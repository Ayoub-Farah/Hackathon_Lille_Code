"""Host checks for startup discovery using the firmware's actual control/RX functions.
Run with Python and clang++ (no board required).
"""
from pathlib import Path
import re
import subprocess
import tempfile

def function(source, name):
    match = re.search(r"^(?:static )?(?:inline )?\w+ " + name + r"\([^;{}]*\)\s*\{", source, re.M)
    assert match, name
    start = source.index("{", match.start())
    depth = 1
    end = start + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end]


DEFINITIONS = r'''
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#undef assert
#define assert(condition) do { if (!(condition)) { std::fprintf(stderr, "Assertion at line %d: %s\n", __LINE__, #condition); std::exit(1); } } while (0)
#define __packed __attribute__((packed))
using float32_t = float;
constexpr uint8_t MMC_SM1 = 1, MMC_SM2 = 2, MMC_SM5 = 5, MMC_SM6 = 6;
constexpr uint8_t MMC_SM_FIRST = 1, MMC_SM_LAST = 10, MMC_SM_COUNT = 10;
constexpr uint8_t total_number_of_modules_arm = 5;
constexpr uint8_t IDLE = 0, POWER = 1, COMMUNICATION_ERROR = 6;
constexpr uint8_t OVER_VOLTAGE = 3, OVER_CURRENT = 5;
constexpr uint8_t ACTION_UNAVAILABLE = 255;
constexpr float Cap_voltage_SCALE = 160, Arm_current_SCALE = 20, Arm_current_OFFSET = 10;
constexpr float overvoltage_tolerance = 80, overcurrent_tolerance = 8;
float Cap_voltage = 40, Arm_current = 0;
float a = 1, m = 1;
uint8_t module_ID = 1;
uint16_t capacitor_voltage_raw[10] = {};
uint16_t arm_current_raw[10] = {};
uint8_t module_status[10] = {}, module_previous_status[10] = {};
uint16_t previous_inserted_mask = 0, action_valid_mask = 0;
uint16_t previous_action_cycle_id = 0;
bool previous_action_valid = false;
uint8_t previous_action_status = 255;
bool previous_action_inserted = false;
constexpr uint8_t debug_module_id = MMC_SM2;
bool measurement_received[10] = {};
uint8_t received_module_count = 0, communication_fault = 0;
bool cycle_started = false;
float upper_arm_current_reference = 0, lower_arm_current_reference = 0;
enum serial_interface_menu_mode { IDLEMODE, POWERMODE };
volatile serial_interface_menu_mode mode = IDLEMODE;
bool pwm_enable = false, enable_acq = false, is_downloading = false;
bool scope_ready = false, scope_fault_armed = false, scope_fault_triggered = false;
bool scope_rearm_requested = false;
constexpr uint16_t NB_DATAS = 512;
uint16_t scope_samples_recorded = 0;
uint16_t scope_write_index = 0;
uint8_t received_serial_char = 0, console_char = 0;
int console_getchar() { return console_char; }
#include <cstdarg>
#include <string>
std::string output;
void printk(const char* format, ...) {
    char text[512]; va_list args; va_start(args, format);
    vsnprintf(text, sizeof(text), format, args); va_end(args); output += text;
}
constexpr int LEG2 = 2, ALL = 0;
struct Power {
    bool running = false;
    float duty = 0;
    void setDutyCycle(int, float value) { duty = value; }
    void start(int) { running = true; }
    void stop(int) { running = false; }
};
struct { Power power; } shield;
struct ScopeMimicry {
    uint8_t memory[512*24*4] = {};
    uint8_t* get_buffer() { return memory; }
    uint16_t get_nb_channel() { return 24; }
    const char* get_channel_name(int) { return "channel"; }
} scope;
struct { struct { unsigned transmissions = 0; void startTransmission() { ++transmissions; } } rs485; } communication;
struct { struct { void turnOff() {} void toggle() {} } led; } spin;
struct { void suspendBackgroundMs(unsigned) {} void suspendBackgroundUs(unsigned) {} } task;
unsigned dumps = 0;

float module_command = 0, upper_insert_count_scope = 0, lower_insert_count_scope = 0;
float angle = 0, w0 = 314.159F, Ts = 0.0002F;
unsigned scope_timer = 0, scope_period = 1, critical_task_timer = 0, counter_timer = 0;
unsigned counter_receive = 0;
float ot_modulo_2pi(float value) { return std::fmod(value, 6.283185F); }
float ot_sin(float value) { return std::sin(value); }
void update_measurements() {}
void Led_turnON_LL() {}

bool scope_active=false, scope_stop_requested=false;
constexpr int LEG1=1; void Led_turnOFF_LL() {}
'''

def main(checks=None):
    root = Path(__file__).resolve().parents[1]
    source = (root / 'src/main.cpp').read_text(encoding='utf-8')
    startup = source[source.index('static constexpr uint32_t control_task_period'):source.index('static float32_t Ts')]
    discovery = source[source.index('static volatile bool bus_ready'):source.index('constexpr size_t MMC_FRAME_SIZE')]
    frame = source[source.index('struct MMC_frame_t'):source.index('} __packed;')+len('} __packed;')]
    debug = re.search(r'struct DebugScopeData\s*\{[^}]+\};', source).group()
    functions = '\n'.join(function(source, name) for name in (
        'mmc_is_upper_arm_module', 'mmc_encode_voltage', 'mmc_decode_voltage', 'mmc_encode_current',
        'mmc_compute_insertion_counts', 'begin_cycle', 'store_cycle_sample', 'store_module_measurements',
        'mmc_local_insertion', 'mmc_check_power_limits', 'send_own_measurements', 'reception_function',
        'mmc_latch_action', 'mmc_update_debug_scope', 'dump_scope_datas', 'loop_communication_task',
        'loop_background_task', 'loop_critical_task'))
    program = (DEFINITIONS + '\nconstexpr uint8_t DISCOVERY=2;\n' + startup + discovery + frame
        + '\nMMC_frame_t cycle_command,next_command;\nuint8_t buffer_tx[sizeof(MMC_frame_t)],buffer_rx[sizeof(MMC_frame_t)];\n'
        + debug + '\nDebugScopeData debug_scope;\n' + functions + (CHECKS if checks is None else checks))
    with tempfile.TemporaryDirectory(prefix='mmc-discovery-') as temp:
        cpp = Path(temp) / 'test.cpp'
        exe = Path(temp) / 'test.exe'
        cpp.write_text(program)
        subprocess.run(['clang++', '-std=c++17', str(cpp), '-o', str(exe)], check=True)
        subprocess.run([str(exe)], check=True)

CHECKS = r'''
void key(char c) { console_char=c; loop_communication_task(); }
void receive(uint8_t id, uint16_t cycle, uint8_t status=IDLE) {
    MMC_frame_t f=cycle_command;
    f.sm_id=id; f.cycle_id=cycle; f.status=status;
    memcpy(buffer_rx,&f,sizeof(f)); reception_function();
}
void clear_window() {
    received_module_count=0;
    std::fill_n(measurement_received,10,false);
}
int main() {
    assert(communication_startup_cycles==5000);
    for(unsigned i=0;i<communication_startup_cycles;++i) {
        key('p'); loop_critical_task();
        assert(mode==IDLEMODE && !pwm_enable && !shield.power.running);
        assert(communication.rs485.transmissions==0 && !cycle_started);
    }
    loop_critical_task();
    assert(cycle_command.status==DISCOVERY && cycle_command.upper_insert_count==2);
    assert(communication.rs485.transmissions==1);
    // A missing SM2 does not prevent probing SM3..10.
    for(unsigned id=2;id<=10;++id) {
        assert(cycle_command.status==DISCOVERY && cycle_command.upper_insert_count==id);
        key('p'); assert(mode==IDLEMODE);
        if(id!=2) receive(id,cycle_command.cycle_id);
        loop_critical_task();
        assert(!bus_ready && !shield.power.running && communication_fault==0);
    }
    assert(discovery_present_mask==(ALL_MODULES_MASK & ~2U));
    output.clear(); loop_background_task();
    assert(output.find(" SM2")!=std::string::npos);
    // A stale reply cannot make discovery succeed.
    receive(2,cycle_command.cycle_id-1);
    assert(!measurement_received[1]);
    // A subsequent complete sweep unlocks normal IDLE exchanges.
    for(unsigned id=2;id<=10;++id) {
        receive(id,cycle_command.cycle_id);
        loop_critical_task();
        assert(!shield.power.running);
        if(id<10) assert(!bus_ready);
    }
    assert(bus_ready && discovery_present_mask==ALL_MODULES_MASK);
    assert(cycle_command.status==IDLE);
    for(unsigned id=2;id<=10;++id) receive(id,cycle_command.cycle_id);
    key('p'); loop_critical_task();
    assert(cycle_command.status==POWER && !shield.power.running);
    for(unsigned id=2;id<=10;++id) receive(id,cycle_command.cycle_id,POWER);
    loop_critical_task(); assert(shield.power.running);
    // Loss in normal operation stops power and returns to discovery.
    loop_critical_task();
    assert(!bus_ready && !shield.power.running && mode==IDLEMODE);
    assert(cycle_command.status==DISCOVERY && cycle_command.upper_insert_count==2);
    key('p'); assert(mode==IDLEMODE);
    // Followers answer only their own poll, directly to SM1, even in fault.
    module_ID=3; clear_window(); cycle_started=false; communication_fault=0;
    unsigned before=communication.rs485.transmissions;
    MMC_frame_t poll{}; poll.sm_id=1; poll.status=DISCOVERY;
    poll.cycle_id=100; poll.upper_insert_count=2;
    memcpy(buffer_rx,&poll,sizeof(poll)); reception_function();
    assert(communication.rs485.transmissions==before);
    loop_critical_task(); // No complete-round fault during discovery.
    assert(communication_fault==0 && mode==IDLEMODE);
    poll.cycle_id=101; poll.upper_insert_count=3;
    Cap_voltage=81;
    memcpy(buffer_rx,&poll,sizeof(poll)); reception_function();
    assert(communication.rs485.transmissions==before+1 && !pwm_enable);
    MMC_frame_t response; memcpy(&response,buffer_tx,sizeof(response));
    assert(response.sm_id==3 && response.cycle_id==101 && response.status==OVER_VOLTAGE);
    reception_function(); assert(communication.rs485.transmissions==before+1);
    puts("PASS: startup delay, independent polls, missing/stale/duplicate replies, power interlock, recovery and follower fault reply");
}
'''

if __name__ == '__main__':
    main()
