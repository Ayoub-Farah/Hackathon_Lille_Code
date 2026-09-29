/*
 * Copyright (c) 2026-present LAAS-CNRS
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 2.1 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @brief  Distributed MMC voltage balancing. SM1 broadcasts arm insertion counts.
 *         This research was funded in whole by the French National Research Agency (ANR) under the project CARROTS "ANR-24-CE05-0920-01".
 *
 * @author Ayoub Farah Hassan <ayoub.farah-hassan@laas.fr>
 * @author Ana Luiza Haas Bezerra <ana-luiza.haas-bezerra@centralesupelec.fr>
 * @author Zaid Jabbar <zaid.jabbar@grenoble-inp.fr>
 * @author Luiz Villa <luiz.villa@laas.fr>
 * @author Jean Alinei <jean.alinei@owntech.org>
 * @author Noemi Lanciotti <noemi.lanciotti@centralesupelec.fr>
 * @author Loïc Quéval <loic.queval@centralesupelec.fr>
 */

/* --------------OWNTECH APIs---------------------------------- */
#include "SpinAPI.h"
#include "TaskAPI.h"
#include "ShieldAPI.h"
#include "CommunicationAPI.h"

/*--------------OWNTECH Libraries----------------------------- */
#include "trigo.h"
#include "arm_math_types.h"
#include <ScopeMimicry.h>
#include <math.h>

/*-- Zephyr includes --*/
#include "zephyr/console/console.h"


#define MMC_SM1 1
#define MMC_SM2 2
#define MMC_SM3 3
#define MMC_SM4 4
#define MMC_SM5 5
#define MMC_SM6 6
#define MMC_SM7 7
#define MMC_SM8 8
#define MMC_SM9 9
#define MMC_SM10 10

#define IDLE 0
#define POWER 1
#define DISCOVERY 2 // SM1 polls the module named by upper_insert_count; PWM stays off.
#define OVER_VOLTAGE 3
#define OVER_CURRENT 5
#define COMMUNICATION_ERROR 6

constexpr uint8_t MMC_SM_COUNT = 10;
constexpr uint8_t MMC_SM_FIRST = MMC_SM1;
constexpr uint8_t MMC_SM_LAST = MMC_SM10;

/* -------------- GENERAL MMC DEFINITIONS -------------------- */
/* --------------- To be changed by user --------------------- */

static const float32_t f0 = 50.F; //[Hz] Output frequency used to generate the sinusoidal reference for open-loop control
static const uint8_t total_number_of_modules_arm = 5; //[-] Number of modules per arm
constexpr float32_t Vcap_expected = 80.0F; //[V] Capacitor DC voltage expected during the test (used to set voltage measurement scale for 12 bits)
constexpr float32_t i_expected = 10.0F; //[A] Expected current amplitude during test (used to set current measurement scale for 12 bits)
constexpr float32_t overvoltage_tolerance = 80.0F; //[V] Set overvoltage tolerance (default max TWIST voltage)
constexpr float32_t overcurrent_tolerance = 8.0F; //[A] Set overcurrent tolerance (default max TWIST current)
constexpr uint8_t debug_module_id = MMC_SM2; // Peer compared with SM1 in ScopeMimicry.
static_assert(debug_module_id >= MMC_SM2 && debug_module_id <= MMC_SM_LAST,
              "Select a debug peer from SM2 to SM10");
constexpr uint8_t ACTION_UNAVAILABLE = 255;

/* -------------- BOARD IDENTIFICATION ----------------------- */
/* --------------- To be changed by user --------------------- */

constexpr uint32_t UID_MMC_SM1_BOARD = 0x0033004C;
constexpr uint32_t UID_MMC_SM2_BOARD = 0x0031001B;
constexpr uint32_t UID_MMC_SM3_BOARD = 0x00330049;
constexpr uint32_t UID_MMC_SM4_BOARD = 0x0033004B;
constexpr uint32_t UID_MMC_SM5_BOARD = 0x00330054;
constexpr uint32_t UID_MMC_SM6_BOARD = 0x0032003F;
constexpr uint32_t UID_MMC_SM7_BOARD = 0x004E0048;
constexpr uint32_t UID_MMC_SM8_BOARD = 0x00470026;
constexpr uint32_t UID_MMC_SM9_BOARD = 0x004C001D;
constexpr uint32_t UID_MMC_SM10_BOARD = 0x0047002B;

/* --------- BOARD IDENTIFICATION functions ------------------ */
static uint32_t read_board_uid()
{
    static volatile uint32_t *const uid0 =
        reinterpret_cast<volatile uint32_t *>(0x1FFF7590UL);
    return *uid0;
}

static uint8_t detect_module_id()
{
    switch (read_board_uid())
    {
    case UID_MMC_SM1_BOARD:
        return MMC_SM1;
    case UID_MMC_SM2_BOARD:
        return MMC_SM2;
    case UID_MMC_SM3_BOARD:
        return MMC_SM3;
    case UID_MMC_SM4_BOARD:
        return MMC_SM4;
    case UID_MMC_SM5_BOARD:
        return MMC_SM5;
    case UID_MMC_SM6_BOARD:
        return MMC_SM6;
    case UID_MMC_SM7_BOARD:
        return MMC_SM7;
    case UID_MMC_SM8_BOARD:
        return MMC_SM8;
    case UID_MMC_SM9_BOARD:
        return MMC_SM9;
    case UID_MMC_SM10_BOARD:
        return MMC_SM10;
    default:
        return 0; // An unknown board must never become a second SM1.
    }
}

/* -------------- DATA PACKING HELPERS ----------------------- */

constexpr float32_t Cap_voltage_SCALE = Vcap_expected*2; //[V] Scale for 12-bit voltage measurements
constexpr float32_t Arm_current_SCALE = i_expected*2; //[A] Scale for 12-bit current measurements
constexpr float32_t Arm_current_OFFSET = i_expected; //[A] Offset for signed current measurements

/**
 * @brief Encode an capacitor voltage into the 12-bit transport format.
 *
 * @param current Physical capacitor voltage in volts.
 * @return 12-bit encoded voltage suitable for MMC frames.
 */
static inline uint16_t mmc_encode_voltage(float32_t voltage)
{
    if (voltage <= 0.0F) return 0;
    if (voltage >= Cap_voltage_SCALE) return 0x0FFF;
    return static_cast<uint16_t>((voltage * 4095.0F) / Cap_voltage_SCALE);
}

/**
 * @brief Decode a raw capacitor voltage value from an MMC frame.
 *
 * @param raw 12-bit encoded capacitor voltage.
 * @return Physical capacitor voltage in volts.
 */
static inline float32_t mmc_decode_voltage(uint16_t raw)
{
    return (Cap_voltage_SCALE * static_cast<float32_t>(raw & 0x0FFF)) / 4095.0F;
}

/**
 * @brief Encode an arm current into the 12-bit transport format.
 *
 * @param current Physical arm current in amperes.
 * @return 12-bit encoded current suitable for MMC frames.
 */
static inline uint16_t mmc_encode_current(float32_t current)
{
    if (current <= -Arm_current_OFFSET) return 0;
    if (current >= Arm_current_SCALE - Arm_current_OFFSET) return 0x0FFF;
    return static_cast<uint16_t>(((current + Arm_current_OFFSET) * 4095.0F) / Arm_current_SCALE);
}




/* --------------SETUP FUNCTIONS DECLARATION------------------- */

/* Setups the hardware and software of the system */
void setup_routine();

/* --------------LOOP FUNCTIONS DECLARATION-------------------- */

/* Code to be executed in the background task - only sets up boards LEDs */
void loop_background_task();
/* Code to be executed in real time in the critical task - executes the control of each module */
void loop_critical_task();
/* Code to be executed in the communication task - serves to send command to board via PC using USB-C cable */
void loop_communication_task();

/* --------------USER VARIABLES DECLARATIONS------------------- */

// Only known boards can participate; SM1 generates the arm insertion counts.
uint8_t module_ID = detect_module_id();
static float32_t module_command = 0.0F; // 1 = inserted, 0 = bypassed.

// SM1 supplies the two global insertion counts.
// 12-byte debug protocol: update all ten boards together.
struct MMC_frame_t
{
    uint8_t upper_insert_count;
    uint8_t lower_insert_count;
    uint16_t cycle_id;
    uint16_t capacitor_voltage_raw : 12;
    uint16_t arm_current_raw : 12;
    uint8_t status;
    uint8_t sm_id;
    // Only bit (sm_id - 1) belongs to this sender; bits 10..15 are zero.
    // Frame C reports the software action applied for C-1, not sensed gate state.
    uint16_t previous_inserted_mask;
    uint8_t previous_status; // Applied POWER/IDLE/fault, or ACTION_UNAVAILABLE.
} __packed;
static_assert(sizeof(MMC_frame_t) == 12, "All modules must use the same 12-byte frame");

static bool mmc_is_upper_arm_module(uint8_t id)
{
    return id >= MMC_SM1 && id <= MMC_SM5;
}

// Exact transmitted samples, including our own, ensure identical local ranks.
static uint16_t capacitor_voltage_raw[MMC_SM_COUNT] = {};
static bool measurement_received[MMC_SM_COUNT] = {};
static uint8_t received_module_count = 0;
static uint16_t arm_current_raw[MMC_SM_COUNT] = {};
static uint8_t module_status[MMC_SM_COUNT] = {};
static uint8_t module_previous_status[MMC_SM_COUNT] = {};
static uint16_t previous_inserted_mask = 0;
static uint16_t action_valid_mask = 0;
static uint16_t previous_action_cycle_id = 0;
static bool previous_action_valid = false;
static uint8_t previous_action_status = ACTION_UNAVAILABLE;
static bool previous_action_inserted = false;
// SM1 supplies the upper-arm current; SM6 supplies the lower-arm current.
static float32_t upper_arm_current_reference = 0.0F;
static float32_t lower_arm_current_reference = 0.0F;

// Scheduling assumption: RS485 reception and control never overlap.
// Each exchange finishes before the next control tick; no interrupt locking.
static MMC_frame_t cycle_command;
static MMC_frame_t next_command; // Prepared by SM1 for the next communication window.
static bool cycle_started = false;
static volatile uint8_t communication_fault = 0;
static volatile bool bus_ready = false;
static volatile uint16_t discovery_present_mask = 1U; // SM1 is local.
static uint16_t discovery_sweep_mask = 1U;
static uint8_t discovery_target = MMC_SM2;
static constexpr uint16_t ALL_MODULES_MASK = (1U << MMC_SM_COUNT) - 1U;

constexpr size_t MMC_FRAME_SIZE = sizeof(MMC_frame_t);
uint8_t buffer_tx[MMC_FRAME_SIZE];
uint8_t buffer_rx[MMC_FRAME_SIZE];

// Finite sensor/control values are assumed. Keep the last value on NO_VALUE.
float32_t Cap_voltage = 0.0F;
static float32_t Arm_current = 0.0F;

uint32_t counter_timer = 0;
uint32_t counter_receive = 0;

uint8_t received_serial_char; // Variable to store the received character from the serial interface
int8_t CommTask_num;


/* --------------- LIST OF POSSIBLE BOARD MODES ------------------*/
enum serial_interface_menu_mode
{
    IDLEMODE = 0, // Related to blocked state
    POWERMODE = 1, // Related to connected/disconnected state
};

volatile serial_interface_menu_mode mode = IDLEMODE;

/* --------------- Distributed insertion variables ------------------*/

/* [us] period of the control task (=critical task) */
static constexpr uint32_t control_task_period = 200; // us
// Let followers enter their synchronized control task before SM1 opens the bus.
// This is a boot grace period, not an acknowledgement that every board is ready.
static constexpr uint32_t communication_startup_delay_us = 1000000; // 1 s
static constexpr uint32_t communication_startup_cycles =
    (communication_startup_delay_us + control_task_period - 1) / control_task_period;
static uint32_t communication_startup_ticks = 0;
static float32_t Ts = control_task_period * 1e-6F; // s
/* [bool] state of the PWM (ctrl task) */
static bool pwm_enable = false;

static uint32_t critical_task_timer = 0; 

/* Scope variables */
// Linear capture on SM1: p starts a new run from IDLE; a/s restart capture.
// i, r, a fault or a full buffer freeze the trace. r exports in IDLE.
// Only main.cpp writes samples, from index 0 onward: no circular acquisition.
// 24 float channels x 512 points = 49,152 bytes, below the library's 64 KiB limit.
// At each row C, vc/i/status and N_* describe exchange C. *_prev describes
// the action for C-1: compare it with N_*/vc/i from row C-1. `inserted` is the
// new SM1 action for C. The peer is selected by debug_module_id above.
// Mask bit 0 = SM1, ... bit 9 = SM10. rx_mask includes SM1's own transmission.
// A zero insertion bit means bypass/stopped ONLY if action_valid_mask has that
// bit set. Missing measurements/actions are NaN in the self/peer channels.
// prev_status: 0 stopped, 1 PWM enabled (inserted OR bypass), 3/5/6 fault.
// fault_mask identifies frames REPORTING faults; propagated faults need not
// originate in the reporting module. `fault` is SM1's current local/received fault.
static const uint16_t NB_DATAS = 512; // 102.4 ms at 200 us/sample.
static constexpr uint16_t SCOPE_CHANNELS = 24;
static_assert(NB_DATAS * SCOPE_CHANNELS * sizeof(float32_t) <= UINT16_MAX,
              "ScopeMimicry exports its buffer size as uint16_t");
static ScopeMimicry scope(NB_DATAS, SCOPE_CHANNELS);
struct DebugScopeData
{
    float32_t cycle_id, N_upper, N_lower, i_upper_ref, i_lower_ref;
    float32_t inserted_prev_mask, rx_mask, action_valid_mask, fault_mask, fault, mode, debug_sm;
    float32_t vc_self, vc_peer, i_self, i_peer, status_self, status_peer;
    float32_t prev_status_self, prev_status_peer, inserted_self_prev, inserted_peer_prev;
    float32_t inserted, rx_count;
};
static DebugScopeData debug_scope = {};
static_assert(sizeof(DebugScopeData) == SCOPE_CHANNELS * sizeof(float32_t),
              "Debug fields must match the connected channel order");
static bool scope_ready = true;
static bool scope_active = false;
static bool scope_stop_requested = false;
static bool scope_rearm_requested = false;
static uint16_t scope_samples_recorded = 0;
static bool is_downloading; // Export requested; keep the buffer frozen until done.
static uint32_t scope_timer = 0;
static uint32_t scope_period = 1; // scope acquire data every t = scope_period * critical_task_period (200 µs) s;

/* NLM modulation on SM1; preserve the firmware's upper/lower sine signs. */
static float32_t m = 1; // Modulation amplitude, used by SM1
static float32_t a = 1; // Modulation dc part, used by SM1
static float32_t angle;
static const float32_t w0 = 2 * PI * f0; // Angular frequency

static void mmc_compute_insertion_counts(float32_t sine, MMC_frame_t &frame)
{
    const float32_t upper = 0.5F * (a + m * sine);
    const float32_t lower = 0.5F * (a - m * sine);
    frame.upper_insert_count = static_cast<uint8_t>(roundf(total_number_of_modules_arm * upper));
    frame.lower_insert_count = static_cast<uint8_t>(roundf(total_number_of_modules_arm * lower));
}

/* --------------SETUP FUNCTIONS------------------------------- */

/* Function to control the LEDs in the low level */
void config_led_LL()
{
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_5, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_5, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_5, LL_GPIO_PULL_NO);
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);
}

inline void Led_turnON_LL()
{
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5);
}

inline void Led_turnOFF_LL()
{
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);
}

/* Records scope data */
void dump_scope_datas(ScopeMimicry &scope)
{
    uint8_t *buffer = scope.get_buffer();
    const uint16_t buffer_size = scope_samples_recorded * scope.get_nb_channel();
    printk("begin record\n");
    printk("#");
    for (uint16_t k = 0; k < scope.get_nb_channel(); k++)
    {
        printk("%s,", scope.get_channel_name(k));
    }
    printk("\n");
    printk("# -1\n"); // Already chronological: readers must not rotate rows.
    for (uint16_t k = 0; k < buffer_size; k++)
    {
        uint32_t value;
        memcpy(&value, buffer + k * sizeof(value), sizeof(value));
        printk("%08x\n", value);
        task.suspendBackgroundUs(100);
    }
    printk("end record\n");
}

static void update_measurements(void)
{
    float32_t latest = shield.sensors.getLatestValue(V_HIGH);
    if (latest != NO_VALUE)
        Cap_voltage = latest;
    latest = shield.sensors.getLatestValue(I2_LOW);
    if (latest != NO_VALUE)
        Arm_current = -latest; // Preserve the arm-current sign convention on LEG2.
}

static void mmc_check_power_limits()
{
    if (fabsf(Arm_current) > overcurrent_tolerance) communication_fault = OVER_CURRENT;
    else if (Cap_voltage > overvoltage_tolerance) communication_fault = OVER_VOLTAGE;
}

// Called once per reception window, including on SM1.
static bool begin_cycle(const MMC_frame_t &frame)
{
    if (frame.sm_id != MMC_SM1) return false;
    if (received_module_count != 0)
    {
        if (frame.cycle_id != cycle_command.cycle_id)
            communication_fault = COMMUNICATION_ERROR;
        return false;
    }
    if (frame.status == IDLE || frame.status == DISCOVERY) communication_fault = 0;
    if (cycle_started)
    {
        const uint16_t advance = static_cast<uint16_t>(frame.cycle_id - cycle_command.cycle_id);
        if (advance != 1)
            communication_fault = COMMUNICATION_ERROR;
    }

    cycle_command = frame;
    cycle_started = true;
    return true;
}

static void store_cycle_sample(const MMC_frame_t &frame)
{
    const uint8_t index = frame.sm_id - MMC_SM_FIRST;
    const uint16_t bit = static_cast<uint16_t>(1U << index);
    capacitor_voltage_raw[index] = frame.capacitor_voltage_raw;
    arm_current_raw[index] = frame.arm_current_raw;
    module_status[index] = frame.status;
    module_previous_status[index] = frame.previous_status;
    // Never trust a sender's copy of another module's action.
    previous_inserted_mask &= static_cast<uint16_t>(~bit);
    action_valid_mask &= static_cast<uint16_t>(~bit);
    if (frame.previous_status != ACTION_UNAVAILABLE)
    {
        previous_inserted_mask |= frame.previous_inserted_mask & bit;
        action_valid_mask |= bit;
    }
    const float32_t current = Arm_current_SCALE * static_cast<float32_t>(frame.arm_current_raw) / 4095.0F
        - Arm_current_OFFSET;
    if (frame.sm_id == MMC_SM1) upper_arm_current_reference = current;
    if (frame.sm_id == MMC_SM6) lower_arm_current_reference = current;
}

// Count accepted frames and keep measurements from this exchange.
static bool store_module_measurements(const MMC_frame_t &frame)
{
    if (!cycle_started || frame.sm_id < MMC_SM_FIRST || frame.sm_id > MMC_SM_LAST)
        return false;
    if (frame.sm_id != MMC_SM1 && !measurement_received[0]) return false;
    if (frame.cycle_id != cycle_command.cycle_id)
    {
        communication_fault = COMMUNICATION_ERROR;
        return false;
    }
    const uint8_t index = frame.sm_id - MMC_SM_FIRST;
    if (measurement_received[index]) return false;

    if (frame.status == OVER_VOLTAGE || frame.status == OVER_CURRENT ||
        frame.status == COMMUNICATION_ERROR)
        communication_fault = frame.status;
    measurement_received[index] = true;
    received_module_count++;
    store_cycle_sample(frame);
    return true;
}

static void send_own_measurements()
{
    const uint8_t index = module_ID - MMC_SM_FIRST;
    if (measurement_received[index]) return;
    mmc_check_power_limits();
    MMC_frame_t frame = cycle_command;
    frame.sm_id = module_ID;
    frame.capacitor_voltage_raw = mmc_encode_voltage(Cap_voltage);
    frame.arm_current_raw = mmc_encode_current(Arm_current);
    // SM1 must preserve the discovery request even after an incomplete run.
    if (communication_fault && !(module_ID == MMC_SM1 && cycle_command.status == DISCOVERY))
        frame.status = communication_fault;
    const bool action_is_previous = previous_action_valid &&
        static_cast<uint16_t>(previous_action_cycle_id + 1U) == frame.cycle_id;
    frame.previous_status = action_is_previous ? previous_action_status : ACTION_UNAVAILABLE;
    frame.previous_inserted_mask = action_is_previous && previous_action_inserted ?
        static_cast<uint16_t>(1U << index) : 0;
    store_cycle_sample(frame);
    // Include our own slot in the round without waiting for an echo.
    measurement_received[index] = true;
    received_module_count++;
    memcpy(buffer_tx, &frame, sizeof(frame));
    communication.rs485.startTransmission();
}

void reception_function()
{
    MMC_frame_t frame;
    memcpy(&frame, buffer_rx, sizeof(frame));
    if (frame.sm_id == module_ID) return;
    if (frame.sm_id == MMC_SM1)
    {
        if (module_ID == MMC_SM1 || !begin_cycle(frame)) return;
        mode = frame.status == POWER ? POWERMODE : IDLEMODE;
    }
    const bool accepted = store_module_measurements(frame);
    // Bus order: SM1 -> SM2 -> ... -> SM10, across both arms.
    if (module_ID != MMC_SM1 && accepted)
    {
        if (cycle_command.status == DISCOVERY)
        {
            // Independent polling: a missing SM2 must not silence SM3..SM10.
            if (frame.sm_id == MMC_SM1 && cycle_command.upper_insert_count == module_ID)
                send_own_measurements();
        }
        else if (frame.sm_id == module_ID - 1)
            send_own_measurements();
    }
    counter_receive++;
}

/**
 * This is the setup routine.
 * It is used to call functions that will initialize your spin, power shields
 * and tasks.
 */
void setup_routine()
{
    /* Informs the module ID in the terminal */
    const uint32_t board_uid = read_board_uid();
    printk("Board UID: 0x%08" PRIX32 "\n", board_uid);
    printk("Module ID : %u \n", module_ID);
    if (module_ID < MMC_SM_FIRST || module_ID > MMC_SM_LAST)
    {
        printk("Unknown board: control and communication disabled\n");
        return;
    }

    config_led_LL(); // Configure the LED pin in Low Level

    // PWMA remains the synchronization reference; PWMC drives LEG2.
    shield.power.initBuck(ALL);
    /* Declare task */
    uint32_t background_task_number =
        task.createBackground(loop_background_task);

    task.createCritical(loop_critical_task, control_task_period);

    shield.sensors.enableDefaultTwistSensors();

    if(module_ID == MMC_SM1)
    {
        shield.sensors.setConversionParametersLinear(V_HIGH, 0.0298174F, -0.116135F);
        shield.sensors.setConversionParametersLinear(I_HIGH, 0.00542287F, -10.9166F);
        shield.sensors.setConversionParametersLinear(V1_LOW, 0.0452099F, -89.9364F);
        shield.sensors.setConversionParametersLinear(I1_LOW, 0.0056548F, -11.6536F);
        shield.sensors.setConversionParametersLinear(V2_LOW, 0.045172F, -88.5664F);
        shield.sensors.setConversionParametersLinear(I2_LOW, 0.00576626F, -12.6195F);

    }
    if(module_ID == MMC_SM6)
    {
        shield.sensors.setConversionParametersLinear(V_HIGH, 0.029749F, 0.17888F);
        shield.sensors.setConversionParametersLinear(I_HIGH, 0.00566004F, -10.9416F);
        shield.sensors.setConversionParametersLinear(V1_LOW, 0.045442F, -86.4199F);
        shield.sensors.setConversionParametersLinear(I1_LOW, 0.00569674F, -11.8135F);
        shield.sensors.setConversionParametersLinear(V2_LOW, 0.0453543F, -86.2705F);
        shield.sensors.setConversionParametersLinear(I2_LOW, 0.00478794F, -10.687F);
    }

    /* Disconnect electrolytical capacitors from low-side */
    shield.power.disconnectCapacitor(LEG1);
    shield.power.disconnectCapacitor(LEG2);

    /* Enable switch control with max and min duty cycle of 1 and 0 */
    shield.power.setDutyCycleMax(ALL,1.0);
    shield.power.setDutyCycleMin(ALL,0.0);

    CommTask_num = task.createBackground(loop_communication_task);

    communication.rs485.configure(buffer_tx, buffer_rx, sizeof(buffer_rx),
                                  reception_function,
                                  SPEED_20M); // custom configuration for RS485
                                              /* Configure scope channels, what measurements do you want to acquire? */
    if (module_ID == MMC_SM1)
    {
        /* SM1 supplies the common control clock. */
        communication.sync.initMaster();

        /* Configures scopemimicry measured variables */
        scope.connectChannel(debug_scope.cycle_id, "cycle_id");
        scope.connectChannel(debug_scope.N_upper, "N_upper");
        scope.connectChannel(debug_scope.N_lower, "N_lower");
        scope.connectChannel(debug_scope.i_upper_ref, "i_upper_ref");
        scope.connectChannel(debug_scope.i_lower_ref, "i_lower_ref");
        scope.connectChannel(debug_scope.inserted_prev_mask, "inserted_prev_mask");
        scope.connectChannel(debug_scope.rx_mask, "rx_mask");
        scope.connectChannel(debug_scope.action_valid_mask, "action_valid_mask");
        scope.connectChannel(debug_scope.fault_mask, "fault_mask");
        scope.connectChannel(debug_scope.fault, "fault");
        scope.connectChannel(debug_scope.mode, "mode");
        scope.connectChannel(debug_scope.debug_sm, "debug_sm");
        scope.connectChannel(debug_scope.vc_self, "vc_self");
        scope.connectChannel(debug_scope.vc_peer, "vc_peer");
        scope.connectChannel(debug_scope.i_self, "i_self");
        scope.connectChannel(debug_scope.i_peer, "i_peer");
        scope.connectChannel(debug_scope.status_self, "status_self");
        scope.connectChannel(debug_scope.status_peer, "status_peer");
        scope.connectChannel(debug_scope.prev_status_self, "prev_status_self");
        scope.connectChannel(debug_scope.prev_status_peer, "prev_status_peer");
        scope.connectChannel(debug_scope.inserted_self_prev, "inserted_self_prev");
        scope.connectChannel(debug_scope.inserted_peer_prev, "inserted_peer_prev");
        scope.connectChannel(debug_scope.inserted, "inserted");
        scope.connectChannel(debug_scope.rx_count, "rx_count");
        // The buffer and channel names come from ScopeMimicry; main.cpp owns
        // linear writes and export length. Do not call its circular acquire().

    }
    else{
        /* Defines module as follower for communication synchorinization */
        communication.sync.initSlave();
    }
    shield.power.stop(ALL);
    // Binary insertion uses the same switching phase on every module.
    shield.power.setPhaseShift(LEG1, 0);
    task.startBackground(background_task_number);
    task.startBackground(CommTask_num);
    task.startCritical();
}

/* --------------LOOP FUNCTIONS-------------------------------- */

/**
 * This is the communication task.
 * It is used to send to the board via the computer the desired mode
 * IDLE (i) = block all modules or POWER (p) = operate with distributed capacitor-voltage sorting.
 * 
 * Scope: p starts from IDLE, a/s restart, i/r freeze, r exports in IDLE.
 */
void loop_communication_task()
{
    received_serial_char = console_getchar();

    switch (received_serial_char)
    {
    case 'h':
        /*----------SERIAL INTERFACE MENU----------------------- */
        printk(" ________________________________________ \n"
               "|     ---- MENU buck voltage mode ----   |\n"
               "|     press i : idle mode                |\n"
               "|     press p : power mode               |\n"
               "|     press r : freeze/export in idle    |\n"
               "|     press a : restart linear capture   |\n"
               "|     press s : restart linear capture   |\n"
               "|________________________________________|\n\n");
        /*------------------------------------------------------ */
        break;
    case 'i':
        mode = IDLEMODE;
        if (module_ID == MMC_SM1) scope_stop_requested = true;
        printk("idle mode\n");
        break;
    case 'p':
        if (module_ID == MMC_SM1)
        {
            if (!bus_ready)
            {
                printk("Power blocked: bus discovery incomplete (present mask 0x%03x)\n",
                       static_cast<unsigned>(discovery_present_mask));
                break;
            }
            if (mode == IDLEMODE && !is_downloading)
            {
                scope_rearm_requested = true;
                scope_stop_requested = false;
            }
            mode = POWERMODE;
            printk("power mode\n");
        }
        break;
    case 'r':
        if (module_ID == MMC_SM1)
        {
            scope_stop_requested = true;
            is_downloading = true;
        }
        break;
    case 'a':
    case 's':
        if (module_ID == MMC_SM1 && !is_downloading)
        {
            scope_rearm_requested = true;
            scope_stop_requested = false;
        }
        break;
    default:
        break;
    }
}

/**
 * This is the code loop of the background task
 * It runs perpetually. Here a `suspendBackgroundMs` is used to pause during
 * 2000ms between each LED toggles.
 * Hence we expect the LED to blink each 2 seconds.
 */
void loop_background_task()
{
    if (module_ID == MMC_SM1)
    {
        static uint16_t last_present_mask = 0;
        static bool last_ready = false;
        const uint16_t present = discovery_present_mask;
        const bool ready = bus_ready;
        if (present != last_present_mask || ready != last_ready)
        {
            printk("Bus %s; present mask 0x%03x; missing:",
                   ready ? "ready (press p)" : "discovery", static_cast<unsigned>(present));
            for (uint8_t id = MMC_SM2; id <= MMC_SM_LAST; ++id)
                if (!(present & (1U << (id - 1)))) printk(" SM%u", id);
            printk("\n");
            last_present_mask = present;
            last_ready = ready;
        }
        if (mode == IDLEMODE)
        {
            spin.led.turnOff();
            if (is_downloading && scope_ready)
            {
                if (scope_samples_recorded != 0) dump_scope_datas(scope);
                else printk("No scope samples: press p or a to start capture\n");
                is_downloading = false;
            }
        }
        if (mode == POWERMODE)
        {
            spin.led.toggle();
        }
    }

    task.suspendBackgroundMs(2000);
}

// Rank against shared quantized voltages; ties favor the smaller SM identifier.
static float32_t mmc_local_insertion()
{
    const bool upper = mmc_is_upper_arm_module(module_ID);
    const uint8_t first = upper ? MMC_SM1 : MMC_SM6;
    const uint8_t count = upper ? cycle_command.upper_insert_count : cycle_command.lower_insert_count;
    const float32_t current = upper ? upper_arm_current_reference : lower_arm_current_reference;
    const uint16_t own_voltage = capacitor_voltage_raw[module_ID - MMC_SM_FIRST];
    uint8_t rank = 0;
    for (uint8_t id = first; id < first + total_number_of_modules_arm; ++id)
    {
        const uint16_t voltage = capacitor_voltage_raw[id - MMC_SM_FIRST];
        const bool ahead = current >= 0.0F ? voltage < own_voltage : voltage > own_voltage;
        if (ahead || (voltage == own_voltage && id < module_ID)) ++rank;
    }
    return rank < count ? 1.0F : 0.0F;
}

// Latch the software action AFTER the PWM calls, for the next exchange.
static void mmc_latch_action()
{
    previous_action_valid = cycle_started;
    previous_action_cycle_id = cycle_command.cycle_id;
    previous_action_status = communication_fault ? communication_fault : (pwm_enable ? POWER : IDLE);
    previous_action_inserted = pwm_enable && module_command == 1.0F;
}

// One coherent snapshot of the consumed exchange, before reception flags reset.
static void mmc_update_debug_scope()
{
    uint16_t received_mask = 0;
    uint16_t fault_mask = 0;
    for (uint8_t index = 0; index < MMC_SM_COUNT; ++index)
    {
        if (!measurement_received[index]) continue;
        const uint16_t bit = static_cast<uint16_t>(1U << index);
        received_mask |= bit;
        const uint8_t status = module_status[index];
        if (status == OVER_VOLTAGE || status == OVER_CURRENT || status == COMMUNICATION_ERROR)
            fault_mask |= bit;
    }
    const uint8_t peer = debug_module_id - MMC_SM_FIRST;
    const bool self_received = measurement_received[0];
    const bool peer_received = measurement_received[peer];
    const uint16_t valid_actions = action_valid_mask & received_mask;
    const bool self_action_valid = (valid_actions & 1U) != 0;
    const bool peer_action_valid = (valid_actions & (1U << peer)) != 0;
    debug_scope.cycle_id = cycle_started ? cycle_command.cycle_id : NAN;
    debug_scope.N_upper = self_received ? cycle_command.upper_insert_count : NAN;
    debug_scope.N_lower = self_received ? cycle_command.lower_insert_count : NAN;
    debug_scope.i_upper_ref = self_received ? upper_arm_current_reference : NAN;
    debug_scope.i_lower_ref = measurement_received[MMC_SM6 - MMC_SM_FIRST] ? lower_arm_current_reference : NAN;
    debug_scope.inserted_prev_mask = previous_inserted_mask & valid_actions;
    debug_scope.rx_mask = received_mask;
    debug_scope.action_valid_mask = valid_actions;
    debug_scope.fault_mask = fault_mask;
    debug_scope.fault = communication_fault;
    debug_scope.mode = mode;
    debug_scope.debug_sm = debug_module_id;
    debug_scope.vc_self = self_received ? mmc_decode_voltage(capacitor_voltage_raw[0]) : NAN;
    debug_scope.vc_peer = peer_received ? mmc_decode_voltage(capacitor_voltage_raw[peer]) : NAN;
    debug_scope.i_self = self_received ? Arm_current_SCALE * arm_current_raw[0] / 4095.0F - Arm_current_OFFSET : NAN;
    debug_scope.i_peer = peer_received ? Arm_current_SCALE * arm_current_raw[peer] / 4095.0F - Arm_current_OFFSET : NAN;
    debug_scope.status_self = self_received ? module_status[0] : NAN;
    debug_scope.status_peer = peer_received ? module_status[peer] : NAN;
    debug_scope.prev_status_self = self_action_valid ? module_previous_status[0] : NAN;
    debug_scope.prev_status_peer = peer_action_valid ? module_previous_status[peer] : NAN;
    debug_scope.inserted_self_prev = self_action_valid ? ((previous_inserted_mask & 1U) ? 1.0F : 0.0F) : NAN;
    debug_scope.inserted_peer_prev = peer_action_valid ? ((previous_inserted_mask & (1U << peer)) ? 1.0F : 0.0F) : NAN;
    debug_scope.inserted = module_command;
    debug_scope.rx_count = received_module_count;
}

/**
 * This is the code loop of the critical task
 * It is executed every 200 micro-seconds defined in the setup_software
 * function.
 *
 * In the critical task, we implement the MMC control algorithms that will
 * run in Real Time.
 */
void loop_critical_task()
{
    if (module_ID == MMC_SM1 && communication_startup_ticks < communication_startup_cycles)
    {
        ++communication_startup_ticks;
        mode = IDLEMODE;
        module_command = 0.0F;
        if (pwm_enable) shield.power.stop(ALL);
        pwm_enable = false;
        update_measurements();
        mmc_check_power_limits();
        // Keep the control/synchronization timers running; only defer exchanges.
        return;
    }
    if (module_ID == MMC_SM1 && scope_rearm_requested)
    {
        if (!is_downloading)
        {
            scope_ready = false;
            scope_active = true;
            scope_timer = 0;
            scope_samples_recorded = 0;
        }
        scope_rearm_requested = false;
    }
    // The preceding exchange is finished before control starts.
    const bool round_complete = received_module_count == MMC_SM_COUNT;
    const bool discovery_round = cycle_started && cycle_command.status == DISCOVERY;
    if (module_ID == MMC_SM1 && discovery_round)
    {
        if (discovery_target == MMC_SM2) discovery_sweep_mask = 1U;
        if (measurement_received[discovery_target - 1])
            discovery_sweep_mask |= (1U << (discovery_target - 1));
        if (discovery_target == MMC_SM_LAST)
        {
            discovery_present_mask = discovery_sweep_mask;
            bus_ready = discovery_sweep_mask == ALL_MODULES_MASK;
            discovery_target = MMC_SM2;
        }
        else ++discovery_target;
    }
    if (cycle_started && !discovery_round && !round_complete)
    {
        communication_fault = COMMUNICATION_ERROR;
        if (module_ID == MMC_SM1)
        {
            bus_ready = false;
            discovery_target = MMC_SM2;
            discovery_present_mask = 1U;
        }
    }
    update_measurements();
    mmc_check_power_limits();

    if (communication_fault || discovery_round || (module_ID == MMC_SM1 && !bus_ready))
        mode = IDLEMODE;

    // Apply insertion only once all samples for the POWER cycle are available.
    const bool apply_power = mode == POWERMODE && round_complete && cycle_command.status == POWER;
    module_command = 0.0F;
    if (apply_power)
    {
        module_command = mmc_local_insertion();
        Led_turnON_LL();
    }
    if (apply_power)
    {
        shield.power.setDutyCycle(LEG1, module_command);
        if (!pwm_enable) shield.power.start(LEG1);
        pwm_enable = true;
    }
    else
    {
        if (pwm_enable) shield.power.stop(ALL);
        pwm_enable = false;
        module_command = 0.0F;
        if (!module_ID == MMC_SM1)
        {
            Led_turnOFF_LL();
        }
    }

    mmc_latch_action();
    if (module_ID == MMC_SM1 && scope_active)
    {
        const bool stop_capture = scope_stop_requested || communication_fault != 0;
        if (++scope_timer >= scope_period || stop_capture)
        {
            mmc_update_debug_scope();
            if (scope_samples_recorded < NB_DATAS)
            {
                memcpy(scope.get_buffer() + scope_samples_recorded * sizeof(DebugScopeData),
                       &debug_scope, sizeof(debug_scope));
                ++scope_samples_recorded;
            }
            scope_timer = 0;
        }
        if (stop_capture || scope_samples_recorded == NB_DATAS)
        {
            scope_active = false;
            scope_ready = true;
        }
    }
    if (apply_power) critical_task_timer++;
    counter_timer++;

    // Open the next exchange only after consuming the previous window.
    received_module_count = 0;
    previous_inserted_mask = 0;
    action_valid_mask = 0;
    for (uint8_t i = 0; i < MMC_SM_COUNT; ++i) measurement_received[i] = false;
    if (module_ID == MMC_SM1)
    {
        next_command = {};
        next_command.sm_id = MMC_SM1;
        next_command.cycle_id = static_cast<uint16_t>(cycle_command.cycle_id + 1U);
        if (!bus_ready)
        {
            next_command.status = DISCOVERY;
            next_command.upper_insert_count = discovery_target;
            angle = 0.0F;
        }
        else if (mode == POWERMODE)
        {
            angle = ot_modulo_2pi(angle + w0 * Ts);
            mmc_compute_insertion_counts(ot_sin(angle), next_command);
            next_command.status = POWER;
        }
        else angle = 0.0F;
        if (begin_cycle(next_command)) send_own_measurements();
    }
}

/**
 * This is the main function of this example
 * This function is generic and does not need editing.
 */
int main(void)
{
    setup_routine();

    return 0;
}
