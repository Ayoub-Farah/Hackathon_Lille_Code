"""Exercise the actual control loop and chronological export with a delayed fault.

Run: py tests/test_scope_capture.py (requires clang++).
"""
from test_bus_discovery import CHECKS, main


SCOPE_CHECKS = CHECKS.split('int main() {')[0] + r'''
#include <vector>
#include <sstream>

void complete_cycle() {
    for (unsigned id=2; id<=10; ++id)
        receive(id, cycle_command.cycle_id, cycle_command.status);
    loop_critical_task();
}

void check_export(const std::vector<DebugScopeData>& expected) {
    output.clear(); dump_scope_datas(scope);
    std::istringstream stream(output);
    std::string line;
    std::getline(stream,line); assert(line=="begin record");
    std::getline(stream,line); assert(line[0]=='#');
    std::getline(stream,line); assert(line=="# -1");
    const size_t first=expected.size()>NB_DATAS ? expected.size()-NB_DATAS : 0;
    for(size_t row=first; row<expected.size(); ++row) {
        for(unsigned col=0; col<24; ++col) {
            uint32_t bits;
            memcpy(&bits,reinterpret_cast<const uint8_t*>(&expected[row])+col*4,4);
            assert(static_cast<bool>(std::getline(stream,line)));
            assert(std::stoul(line,nullptr,16)==bits);
        }
    }
    std::getline(stream,line); assert(line=="end record");
}

int main() {
    communication_startup_ticks=communication_startup_cycles;
    bus_ready=true;
    loop_critical_task(); // Open first IDLE exchange.
    key('p');
    std::vector<DebugScopeData> history;
    for(unsigned n=0; n<NB_DATAS*3+17; ++n) {
        complete_cycle(); history.push_back(debug_scope);
        assert(scope_active && !scope_ready && communication_fault==0);
        if(n+1==NB_DATAS) check_export(history); // Exact capacity boundary.
    }
    assert(scope_samples_recorded==NB_DATAS);
    // Missing SM7 in a late exchange: fault must be captured after several wraps.
    for(unsigned id=2; id<=10; ++id)
        if(id!=7) receive(id,cycle_command.cycle_id,POWER);
    loop_critical_task(); history.push_back(debug_scope);
    assert(!scope_active && scope_ready && mode==IDLEMODE);
    assert(debug_scope.fault==COMMUNICATION_ERROR && debug_scope.rx_count==9);
    assert(static_cast<unsigned>(debug_scope.rx_mask)==(1023U & ~(1U<<6)));
    check_export(history);
    const auto frozen_index=scope_write_index;
    for(unsigned n=0;n<30;++n) loop_critical_task(); // Discovery must not overwrite.
    assert(scope_write_index==frozen_index);
    check_export(history);
    key('r'); output.clear(); loop_background_task();
    assert(!is_downloading && output.find("begin record")!=std::string::npos);

    // Rearm after a fault, then manually stop a short capture.
    bus_ready=true; cycle_started=false; communication_fault=0;
    clear_window(); scope_stop_requested=false;
    loop_critical_task();
    key('a'); history.clear();
    complete_cycle(); history.push_back(debug_scope);
    assert(scope_samples_recorded==1 && scope_write_index==1 && scope_active);
    key('i'); complete_cycle(); history.push_back(debug_scope);
    assert(!scope_active && scope_ready && scope_samples_recorded==2);
    check_export(history);

    // A fault must be sampled even between decimated acquisition ticks.
    key('s'); scope_period=10;
    complete_cycle();
    assert(scope_active && scope_samples_recorded==0);
    loop_critical_task();
    assert(!scope_active && scope_samples_recorded==1 && debug_scope.fault==COMMUNICATION_ERROR);
    history={debug_scope}; check_export(history);
    puts("PASS: circular capture, late fault, chronological export, frozen recovery, rearm, manual stop, forced fault sample");
}
'''

if __name__ == '__main__':
    main(SCOPE_CHECKS)
