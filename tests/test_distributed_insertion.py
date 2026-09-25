"""Run the firmware's actual decision/receive functions on a host C++ compiler.

Usage: python tests/test_distributed_insertion.py [--compiler clang++]
No board or third-party Python packages are required.
"""
import argparse
from pathlib import Path
import re
import subprocess
import tempfile


def function(source, name):
    match = re.search(r"^(?:static )?(?:inline )?\w+ " + name + r"\(", source, re.M)
    assert match, name
    start = source.index("{", match.start())
    depth = 1
    end = start + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", default="clang++")
    args = parser.parse_args()
    source = (Path(__file__).resolve().parents[1] / "src/main.cpp").read_text(encoding="utf-8")
    frame = source[source.index("struct MMC_frame_t"):source.index("} __packed;") + len("} __packed;")]
    definitions = r'''
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#define __packed __attribute__((packed))
using float32_t = float;
constexpr uint8_t MMC_SM1 = 1, MMC_SM5 = 5, MMC_SM6 = 6;
constexpr uint8_t MMC_SM_FIRST = 1, MMC_SM_LAST = 10, MMC_SM_COUNT = 10;
constexpr uint8_t total_number_of_modules_arm = 5;
constexpr uint8_t IDLE = 0, POWER = 1, COMMUNICATION_ERROR = 6;
constexpr float Cap_voltage_SCALE = 160, Arm_current_SCALE = 20, Arm_current_OFFSET = 10;
float a = 1, m = 1;
uint8_t module_ID = 1;
uint16_t capacitor_voltage_raw[10] = {};
bool measurement_received[10] = {};
uint8_t received_module_count = 0, communication_fault = 0;
bool cycle_started = false;
float upper_arm_current_reference = 0, lower_arm_current_reference = 0;
'''
    functions = "\n".join(function(source, name) for name in (
        "mmc_is_upper_arm_module", "mmc_encode_voltage", "mmc_encode_current",
        "mmc_compute_insertion_counts", "begin_cycle", "store_cycle_sample",
        "store_module_measurements", "mmc_local_insertion"))
    checks = r'''
void reset() {
    received_module_count = communication_fault = 0;
    cycle_started = false;
    std::fill_n(measurement_received, 10, false);
    cycle_command = {};
}
int main() {
    MMC_frame_t command{};
    assert(mmc_compute_insertion_counts(-1, command));
    assert(command.upper_insert_count == 0 && command.lower_insert_count == 5);
    assert(mmc_compute_insertion_counts(1, command));
    assert(command.upper_insert_count == 5 && command.lower_insert_count == 0);
    assert(mmc_compute_insertion_counts(0, command));
    assert(command.upper_insert_count == 3 && command.lower_insert_count == 3);
    assert(!mmc_compute_insertion_counts(NAN, command));
    m = 2;
    assert(!mmc_compute_insertion_counts(1, command));
    m = 1;

    // Exhaustive tied/untied voltage patterns; compare with an independent sort.
    unsigned cases = 0;
    for (int pattern = 0; pattern < 243; ++pattern) {
        int digits = pattern;
        for (int i = 0; i < 5; ++i) {
            capacitor_voltage_raw[i] = digits % 3;
            capacitor_voltage_raw[5 + i] = 2 - digits % 3;
            digits /= 3;
        }
        for (float direction : {-1.0F, 0.0F, 1.0F}) {
            upper_arm_current_reference = direction;
            lower_arm_current_reference = -direction;
            for (int upper_count = 0; upper_count <= 5; ++upper_count) {
                for (int lower_count = 0; lower_count <= 5; ++lower_count) {
                    cycle_command.upper_insert_count = upper_count;
                    cycle_command.lower_insert_count = lower_count;
                    for (int arm = 0; arm < 2; ++arm) {
                        int ids[5];
                        for (int i = 0; i < 5; ++i) ids[i] = arm * 5 + i;
                        const float current = arm ? lower_arm_current_reference : upper_arm_current_reference;
                        std::stable_sort(ids, ids + 5, [current](int x, int y) {
                            return current >= 0 ? capacitor_voltage_raw[x] < capacitor_voltage_raw[y]
                                                : capacitor_voltage_raw[x] > capacitor_voltage_raw[y];
                        });
                        int inserted = 0;
                        const int count = arm ? lower_count : upper_count;
                        for (int rank = 0; rank < 5; ++rank) {
                            module_ID = ids[rank] + 1;
                            const float decision = mmc_local_insertion();
                            assert(decision == (rank < count ? 1.0F : 0.0F));
                            inserted += static_cast<int>(decision);
                        }
                        assert(inserted == count);
                        ++cases;
                    }
                }
            }
        }
    }

    // Receive a complete exchange; current sources are exclusively SM1 and SM6.
    reset();
    command = {};
    command.sm_id = 1; command.status = POWER; command.cycle_id = 42;
    command.upper_insert_count = 2; command.lower_insert_count = 4;
    assert(begin_cycle(command));
    for (int id = 1; id <= 10; ++id) {
        MMC_frame_t frame = command;
        frame.sm_id = id;
        frame.capacitor_voltage_raw = mmc_encode_voltage(40.0F + id);
        frame.arm_current_raw = mmc_encode_current(id == 1 ? 3.0F : id == 6 ? -2.0F : 8.0F);
        assert(store_module_measurements(frame));
        assert(!store_module_measurements(frame)); // duplicates never complete a round
        assert(capacitor_voltage_raw[id - 1] == frame.capacitor_voltage_raw);
        assert(received_module_count == id);
    }
    assert(communication_fault == 0 && received_module_count == 10);
    assert(std::fabs(upper_arm_current_reference - 3.0F) < 0.005F);
    assert(std::fabs(lower_arm_current_reference + 2.0F) < 0.005F);
    // Own sample uses exactly the same storage/quantization as remote samples.
    command.capacitor_voltage_raw = mmc_encode_voltage(40.01F);
    store_cycle_sample(command);
    assert(capacitor_voltage_raw[0] == command.capacitor_voltage_raw);

    for (int fault = 0; fault < 3; ++fault) {
        reset();
        assert(begin_cycle(command));
        assert(store_module_measurements(command));
        MMC_frame_t bad = command;
        bad.sm_id = 2;
        if (fault == 0) ++bad.cycle_id;
        if (fault == 1) ++bad.upper_insert_count;
        if (fault == 2) ++bad.lower_insert_count;
        assert(!store_module_measurements(bad));
        assert(communication_fault == COMMUNICATION_ERROR && received_module_count == 1);
    }
    for (int arm = 0; arm < 2; ++arm) {
        reset();
        MMC_frame_t bad = command;
        if (arm == 0) bad.upper_insert_count = 6;
        else bad.lower_insert_count = 6;
        begin_cycle(bad);
        assert(communication_fault == COMMUNICATION_ERROR);
    }
    std::printf("PASS: %u arm selections; NLM counts, shared currents and frame validation\n", cases);
}
'''
    with tempfile.TemporaryDirectory(prefix="mmc-insertion-") as temporary:
        directory = Path(temporary)
        cpp = directory / "test.cpp"
        executable = directory / "test.exe"
        cpp.write_text(definitions + frame + "\nMMC_frame_t cycle_command;\n" + functions + checks, encoding="utf-8")
        subprocess.run([args.compiler, "-std=c++17", str(cpp), "-o", str(executable)], check=True)
        subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
