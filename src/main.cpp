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
 * @brief  No-power MMC insertion-count agreement test. SM1 broadcasts the sine.
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
#define CONSENSUS_TEST 1
#define COMMUNICATION_ERROR 6

constexpr uint8_t MMC_SM_COUNT = 10;
constexpr uint8_t MMC_SM_FIRST = MMC_SM1;
constexpr uint8_t MMC_SM_LAST = MMC_SM10;

/* -------------- GENERAL MMC DEFINITIONS -------------------- */
/* --------------- To be changed by user --------------------- */

static const float f0 = 50.F; //[Hz] Output frequency used to generate the sinusoidal reference for open-loop control
static const uint8_t total_number_of_modules_arm = 5; //[-] Number of modules per arm

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

/* -------------- NO-POWER TEST CONFIGURATION ----------------- */
// Flash this firmware on all 10 boards and use auxiliary supplies only.
// On SM1's serial monitor: p starts the sine and a 512-sample capture;
// wait >=120 ms, then r exports it through the existing recorded_datas filter.
// a rearms capture, i stops the sine. PWM/gate drivers always remain disabled.
// Expected: the 10 N_u_SM* curves agree, as do the 10 N_l_SM* curves;
// reports_received=10, mismatch_count=0, round_ok=1. -1 means missing/invalid.
// Same parameters and firmware on all ten boards: each calculates both arms.
static const float32_t m = 1.0F;
static const float32_t a = 1.0F;
static constexpr uint32_t control_task_period = 200; // us
static constexpr float32_t Ts = control_task_period * 1e-6F;
static const float32_t w0 = 2 * PI * f0;
static float32_t angle;

constexpr int16_t MMC_SINE_REFERENCE_MAX = 32767;
constexpr int16_t MMC_SINE_REFERENCE_INVALID = -32768;
constexpr uint8_t MMC_COUNT_INVALID = 0xFF;
constexpr uint8_t MMC_TEST_PROTOCOL = 0xC7;

static inline int16_t mmc_encode_sine_reference(float32_t sine)
{
    if (!isfinite(sine)) return MMC_SINE_REFERENCE_INVALID;
    if (sine >= 1.0F) return MMC_SINE_REFERENCE_MAX;
    if (sine <= -1.0F) return -MMC_SINE_REFERENCE_MAX;
    return static_cast<int16_t>(roundf(sine * MMC_SINE_REFERENCE_MAX));
}

static inline float32_t mmc_decode_sine_reference(int16_t raw)
{
    return static_cast<float32_t>(raw) / MMC_SINE_REFERENCE_MAX;
}

static bool mmc_compute_insertion_counts(int16_t sine_reference_raw,
                                         uint8_t &n_insert_upper,
                                         uint8_t &n_insert_lower)
{
    if (sine_reference_raw == MMC_SINE_REFERENCE_INVALID) return false;
    const float32_t sine = mmc_decode_sine_reference(sine_reference_raw);
    const float32_t upper = roundf(total_number_of_modules_arm * ((a + m * sine) / 2.0F));
    const float32_t lower = roundf(total_number_of_modules_arm * ((a - m * sine) / 2.0F));
    if (!(upper >= 0.0F && upper <= total_number_of_modules_arm &&
          lower >= 0.0F && lower <= total_number_of_modules_arm)) return false;
    n_insert_upper = static_cast<uint8_t>(upper);
    n_insert_lower = static_cast<uint8_t>(lower);
    return true;
}

// Test-only wire format. Flash ALL boards; incompatible with the power firmware.
struct MMC_frame_t
{
    uint8_t protocol;
    int16_t sine_reference_raw;
    uint16_t cycle_id;
    uint8_t n_insert_upper;
    uint8_t n_insert_lower;
    uint8_t status;
    uint8_t sm_id;
} __packed;
static_assert(sizeof(MMC_frame_t) == 9, "Keep the RS485 test frame at 9 bytes");

uint8_t module_ID = detect_module_id();
static MMC_frame_t cycle_command;
static bool cycle_started;
static volatile bool round_open;
static bool report_received[MMC_SM_COUNT];
static uint8_t received_module_count;
static bool communication_fault;
static float32_t reported_upper[MMC_SM_COUNT];
static float32_t reported_lower[MMC_SM_COUNT];
static uint8_t buffer_tx[sizeof(MMC_frame_t)];
static uint8_t buffer_rx[sizeof(MMC_frame_t)];

static volatile char requested_command;
static volatile bool test_requested;
static volatile bool is_downloading;
static volatile bool scope_ready;
static bool scope_armed;
static bool scope_primed;
static bool enable_acq;
// ScopeMimicry's buffer-size API is uint16_t. 512 * 25 * 4 = 51200 bytes.
static constexpr uint16_t NB_DATAS = 512;
static constexpr uint16_t NB_CHANNELS = 5 + 2 * MMC_SM_COUNT;
static_assert(NB_DATAS * NB_CHANNELS * sizeof(float32_t) <= UINT16_MAX,
              "ScopeMimicry buffer-size overflow");
static ScopeMimicry scope(NB_DATAS, NB_CHANNELS);
// Snapshot variables are only written by SM1's critical task, never by RX.
static float32_t scope_sine_reference;
static float32_t scope_test_active;
static float32_t scope_reports_received;
static float32_t scope_mismatch_count;
static float32_t scope_round_ok;
static float32_t scope_upper[MMC_SM_COUNT];
static float32_t scope_lower[MMC_SM_COUNT];
static const char *const upper_channel_names[MMC_SM_COUNT] = {
    "N_u_SM1", "N_u_SM2", "N_u_SM3", "N_u_SM4", "N_u_SM5",
    "N_u_SM6", "N_u_SM7", "N_u_SM8", "N_u_SM9", "N_u_SM10"};
static const char *const lower_channel_names[MMC_SM_COUNT] = {
    "N_l_SM1", "N_l_SM2", "N_l_SM3", "N_l_SM4", "N_l_SM5",
    "N_l_SM6", "N_l_SM7", "N_l_SM8", "N_l_SM9", "N_l_SM10"};

static bool a_trigger()
{
    return enable_acq;
}

static void reset_round()
{
    received_module_count = 0;
    communication_fault = false;
    for (uint8_t i = 0; i < MMC_SM_COUNT; ++i)
    {
        report_received[i] = false;
        reported_upper[i] = -1.0F;
        reported_lower[i] = -1.0F;
    }
}

static bool begin_cycle(const MMC_frame_t &frame)
{
    if (frame.protocol != MMC_TEST_PROTOCOL || frame.sm_id != MMC_SM1 ||
        (frame.status != IDLE && frame.status != CONSENSUS_TEST)) return false;
    if (cycle_started)
    {
        const uint16_t advance = static_cast<uint16_t>(frame.cycle_id - cycle_command.cycle_id);
        if (advance == 0 || advance >= 32768U) return false;
    }
    // Followers reset on the command itself, not in their timer interrupt:
    // a later timer tick must not erase reports already received for this cycle.
    reset_round();
    cycle_command = frame;
    cycle_started = true;
    communication_fault = frame.sine_reference_raw == MMC_SINE_REFERENCE_INVALID;
    round_open = true;
    return true;
}

static bool store_module_report(const MMC_frame_t &frame)
{
    if (!round_open) return false;
    if (frame.protocol != MMC_TEST_PROTOCOL ||
        frame.sm_id < MMC_SM_FIRST || frame.sm_id > MMC_SM_LAST ||
        frame.cycle_id != cycle_command.cycle_id)
    {
        communication_fault = true;
        return false;
    }
    const uint8_t index = frame.sm_id - MMC_SM_FIRST;
    if (report_received[index]) return false; // Includes our own RS485 echo.
    if (frame.sm_id != MMC_SM1 && !report_received[0]) return false;

    const bool valid = frame.status == cycle_command.status &&
        frame.sine_reference_raw == cycle_command.sine_reference_raw &&
        frame.sine_reference_raw != MMC_SINE_REFERENCE_INVALID &&
        frame.n_insert_upper <= total_number_of_modules_arm &&
        frame.n_insert_lower <= total_number_of_modules_arm;
    if (valid)
    {
        reported_upper[index] = frame.n_insert_upper;
        reported_lower[index] = frame.n_insert_lower;
    }
    else communication_fault = true;
    report_received[index] = true;
    ++received_module_count;
    return true;
}

static void send_own_report()
{
    MMC_frame_t frame = cycle_command;
    frame.sm_id = module_ID;
    uint8_t upper = MMC_COUNT_INVALID;
    uint8_t lower = MMC_COUNT_INVALID;
    // EVERY board (including SM1) computes from the same quantized sine.
    // Never reuse the counts received from SM1 or from the previous cycle.
    if (!mmc_compute_insertion_counts(frame.sine_reference_raw, upper, lower))
        frame.status = COMMUNICATION_ERROR;
    frame.n_insert_upper = upper;
    frame.n_insert_lower = lower;
    store_module_report(frame); // Do not depend on receiving our own echo.
    memcpy(buffer_tx, &frame, sizeof(frame));
    communication.rs485.startTransmission();
}

void reception_function()
{
    MMC_frame_t frame;
    memcpy(&frame, buffer_rx, sizeof(frame));
    if (frame.sm_id == MMC_SM1)
    {
        if (module_ID == MMC_SM1 || !begin_cycle(frame)) return;
    }
    const bool accepted = store_module_report(frame);
    if (module_ID != MMC_SM1 && accepted && frame.sm_id == module_ID - 1)
        send_own_report();
}

static void snapshot_round()
{
    scope_sine_reference = cycle_command.sine_reference_raw == MMC_SINE_REFERENCE_INVALID
        ? NAN : mmc_decode_sine_reference(cycle_command.sine_reference_raw);
    scope_test_active = cycle_started && cycle_command.status == CONSENSUS_TEST ? 1.0F : 0.0F;
    scope_reports_received = received_module_count;
    const bool reference_valid = reported_upper[0] >= 0.0F && reported_lower[0] >= 0.0F;
    scope_mismatch_count = reference_valid ? 0.0F : -1.0F;
    for (uint8_t i = 0; i < MMC_SM_COUNT; ++i)
    {
        scope_upper[i] = reported_upper[i];
        scope_lower[i] = reported_lower[i];
        if (reference_valid && reported_upper[i] >= 0.0F && reported_lower[i] >= 0.0F &&
            (reported_upper[i] != reported_upper[0] || reported_lower[i] != reported_lower[0]))
            scope_mismatch_count += 1.0F;
    }
    scope_round_ok = cycle_command.status == CONSENSUS_TEST &&
        received_module_count == MMC_SM_COUNT && !communication_fault &&
        scope_mismatch_count == 0.0F ? 1.0F : 0.0F;
}

static void arm_scope()
{
    scope.start();
    enable_acq = false;
    scope_ready = false;
    scope_primed = false;
    scope_armed = true;
}

static void acquire_scope()
{
    if (!scope_armed || !cycle_started) return;
    if (!scope_primed)
    {
        if (cycle_command.status != CONSENSUS_TEST) return;
        // Prime one real sample before the trigger: this library then captures
        // NB_DATAS-1 further samples, so no stale/zero row remains in the dump.
        scope.acquire();
        scope_primed = true;
        enable_acq = true;
    }
    else if (scope.acquire() == 2)
    {
        scope_armed = false;
        scope_ready = true; // Buffer is now frozen until p/a explicitly rearms it.
    }
}

void loop_critical_task()
{
    // No path in this firmware enables the gate drivers or any PWM output.
    shield.power.stop(ALL);
    if (module_ID != MMC_SM1) return;

    // RX may preempt this task. Close reception before reading/copying the
    // completed window; late replies cannot overwrite the scope snapshot.
    round_open = false;
    snapshot_round();
    acquire_scope();

    const char request = requested_command;
    requested_command = 0;
    if (request == 'i') test_requested = false;
    else if (request == 'p' && !is_downloading)
    {
        test_requested = true;
        angle = 0.0F;
        arm_scope();
    }
    else if (request == 'a' && test_requested && !is_downloading) arm_scope();
    // Capture and download ownership change in this task only. A console
    // request cannot race with rearming and expose a buffer still being filled.
    else if (request == 'r' && scope_ready) is_downloading = true;

    MMC_frame_t next_command = {};
    next_command.protocol = MMC_TEST_PROTOCOL;
    next_command.sm_id = MMC_SM1;
    next_command.cycle_id = static_cast<uint16_t>(cycle_command.cycle_id + 1U);
    if (test_requested)
    {
        angle = ot_modulo_2pi(angle + w0 * Ts);
        next_command.sine_reference_raw = mmc_encode_sine_reference(ot_sin(angle));
        next_command.status = CONSENSUS_TEST;
    }
    else angle = 0.0F;
    // Continue the diagnostic test after missing/mismatching reports. There
    // is no power to control; round_ok and -1 counts expose the failed cycles.
    if (begin_cycle(next_command)) send_own_report();
}

static void dump_scope_datas()
{
    uint8_t *buffer = scope.get_buffer();
    const uint16_t buffer_size = scope.get_buffer_size() / sizeof(float32_t);
    printk("begin record\n#");
    for (uint16_t k = 0; k < scope.get_nb_channel(); ++k)
        printk("%s,", scope.get_channel_name(k));
    printk("\n# %d\n", scope.get_final_idx());
    for (uint16_t k = 0; k < buffer_size; ++k)
    {
        printk("%08x\n", *((uint32_t *)buffer + k));
        task.suspendBackgroundUs(100);
    }
    printk("end record\n");
}

void loop_communication_task()
{
    const char command = console_getchar();
    // Avoid interleaving console messages with the scope's serial record.
    if (is_downloading) return;
    if (command == 'h')
    {
        printk("CONSENSUS TEST - PWM and gate drivers DISABLED\n"
               "SM1: p = start sine + capture, a = new capture, i = stop sine\n"
               "     r = download completed capture (512 samples, 200 us)\n"
               "All 10 boards must run the test firmware.\n");
        return;
    }
    if (module_ID != MMC_SM1) return;
    if (requested_command != 0) return; // Consume the previous request first.
    switch (command)
    {
    case 'p':
        requested_command = 'p';
        printk("No-power test requested; capture armed.\n");
        break;
    case 'a':
        if (test_requested)
        {
            requested_command = 'a';
            printk("New capture requested.\n");
        }
        else printk("Start the test with p first.\n");
        break;
    case 'i':
        requested_command = 'i';
        printk("Stop sine requested; PWM remains disabled.\n");
        break;
    case 'r':
        if (scope_ready) requested_command = 'r';
        else printk("No completed capture. Press p, wait at least 120 ms, then r.\n");
        break;
    default:
        break;
    }
}

void loop_background_task()
{
    if (module_ID == MMC_SM1)
    {
        if (is_downloading)
        {
            dump_scope_datas();
            is_downloading = false;
        }
        if (test_requested) spin.led.toggle();
        else spin.led.turnOff();
    }
    task.suspendBackgroundMs(100);
}

void setup_routine()
{
    // Initialize the timer hardware needed by synchronization, with outputs
    // explicitly blocked even on an unknown board. No ADC input is needed.
    shield.power.initBuck(ALL);
    shield.power.stop(ALL);
    shield.power.disconnectCapacitor(LEG1);
    shield.power.disconnectCapacitor(LEG2);
    printk("Board UID: 0x%08" PRIX32 "\n", read_board_uid());
    printk("Module ID: %u - CONSENSUS TEST, POWER DISABLED\n", module_ID);
    if (module_ID < MMC_SM_FIRST || module_ID > MMC_SM_LAST)
    {
        printk("Unknown board: test and communication disabled\n");
        return;
    }
    reset_round();
    task.createCritical(loop_critical_task, control_task_period);
    communication.rs485.configure(buffer_tx, buffer_rx, sizeof(buffer_rx),
                                  reception_function, SPEED_20M);
    if (module_ID == MMC_SM1)
    {
        communication.sync.initMaster();
        scope.connectChannel(scope_sine_reference, "sine_ref");
        // Cycle IDs align frames internally; expose a 0/1 test flag so the
        // existing monitor's common plot axis stays readable for 0..5 counts.
        scope.connectChannel(scope_test_active, "test_active");
        scope.connectChannel(scope_reports_received, "reports_received");
        scope.connectChannel(scope_mismatch_count, "mismatch_count");
        scope.connectChannel(scope_round_ok, "round_ok");
        for (uint8_t i = 0; i < MMC_SM_COUNT; ++i)
            scope.connectChannel(scope_upper[i], upper_channel_names[i]);
        for (uint8_t i = 0; i < MMC_SM_COUNT; ++i)
            scope.connectChannel(scope_lower[i], lower_channel_names[i]);
        scope.set_trigger(&a_trigger);
        scope.set_delay(0.0F);
    }
    else communication.sync.initSlave();
    const int8_t background_task = task.createBackground(loop_background_task);
    const int8_t console_task = task.createBackground(loop_communication_task);
    task.startBackground(background_task);
    task.startBackground(console_task);
    task.startCritical(false); // Timers and communication only; no ADC acquisition.
}

int main(void)
{
    setup_routine();
    return 0;
}
