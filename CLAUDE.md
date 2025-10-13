# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

TI C2000 F28377D dual-core DSP firmware for a 30kW battery power conversion system (Master Controller). The system manages up to 31 slave modules via CAN bus, with real-time PI control running on the CLA (Control Law Accelerator) at 100kHz.

**Hardware Platform**: TI F28377D (200MHz dual-core C28x + CLA)
**Build Tool**: Texas Instruments Code Composer Studio (CCS)
**Target Configuration**: Flash or RAM execution modes

## Build System

This is a CCS (Eclipse-based IDE) project. Build commands must be executed within CCS or via command-line tools provided by TI.

### Build Configurations
- **CPU1_FLASH**: Production build targeting on-chip Flash memory
- **CPU1_RAM**: Debug build running from RAM (faster iteration)

### Linker Command Files
- `2837xD_FLASH_CLA_lnk_cpu1.cmd`: Flash build with CLA sections
- `2837xD_RAM_CLA_lnk_cpu1.cmd`: RAM build with CLA sections
- `F2837xD_Headers_nonBIOS_cpu1.cmd`: Memory-mapped peripheral headers

### Key Dependencies
- **C2000Ware**: TI's software development kit (device support, driverlib)
- **driverlib.lib**: Pre-compiled peripheral driver library
- **DCL (Digital Control Library)**: TI's optimized control algorithms for CLA
- Path variables: `COM_TI_C2000WARE_INSTALL_DIR`, `C2000WARE_DLIB_ROOT`

### DCL (Digital Control Library) Integration

**Include Path** (CCS Project Settings → Build → C2000 Compiler → Include Options):
```
${COM_TI_C2000WARE_INSTALL_DIR}/libraries/control/DCL/c28/include
```

**Required Source Files** (Add to project):
```
${COM_TI_C2000WARE_INSTALL_DIR}/libraries/control/DCL/c28/source/DCL_PI_L1.asm
${COM_TI_C2000WARE_INSTALL_DIR}/libraries/control/DCL/c28/source/DCL_PI_L2.asm
${COM_TI_C2000WARE_INSTALL_DIR}/libraries/control/DCL/c28/source/DCL_clamp_L1.asm
${COM_TI_C2000WARE_INSTALL_DIR}/libraries/control/DCL/c28/source/DCL_error.c
```

**Alternative** (if prebuilt library exists):
Link against `dcl_cla.lib` or similar prebuilt DCL library for F28377D.

**Header Files**:
- `DCLCLA.h`: CLA-specific DCL functions (PI, PID controllers)
- `DCL.h`: Common DCL types and utilities

## Code Architecture

### Module Organization

```
HABA_main.c                        Main entry point and control loop
├── HABA_setup.c/h                 Hardware initialization and system setup (GPIO, PWM, CAN, SCI, SPI, ADC, CLA)
├── HABA_control.c/h               Control algorithms, communication handlers, ISRs
├── HABA_globals.c/h               Global variables, constants, shared data definitions
└── HABA_cla_tasks.cla             CLA assembly tasks for high-speed PI control
```

### Real-Time Control Architecture

**Primary ISR**: `INT_EPWM1_ISR` @ 100kHz (EPWM1 period interrupt)

**5-Phase Task Scheduler with Pipelining** (20kHz effective per task):
The control loop uses a pipelined architecture for optimal CPU-CLA parallelism:

```c
// Function pointer table for clean phase execution
static const PhaseFunction control_phase_table[5] = {
    Sensing_And_Trigger_PI,         // Phase 0: Voltage sensing → LPF → CLA Force
    Apply_PI_And_Convert_DAC,        // Phase 1: Use CLA results → DAC conversion
    Transmit_Current_Command,        // Phase 2: RS485 transmission
    Check_System_Safety,             // Phase 3: Fault detection & relay control
    Update_Monitoring_And_Sequence   // Phase 4: Averaging & sequence control
};

// ISR execution (replaces switch-case for clarity)
control_phase_table[control_phase]();
```

**Pipeline Flow**:
```
Phase 0 (n-th cycle):
  ├─ Voltage sensing & calibration
  ├─ Soft-start ramp calculation
  ├─ LPF filtering
  ├─ Anti-windup limit update
  └─ CLA Task Force (non-blocking)
      [CLA executes in background]

Phase 1 (n-th cycle, 10us later):
  ├─ CLA results ready (I_PI_charge_out, I_PI_discharge_out)
  ├─ Operating mode selection
  ├─ Current limiting
  └─ DAC output conversion

Phase 2-4: Communication & Monitoring
```

**Pipeline Benefits**:
- **Latency**: 10μs (vs 50μs without pipelining)
- **CPU-CLA Parallelism**: CLA runs while CPU executes Phase 1-4
- **No Blocking**: Force-only calls, no ForceAndWait
- **Deterministic**: CLA completion guaranteed in 10μs (CLA needs only 0.1-0.2μs)

### Critical Timing Loops

**1ms Loop** (`flag_1ms`):
- CAN bus communication with slave modules
- Watchdog service
- Emergency stop monitoring

**10ms Loop** (`flag_10ms`):
- SCADA data reception and parsing
- Slave status updates (channels 1-16)
- System data transmission to SCADA

**50ms Loop** (`flag_50ms`):
- System voltage reporting
- Master ID selection
- Extended diagnostics

### CLA (Control Law Accelerator) Integration

The CLA runs **independent of CPU** for deterministic, low-latency PI control using **TI's DCL (Digital Control Library)**:

**File**: `HABA_cla_tasks.cla`

**Tasks**:
- `Cla1Task1`: Charge mode PI controller (V_max_cmd reference)
- `Cla1Task2`: Discharge mode PI controller (V_min_cmd reference)
- Tasks 3-8: Reserved (currently unused)

**DCL PI Controller (DCL_PI_CLA)**:
```c
// TI's optimized PI controller structure
typedef struct {
    float32_t Kp;       // Proportional gain
    float32_t Ki;       // Integral gain (discrete: Ki_continuous * Ts)
    float32_t i10;      // Integrator storage
    float32_t Umax;     // Upper control saturation limit
    float32_t Umin;     // Lower control saturation limit
    float32_t i6;       // Saturation storage
    float32_t i11;      // Integrator storage
    float32_t Imax;     // Upper integrator saturation limit (Anti-windup)
    float32_t Imin;     // Lower integrator saturation limit (Anti-windup)
} DCL_PI_CLA;
```

**Usage in CLA Tasks**:
```c
// Task 1: Charge mode
I_PI_charge_out = DCL_runPI_L1(&pi_charge, V_max_cmd, V_fb);

// Task 2: Discharge mode
I_PI_discharge_out = DCL_runPI_L1(&pi_discharge, V_min_cmd, V_fb);
```

**Advantages of DCL**:
- TI-optimized assembly code (~10-20 cycles per PI calculation)
- Automatic Anti-windup handling
- Standardized interface
- Better maintainability

**CPU-CLA Shared Memory**:
- **CpuToCla1MsgRAM**: `pi_charge`, `pi_discharge` (DCL_PI_CLA structures), `V_max_cmd`, `V_min_cmd`, `V_fb`
- **Cla1ToCpuMsgRAM**: `I_PI_charge_out`, `I_PI_discharge_out`, `cla_cnt` (debug counter)

**CLA + DCL Initialization**: `Init_CPU1_CLA1()` in `HABA_setup.c`
```c
// Discrete-time PI (Ki = Ki_continuous * Ts)
// Charge mode controller
pi_charge.Kp = 1.0f;
pi_charge.Ki = 3000.0f * 50e-6f;  // = 0.15
pi_charge.Umax = 80.0f;
pi_charge.Umin = -2.0f;
pi_charge.Imax = 80.0f;  // Dynamic update in Sensing_And_Trigger_PI()
pi_charge.Imin = -2.0f;
// Manual reset (DCL_PI_CLA has no reset function)
pi_charge.i10 = 0.0f;
pi_charge.i6  = 1.0f;
pi_charge.i11 = 0.0f;
```
- Memory mapping for message RAMs
- Task trigger configuration
- Scratchpad memory setup

### Communication Protocols

**CAN Bus** (CANA @ 500kbps):
- **TX**: Master commands to slaves (0xE0 base ID)
- **RX**: Slave status feedback (0xF1-0xFF, mailboxes 2-16)
- Protocol: 4-byte messages with current, temperature, status flags
- Function: `Send_CANA_Message()`, `Read_CAN_Slave()`

**RS485A/B** (SCIA/B @ 5.625Mbps):
- Current reference transmission to legacy modules
- DE (Driver Enable) GPIO control for half-duplex
- Functions: `Send_485A_Current_Command()`, `Send_485B_Current_Command()`

**SPI Channels**:
- **SPIA**: DAC80502 control (5MHz, 8-bit frames) - analog output
- **SPIC**: FPGA communication (ADC data reception: Vo, Vbat, Io)
- Function: `Read_FPGA_Data()` called every 100kHz ISR

**SCID (SCADA Interface)** @ 115200 baud:
- Protocol: Custom 10-byte packet with STX/ETX framing
- Commands: run/Stop, Mode selection, Voltage/Current setpoints
- Modbus-like checksum validation
- Functions: `Process_SCADA_Command()`, `Send_Slave_Data_To_SCADA()`, `Send_System_Voltage_To_SCADA()`

### Operating Modes

Controlled by `operation_mode` enum:

```c
MODE_STOP (0)        // System disabled
MODE_INDEPENDENT (1) // CH1/CH2 operate with independent current references
MODE_PARALLEL (2)    // Upper master runs PI, lower master passes through commands
```

Mode selection affects:
- PI controller activation (which master runs closed-loop)
- Relay configuration (independent vs parallel connection)
- Current command routing

### Sequence Control State Machine

**Function**: `Sequence_Module()` in `HABA_Ctrl.c`

State progression controlled by `sequence_step`:
- **Step 0**: Idle, waiting for start command
- **Step 10**: Pre-charging sequence (capacitor charging via limiting resistor)
- **Step 20**: Main contactor close, normal operation
- **Fault states**: Automatic shutdown on over-voltage/current/temperature

**Protection Thresholds**:
```c
OVER_VOLTAGE: 1400V
OVER_CURRENT: 88.0A
OVER_TEMP:    120°C
```

## Critical Code Sections

### RAMFUNC Placement

Performance-critical functions are placed in RAM using `#pragma CODE_SECTION`:

```c
#pragma CODE_SECTION(INT_EPWM1_ISR, ".TI.ramfunc");
#pragma CODE_SECTION(sensing_task, ".TI.ramfunc");
#pragma CODE_SECTION(pi_control_task, ".TI.ramfunc");
```

This eliminates Flash wait states for 100kHz ISR execution.

### Soft-Start Ramping

Current commands undergo **soft-start limiting** to prevent inrush:

```c
I_ss_ramp += CURRENT_LIMIT * 0.00005f;  // 20kHz ramp rate
if (I_ss_ramp > CURRENT_LIMIT) I_ss_ramp = CURRENT_LIMIT;
```

Then low-pass filtered before PI control:
```c
I_cmd_filtered = lpf_coeff_a * (I_cmd_ramped + I_cmd_prev) + lpf_coeff_b * I_cmd_filtered;
```

Filter coefficients calculated at init: `wc=1kHz`, `Ts=50μs`
- `lpf_coeff_a`: Forward path coefficient
- `lpf_coeff_b`: Feedback coefficient

### Voltage Feedback Selection

Feedback voltage switches based on operating sequence:

```c
if (sequence_step == 20) V_fb = V_batt;  // Normal: regulate battery side
else V_fb = V_out;                       // Pre-charge: regulate output side
```

This prevents instability during contactor transitions.

## Hardware Abstraction

### GPIO Mapping

**Status LEDs** (Front Panel):
```c
LED_PWR        (46) // Power indicator
LED_CHARGE     (47) // Charging mode
LED_DISCHARGE  (42) // Discharging mode
LED_FAULT      (43) // Fault condition
LED_SINGLE     (67) // Independent mode
LED_DUAL       (68) // Parallel mode
```

**Relays**:
- Relay8 (GPIO8): Independent operation mode
- Relay7 (GPIO9): Parallel operation mode
- Pre-charge relay: Code commented out (requires restoration)

**Digital Inputs**:
GPIO36-39 used for Master ID DIP switches (directly read via `Select_master_id()`).

### ADC Configuration

**ADCA**: NTC temperature sensors (channels added 07.11)
**FPGA SPI ADC**: Primary voltage/current sensing (higher speed, lower noise)

Calibration constants in `sensing_task()`:
```c
Vo_sen = (fADC_voltage_out + 0.091694057f) * 0.9926000888f;
Vbat_sen = (fADC_voltage_Bat - 0.3058461657f) * 0.9945009708f;
```

### Master ID Selection

Physical hardware strapping determines if this board is "upper" or "lower" master:

```c
void Select_master_id(void); // Reads GPIO pins to determine master_id (0 or 1)
```

Affects:
- Which master enables RS485 driver (upper only)
- PI control authority in parallel mode

## Known Issues and TODOs

### From Code History Comments (Main File Header)

**Verified Working** (as of 10.01):
- EPWM3 interrupt (07.03)
- CAN slave communication (07.07)
- SCI/RS485 integration (07.08)
- CLA-based PI control (07.09)
- DAC80502 SPI (07.10)
- NTC temperature sensing (07.11)
- Digital I/O (07.12)
- SCADA protocol rev 2.0 (09.11, 09.15)
- UI data transmission fixes (09.23)
- Sequence module updates (09.30)
- UI channels 1-15 operation (10.01)

**Pending Verification**:
- UI channels 16-31: `while` loop validation needed (main.c:48)
- Pre-charge relay code restoration (commented 20250918)

### Debug Infrastructure

Debug flags in `HABA_Ctrl.c` (disabled by default):
```c
_DEBUG_CAN_STATUS_ENABLE_  (0)
_DEBUG_SCI_STATUS_ENABLE_  (0)
_DEBUG_SPI_STATUS_ENABLE_  (0)
_DEBUG_CLK_STATUS_ENABLE_  (0)
```

Set to `1` to populate debug variables: `debug_sysclk_freq`, `debug_can_es`, `debug_sci_rx_error`, etc.

### GPIO Debug Pins

ISR and task timing monitored via GPIO toggles:
```c
GPIO90: EPWM1 ISR execution
GPIO91: 1ms task
GPIO92: 10ms task
```

**Recommendation**: Wrap in `#ifdef DEBUG` for production builds.

## Development Notes

### When Modifying Control Algorithms

1. **Shared Variables**: Add to `HABA_shared.h` with `extern`, define in `HABA_shared.c`
2. **CLA Access**: Variables accessed by CLA must be in message RAM (see `Init_CPU1_CLA1()`)
3. **ISR Functions**: Use `#pragma CODE_SECTION(..., ".TI.ramfunc")` for time-critical code
4. **Naming Convention**: Project uses Pascal_Snake_Case (e.g., `Update_Voltage_Sensing`, `Send_CANA_Message`) with uppercase preserved for acronyms (CAN, SPI, ADC, GPIO, etc.) and physical quantities (V, I)
5. **Task Timing**: Respect 10μs ISR budget - offload heavy work to background loops

### When Adding Communication Features

- **CAN**: Extend mailbox configuration in `Init_CANA()`, update `RX_MSG_OBJ_COUNT`
- **SCID Protocol**: Modify `HMI_PACKET` struct and `Modbus_Parse()` function
- **Timing**: SCI transmission moved from 10μs to 10/50ms to prevent ISR overruns (09.23 fix)

### Calibration Procedure

Voltage sensor calibration constants are empirically derived:
1. Measure actual system voltage with calibrated reference
2. Read raw ADC average value
3. Update linear correction in `sensing_task()`:
   ```c
   Vo_sen = (raw_voltage + offset) * gain;
   ```

Current calibration follows same process in temperature/current tasks.

## Special Considerations

### CLA Programming Model

- CLA code (`.cla` files) uses **C syntax subset** - no pointers, limited stdlib
- CLA has **independent 32-bit floating-point pipeline** - no type promotion issues
- CPU cannot directly access CLA registers - all communication via message RAM
- CLA tasks are **non-preemptive** - design for deterministic execution time

### Dual-Core CPU2 Usage

CPU2 is **not currently active** (boot call commented out):
```c
// Device_bootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_FLASH);
```

If activating CPU2:
- Separate linker command file required
- IPC (Inter-Processor Communication) for CPU1↔CPU2 messaging
- Careful GPIO ownership management (shared peripheral arbitration)

### Watchdog Configuration

Watchdog serviced every 1ms in `INT_EPWM1_ISR`:
```c
SysCtl_serviceWatchdog();
```

Timeout configured in `Device_init()` (device library). Do not remove service calls without reconfiguring watchdog timeout.

### Performance Optimization

**GPIO Direct Register Access (10x faster than driverlib)**

All GPIO operations in ISR/RAMFUNC use direct register access instead of `GPIO_writePin()`:

```c
// Fast macros defined in HABA_globals.h
GPIO8_SET()     // Relay 8 ON  (5-10 cycles vs 50-100 cycles)
GPIO8_CLEAR()   // Relay 8 OFF
GPIO90_SET()    // Debug pin HIGH
GPIO90_CLEAR()  // Debug pin LOW
```

**Performance Impact:**
- **driverlib GPIO_writePin()**: ~50-100 cycles (~0.25-0.5μs @ 200MHz)
- **Direct register access**: ~5-10 cycles (~0.025-0.05μs @ 200MHz)
- **Speed improvement**: ~10x faster

**Timing Debug GPIO (Conditional Compilation)**

Debug GPIO toggles (GPIO 90, 91, 92) are used for oscilloscope timing measurement:

- **GPIO 90**: ISR execution timing (100kHz)
- **GPIO 91**: 1ms task timing (1kHz)
- **GPIO 92**: 10ms task timing (100Hz)

Control via `ENABLE_TIMING_DEBUG` flag in `HABA_globals.h`:
```c
#define ENABLE_TIMING_DEBUG     0    // 0=disabled (release), 1=enabled (debug)
```

Usage in code:
```c
DEBUG_ISR_START();    // Expands to GPIO90_SET() or nothing
// ... ISR code ...
DEBUG_ISR_END();      // Expands to GPIO90_CLEAR() or nothing
```

When **disabled** (release build):
- Debug macros expand to empty statements
- No overhead at all
- Recommended for production firmware

When **enabled** (debug build):
- GPIO toggles active for timing analysis
- Use oscilloscope to measure execution time
- Still ~10x faster than driverlib calls