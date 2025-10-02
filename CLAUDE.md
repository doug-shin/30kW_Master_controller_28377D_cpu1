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
- Path variables: `COM_TI_C2000WARE_INSTALL_DIR`, `C2000WARE_DLIB_ROOT`

## Code Architecture

### Module Organization

```
30kW_Master_controller_28377D.c    Main entry point and control loop
├── HABA_Init.c/h                  Hardware initialization (GPIO, PWM, CAN, SCI, SPI, ADC, CLA)
├── HABA_Ctrl.c/h                  Control algorithms, communication handlers, ISRs
├── HABA_shared.c/h                Global variables, constants, type definitions
└── HABA_cla_tasks.cla             CLA assembly tasks for high-speed PI control
```

### Real-Time Control Architecture

**Primary ISR**: `INT_Init_EPWM1_ISR` @ 100kHz (EPWM1 period interrupt)

**5-Phase Task Scheduler** (20kHz effective per task):
The control loop uses function pointers to cycle through 5 tasks at 1/5 the ISR frequency:

```c
void (*control_task_functions[5])(void) = {
    sensing_task,           // Phase 0: Voltage/current ADC averaging
    pi_control_task,        // Phase 1: CLA PI controller + soft-start
    current_command_task,   // Phase 2: Current reference processing
    temperature_task,       // Phase 3: NTC temperature monitoring
    average_task           // Phase 4: Long-term averaging
};
```

Execution flow in ISR:
```c
control_task_functions[control_phase]();
if (++control_phase >= 5) control_phase = 0;
```

### Critical Timing Loops

**1ms Loop** (`_1ms_flag`):
- CAN bus communication with slave modules
- Watchdog service
- Emergency stop monitoring

**10ms Loop** (`flag_10ms`):
- HMI data reception and parsing
- Slave status updates (channels 1-16)
- System data transmission to HMI

**50ms Loop** (`flag_50ms`):
- System voltage reporting
- Master ID selection
- Extended diagnostics

### CLA (Control Law Accelerator) Integration

The CLA runs **independent of CPU** for deterministic, low-latency PI control:

**File**: `HABA_cla_tasks.cla`

**Tasks**:
- `Cla1Task1`: High-voltage PI controller (charging mode)
- `Cla1Task2`: Low-voltage PI controller (discharging mode)
- Tasks 3-8: Reserved (currently unused)

**CPU-CLA Shared Memory**:
- Command: `Voh_cmd`, `Vol_cmd`, `Kp`, `Ki`, `T_sample`, `I_sat`, `I_MAX`
- Feedback: `Vfb` (voltage), `Voh_pi_out`, `Vol_pi_out` (control outputs)
- Variables declared in `HABA_shared.h` with `extern` linkage

**CLA Initialization**: `initCpu1Cla1()` in `HABA_Init.c`
- Memory mapping for message RAMs
- Task trigger configuration
- Scratchpad memory setup

### Communication Protocols

**CAN Bus** (CANA @ 500kbps):
- **TX**: Master commands to slaves (0xE0 base ID)
- **RX**: Slave status feedback (0xF1-0xFF, mailboxes 2-16)
- Protocol: 4-byte messages with current, temperature, status flags
- Function: `send_CANA_Message()`, `CAN_SlaveRead()`

**RS485A/B** (SCIA/B @ 5.625Mbps):
- Current reference transmission to legacy modules
- DE (Driver Enable) GPIO control for half-duplex
- Functions: `send_485A_CurrentCommand()`, `send_485B_CurrentCommand()`

**SPI Channels**:
- **SPIA**: DAC80502 control (5MHz, 8-bit frames) - analog output
- **SPIC**: FPGA communication (ADC data reception: Vo, Vbat, Io)
- Function: `read_FPGA_data()` called every 100kHz ISR

**SCID (HMI Interface)** @ 115200 baud:
- Protocol: Custom 10-byte packet with STX/ETX framing
- Commands: Run/Stop, Mode selection, Voltage/Current setpoints
- Modbus-like checksum validation
- Functions: `UI_monitoring()`, `send_slave_data_to_hmi()`, `send_system_voltage_to_hmi()`

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
#pragma CODE_SECTION(INT_Init_EPWM1_ISR, ".TI.ramfunc");
#pragma CODE_SECTION(sensing_task, ".TI.ramfunc");
#pragma CODE_SECTION(pi_control_task, ".TI.ramfunc");
```

This eliminates Flash wait states for 100kHz ISR execution.

### Soft-Start Ramping

Current commands undergo **soft-start limiting** to prevent inrush:

```c
soft_start_limit += CURRENT_LIMIT * 0.00005f;  // 20kHz ramp rate
if (soft_start_limit > CURRENT_LIMIT) soft_start_limit = CURRENT_LIMIT;
```

Then low-pass filtered before PI control:
```c
I_cmd_filt = La_Ee * (I_cmd_ss + I_cmd_old) + Lb_Ee * I_cmd_filt;
```

Filter coefficients calculated at init: `wc=1kHz`, `Ts=50μs`

### Voltage Feedback Selection

Feedback voltage switches based on operating sequence:

```c
if (sequence_step == 20) Vfb = Vbat;  // Pre-charge: regulate battery side
else Vfb = Vo;                         // Normal: regulate output side
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
Defined in `DIGITAL_REG` union (GPIO36-39) for DIP switches or discrete sensors.

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
void Master_ID_Select(void); // Reads GPIO pins to determine Master_ID (0 or 1)
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
- HMI protocol rev 2.0 (09.11, 09.15)
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
2. **CLA Access**: Variables accessed by CLA must be in message RAM (see `initCpu1Cla1()`)
3. **ISR Functions**: Use `#pragma CODE_SECTION(..., ".TI.ramfunc")` for time-critical code
4. **Task Timing**: Respect 10μs ISR budget - offload heavy work to background loops

### When Adding Communication Features

- **CAN**: Extend mailbox configuration in `initCANA()`, update `RX_MSG_OBJ_COUNT`
- **SCID Protocol**: Modify `HMI_PACKET` struct and `modbus_parse()` function
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

Watchdog serviced every 1ms in `INT_Init_EPWM1_ISR`:
```c
SysCtl_serviceWatchdog();
```

Timeout configured in `Device_init()` (device library). Do not remove service calls without reconfiguring watchdog timeout.
