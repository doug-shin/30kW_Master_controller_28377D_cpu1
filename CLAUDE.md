# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 프로젝트 개요

**팩사이클러(Pack Cycler) 시스템 충방전 제어용 마스터 컨트롤러**

TI F28377D 듀얼코어 DSP 기반 30kW 배터리 팩 충방전 제어 시스템의 마스터용 펌웨어입니다. 실시간 PI 제어를 CLA(Control Law Accelerator)에서 100kHz로 수행하며, 최대 12개의 슬레이브 모듈을 통합 제어합니다.

### 시스템 구성
- **마스터 컨트롤러**: 2개 (CH1, CH2) - 각 채널당 1개
- **슬레이브 모듈**: 채널당 6개, 총 12개 (2채널 × 6모듈)
- **모듈 출력**: 슬레이브당 80A, 30kW
- **전체 용량**:
  - 개별 모드: 채널당 480A (6모듈 × 80A)
  - 병렬 모드: 960A (12모듈 × 80A, Master1이 Master2 제어)

### 하드웨어 플랫폼
- **MCU**: TI TMS320F28377D (200MHz 듀얼코어 C28x + CLA)
- **빌드 도구**: Code Composer Studio (CCS)
- **실행 모드**: Flash 또는 RAM

---

## 빌드 시스템

### CCS 프로젝트 빌드

CCS IDE 또는 TI 제공 커맨드라인 도구로 빌드합니다.

**빌드 구성**:
- `CPU1_FLASH`: 프로덕션 빌드 (Flash 메모리 실행)
- `CPU1_RAM`: 디버그 빌드 (RAM 실행, 빠른 반복 개발)

**Linker 명령 파일**:
- `2837xD_FLASH_CLA_lnk_cpu1.cmd`: Flash + CLA 섹션
- `2837xD_RAM_CLA_lnk_cpu1.cmd`: RAM + CLA 섹션
- `F2837xD_Headers_nonBIOS_cpu1.cmd`: 주변장치 메모리 맵

### 의존성

**필수 라이브러리**:
- **C2000Ware**: TI SDK (device support, driverlib)
- **driverlib.lib**: 주변장치 드라이버 (사전 컴파일)
- **DCL (Digital Control Library)**: CLA용 최적화 제어 알고리즘

**환경 변수**:
```
COM_TI_C2000WARE_INSTALL_DIR = /Applications/ti/C2000Ware_6_00_00_00
C2000WARE_DLIB_ROOT = ${COM_TI_C2000WARE_INSTALL_DIR}/driverlib
```

**DCL 통합** (CCS 프로젝트 설정):
```
Include Path:
  ${COM_TI_C2000WARE_INSTALL_DIR}/libraries/control/DCL/c28/include

Source Files (프로젝트에 추가):
  ${COM_TI_C2000WARE_INSTALL_DIR}/libraries/control/DCL/c28/source/DCL_PI_L1.asm
  ${COM_TI_C2000WARE_INSTALL_DIR}/libraries/control/DCL/c28/source/DCL_PI_L2.asm
  ${COM_TI_C2000WARE_INSTALL_DIR}/libraries/control/DCL/c28/source/DCL_clamp_L1.asm
  ${COM_TI_C2000WARE_INSTALL_DIR}/libraries/control/DCL/c28/source/DCL_error.c
```

---

## 코드 아키텍처

### 핵심 모듈 구조

```
HABA_main.c              시스템 엔트리 포인트 및 메인 루프
├── HABA_setup.c/h       하드웨어 초기화 (GPIO, PWM, CAN, SCI, SPI, ADC, CLA)
├── HABA_control.c/h     제어 알고리즘, 통신 핸들러, ISR
├── HABA_globals.c/h     전역 변수, 상수, 공유 데이터
└── HABA_cla_tasks.cla   CLA PI 제어기 (DCL 기반, 100kHz)
```

### 실시간 제어 아키텍처

**메인 ISR**: `INT_EPWM1_ISR` @ 100kHz (10μs 주기)

**5-Phase 파이프라인 스케줄러** (20kHz 실효):

CPU-CLA 병렬 실행을 위한 파이프라인 구조로, 함수 포인터 테이블로 구현:

```c
static const PhaseFunction control_phase_table[5] = {
    Sensing_And_Trigger_PI,         // Phase 0: 센싱 + CLA Force (비블로킹)
    Apply_PI_And_Convert_DAC,       // Phase 1: CLA 결과 사용 + DAC 변환
    Transmit_Current_Command,       // Phase 2: RS485 전류 지령 전송
    Check_System_Safety,            // Phase 3: 고장 감지 + 릴레이 제어
    Update_Monitoring_And_Sequence  // Phase 4: 평균 계산 + 시퀀스
};

control_phase_table[control_phase]();  // ISR에서 순환 호출
```

**파이프라인 동작**:
```
Phase 0 (n-th):
  전압 센싱 → 캘리브레이션 → LPF → CLA Force (백그라운드)

Phase 1 (n-th, 10μs 후):
  CLA 결과(I_PI_*_out) → 전류 제한 → DAC 변환

Phase 2-4: 통신 및 모니터링
```

**파이프라인 이점**:
- 레이턴시: 10μs (파이프라인 없으면 50μs)
- CPU-CLA 병렬 실행 (CLA 0.1~0.2μs 실행 중 CPU는 다른 Phase 처리)
- 결정론적 완료 보장

### 타이밍 루프

**1ms**: CAN 슬레이브 통신, Watchdog 서비스, 비상 정지 모니터링
**10ms**: SCADA 패킷 파싱, 슬레이브 상태 업데이트, 상태 송신
**50ms**: 시스템 전압 보고, Master ID 읽기

---

## CLA 제어 로직

**CLA Tasks** (`HABA_cla_tasks.cla`):
- `Cla1Task1`: 충전 모드 PI (V_max_cmd 기준)
- `Cla1Task2`: 방전 모드 PI (V_min_cmd 기준)
- `Cla1Task3`: 배터리 모드 CV (V_cmd 기준)

**DCL PI 제어기 사용법** (Parallel form):
```c
// CLA Task 예시 - Parallel form PI (L2)
I_PI_charge_out = DCL_runPI_L2(&pi_charge, V_max_cmd, V_fb);
```

**PI 제어 형태**:
- **Parallel form** (L2): `u(k) = Kp×e(k) + I(k)`, `I(k+1) = I(k) + Ki×e(k)`
- DCL 전환 전 사용하던 parallel form을 유지
- L1(ideal form)은 `u(k) = Kp×(e(k) + I(k))`로 비례항이 적분에도 영향

**CPU-CLA 공유 메모리**:
- `CpuToCla1MsgRAM`: PI 구조체, 지령값, 피드백
- `Cla1ToCpuMsgRAM`: PI 출력, 디버그 카운터

**초기화** (`Init_CPU1_CLA1()` in `HABA_setup.c`):
```c
// 이산시간 PI (Ki = Ki_연속 × Ts)
pi_charge.Kp = 1.0f;
pi_charge.Ki = 3000.0f * 50e-6f;  // = 0.15
pi_charge.Umax = 80.0f;           // 출력 상한
pi_charge.Imax = 80.0f;           // 적분기 상한 (동적 업데이트)
```

---

## 통신 프로토콜

### CAN (CANA @ 500kbps)
- **마스터 → 슬레이브**: 0xE0 (4바이트, Buck_EN 제어)
- **슬레이브 → 마스터**: 0xF1~0xFF (메일박스 2~16, 전류/온도/상태)
- 함수: `Send_CANA_Message()`, `Read_CAN_Slave()`

### RS485 (5.625Mbps)
- **SCIA (Master-to-Master)**: CH1 → CH2 전류 지령 (병렬 모드)
- **SCIB (Master-to-Slave)**: Master → 슬레이브 전류 지령
- DE(Driver Enable) GPIO 제어로 half-duplex
- 함수: `Send_RS485_MM_Current()`, `Send_RS485_MS_Current()`

### SPI
- **SPIA**: DAC80502 제어 (5MHz, 슬레이브 전류 지령 출력)
- **SPIC**: FPGA ADC 통신 (V_out, V_batt, I_out 수신)

### SCADA (SCID @ 115200 baud)
- 13바이트 패킷 (STX/ETX 프레이밍, CRC-32 검증)
- 제어 명령: 모드 선택, Run/Stop, 전압/전류 설정값
- 함수: `Parse_SCADA_Command()`, `Send_Slave_Status_To_SCADA()`
- 프로토콜 명세: `docs/RS232_interface_protocol_rev4.md`

---

## 운전 모드

### Operation Mode (`operation_mode`)
```c
MODE_STOP (0)        // 시스템 정지
MODE_INDIVIDUAL (1)  // CH1, CH2 독립 제어 (각각 6모듈씩)
MODE_PARALLEL (2)    // CH1이 CH2 제어 (12모듈 통합, Master1이 PI, Master2는 패스스루)
```

### Control Mode (`control_mode`)
```c
CONTROL_MODE_CHARGE_DISCHARGE (0)  // 충방전: V_max/V_min 한계, I_cmd 제어
CONTROL_MODE_BATTERY (1)           // 배터리: V_cmd CV 제어, I_max/I_min 한계
```

**모드 영향**:
- Operation Mode → PI 제어 활성화, 릴레이 구성, 전류 라우팅
- Control Mode → CLA Task 선택(1+2 vs 3), 제어 파라미터 사용

**병렬 모드 동작**:
- Master1 (CH1): PI 제어 실행 → RS485로 CH2에 지령 전달
- Master2 (CH2): Master1 지령 수신 → 패스스루 (PI 미실행)
- 전류 분배: 전체 전류 ÷ 2(채널) ÷ 6(모듈) = 모듈당 전류

---

## 시퀀스 제어

`Update_System_Sequence()` 상태 머신:

```c
SEQ_STEP_IDLE (0)             // Precharge 진행 중 (V_out ≈ V_batt ±2V 대기)
SEQ_STEP_PRECHARGE_DONE (10)  // Precharge 완료, 1초 대기
SEQ_STEP_NORMAL_RUN (20)      // 메인 릴레이 ON, 정상 운전
```

**Precharge 완료 조건**:
```c
if (abs(V_out_display - V_batt_display) < 2.0f)
    sequence_step = SEQ_STEP_PRECHARGE_DONE;
```

**보호 임계값** (`HABA_globals.h:106-109`):
```c
OVER_VOLTAGE: 1400V    // 배터리 시스템 최대 전압
OVER_CURRENT: 88.0A    // 슬레이브 최대 전류 (80A + 10% 여유)
OVER_TEMP:    120°C    // NTC 온도 한계
```

⚠️ **안전 권장사항**: `docs/POWER_CONTROL_REVIEW.md` 참조

---

## 중요 코드 섹션

### RAMFUNC 배치

성능 중요 함수는 RAM 배치 (Flash 대기 제거):
```c
#pragma CODE_SECTION(INT_EPWM1_ISR, ".TI.ramfunc");
#pragma CODE_SECTION(Sensing_And_Trigger_PI, ".TI.ramfunc");
```

### Soft-Start 램핑

돌입 전류 방지:
```c
I_ss_ramp += CURRENT_LIMIT * 0.00005f;  // 20kHz 램프
// 개별: 480A/9.6초, 병렬: 960A/19.2초
```

LPF 필터 (fc=1kHz, Ts=50μs):
```c
I_cmd_filtered = lpf_coeff_a * (I_cmd_ramped + I_cmd_prev)
               + lpf_coeff_b * I_cmd_filtered;
```

### 전압 피드백 선택

```c
if (sequence_step == SEQ_STEP_NORMAL_RUN)
    V_fb = V_batt;  // 정상: 배터리 전압 제어
else
    V_fb = V_out;   // Precharge: 출력 전압 제어
```

---

## 하드웨어 맵핑

### GPIO
**릴레이**:
- GPIO8: 메인 릴레이 (배터리 연결)
- GPIO9: 병렬 연결 릴레이 (CH1↔CH2)

**LED** (전면 패널):
- GPIO46: 전원, GPIO47: 충전, GPIO42: 방전
- GPIO43: 고장, GPIO67: 개별, GPIO68: 병렬

**DIP 스위치**: GPIO36~39 (Master ID 설정)

### 전압 캘리브레이션

`Sensing_And_Trigger_PI()`에서 2단계 변환:
```c
// 1단계: ADC → 물리값
V_out_uncal = V_out_raw_avg * VOLTAGE_ADC_SCALE - VOLTAGE_ADC_OFFSET;

// 2단계: 실측 보정
V_out = (V_out_uncal + 0.091694057f) * 0.9926000888f;
```

**캘리브레이션 절차**:
1. 기준 전압계로 실측
2. 디버거에서 `V_out_uncal` 읽기
3. 선형 회귀로 오프셋/게인 계산
4. 상수 업데이트

---

## 개발 가이드

### 제어 알고리즘 수정

1. **전역 변수**: `HABA_globals.h` (extern) + `HABA_globals.c` (정의)
2. **CLA 접근 변수**: Message RAM 배치 (`Init_CPU1_CLA1()`)
3. **ISR 함수**: `#pragma CODE_SECTION(..., ".TI.ramfunc")` 필수
4. **네이밍**: `Pascal_Snake_Case` (예: `Update_Voltage_Sensing`)
5. **타이밍**: 10μs ISR 예산 준수, 무거운 작업은 백그라운드로

### 통신 추가

- **CAN**: `Init_CANA()`에서 메일박스 확장, `RX_MSG_OBJ_COUNT` 업데이트
- **SCADA**: `SCADA_PACKET` 구조체, `Parse_SCADA_Command()` 수정

### 성능 최적화

**GPIO Direct Register Access** (driverlib 대비 10배 고속):
```c
GPIO8_SET()     // 5-10 사이클 (vs GPIO_writePin() 50-100 사이클)
GPIO8_CLEAR()
```

**타이밍 디버그** (`ENABLE_TIMING_DEBUG` 플래그):
```c
#define ENABLE_TIMING_DEBUG  0  // 릴리스: 0, 디버그: 1

DEBUG_ISR_START();   // GPIO90 토글 (오실로스코프로 측정)
// ... ISR code ...
DEBUG_ISR_END();
```

---

## 특수 고려사항

### CLA 프로그래밍 모델
- C 서브셋 (포인터, stdlib 제한)
- 독립 32비트 FPU 파이프라인
- CPU는 CLA 레지스터 직접 접근 불가 (Message RAM 경유)
- 비선점형 → 결정론적 실행 시간 설계

### CPU2 (현재 비활성)
```c
// Device_bootCPU2(...);  // 주석 처리됨
```

활성화 시:
- 별도 Linker 명령 파일
- IPC 메시징 (CPU1↔CPU2)
- GPIO 소유권 관리

### Watchdog
1ms마다 서비스 (`INT_EPWM1_ISR`):
```c
SysCtl_serviceWatchdog();
```

서비스 제거 시 타임아웃 재설정 필요.

---

## 문서

- **QUICK_REFERENCE.md**: 빠른 참조, 디버깅 팁
- **PROJECT_INDEX.md**: API 레퍼런스, 함수 목록
- **CODE_ANALYSIS_REPORT.md**: 코드 품질/보안/성능 분석
- **POWER_CONTROL_REVIEW.md**: PI 튜닝, 안전 권장사항
- **RS232_interface_protocol_rev4.md**: SCADA 프로토콜 명세

**추가 리소스**:
- TI C2000Ware 문서: `/Applications/ti/C2000Ware_6_00_00_00/docs`
- F28377D 데이터시트: TI 공식 사이트
- DCL 가이드: `C2000Ware/libraries/control/DCL/docs`
