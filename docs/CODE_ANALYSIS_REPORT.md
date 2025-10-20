# 30kW Master Controller 코드 분석 보고서

**분석 대상**: TI F28377D 30kW Battery Power Conversion System Master Controller Firmware
**분석 일자**: 2025년 10월 19일
**총 코드 라인 수**: 약 3,849 라인 (핵심 소스 파일 기준)

---

## 목차

1. [프로젝트 개요](#1-프로젝트-개요)
2. [파일 구조 분석](#2-파일-구조-분석)
3. [코드 품질 분석](#3-코드-품질-분석)
4. [보안 취약점 분석](#4-보안-취약점-분석)
5. [성능 분석](#5-성능-분석)
6. [아키텍처 분석](#6-아키텍처-분석)
7. [권장 사항](#7-권장-사항)
8. [결론](#8-결론)

---

## 1. 프로젝트 개요

### 1.1 시스템 사양
- **하드웨어**: TI F28377D Dual-Core DSP (200MHz C28x + CLA)
- **애플리케이션**: 30kW 배터리 전력 변환 시스템 마스터 제어기
- **제어 주기**: 100kHz ISR (실시간 제어), 20kHz Phase 순환
- **통신**: CAN (500kbps), RS485 (5.625Mbps), SCADA (115200 baud)
- **제어 방식**: DCL PI 제어기 (CLA 가속), 파이프라인 구조

### 1.2 주요 기능
- **실시간 전압/전류 제어**: CLA 기반 PI 제어 (100kHz)
- **멀티 슬레이브 관리**: CAN 통신으로 최대 31개 슬레이브 모듈 제어
- **운전 모드**: 개별/병렬 운전, 충방전/배터리 모드
- **보호 기능**: 과전압/과전류/과온도 감지
- **시퀀스 제어**: 프리차지 → 메인 릴레이 → 정상 운전

---

## 2. 파일 구조 분석

### 2.1 핵심 소스 파일 (HABA_*.c/h)

```
HABA_main.c          (277 라인)  - 시스템 엔트리 포인트, 메인 루프
├── HABA_setup.c/h               - 하드웨어 초기화 (GPIO, PWM, CAN, SCI, SPI, ADC, CLA)
├── HABA_control.c/h             - 제어 알고리즘, 통신 핸들러, ISR 구현
├── HABA_globals.c/h (685 라인) - 전역 변수, 상수, 구조체 정의
└── HABA_cla_tasks.cla (83 라인) - CLA PI 제어기 (DCL 기반)
```

**총 라인 수**: 약 3,849 라인 (주석 포함)

### 2.2 의존성
- **TI C2000Ware**: driverlib, device support, DCL (Digital Control Library)
- **외부 라이브러리**: VCU2 CRC-32 (SCADA 프로토콜)
- **Linker**: Flash/RAM 빌드 구성 (`.cmd` 파일)

### 2.3 모듈 분리도
✅ **장점**: 기능별 모듈 분리 명확 (setup, control, globals)
⚠️ **개선 필요**: `HABA_globals.h`가 685라인으로 비대 (리팩토링 필요)

---

## 3. 코드 품질 분석

### 3.1 강점

#### 3.1.1 명확한 주석 및 문서화
```c
/**
 * @brief Phase 0 제어 태스크 (전압 센싱 및 CLA PI 제어기 트리거)
 * @details 파이프라인 제어 구조: ...
 * @note 호출 주기: 20kHz, 실행 시간: ~2μs
 */
```
- Doxygen 스타일 주석 일관 사용
- 파라미터, 반환값, 실행 시간, 호출 위치 명시
- 한글 주석과 영문 변수명 혼용 (팀 커뮤니케이션 효율적)

#### 3.1.2 함수 포인터 기반 Phase 제어
```c
typedef void (*PhaseFunction)(void);
static const PhaseFunction control_phase_table[5] = {
    Sensing_And_Trigger_PI,
    Apply_PI_And_Convert_DAC,
    Transmit_Current_Command,
    Check_System_Safety,
    Update_Monitoring_And_Sequence
};
// ISR 내부
control_phase_table[control_phase]();
```
✅ **장점**: switch-case 제거, 코드 가독성 향상, 유지보수 용이

#### 3.1.3 성능 최적화
- **Direct Register Access**: GPIO 제어를 driverlib 대신 레지스터 직접 액세스 (~10배 고속)
- **RAMFUNC 배치**: ISR 및 제어 함수를 RAM에 배치 (Flash 대기 시간 제거)
- **조건부 컴파일**: 디버그 GPIO 토글을 `ENABLE_TIMING_DEBUG` 플래그로 제어

```c
#if ENABLE_TIMING_DEBUG
    #define DEBUG_ISR_START()    GPIO90_SET()
#else
    #define DEBUG_ISR_START()    // 릴리스 빌드에서 제거
#endif
```

#### 3.1.4 안전 지향 설계
- **Fail-Safe 릴레이 제어**: `run == 0` 시 모든 릴레이 강제 OFF
- **다단계 전류 제한**: PI 출력 → 시스템 레벨 → 모드별 제한
- **고장 플래그 래치**: 과전압/과전류 발생 시 수동 리셋 필요

### 3.2 개선 필요 사항

#### 3.2.1 Magic Number 사용
❌ **문제점**:
```c
if (sequence_step == 20)  // SEQ_STEP_NORMAL_RUN 상수 대신 숫자 사용
    V_fb = V_batt;
```
✅ **권장**:
```c
if (sequence_step == SEQ_STEP_NORMAL_RUN)
    V_fb = V_batt;
```
**영향**: 가독성 저하, 유지보수 어려움 (실제로는 상수 정의되어 있으나 일부 코드에서 숫자 직접 사용)

#### 3.2.2 복잡한 함수 (사이클로매틱 복잡도)
```c
void Apply_PI_And_Convert_DAC(void)  // ~150 라인, 복잡도 높음
{
    // 제어 모드 분기
    if (control_mode == CONTROL_MODE_BATTERY) {
        // 운전 모드 분기
        if (operation_mode == MODE_INDIVIDUAL) { ... }
        else if (operation_mode == MODE_PARALLEL) {
            if (IS_CH1) { ... }
            else { ... }
        }
    }
    else if (control_mode == CONTROL_MODE_CHARGE_DISCHARGE) { ... }

    // DAC 변환 분기
    if (operation_mode == MODE_PARALLEL && IS_CH2) { ... }
    else {
        if (operation_mode == MODE_INDIVIDUAL) { ... }
        else if (operation_mode == MODE_PARALLEL && IS_CH1) { ... }
    }
}
```

⚠️ **문제점**: 중첩 if-else 구조 (최대 3단계 중첩), 읽기 어려움
✅ **권장**: 전략 패턴 또는 상태 머신으로 리팩토링

#### 3.2.3 전역 변수 과다 사용
- **HABA_globals.h**: 113개의 volatile/extern 변수 선언
- **영향**: 스레드 안전성 문제 (멀티코어 확장 시), 디버깅 어려움

✅ **권장**:
- 구조체로 그룹화 (`SystemState_t`, `ControlParams_t`)
- CPU1/CPU2 공유 변수는 IPC 메커니즘 사용

#### 3.2.4 디버그 코드 혼재
```c
#if _DEBUG_CAN_STATUS_ENABLE_
extern volatile uint16_t debug_tx_err_cnt;
extern volatile uint16_t debug_rx_err_cnt;
// ... 15개 이상의 디버그 변수
#endif
```
✅ **권장**: 디버그 전용 모듈로 분리 (`debug.c/h`)

---

## 4. 보안 취약점 분석

### 4.1 버퍼 오버플로우 위험도: 낮음 ✅

#### 4.1.1 고정 크기 버퍼 사용
```c
volatile uint8_t rxBuffer[4];          // SCIA 수신 버퍼
uint8_t scia_rs485_mm_tx_buf[4];      // RS485 송신 버퍼
volatile uint8_t scada_rx_buffer[SCADA_RX_BUFFER_SIZE];  // SCADA 수신
```
✅ **안전 요인**:
- 모든 버퍼가 고정 크기 (`#define` 상수로 정의)
- 동적 메모리 할당 없음 (`malloc`, `free` 미사용)
- 문자열 조작 함수 미사용 (`strcpy`, `sprintf` 없음)

#### 4.1.2 배열 인덱스 검증 부족
⚠️ **잠재적 위험**:
```c
void Read_CAN_Slave(uint16_t mbox)
{
    // mbox 범위 검증 없음 (호출자에 의존)
    CAN_readMessage(CANA_BASE, mbox, rxData);

    if (msgStatus & CAN_STATUS_RXOK)
    {
        uint16_t slave_id = mbox - 1;
        I_out_slave_raw[slave_id] = ...;  // slave_id가 31을 초과하면 버퍼 오버플로우
    }
}
```

✅ **권장 수정**:
```c
void Read_CAN_Slave(uint16_t mbox)
{
    if (mbox < CAN_MAILBOX_MIN || mbox > CAN_MAILBOX_MAX)
        return false;  // 범위 외 메일박스 거부

    uint16_t slave_id = mbox - 1;
    if (slave_id >= MAX_SLAVES)
        return false;  // 배열 경계 검증

    // ... 안전한 접근
}
```

#### 4.1.3 memcpy 사용 검토
```c
// HABA_setup.c:399
memcpy((void*)&Cla1funcsRunStart, (void*)&Cla1funcsLoadStart, (uint32_t)&Cla1funcsLoadSize);
```
✅ **안전 평가**:
- Linker 심볼 주소 사용 (컴파일 타임 결정)
- 크기도 Linker 심볼로 정확 (`&Cla1funcsLoadSize`)
- 버퍼 오버플로우 위험 없음

### 4.2 정수 오버플로우

#### 4.2.1 전압/전류 변환 시 포화 처리
✅ **안전**:
```c
if (I_cmd_to_slave_tmp < DAC_MIN_VALUE) I_cmd_to_slave_tmp = DAC_MIN_VALUE;
if (I_cmd_to_slave_tmp > DAC_MAX_VALUE) I_cmd_to_slave_tmp = DAC_MAX_VALUE;
```
- 모든 DAC 변환에서 명시적 포화 처리
- 부호 없는/부호 있는 타입 변환 시 검증

#### 4.2.2 누적 연산 검증 부족
⚠️ **잠재적 위험**:
```c
V_out_raw_sum += V_out_raw;    // int32_t 누적
V_batt_raw_sum += V_batt_raw;
```
- 5회 누적 후 나누기 → 오버플로우 가능성 낮음
- 그러나 장기 운전 시 검증 필요

### 4.3 타입 안전성

✅ **강점**:
- `enum` 타입 사용 (`OperationMode_t`, `ControlMode_t`, `SystemState`)
- 명시적 타입 캐스팅 (`(uint16_t)`, `(float32_t)`)

⚠️ **주의**:
```c
volatile uint8_t control_mode;  // SCADA에서 uint8_t 수신
// 사용 시 ControlMode_t로 비교
if (control_mode == CONTROL_MODE_BATTERY) { ... }
```
✅ **권장**: `volatile ControlMode_t control_mode;`로 변경

### 4.4 CRC 검증 (SCADA 프로토콜)

✅ **보안 강점**:
```c
// VCU2 CRC-32 하드웨어 가속 사용
CRC_run32BitPoly1(handleCRC_SCADA, parity, &crc, len);
if (crc != expected_crc)
    scada_crc_error_cnt++;  // CRC 에러 카운터 증가
```
- 패킷 무결성 검증
- 하드웨어 CRC (polynomial 0x04C11DB7)

⚠️ **개선 필요**:
- CRC 실패 시 대응 로직 부족 (카운터만 증가, 데이터는 버림)
- 재전송 메커니즘 없음 (SCADA 통신 신뢰성 저하 가능)

---

## 5. 성능 분석

### 5.1 실시간 성능

#### 5.1.1 ISR 실행 시간 분석
```
INT_EPWM1_ISR (100kHz, 10μs 주기)
├── Phase 0: 전압 센싱 + CLA Force      ~2.0μs
├── Phase 1: PI 적용 + DAC 변환         ~3.0μs
├── Phase 2: RS485 전류 지령 전송       ~1.5μs
├── Phase 3: 안전 체크 + 릴레이 제어    ~1.5μs
└── Phase 4: 평균 계산 + 시퀀스         ~1.0μs
──────────────────────────────────────────────
합계 (5 Phase @ 20kHz):                ~9.0μs
```

✅ **성능 평가**:
- ISR 실행 시간: ~1.8μs/phase (평균), 10μs 주기 대비 18% CPU 사용
- 여유: ~8.2μs (82%), 안정적인 실시간 성능

#### 5.1.2 CLA 파이프라인 효율
```
Phase 0 (n-th cycle):
  └─ CLA Force (비블로킹) → Phase 1~4 동안 CLA 백그라운드 실행

Phase 1 (n-th cycle, 10us 후):
  └─ CLA 결과 사용 (I_PI_charge_out, I_PI_discharge_out)
```

✅ **최적화 효과**:
- **레이턴시**: 10μs (vs 50μs without pipelining)
- **CPU-CLA 병렬 실행**: CLA 실행 중 CPU는 Phase 2~4 처리
- **결정론적 완료 보장**: CLA 실행 0.1~0.2μs, 10μs 내 완료 확실

#### 5.1.3 GPIO 성능 최적화
```c
// driverlib (느림, ~50-100 사이클)
GPIO_writePin(8, 1);

// Direct Register Access (빠름, ~5-10 사이클)
GpioDataRegs.GPASET.bit.GPIO8 = 1;
```
✅ **성능 향상**: ~10배 고속 (ISR 내 릴레이 제어 필수)

### 5.2 메모리 사용

#### 5.2.1 RAM 배치 전략
- **RAMFUNC 섹션**: ISR 및 제어 함수 (~2KB)
- **CLA 메시지 RAM**: CPU↔CLA 공유 변수 (~256 bytes)
- **스택**: 정적 할당 (동적 메모리 없음)

✅ **장점**: Flash 대기 시간 제거, 결정론적 실행

#### 5.2.2 글로벌 변수 메모리
- **volatile 변수**: 113개 (대부분 float32_t, uint16_t)
- **추정 크기**: ~800 bytes (최적화 여지 있음)

### 5.3 통신 성능

#### 5.3.1 병목 현상
⚠️ **문제점**:
```c
// Phase 2: RS485 전송 (블로킹)
Send_RS485_MM_Current(I_cmd_to_slave);  // SCIA 송신 완료 대기
Send_RS485_MS_Current(I_cmd_to_slave);  // SCIB 송신 완료 대기
```
- 5.625Mbps RS485: 4바이트 전송 ~6μs (이론값)
- 실제 블로킹 시간 측정 필요

✅ **개선 방안**:
- DMA 기반 전송으로 변경 (블로킹 제거)
- 또는 전송 완료 플래그 폴링 제거

#### 5.3.2 CAN 버스 부하
- 500kbps CAN: 1ms마다 슬레이브 제어 메시지 전송
- 31개 슬레이브 × 4바이트 = 124바이트/1ms
- **이론 부하**: (124 bytes × 10 bits × 1000) / 500000 = 24.8% (여유 충분)

---

## 6. 아키텍처 분석

### 6.1 모듈 구조

```
┌─────────────────────────────────────────────┐
│          HABA_main.c (엔트리 포인트)         │
│  - 초기화                                    │
│  - 메인 루프 (1ms/10ms/50ms 태스크)         │
│  - ISR 등록                                  │
└───────────┬─────────────────────────────────┘
            │
    ┌───────┴───────┬───────────────┬─────────┐
    │               │               │         │
┌───▼──────┐  ┌────▼────┐  ┌───────▼──┐  ┌───▼───┐
│ HABA_    │  │ HABA_   │  │ HABA_    │  │ CLA   │
│ setup.c  │  │control.c│  │globals.c │  │ tasks │
│          │  │         │  │          │  │       │
│ - GPIO   │  │ - ISR   │  │ - 변수   │  │ - PI  │
│ - PWM    │  │ - Phase │  │ - 상수   │  │   제어│
│ - CAN    │  │ - CAN   │  │ - 구조체 │  │       │
│ - SCI    │  │ - SPI   │  │          │  │       │
│ - SPI    │  │ - RS485 │  │          │  │       │
│ - ADC    │  │ - SCADA │  │          │  │       │
│ - CLA    │  │ - Fault │  │          │  │       │
└──────────┘  └─────────┘  └──────────┘  └───────┘
```

✅ **장점**:
- 모듈화 명확: 초기화(setup), 제어(control), 공유 데이터(globals) 분리
- 단일 책임 원칙 준수

⚠️ **개선점**:
- `HABA_control.c` 비대 (1000+ 라인 추정)
- `HABA_globals.h` 685라인 → 구조체로 그룹화 필요

### 6.2 의존성 분석

```
HABA_main.c
    ├── HABA_setup.h    (초기화 함수)
    ├── HABA_control.h  (제어 함수)
    └── HABA_globals.h  (공유 데이터)
        └── driverlib.h (TI 라이브러리)
            └── device.h (디바이스 지원)

HABA_control.c
    ├── HABA_control.h
    ├── HABA_globals.h
    └── vcu2/vcu2_crc.h (CRC-32)

HABA_cla_tasks.cla
    ├── driverlib.h
    └── HABA_globals.h  (pi_charge, pi_discharge, pi_cv)
```

✅ **순환 의존성 없음**
⚠️ **전역 상태 의존**: 대부분의 함수가 `HABA_globals.h`에 의존 (리팩토링 시 주의)

### 6.3 확장성

#### 6.3.1 슬레이브 확장
- **현재**: 최대 31개 슬레이브 (CAN 메일박스 제약)
- **코드**: 배열 크기 `MAX_SLAVES` 상수로 정의
- **확장 용이성**: ✅ 상수 변경만으로 확장 가능

#### 6.3.2 듀얼 코어 확장 (CPU2)
⚠️ **현재 상태**:
```c
// main.c: CPU2 부팅 주석 처리됨
// Device_bootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_FLASH);
```
- CPU2 비활성화
- IPC (Inter-Processor Communication) 미구현

✅ **확장 가능성**:
- 통신 프로토콜 처리 → CPU2 오프로드 가능
- SCADA 파싱, CAN 송수신 → CPU2 전담

#### 6.3.3 제어 모드 확장
- **현재**: Charge/Discharge, Battery 모드
- **확장**: CLA Task 4~8 예약 (미사용)

✅ **설계 우수**: 새 모드 추가 시 CLA Task만 구현하면 됨

### 6.4 테스트 가능성

⚠️ **단위 테스트 어려움**:
- 전역 변수 다수 (모킹 어려움)
- 하드웨어 의존성 높음 (GPIO, SPI, CAN 직접 접근)

✅ **개선 방안**:
- HAL (Hardware Abstraction Layer) 도입
- 테스트 시뮬레이터 제작 (CCS 제공 도구 활용)

---

## 7. 권장 사항

### 7.1 즉시 적용 (우선순위 높음)

#### 7.1.1 배열 경계 검증 추가
```c
// HABA_control.c: Read_CAN_Slave()
bool Read_CAN_Slave(uint16_t mbox)
{
    if (mbox < CAN_MAILBOX_MIN || mbox > CAN_MAILBOX_MAX)
        return false;  // ✅ 추가

    uint16_t slave_id = mbox - 1;
    if (slave_id >= MAX_SLAVES)
        return false;  // ✅ 추가

    // ... 기존 코드
}
```

#### 7.1.2 Magic Number 제거
```c
// HABA_main.c:219 (현재)
if (sequence_step == 20) V_fb = V_batt;

// ✅ 개선
if (sequence_step == SEQ_STEP_NORMAL_RUN) V_fb = V_batt;
```

#### 7.1.3 CRC 실패 시 대응 추가
```c
// HABA_control.c: Parse_SCADA_Command()
if (crc != expected_crc)
{
    scada_crc_error_cnt++;
    // ✅ 추가: 마지막 유효 명령 유지, 에러 플래그 설정
    scada_packet_error_flag = 1;
    return;  // 잘못된 명령 무시
}
```

### 7.2 단기 개선 (1~2주)

#### 7.2.1 함수 복잡도 리팩토링
```c
// 현재: Apply_PI_And_Convert_DAC() (~150 라인)

// ✅ 개선: 함수 분할
static float32_t Calculate_Current_Command_Battery_Mode(void);
static float32_t Calculate_Current_Command_CD_Mode(void);
static uint16_t Convert_Current_To_DAC(float32_t I_cmd);

void Apply_PI_And_Convert_DAC(void)
{
    float32_t I_cmd_limited;

    if (control_mode == CONTROL_MODE_BATTERY)
        I_cmd_limited = Calculate_Current_Command_Battery_Mode();
    else
        I_cmd_limited = Calculate_Current_Command_CD_Mode();

    I_cmd_to_slave = Convert_Current_To_DAC(I_cmd_limited);

    if (run == 0) Reset_Control_State();
}
```

#### 7.2.2 구조체로 그룹화
```c
// 현재: 개별 변수 113개
extern float32_t V_out, V_batt, I_cmd, I_ss_ramp, ...;

// ✅ 개선: 구조체 그룹화
typedef struct {
    float32_t V_out;
    float32_t V_batt;
    float32_t V_out_display;
    float32_t V_batt_display;
} VoltageSensing_t;

typedef struct {
    float32_t I_cmd;
    float32_t I_cmd_ramped;
    float32_t I_ss_ramp;
    uint16_t I_cmd_to_slave;
} CurrentControl_t;

extern VoltageSensing_t voltage;
extern CurrentControl_t current;
```

### 7.3 중장기 개선 (1~3개월)

#### 7.3.1 HAL 계층 도입
```c
// hal_gpio.h
typedef enum { RELAY_MAIN, RELAY_PARALLEL } Relay_t;
void HAL_GPIO_SetRelay(Relay_t relay, bool state);
bool HAL_GPIO_GetRelay(Relay_t relay);

// 테스트 시뮬레이터에서 모킹 가능
#ifdef UNIT_TEST
    #define HAL_GPIO_SetRelay(r, s)  mock_set_relay(r, s)
#endif
```

#### 7.3.2 상태 머신 기반 시퀀스 제어
```c
// 현재: if-else 중첩 (sequence_step == 0, 10, 20)

// ✅ 개선: 상태 머신
typedef enum {
    SEQ_STATE_IDLE,
    SEQ_STATE_PRECHARGING,
    SEQ_STATE_PRECHARGE_DONE,
    SEQ_STATE_NORMAL_RUN,
    SEQ_STATE_FAULT
} SequenceState_t;

typedef struct {
    SequenceState_t state;
    void (*enter)(void);    // 상태 진입 시 호출
    void (*execute)(void);  // 상태 실행
    void (*exit)(void);     // 상태 종료 시 호출
} SequenceFSM_t;
```

#### 7.3.3 단위 테스트 프레임워크
- **도구**: Unity Test Framework (C 임베디드 표준)
- **타겟**: 제어 로직 함수 (CLA 제외)
- **CI/CD**: Git hook으로 커밋 전 자동 테스트

---

## 8. 결론

### 8.1 종합 평가

| 항목               | 평가  | 상세                                      |
|--------------------|-------|-------------------------------------------|
| **코드 품질**      | ⭐⭐⭐⭐ | 명확한 주석, 모듈화, 성능 최적화 우수      |
| **보안**           | ⭐⭐⭐⭐ | 버퍼 오버플로우 위험 낮음, CRC 검증 있음  |
| **성능**           | ⭐⭐⭐⭐⭐ | 실시간 성능 우수, CLA 파이프라인 효율적   |
| **아키텍처**       | ⭐⭐⭐  | 모듈 분리 양호, 전역 변수 과다 사용       |
| **유지보수성**     | ⭐⭐⭐  | 함수 복잡도 높음, 리팩토링 필요           |
| **확장성**         | ⭐⭐⭐⭐ | 슬레이브/모드 확장 용이, CPU2 확장 가능   |
| **테스트 가능성**  | ⭐⭐   | 하드웨어 의존성 높음, HAL 필요            |

**총평**: ⭐⭐⭐⭐ (4/5) - 실전 투입 가능한 안정적인 펌웨어, 개선 여지 있음

### 8.2 주요 강점
1. **실시간 성능**: 100kHz ISR 안정 동작, 여유 82%
2. **안전 설계**: Fail-Safe 릴레이, 다단계 전류 제한, 고장 래치
3. **최적화**: RAMFUNC, Direct GPIO, CLA 파이프라인
4. **문서화**: Doxygen 주석, CLAUDE.md 상세 가이드

### 8.3 주요 개선점
1. **코드 복잡도**: 함수 분할, Magic Number 제거
2. **구조화**: 전역 변수 → 구조체 그룹화
3. **경계 검증**: 배열 인덱스, CRC 실패 처리
4. **테스트**: HAL 계층, 단위 테스트 프레임워크

### 8.4 권장 로드맵

#### Phase 1 (즉시, 1일)
- [ ] 배열 경계 검증 추가 (Read_CAN_Slave)
- [ ] Magic Number 제거 (sequence_step)
- [ ] CRC 실패 처리 개선

#### Phase 2 (단기, 1주)
- [ ] Apply_PI_And_Convert_DAC() 함수 분할
- [ ] 전역 변수 → 구조체 그룹화 (시작)

#### Phase 3 (중기, 1개월)
- [ ] HAL 계층 도입 (GPIO, SPI, CAN)
- [ ] 상태 머신 기반 시퀀스 제어

#### Phase 4 (장기, 3개월)
- [ ] 단위 테스트 프레임워크 구축
- [ ] CPU2 확장 (통신 오프로드)
- [ ] MISRA-C 규칙 준수 검토

---

**분석자**: Claude Code (AI)
**문의**: 추가 분석이나 코드 개선 제안이 필요하시면 요청해주세요.
**참고 문서**:
- `CLAUDE.md` - 프로젝트 개발 가이드
- `POWER_CONTROL_REVIEW.md` - 전력 제어 전문가 리뷰
- `RS232_interface_protocol_rev4.md` - SCADA 프로토콜 명세
