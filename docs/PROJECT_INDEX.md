# 30kW Master Controller 프로젝트 문서 인덱스

**프로젝트명**: TI F28377D 30kW Battery Power Conversion System Master Controller
**플랫폼**: Texas Instruments F28377D Dual-Core DSP
**버전**: 1.0
**최종 업데이트**: 2025년 10월 19일

---

## 📚 문서 개요

이 프로젝트는 30kW 배터리 전력 변환 시스템의 마스터 제어기 펌웨어입니다. 본 인덱스는 프로젝트 구조, API 레퍼런스, 개발 가이드를 포함한 전체 문서의 진입점입니다.

---

## 📖 핵심 문서

### 1. 시작 가이드
- **[CLAUDE.md](../CLAUDE.md)** - AI 개발 어시스턴트용 프로젝트 가이드
  - 프로젝트 개요 및 아키텍처
  - 빌드 시스템 설명
  - 코드 구조 및 중요 섹션
  - 개발 노트 및 주의사항

### 2. 기술 분석 문서
- **[CODE_ANALYSIS_REPORT.md](./CODE_ANALYSIS_REPORT.md)** - 코드 품질 및 보안 분석 보고서
  - 코드 품질 평가 (복잡도, 유지보수성)
  - 보안 취약점 분석 (버퍼 오버플로우, 메모리 관리)
  - 성능 분석 (타이밍, 최적화 기회)
  - 아키텍처 분석 (모듈 구조, 의존성)
  - 권장 개선 로드맵

- **[POWER_CONTROL_REVIEW.md](./POWER_CONTROL_REVIEW.md)** - 전력 제어 전문가 리뷰
  - PI 제어기 튜닝 분석
  - 안전 임계값 검토
  - 성능 최적화 제안

### 3. 통신 프로토콜
- **[RS232_interface_protocol_rev4.md](./RS232_interface_protocol_rev4.md)** - SCADA 통신 프로토콜 명세 (최신)
  - 패킷 구조 (13바이트)
  - CRC-32 검증 방식
  - 제어 명령 및 상태 보고
  - 배터리 모드 추가 (Rev 4.0)

- **[RS232_interface_protocol_rev2.1.md](./RS232_interface_protocol_rev2.1.md)** - 레거시 SCADA 프로토콜 (참고용)

### 4. 시스템 제어 (레거시)
- **[System_Control.MD](./System_Control.MD)** - ⚠️ 구버전 개발 노트 (역사적 참고용)

---

## 🗂️ 프로젝트 파일 구조

### 핵심 소스 파일
```
30kW_Master_controller_28377D_cpu1/
├── HABA_main.c              - 시스템 엔트리 포인트 및 메인 루프
├── HABA_setup.c/h           - 하드웨어 초기화 모듈
├── HABA_control.c/h         - 제어 알고리즘 및 통신 핸들러
├── HABA_globals.c/h         - 전역 변수 및 상수 정의
├── HABA_cla_tasks.cla       - CLA PI 제어기 (DCL 기반)
├── device/                  - TI Device Support Files
│   ├── device.c/h           - 디바이스 초기화
│   └── driverlib/           - TI DriverLib (주변장치 드라이버)
├── docs/                    - 프로젝트 문서 (이 폴더)
├── .git/                    - Git 버전 관리
└── *.cmd                    - Linker Command Files (Flash/RAM)
```

### 빌드 구성 파일
- `2837xD_FLASH_CLA_lnk_cpu1.cmd` - Flash 빌드용 Linker 스크립트
- `2837xD_RAM_CLA_lnk_cpu1.cmd` - RAM 빌드용 Linker 스크립트 (디버깅)
- `F2837xD_Headers_nonBIOS_cpu1.cmd` - 메모리 매핑 주변장치 헤더

---

## 📋 API 레퍼런스

### 제어 함수 (HABA_control.h)

#### 5-Phase 제어 함수 (20kHz @ RAMFUNC)
| 함수 | 설명 | 실행 시간 |
|------|------|----------|
| `Sensing_And_Trigger_PI()` | Phase 0: 전압 센싱 + CLA PI 트리거 | ~2.0μs |
| `Apply_PI_And_Convert_DAC()` | Phase 1: PI 결과 적용 + DAC 변환 | ~3.0μs |
| `Transmit_Current_Command()` | Phase 2: RS485 전류 지령 전송 | ~1.5μs |
| `Check_System_Safety()` | Phase 3: 고장 체크 + 릴레이 제어 | ~1.5μs |
| `Update_Monitoring_And_Sequence()` | Phase 4: 모니터링 + 시퀀스 | ~1.0μs |

#### 제어 알고리즘
| 함수 | 설명 |
|------|------|
| `Check_Fault()` | 고장 감지 및 LED 제어 (과전압/과전류/과온) |
| `Apply_Current_Reference_Limit()` | 시스템 상태 업데이트 (슬레이브 모니터링) |
| `Update_System_Sequence()` | 시퀀스 제어 (Precharge → Run) |

#### CAN 통신 (슬레이브 제어)
| 함수 | 설명 |
|------|------|
| `Send_CANA_Message(int8_t CAN_CMD)` | CAN 메시지 송신 (Master → Slave) |
| `Read_CAN_Slave(uint16_t mbox)` | 개별 슬레이브 데이터 읽기 (mailbox 2~32) |
| `Init_Slave_Variables()` | 슬레이브 초기화 |

#### SPI 통신
| 함수 | 설명 |
|------|------|
| `Write_SPI_DAC1(uint8_t cmd, uint16_t data)` | DAC80502 제어 (SPIA, 5MHz) |
| `Read_FPGA_Data()` | FPGA ADC 데이터 읽기 (SPIC, V_out/V_batt/I_out) |

#### RS485 통신
| 함수 | 설명 |
|------|------|
| `Send_RS485_MM_Current(uint16_t current)` | SCIA: Master-to-Master 전류 지령 |
| `Send_RS485_MS_Current(uint16_t current)` | SCIB: Master-to-Slave 전류 지령 |

#### SCADA 통신 (SCID)
| 함수 | 설명 |
|------|------|
| `Parse_SCADA_Command()` | SCADA 패킷 파싱 (CRC-32 검증) |
| `Send_Slave_Status_To_SCADA()` | 슬레이브 상태 → SCADA 송신 |
| `Send_System_Voltage_To_SCADA()` | 시스템 전압 → SCADA 송신 |

#### GPIO 유틸리티
| 함수 | 설명 |
|------|------|
| `Read_Master_ID_From_DIP()` | Master ID 읽기 (GPIO36~39 DIP 스위치) |

#### 인터럽트 서비스 루틴 (ISR)
| 함수 | 설명 |
|------|------|
| `INT_EPWM1_ISR()` | 메인 제어 ISR (100kHz) |
| `CLA1_ISR1()` | CLA Task1 완료 ISR (충전 모드) |
| `CLA1_ISR2()` | CLA Task2 완료 ISR (방전 모드) |
| `SCIA_RS485_MM_Rx_ISR()` | SCIA RS485 수신 ISR |
| `SPIC_FPGA_Rx_ISR()` | SPIC FPGA 수신 ISR |
| `SCID_SCADA_Rx_ISR()` | SCID SCADA 수신 ISR |

---

### 초기화 함수 (HABA_setup.h)

| 함수 | 설명 |
|------|------|
| `Init_System()` | 전체 시스템 초기화 (메인에서 호출) |
| `Init_GPIO_CPU1()` | CPU1 GPIO 핀 구성 |
| `Init_EPWM()` | PWM 모듈 초기화 (EPWM1: 100kHz ISR, EPWM3: ADC 트리거) |
| `Init_CANA()` | CAN 버스 초기화 (500kbps, 메일박스 설정) |
| `Init_SCIA()` | SCIA RS485 초기화 (5.625Mbps, Master-to-Master) |
| `Init_SCIB()` | SCIB RS485 초기화 (5.625Mbps, Master-to-Slave) |
| `Init_SCID_SCADA()` | SCID SCADA 초기화 (115200 baud) |
| `Init_SPIA()` | SPIA DAC 초기화 (5MHz, DAC80502) |
| `Init_SPIC()` | SPIC FPGA 초기화 (ADC 데이터 수신) |
| `Init_ADCA()` | ADCA 초기화 (NTC 온도 센싱용) |
| `Init_INTERRUPT()` | 인터럽트 벡터 테이블 설정 |
| `Init_CPU1_CLA1()` | CLA 초기화 (PI 제어기 설정) |

---

### CLA Tasks (HABA_cla_tasks.cla)

| Task | 설명 | 제어기 |
|------|------|--------|
| `Cla1Task1()` | 충전 모드 PI 제어 | `pi_charge` (V_max_cmd 기준) |
| `Cla1Task2()` | 방전 모드 PI 제어 | `pi_discharge` (V_min_cmd 기준) |
| `Cla1Task3()` | 배터리 모드 CV 제어 | `pi_cv` (V_cmd 기준) |
| `Cla1Task4~8()` | 예약 (미사용) | - |

---

## 🔧 시스템 상수 및 설정 (HABA_globals.h)

### 제어 상수
```c
#define CURRENT_LIMIT_INDIVIDUAL  480.0f   // 개별 모드 최대 전류 (A)
#define CURRENT_LIMIT_PARALLEL    960.0f   // 병렬 모드 최대 전류 (A)
```

### 보호 임계값
```c
#define OVER_VOLTAGE   1400        // 과전압 (V)
#define OVER_CURRENT   88.0f       // 과전류 (A)
#define OVER_TEMP      120         // 과온도 (°C)
```

### 시퀀스 단계
```c
#define SEQ_STEP_IDLE              0    // 대기 (Precharge 준비)
#define SEQ_STEP_PRECHARGE_DONE   10    // Precharge 완료
#define SEQ_STEP_NORMAL_RUN       20    // 정상 운전
```

### 타이밍 상수
```c
#define ISR_FREQUENCY_HZ          100000  // EPWM1 ISR: 100kHz
#define PHASE_CYCLE_FREQUENCY_HZ  20000   // Phase 순환: 20kHz
#define TIMING_1MS_AT_100KHZ      100     // 1ms = 100 counts
#define TIMING_10MS_AT_100KHZ     1000    // 10ms = 1000 counts
```

### GPIO 핀 매핑
```c
// 릴레이
#define MAIN_RELAY_ON()       GPIO8_SET()    // 메인 릴레이
#define PARALLEL_LINK_ON()    GPIO9_SET()    // 병렬 연결

// LED (전면 패널)
#define LED_PWR        46   // 전원 LED
#define LED_CHARGE     47   // 충전 LED
#define LED_DISCHARGE  42   // 방전 LED
#define LED_FAULT      43   // 고장 LED
#define LED_SINGLE     67   // 개별 운전 LED
#define LED_DUAL       68   // 병렬 운전 LED
```

---

## 📊 데이터 구조

### 운전 모드 (OperationMode_t)
```c
typedef enum {
    MODE_STOP        = 0,   // 정지
    MODE_INDIVIDUAL  = 1,   // 개별 운전
    MODE_PARALLEL    = 2,   // 병렬 운전
} OperationMode_t;
```

### 제어 모드 (ControlMode_t)
```c
typedef enum {
    CONTROL_MODE_CHARGE_DISCHARGE = 0,  // 충방전 모드 (V_max/V_min)
    CONTROL_MODE_BATTERY          = 1   // 배터리 모드 (V_cmd CV)
} ControlMode_t;
```

### SCADA 패킷 구조
```c
// 시스템 전압 송신 (6바이트)
typedef struct {
    uint8_t stx;            // 0x02
    uint8_t id;             // ID + Channel
    int16_t systemVoltage;  // Big-endian, ÷10
    uint8_t reserved;
    uint8_t checksum;
    uint8_t etx;            // 0x03
} SYSTEM_TX_PACKET;

// 슬레이브 데이터 송신 (7바이트)
typedef struct {
    uint8_t stx;
    uint8_t idAndStatus;    // bit[7:3]: Slave ID, bit[0]: DAB_OK
    int16_t slaveCurrent;   // Big-endian, Center=32768
    uint8_t slaveTemp;      // ×0.5
    uint8_t checksum;
    uint8_t etx;
} SLAVE_TX_PACKET;
```

---

## 🛠️ 빌드 및 개발 환경

### 필수 도구
- **CCS (Code Composer Studio)**: TI 공식 IDE (Eclipse 기반)
- **C2000Ware**: TI 소프트웨어 개발 킷
  - Device Support Files
  - DriverLib (주변장치 드라이버)
  - DCL (Digital Control Library)

### 환경 변수
```
COM_TI_C2000WARE_INSTALL_DIR = /Applications/ti/C2000Ware_6_00_00_00
C2000WARE_DLIB_ROOT = ${COM_TI_C2000WARE_INSTALL_DIR}/driverlib
```

### DCL 통합
```
Include Path:
  ${COM_TI_C2000WARE_INSTALL_DIR}/libraries/control/DCL/c28/include

Source Files:
  DCL_PI_L1.asm
  DCL_PI_L2.asm
  DCL_clamp_L1.asm
  DCL_error.c
```

### 빌드 구성
- **CPU1_FLASH**: 프로덕션 빌드 (Flash 실행)
- **CPU1_RAM**: 디버그 빌드 (RAM 실행, 빠른 반복)

---

## 🔬 테스트 및 디버깅

### 타이밍 측정 GPIO
```c
#define ENABLE_TIMING_DEBUG  0   // 1=활성화, 0=비활성화
```

활성화 시:
- **GPIO90**: ISR 실행 타이밍 (100kHz)
- **GPIO91**: 1ms 태스크 타이밍
- **GPIO92**: 10ms 태스크 타이밍

### 디버그 플래그
```c
#define _DEBUG_CAN_STATUS_ENABLE_  0
#define _DEBUG_SCI_STATUS_ENABLE_  0
#define _DEBUG_SPI_STATUS_ENABLE_  0
```

---

## 🚦 개발 워크플로우

### 1. 제어 알고리즘 수정 시
1. **전역 변수 추가**: `HABA_globals.h` (extern), `HABA_globals.c` (정의)
2. **CLA 접근 변수**: Message RAM 배치 (`Init_CPU1_CLA1()`)
3. **ISR 함수**: `#pragma CODE_SECTION(..., ".TI.ramfunc")` 사용
4. **네이밍**: Pascal_Snake_Case (예: `Update_Voltage_Sensing`)

### 2. 통신 기능 추가 시
- **CAN**: `Init_CANA()`에서 메일박스 확장
- **SCADA**: `SCADA_PACKET` 구조체 수정, `Parse_SCADA_Command()` 업데이트

### 3. 캘리브레이션
```c
// HABA_control.c: Sensing_And_Trigger_PI()
V_out = (V_out_uncal + 0.091694057f) * 0.9926000888f;  // 실측 보정
```
1. 기준 전압계로 실제 전압 측정
2. 원시 ADC 평균값 읽기
3. 선형 보정 계수 업데이트

---

## 🔐 보안 및 안전

### 보안 강점
✅ 고정 크기 버퍼 사용 (동적 할당 없음)
✅ CRC-32 하드웨어 검증 (SCADA)
✅ Fail-Safe 릴레이 제어
✅ 다단계 전류 제한

### 주의 사항
⚠️ 배열 인덱스 경계 검증 부족 (CAN 슬레이브)
⚠️ CRC 실패 시 재전송 메커니즘 없음

→ 자세한 내용은 [CODE_ANALYSIS_REPORT.md](./CODE_ANALYSIS_REPORT.md) 참조

---

## 📈 성능 특성

### 실시간 성능
- **ISR 실행 시간**: ~1.8μs/phase (평균)
- **CPU 사용률**: 18% @ 100kHz ISR (여유 82%)
- **CLA 레이턴시**: 10μs (파이프라인 구조)

### 통신 성능
- **CAN**: 500kbps, 31개 슬레이브, 부하 ~25%
- **RS485**: 5.625Mbps, 4바이트 전송 ~6μs
- **SCADA**: 115200 baud, 13바이트 패킷

---

## 🗺️ 로드맵 및 향후 계획

### 단기 (1주)
- [ ] 배열 경계 검증 추가
- [ ] Magic Number 제거
- [ ] 함수 복잡도 리팩토링

### 중기 (1개월)
- [ ] HAL 계층 도입
- [ ] 상태 머신 기반 시퀀스 제어
- [ ] 구조체 그룹화 (전역 변수 정리)

### 장기 (3개월)
- [ ] 단위 테스트 프레임워크
- [ ] CPU2 확장 (통신 오프로드)
- [ ] MISRA-C 규칙 준수

---

## 📞 문의 및 지원

### 문서 업데이트
- 새 기능 추가 시: API 레퍼런스 섹션 업데이트
- 프로토콜 변경 시: 통신 프로토콜 문서 업데이트
- 아키텍처 변경 시: CLAUDE.md 업데이트

### 추가 리소스
- **TI C2000Ware 문서**: `/Applications/ti/C2000Ware_6_00_00_00/docs`
- **DCL 라이브러리 가이드**: C2000Ware/libraries/control/DCL/docs
- **F28377D 데이터시트**: TI 공식 웹사이트

---

**마지막 업데이트**: 2025년 10월 19일
**문서 버전**: 1.0
**작성자**: Claude Code (AI)
