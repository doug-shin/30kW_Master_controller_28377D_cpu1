# 30kW Master Controller 빠른 참조 가이드

**대상**: 신규 개발자, 긴급 디버깅, 빠른 코드 탐색
**최종 업데이트**: 2025년 10월 19일

---

## 🚀 빠른 시작

### 1분 안에 프로젝트 이해하기

```
이 펌웨어는 무엇인가?
→ 30kW 배터리 전력 변환 시스템 마스터 제어기
→ TI F28377D DSP (200MHz) 기반
→ 100kHz 실시간 제어, CLA 가속 PI 제어

주요 기능은?
→ 최대 31개 슬레이브 모듈 제어 (CAN)
→ 과전압/과전류/과온 보호
→ 개별/병렬 운전 모드
→ SCADA 통신 (115200 baud)

어디서부터 읽어야 하나?
1. HABA_main.c (엔트리 포인트, 277라인)
2. HABA_control.c (제어 로직, Phase 함수들)
3. HABA_globals.h (변수/상수 정의)
```

---

## 📍 중요 파일 위치

| 찾고 있는 것 | 파일 | 라인 번호 |
|--------------|------|-----------|
| **메인 루프** | `HABA_main.c` | 115~184 |
| **100kHz ISR** | `HABA_main.c` | 196~272 |
| **Phase 0 (센싱)** | `HABA_control.c` | 105~149 |
| **Phase 1 (PI 적용)** | `HABA_control.c` | 173~323 |
| **PI 제어기 (CLA)** | `HABA_cla_tasks.cla` | 17~53 |
| **전압 캘리브레이션** | `HABA_control.c` | 114~125 |
| **CAN 슬레이브 읽기** | `HABA_control.c` | ~600 |
| **SCADA 파싱** | `HABA_control.c` | ~900 |
| **GPIO 초기화** | `HABA_setup.c` | ~100 |
| **CLA 초기화** | `HABA_setup.c` | ~350 |

---

## ⚡ 핵심 제어 루프 (100kHz)

```
INT_EPWM1_ISR (10μs 주기)
│
├─ Read_FPGA_Data()             // SPI로 V_out, V_batt, I_out 읽기
│
├─ 5-Phase 순환 (20kHz)
│  ├─ Phase 0: Sensing_And_Trigger_PI()      ~2.0μs
│  │   └─ 전압 평균 → 캘리브레이션 → CLA Force
│  │
│  ├─ Phase 1: Apply_PI_And_Convert_DAC()    ~3.0μs
│  │   └─ CLA 결과 사용 → 전류 제한 → DAC 변환
│  │
│  ├─ Phase 2: Transmit_Current_Command()    ~1.5μs
│  │   └─ RS485 전류 지령 송신
│  │
│  ├─ Phase 3: Check_System_Safety()         ~1.5μs
│  │   └─ 고장 체크 → 릴레이 제어
│  │
│  └─ Phase 4: Update_Monitoring_And_Sequence() ~1.0μs
│      └─ 평균 계산 → 소프트스타트 → 시퀀스
│
└─ 타이밍 플래그 생성 (1ms, 10ms, 50ms)
```

---

## 🔧 자주 수정하는 코드

### 1️⃣ 보호 임계값 변경
```c
// HABA_globals.h:106-109
#define OVER_VOLTAGE  1400      // 과전압 (V)
#define OVER_CURRENT  88.0f     // 과전류 (A)
#define OVER_TEMP     120       // 과온도 (°C)
```

### 2️⃣ 전류 제한 변경
```c
// HABA_globals.h:91-92
#define CURRENT_LIMIT_INDIVIDUAL  480.0f   // 개별 모드 (A)
#define CURRENT_LIMIT_PARALLEL    960.0f   // 병렬 모드 (A)
```

### 3️⃣ PI 게인 튜닝
```c
// HABA_setup.c: Init_CPU1_CLA1()
pi_charge.Kp = 1.0f;
pi_charge.Ki = 3000.0f * 50e-6f;  // = 0.15 (이산 시간)
pi_charge.Umax = 80.0f;           // 출력 상한
pi_charge.Umin = -2.0f;           // 출력 하한
pi_charge.Imax = 80.0f;           // 적분기 상한 (동적 업데이트됨)
pi_charge.Imin = -2.0f;           // 적분기 하한
```

### 4️⃣ 전압 캘리브레이션
```c
// HABA_control.c: Sensing_And_Trigger_PI()
// Step 2: 실측 캘리브레이션
V_out = (V_out_uncal + 0.091694057f) * 0.9926000888f;
V_batt = (V_batt_uncal - 0.3058461657f) * 0.9945009708f;
```

**캘리브레이션 절차**:
1. 기준 전압계로 실제 전압 측정
2. `V_out_uncal`, `V_batt_uncal` 값 읽기 (디버거)
3. 선형 회귀로 오프셋/게인 계산
4. 상수 업데이트

### 5️⃣ SCADA 프로토콜 수정
```c
// HABA_control.c: Parse_SCADA_Command()
// 패킷 구조: [STX][CMD][Param1_H][Param1_L][Param2_H][Param2_L]
//             [Param3_H][Param3_L][CRC32_3][CRC32_2][CRC32_1][CRC32_0][ETX]

uint8_t cmd = scada_rx_buffer[1];  // 명령 바이트
// bit[7]: 제어 모드 (0=충방전, 1=배터리)
// bit[6]: Ready 상태
// bit[5]: Run 상태
// bit[4]: Parallel 모드
```

---

## 🐛 디버깅 팁

### 문제: ISR이 너무 느림
```c
// HABA_globals.h:32
#define ENABLE_TIMING_DEBUG  1  // 활성화

// 오실로스코프로 GPIO90, 91, 92 측정
// GPIO90: ISR 실행 시간 (목표 <2μs/phase)
```

### 문제: CAN 슬레이브 응답 없음
```c
// HABA_control.c: Read_CAN_Slave()
// 디버거 브레이크포인트 설정
if (msgStatus & CAN_STATUS_RXOK) {
    // 여기서 멈추면 메시지 수신됨
    // rxData[0~3] 확인
}

// can_rx_fault_cnt[slave_id] 확인 (10000 이상이면 통신 끊김)
```

### 문제: PI 제어 불안정
```c
// HABA_cla_tasks.cla
// CLA 디버그 변수 활성화
cla_cnt++;  // CLA 실행 카운터 (매 사이클 증가해야 함)

// CPU에서 확인
// I_PI_charge_out, I_PI_discharge_out 값 모니터링
// 발산하면 게인 낮추기 (Kp, Ki)
```

### 문제: 릴레이 동작 안 함
```c
// HABA_control.c: Check_System_Safety()
if (run == 1 && sequence_step >= SEQ_STEP_PRECHARGE_DONE)
    MAIN_RELAY_ON();  // GPIO8 = 1

// 체크리스트:
// 1. run == 1 인가? (비상 정지 스위치 확인)
// 2. sequence_step >= 10 인가?
// 3. GPIO8 출력 활성화되었나? (Init_GPIO_CPU1)
```

---

## 📊 중요 변수 위치

### 전압/전류
```c
V_out           // 캘리브레이션된 출력 전압 (V)
V_batt          // 캘리브레이션된 배터리 전압 (V)
V_out_display   // 디스플레이용 평균 (200샘플 = 10ms)
V_fb            // PI 제어 피드백 전압 (V_out 또는 V_batt)

I_cmd           // 전류 지령 (SCADA 수신, A)
I_cmd_ramped    // 소프트스타트 적용 전류 (A)
I_cmd_filtered  // LPF 필터 출력 (A)
I_cmd_to_slave  // DAC 출력값 (0~65535)
```

### 제어 모드
```c
operation_mode  // MODE_STOP(0), MODE_INDIVIDUAL(1), MODE_PARALLEL(2)
control_mode    // CHARGE_DISCHARGE(0), BATTERY(1)
sequence_step   // 0=대기, 10=Precharge 완료, 20=정상 운전
start_stop      // START(1), STOP(0)
run             // 실행 플래그 (비상 정지 스위치 반영)
```

### PI 제어기 (CLA 공유)
```c
pi_charge       // DCL_PI_CLA 구조체 (충전 모드)
pi_discharge    // DCL_PI_CLA 구조체 (방전 모드)
pi_cv           // DCL_PI_CLA 구조체 (배터리 모드)

I_PI_charge_out     // CLA Task 1 출력 (A)
I_PI_discharge_out  // CLA Task 2 출력 (A)
I_PI_cv_out         // CLA Task 3 출력 (A)
```

### 슬레이브 상태
```c
I_out_slave[31]     // 슬레이브 전류 (A)
temp_slave_raw[31]  // 슬레이브 온도 (원시값)
DAB_ok_slave[31]    // DAB OK 플래그
can_rx_fault_cnt[31] // CAN 수신 고장 카운터 (>10000이면 고장)
```

---

## 🔌 통신 프로토콜 요약

### CAN (500kbps)
```
Master → Slave: 0xE0 (4바이트)
  [0] = 0xA0 (Buck_EN=1) 또는 0x00 (OFF)

Slave → Master: 0xF1~0xFF (4바이트)
  [0] = Current_L (0~255)
  [1] = Current_H + DAB_OK (bit7)
  [2] = Temp (0~255)
  [3] = Reserved
```

### RS485 (5.625Mbps)
```
Master1 → Master2 (SCIA): 4바이트
  [0] = STX (0x1B)
  [1] = Current_L
  [2] = Current_H
  [3] = ETX (0x03)

Master → Slave (SCIB): 4바이트 (동일 형식)
```

### SCADA (115200 baud)
```
수신 (SCADA → Master): 13바이트
  [0] = STX (0x1B)
  [1] = CMD (제어 명령)
  [2~7] = Param1~3 (Big-endian)
  [8~11] = CRC-32
  [12] = ETX (0x03)

송신 (Master → SCADA):
  - 시스템 전압: 7바이트 (50ms 주기)
  - 슬레이브 상태: 7바이트×15 (10ms 주기)
```

---

## 🎯 성능 벤치마크

| 항목 | 측정값 | 목표값 | 상태 |
|------|--------|--------|------|
| ISR 실행 시간 | ~1.8μs/phase | <2μs | ✅ |
| CPU 사용률 | 18% | <30% | ✅ |
| CLA 레이턴시 | 10μs | <20μs | ✅ |
| CAN 버스 부하 | 25% | <50% | ✅ |
| 코드 크기 (Flash) | ~50KB | <128KB | ✅ |
| RAM 사용량 | ~8KB | <32KB | ✅ |

---

## 🛡️ 안전 체크리스트

### 시스템 시작 전
- [ ] 비상 정지 스위치 정상 동작 확인
- [ ] 전압 센서 캘리브레이션 검증 (±1% 오차)
- [ ] CAN 통신 정상 (슬레이브 응답 확인)
- [ ] 보호 임계값 설정 확인
- [ ] 릴레이 동작 테스트 (수동 제어)

### 코드 수정 후
- [ ] RAMFUNC 섹션 배치 확인 (ISR 함수)
- [ ] 전역 변수 초기화 확인
- [ ] CLA 공유 변수 메모리 매핑 확인
- [ ] 인터럽트 우선순위 충돌 없음
- [ ] Watchdog 서비스 누락 없음

### 배포 전
- [ ] `ENABLE_TIMING_DEBUG = 0` (릴리스 빌드)
- [ ] 디버그 플래그 모두 비활성화
- [ ] Flash 빌드 구성 확인
- [ ] 버전 번호 업데이트
- [ ] CLAUDE.md 문서 업데이트

---

## 🔗 빠른 링크

- **전체 문서 인덱스**: [PROJECT_INDEX.md](./PROJECT_INDEX.md)
- **코드 분석 보고서**: [CODE_ANALYSIS_REPORT.md](./CODE_ANALYSIS_REPORT.md)
- **SCADA 프로토콜**: [RS232_interface_protocol_rev4.md](./RS232_interface_protocol_rev4.md)
- **개발 가이드**: [../CLAUDE.md](../CLAUDE.md)

---

## 📞 긴급 문제 해결

### 문제: 시스템 부팅 안 됨
1. Watchdog 타임아웃 확인 (`SysCtl_serviceWatchdog()` 호출 여부)
2. Flash 프로그래밍 성공 확인 (CCS 로그)
3. 클럭 설정 확인 (`Device_init()`)
4. RAMFUNC 복사 확인 (`memcpy` 성공 여부)

### 문제: 제어 발산
1. PI 게인 절반으로 감소 (Kp, Ki)
2. 전압 피드백 신호 확인 (`V_fb`)
3. CLA Task 실행 확인 (`cla_cnt` 증가 여부)
4. 전류 제한 확인 (`I_cmd_PI_limited` 범위)

### 문제: 통신 끊김
1. CAN: 터미네이션 저항 확인 (120Ω × 2)
2. RS485: DE (Driver Enable) GPIO 토글 확인
3. SCADA: 보드레이트 일치 확인 (115200)
4. 공통: 오실로스코프로 신호 파형 확인

---

**마지막 업데이트**: 2025년 10월 19일
**문서 버전**: 1.0
