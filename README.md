# 30kW Master Controller Firmware

TI F28377D 기반 30kW 배터리 전력 변환 시스템 마스터 제어기 펌웨어

[![TI F28377D](https://img.shields.io/badge/TI-F28377D-red)](https://www.ti.com/product/TMS320F28377D)
[![C2000Ware](https://img.shields.io/badge/C2000Ware-6.00.00-blue)](https://www.ti.com/tool/C2000WARE)
[![License](https://img.shields.io/badge/license-Proprietary-yellow)]()

---

## 📋 프로젝트 개요

### 시스템 사양
- **MCU**: TI TMS320F28377D Dual-Core DSP
  - CPU1: 200MHz C28x (제어 로직)
  - CLA: Control Law Accelerator (PI 제어)
- **제어 주기**: 100kHz ISR, 20kHz Phase 순환
- **애플리케이션**: 30kW 배터리 충방전 시스템
- **슬레이브 관리**: 최대 31개 모듈 (CAN 통신)

### 주요 기능
- ⚡ **실시간 PI 제어**: CLA 가속, 10μs 레이턴시
- 🔌 **멀티 프로토콜 통신**: CAN (500kbps), RS485 (5.625Mbps), SCADA (115200 baud)
- 🛡️ **보호 기능**: 과전압/과전류/과온 감지, Fail-Safe 릴레이
- 🔄 **유연한 운전 모드**: 개별/병렬, 충방전/배터리 모드
- 📊 **실시간 모니터링**: SCADA 통합, 슬레이브 상태 추적

---

## 🚀 빠른 시작

### 1. 개발 환경 설정

#### 필수 도구
- **CCS (Code Composer Studio)** 12.0 이상
- **C2000Ware** 6.00.00.00
- **Git** (버전 관리)

#### 환경 변수 설정
```bash
export COM_TI_C2000WARE_INSTALL_DIR="/Applications/ti/C2000Ware_6_00_00_00"
export C2000WARE_DLIB_ROOT="${COM_TI_C2000WARE_INSTALL_DIR}/driverlib"
```

### 2. 프로젝트 빌드

#### CCS에서 빌드
1. CCS 실행 → Import Project
2. 빌드 구성 선택:
   - **CPU1_FLASH**: 프로덕션 (Flash 실행)
   - **CPU1_RAM**: 디버깅 (RAM 실행)
3. Project → Build Project

#### 커맨드라인 빌드 (옵션)
```bash
# CCS 설치 경로에서
cd ${CCS_INSTALL_DIR}/utils/bin
./ccsbuild -project /path/to/project -c CPU1_FLASH
```

### 3. 프로그래밍 및 실행
1. XDS100v2/XDS200 JTAG 연결
2. Run → Debug (F11)
3. 디버거에서 Run (F8)

---

## 📁 프로젝트 구조

```
30kW_Master_controller_28377D_cpu1/
├── HABA_main.c              - 메인 엔트리 포인트
├── HABA_setup.c/h           - 하드웨어 초기화
├── HABA_control.c/h         - 제어 알고리즘
├── HABA_globals.c/h         - 전역 변수/상수
├── HABA_cla_tasks.cla       - CLA PI 제어기
│
├── device/                  - TI Device Support
│   ├── device.c/h
│   └── driverlib/           - TI DriverLib
│
├── docs/                    - 프로젝트 문서
│   ├── PROJECT_INDEX.md     - 📚 전체 문서 인덱스
│   ├── QUICK_REFERENCE.md   - ⚡ 빠른 참조 가이드
│   ├── CODE_ANALYSIS_REPORT.md - 코드 분석 보고서
│   ├── POWER_CONTROL_REVIEW.md - 전력 제어 리뷰
│   └── RS232_interface_protocol_rev4.md - SCADA 프로토콜
│
├── CLAUDE.md                - AI 개발 가이드
├── README.md                - 이 파일
└── *.cmd                    - Linker Command Files
```

---

## 📚 문서

### 시작하기
- **[QUICK_REFERENCE.md](docs/QUICK_REFERENCE.md)** - 빠른 참조 및 디버깅 팁
- **[PROJECT_INDEX.md](docs/PROJECT_INDEX.md)** - 전체 문서 인덱스 및 API 레퍼런스
- **[CLAUDE.md](CLAUDE.md)** - 프로젝트 개발 가이드 (AI 어시스턴트용)

### 기술 문서
- **[CODE_ANALYSIS_REPORT.md](docs/CODE_ANALYSIS_REPORT.md)** - 코드 품질/보안/성능 분석
- **[POWER_CONTROL_REVIEW.md](docs/POWER_CONTROL_REVIEW.md)** - 전력 제어 전문가 리뷰
- **[RS232_interface_protocol_rev4.md](docs/RS232_interface_protocol_rev4.md)** - SCADA 통신 프로토콜

---

## 🔧 핵심 기능

### 실시간 제어 (100kHz ISR)
```c
INT_EPWM1_ISR (10μs 주기)
├─ Phase 0: 전압 센싱 + CLA PI 트리거   (~2μs)
├─ Phase 1: PI 결과 적용 + DAC 변환     (~3μs)
├─ Phase 2: RS485 전류 지령 전송        (~1.5μs)
├─ Phase 3: 안전 체크 + 릴레이 제어     (~1.5μs)
└─ Phase 4: 모니터링 + 시퀀스 실행      (~1μs)
```

### CLA 파이프라인 구조
- **레이턴시**: 10μs (vs 50μs without pipelining)
- **CPU-CLA 병렬 실행**: Phase 0에서 CLA Force, Phase 1에서 결과 사용
- **DCL 라이브러리**: TI 최적화 PI 제어기 (0.1~0.2μs 실행)

### 통신 프로토콜
- **CAN**: 500kbps, 최대 31개 슬레이브 제어
- **RS485**: 5.625Mbps, Master-to-Master/Master-to-Slave
- **SCADA**: 115200 baud, CRC-32 검증

---

## ⚙️ 설정 및 캘리브레이션

### 보호 임계값
```c
// HABA_globals.h
#define OVER_VOLTAGE  1400      // 과전압 (V)
#define OVER_CURRENT  88.0f     // 과전류 (A)
#define OVER_TEMP     120       // 과온도 (°C)
```

### PI 제어기 튜닝
```c
// HABA_setup.c: Init_CPU1_CLA1()
pi_charge.Kp = 1.0f;
pi_charge.Ki = 3000.0f * 50e-6f;  // 이산 시간 적분 게인
pi_charge.Umax = 80.0f;           // 출력 상한
```

### 전압 캘리브레이션
```c
// HABA_control.c: Sensing_And_Trigger_PI()
V_out = (V_out_uncal + 0.091694057f) * 0.9926000888f;
V_batt = (V_batt_uncal - 0.3058461657f) * 0.9945009708f;
```

**캘리브레이션 절차**:
1. 기준 전압계로 실제 전압 측정
2. 디버거에서 `V_out_uncal`, `V_batt_uncal` 읽기
3. 선형 회귀로 오프셋/게인 계산
4. 상수 업데이트 및 재빌드

---

## 🧪 테스트 및 디버깅

### 타이밍 측정
```c
// HABA_globals.h
#define ENABLE_TIMING_DEBUG  1  // 디버그 모드 활성화
```
- **GPIO90**: ISR 실행 타이밍 (100kHz)
- **GPIO91**: 1ms 태스크 타이밍
- **GPIO92**: 10ms 태스크 타이밍

### 디버그 플래그
```c
#define _DEBUG_CAN_STATUS_ENABLE_  1  // CAN 상태 모니터링
#define _DEBUG_SCI_STATUS_ENABLE_  1  // SCI 상태 모니터링
```

### 일반적인 문제 해결
| 문제 | 원인 | 해결 방법 |
|------|------|----------|
| ISR 타임아웃 | 실행 시간 초과 | GPIO90으로 타이밍 측정, 코드 최적화 |
| CAN 통신 끊김 | 터미네이션 불량 | 120Ω 저항 확인, 케이블 점검 |
| PI 제어 불안정 | 게인 과다 | Kp, Ki 값 감소 (절반으로 시작) |
| 전압 센서 오차 | 캘리브레이션 필요 | 상수 재측정 및 업데이트 |

---

## 📊 성능 지표

| 항목 | 측정값 | 상태 |
|------|--------|------|
| ISR 실행 시간 | ~1.8μs/phase | ✅ 우수 |
| CPU 사용률 | 18% @ 100kHz | ✅ 여유 82% |
| CLA 레이턴시 | 10μs | ✅ 목표 달성 |
| CAN 버스 부하 | 25% | ✅ 안정 |
| Flash 코드 크기 | ~50KB | ✅ 충분한 여유 |
| RAM 사용량 | ~8KB | ✅ 충분한 여유 |

---

## 🛡️ 안전 기능

### Fail-Safe 설계
- ✅ 비상 정지 스위치 감시 (GPIO11)
- ✅ 과전압/과전류/과온 감지 (래치 방식)
- ✅ Watchdog 타이머 (1ms 서비스)
- ✅ 다단계 전류 제한 (PI → 시스템 → 모드)
- ✅ CRC-32 패킷 검증 (SCADA)

### 시퀀스 제어
```
대기 (Precharge 준비)
    ↓
Precharge 진행 (V_out → V_batt ± 2V)
    ↓
Precharge 완료 (1초 대기)
    ↓
메인 릴레이 ON
    ↓
정상 운전
```

---

## 🔄 개발 워크플로우

### 코드 수정 시
1. **변수 추가**: `HABA_globals.h` (extern) + `HABA_globals.c` (정의)
2. **CLA 변수**: Message RAM 배치 (`Init_CPU1_CLA1()`)
3. **ISR 함수**: `#pragma CODE_SECTION(..., ".TI.ramfunc")`
4. **문서 업데이트**: API 레퍼런스, CLAUDE.md

### Git 커밋 전
- [ ] 빌드 성공 확인 (Flash + RAM)
- [ ] 디버그 플래그 비활성화
- [ ] `ENABLE_TIMING_DEBUG = 0`
- [ ] 문서 업데이트 (변경사항 반영)

### 릴리스 체크리스트
- [ ] 버전 번호 업데이트
- [ ] Flash 빌드 테스트
- [ ] 보호 기능 검증
- [ ] 통신 프로토콜 테스트
- [ ] 캘리브레이션 확인

---

## 📈 로드맵

### 단기 (1주)
- [ ] 배열 경계 검증 추가
- [ ] Magic Number 제거
- [ ] 함수 복잡도 리팩토링

### 중기 (1개월)
- [ ] HAL 계층 도입
- [ ] 상태 머신 기반 시퀀스
- [ ] 전역 변수 구조체 그룹화

### 장기 (3개월)
- [ ] 단위 테스트 프레임워크
- [ ] CPU2 확장 (통신 오프로드)
- [ ] MISRA-C 규칙 준수

---

## 🤝 기여 가이드

### 코딩 스타일
- **네이밍**: Pascal_Snake_Case (예: `Update_Voltage_Sensing`)
- **주석**: Doxygen 스타일 (`@brief`, `@param`, `@return`, `@note`)
- **들여쓰기**: 4 스페이스
- **줄바꿈**: 120자 제한

### Pull Request
1. Feature 브랜치 생성 (`feature/new-feature`)
2. 코드 작성 + 문서 업데이트
3. 빌드 및 테스트 통과 확인
4. PR 생성 (설명 상세히 작성)

---

## 📞 지원 및 문의

### 문서
- **전체 인덱스**: [docs/PROJECT_INDEX.md](docs/PROJECT_INDEX.md)
- **빠른 참조**: [docs/QUICK_REFERENCE.md](docs/QUICK_REFERENCE.md)
- **개발 가이드**: [CLAUDE.md](CLAUDE.md)

### 리소스
- **TI C2000Ware**: `/Applications/ti/C2000Ware_6_00_00_00/docs`
- **F28377D 데이터시트**: [TI 공식 사이트](https://www.ti.com/product/TMS320F28377D)
- **DCL 라이브러리**: C2000Ware/libraries/control/DCL/docs

---

## 📜 라이선스

Proprietary - All Rights Reserved

---

## 🙏 감사의 말

- **Texas Instruments**: C2000 플랫폼 및 개발 도구 제공
- **DCL 라이브러리**: 최적화된 제어 알고리즘
- **개발팀**: 지속적인 개선과 피드백

---

**프로젝트 버전**: 1.0
**마지막 업데이트**: 2025년 10월 19일
**유지보수**: 활발히 진행 중
