# Changelog - 30kW Master Controller

> All notable changes to this project will be documented in this file.

Format based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/)

---

## [Unreleased]

### TODO - High Priority
- PI 게인 재튜닝 검증 (DCL 전환 후 안정성 확인)
- NTC 온도 센싱 구현 (ADC 기반)
- 과전류 임계값 마진 확보 (88A → 96A)
- Battery Mode CV 제어 튜닝
- 마스터 고장 정보 SCADA 피드백 추가
- 슬레이브 과전력(OP) 보호 구현

### TODO - Medium Priority
- RS485 스킵 카운터 모니터링
- 병렬 모드 채널간 전류 불균형 모니터링
- RS485 DMA 전환 (성능 최적화)
- 프리차징 릴레이 복구
- 개별 모드 Master-to-Master 제어 구현 (M1→M2 통신 확장)

### TODO - Low Priority
- Magic Number 제거 (코드 품질)
- 복잡한 함수 리팩토링 (유지보수성)
- CAN 슬레이브 16~31 지원

---

## [2.3.1] - 2025-10-29

### Changed
- **OperationMode_t enum 리팩토링 (코드 명확성 개선)**
  - MODE_STOP, MODE_RESERVED 제거 (미사용 상태 정리)
  - MODE_INDIVIDUAL = 0 (기본값), MODE_PARALLEL = 1
  - Run/Stop 제어와 완전 분리 명확화 (scada_cmd.cmd_run, run 변수로 독립 제어)

- **기본 운전 모드 변경**
  - 초기값: MODE_STOP → MODE_INDIVIDUAL
  - 논리적 기본값 개선 (개별 운전이 안전한 기본 상태)

- **SCADA 타임아웃 처리 개선**
  - operation_mode 강제 변경 제거 (채널 구성은 명시적 명령으로만 변경)
  - Run/Stop 제어만 초기화 (scada_cmd.cmd_run = 0)

### Documentation
- **3가지 독립 제어 계층 명확화**
  - **Operation Mode**: 채널 구성 (개별/병렬) - 하드웨어 토폴로지
  - **Control Mode**: 제어 알고리즘 (충방전/배터리) - 제어 전략
  - **Run Control**: 운전/정지 - 시스템 활성화
- CLAUDE.md, PROJECT_INDEX.md, QUICK_REFERENCE.md 업데이트

### Technical Details
- `HABA_globals.h:322-327`: enum 정의 단순화 (4개 → 2개 상태)
- `HABA_globals.c:81`: 초기값 MODE_INDIVIDUAL로 변경
- `HABA_control.c:1770`: SCADA 타임아웃 시 operation_mode 유지
- 하위 호환성 유지 (SCADA 프로토콜 변경 없음)

---

## [2.3.0] - 2025-10-28

### Added
- **Master-to-Master 프로토콜 Rev 1.0 (병렬 모드 전용)**
  - CRC-32 검증 (VCU2 하드웨어 가속, ~0.5μs)
  - STX + ETX + 고정 바이트 수 프레임 동기화
  - 5ms 타임아웃 감시 (`Update_MM_Watchdog()`)
  - 스파이크 거부 (±30% 변화량 필터링)
  - 통신 통계 추적 (`MM_Statistics_t`)

- **양방향 Master-to-Master 통신**
  - M1 → M2: 전류 지령 (20kHz, Phase 2)
  - M2 → M1: 슬레이브 개수 전송 (100ms, 병렬 모드)
  - `Send_RS485_MM_SlaveCount()` 함수 추가

- **상태 머신 기반 수신 ISR**
  - `SCIA_RS485_MM_Rx_ISR()` 완전 재작성
  - `MM_RxState_t` enum (WAIT_STX, RECEIVING)
  - 프레임 타입 자동 감지 (전류 vs 슬레이브 개수)

### Changed
- **동적 슬레이브 개수 기반 전류 분배**
  - 개별 모드: `I_total / active_slave_list.count`
  - 병렬 모드: 가중 분배 (CH1/CH2 슬레이브 비율 반영)
  - 예시: CH1=6, CH2=4, 480A → 양쪽 모두 48A/모듈 (균등)

- **Precharge 전류 수정**
  - 기존: 2A 시스템 전류 (모듈당 0.33A)
  - 수정: 2A/모듈 × 슬레이브 개수
  - 개별 모드 6모듈: 12A, 병렬 모드 12모듈: 24A

### Fixed
- **병렬 모드 채널간 전류 불균형 해결**
  - M2 슬레이브 개수 미전달로 인한 고정 분배 문제 해결
  - 동적 가중 분배로 실시간 균등 부하 분배 달성

- **Master-to-Master 통신 신뢰성 강화**
  - CRC 오류 자동 거부 (통계 카운터 증가)
  - 타임아웃 시 안전 정지 (I_cmd = 0, IDLE 전환)
  - 프레임 동기화 오류 복구 (STX 대기 상태로 리셋)

---

## [2.2.0] - 2025-10-27

### Added
- **구조 기반 아키텍처 (Structure-Based Architecture)**
  - `SequenceStep_t` enum 추가 (타입 안전성 향상)
  - `SCADA_Command_t` 구조체: SCADA → Master 명령 입력
    - `cmd_ready`, `cmd_run`, `control_mode`, `parallel_mode`
    - `V_max_cmd`, `V_min_cmd`, `I_cmd`, `V_cmd`, `I_max_cmd`, `I_min_cmd`
  - `Master_Status_t` 구조체: Master → SCADA 상태 출력
    - `ready`, `running`, `precharge_ok`, `sequence_step`
    - `fault_latched`, `over_voltage`, `over_current`, `over_temp`
    - `V_out`, `V_batt`, `I_out`
  - 레거시 변수 동기화 코드 유지 (하위 호환성)

- **고장 래칭 메커니즘**
  - `master_status.fault_latched` 플래그 추가
  - 고장 발생 시 `run = 0` 강제 설정 (즉시 정지)
  - IDLE 상태 전환 시 자동 리셋 (프리차지 전압 보존)
  - STOP 상태 유지로 빠른 재시작 지원

### Changed
- **7개 함수 구조체 마이그레이션**
  - `Parse_SCADA_Command()`: `scada_cmd` 구조체 사용
  - `Update_SCADA_Watchdog()`: `scada_cmd` 초기화
  - `Update_System_Sequence()`: `master_status` 상태 관리
  - `Check_Fault()`: `master_status` 고장 플래그
  - `Sensing_And_Trigger_PI()`: `scada_cmd.control_mode` 사용
  - `Apply_PI_And_Convert_DAC()`: `scada_cmd` 파라미터 사용
  - `INT_EPWM1_ISR()`: `master_status.sequence_step` 사용

### Improved
- **데이터 흐름 명확화**
  - Input (scada_cmd) vs Output (master_status) 명확한 분리
  - `ready_state` 역할 개선: `cmd_ready` (명령) vs `ready` (상태)
  - 타입 안전성: enum 사용으로 오타 방지
  - 유지보수성: 관련 변수 그룹화, 코드 가독성 향상

### Removed
- **임시 문서 삭제 (문서 정리)**
  - `docs/IMPROVEMENT_CHECKLIST.md` (TODO.md와 중복)
  - `docs/SAFETY_AND_SEQUENCE_ANALYSIS.md` (모든 이슈 해결 완료)
  - `docs/REV5_IMPLEMENTATION_PLAN.md` (구현 완료)
  - 최종 문서 9개로 정리

---

## [2.1.3] - 2025-10-20

### Fixed
- **과온도 임계값 긴급 수정** (120°C → 85°C)
  - 하드웨어 손상 위험 제거 (Tj ≈ 720°C → 180°C)
  - 강제 공랭 시스템(팬×3) 고려한 안전한 설정
  - OVER_TEMP_WARNING (75°C) 추가로 Derating 단계 구현

---

## [2.1.2] - 2025-10-20

### Added
- **코드 품질 분석 및 개선사항 문서화**
  - 첫 커밋(ec95937) vs 현재(b7dfcca) 심층 비교 분석 완료
  - `docs/IMPROVEMENT_CHECKLIST.md` 생성 (우선순위별 개선사항 정리)
  - 종합 평가: 6.4/10 → 9.0/10 (+2.6점 향상)

### Changed
- **TODO.md 대폭 개선**
  - CRITICAL 우선순위 섹션 신규 추가 (과온도 임계값, 고장 래칭)
  - HIGH 우선순위 항목 추가 (PI 재튜닝, CH2 타임아웃, 과전류 마진)
  - MEDIUM/LOW 우선순위 재구성
  - 작업 우선순위 가이드 추가 (Week 1/2/3 계획)
  - 총 16개 항목으로 확대 (기존 6개 → 16개)

### Fixed
- **신규 발견 안전 이슈 문서화**
  - 과온도 임계값 120°C 위험성 발견 (Tj ≈ 720°C!)
  - CH2 수신 타임아웃 체크 부재 발견 (병렬 모드 통신 끊김 대비)
  - PI 제어기 DCL 전환 후 재튜닝 필요성 확인

---

## [2.1.1] - 2025-10-20

### Added
- **RS485 TX FIFO 블로킹 방지 로직**
  - `Transmit_Current_Command()` 함수에 TX FIFO 상태 확인 추가
  - FIFO 여유 없을 시 송신 스킵 (블로킹 방지)
  - 모니터링 변수 추가: `rs485_mm_skip_cnt`, `rs485_ms_skip_cnt`
  - ISR 실행 시간 안정화 (~1.5μs 유지, 블로킹 제거)

- **타이밍 디버그 모드 활성화**
  - `ENABLE_TIMING_DEBUG = 1` 설정
  - GPIO90: ISR 전체 실행 시간 측정
  - GPIO91: 1ms 작업 실행 시간 측정
  - GPIO92: 10ms 작업 실행 시간 측정
  - 오실로스코프 실측 검증용

### Changed
- 빈 디버그 매크로를 `((void)0)`로 변경 (코드 품질 향상)

### Fixed
- Phase 2 RS485 송신 시 TX FIFO 블로킹 가능성 제거

---

## [2.1.0] - 2025-09-01

### Added
- **Battery 모드 CV 제어**
  - CLA Task 3 구현 (`pi_cv` 제어기)
  - V_cmd, I_max_cmd, I_min_cmd 제어 변수
  - Apply_PI_And_Convert_DAC()에 Battery 모드 분기 추가
  - Update_System_Sequence()에 Battery 모드 데이터 처리

### Changed
- **DCL PI 제어기 전환 (L1 → L2)**
  - Ideal form (L1) → Parallel form (L2)
  - `DCL_runPI_L1()` → `DCL_runPI_L2()`
  - pi_charge, pi_discharge, pi_cv 모두 L2 사용
  - 기존 parallel form 제어 특성 유지

- **문서 구조 개선**
  - README.md 업데이트
  - CLAUDE.md 프로젝트 개요 추가
  - docs/ 디렉토리 정리

### Fixed
- PI 제어기 적분기 상한 동적 업데이트 (I_cmd_filtered 기반)

---

## [2.0.0] - 2025-08-27

### Added
- **RS232 프로토콜 Rev 2.1 적용**
  - 13바이트 패킷 (STX/CMD/Params/CRC32/ETX)
  - CRC-32 검증 (VCU2 하드웨어 가속)
  - Control Mode 비트 추가 (Charge/Discharge vs Battery)
  - IDLE+RUN 비정상 조합 검증

### Changed
- **운전 모드별 전류 분배 로직 개선**
  - 개별 모드: I_total / 6 (채널당 6개 슬레이브)
  - 병렬 모드: I_total / 2 / 6 (총 12개 슬레이브)
  - Master1이 Master2 제어 (병렬 모드)

- **SCADA 패킷 파싱 최적화**
  - ISR: 수신 + STX/ETX 검증만 (~2μs)
  - Main Loop: CRC 검증 + 변수 업데이트 (~5μs)
  - ISR 실행 시간 단축 (10μs → 2μs)

### Removed
- Rev 2.0 파싱 함수 제거 (Rev 2.1로 완전 전환)

---

## [1.5.0] - 2025-09-30

### Added
- **시퀀스 제어 모듈 개선**
  - SEQ_STEP_IDLE, SEQ_STEP_PRECHARGE_DONE, SEQ_STEP_NORMAL_RUN
  - V_out ≈ V_batt (±2V) 조건 자동 감지
  - 1초 대기 후 메인 릴레이 ON

### Changed
- Module_Sequence 동작 신호 변경 (run → Start_Stop)
- PI 제어기 피드백 전압 선택 로직 (V_out vs V_batt)

---

## [1.4.0] - 2025-09-23

### Added
- **전면 패널 LED 구현**
  - LED_PWR (GPIO46): 전원
  - LED_CHARGE (GPIO47): 충전
  - LED_DISCHARGE (GPIO42): 방전
  - LED_FAULT (GPIO43): 고장
  - LED_SINGLE (GPIO67): 개별운전
  - LED_DUAL (GPIO68): 병렬운전

### Changed
- UI 데이터 전송 타이밍 변경 (10μs → 10ms/50ms)

### Fixed
- SCADA 통신 타이밍 이슈 해결

---

## [1.3.0] - 2025-09-18

### Changed
- **릴레이 매핑 수정**
  - GPIO8: 메인 릴레이 (개별/병렬 공통)
  - GPIO9: 병렬 연결 릴레이 (병렬 모드 전용)

### Removed
- 프리차징 릴레이 코드 주석처리 (추후 복구 예정)

---

## [1.2.0] - 2025-09-15

### Added
- HMI 프로토콜 Rev 2.0 통합 완료

### Fixed
- average_task 함수 200번 평균 if 조건문 ++ 위치 버그 수정

---

## [1.1.0] - 2025-09-12

### Added
- PI 제어기 피드백 전압 변경 로직
- sensing_task 함수 Vo_sen, Vbat_sen 캘리브레이션

---

## [1.0.0] - 2025-09-01

### Added
- **5-Phase 파이프라인 제어 구조**
  - 100kHz ISR, 20kHz effective (5-phase rotation)
  - Phase 0: 센싱 + CLA Force
  - Phase 1: PI 결과 적용 + DAC 변환
  - Phase 2: RS485 전류 지령 전송
  - Phase 3: 안전 체크 + 릴레이 제어
  - Phase 4: 모니터링 + 시퀀스 실행

- **CLA 기반 PI 제어기**
  - CLA Task 1: pi_charge (충전 모드)
  - CLA Task 2: pi_discharge (방전 모드)
  - CPU-CLA 병렬 실행 (레이턴시 10μs)

- **통신 프로토콜**
  - CAN (500kbps): 슬레이브 제어
  - RS485 (5.625Mbps): 전류 지령 전송
  - SCADA (115200 baud): 모니터링 및 제어

- **운전 모드**
  - MODE_INDIVIDUAL: 개별 운전 (CH1, CH2 독립)
  - MODE_PARALLEL: 병렬 운전 (CH1이 CH2 제어)

### Changed
- CAN Slave 프레임 형식 (12bit → 16bit)

---

## [0.9.0] - 2025-08-06

### Changed
- EPWM1 ↔ EPWM3 용도 변경
  - EPWM1: 메인 제어 루프 (100kHz)
  - EPWM3: ADC 트리거

---

## [0.8.0] - 2025-07-15

### Added
- EPWM3_ISR 함수 구현 (80% 완료)
  - 함수 포인터 배열 기반 20kHz 분주 제어
  - Phase 0~4 분리 실행

---

## [0.7.0] - 2025-07-12

### Added
- Digital Input/Output (DIO) 구성 및 테스트 완료

---

## [0.6.0] - 2025-07-11

### Added
- NTC 온도 측정용 ADCA 채널 추가 (미완성)
- 전체 코드 분류 (소스/헤더 분리)

---

## [0.5.0] - 2025-07-10

### Added
- DAC80502 SPI 통신 완료
- DAC 출력 확인

---

## [0.4.0] - 2025-07-09

### Added
- CLA Task 예제 추가 및 정상 동작 확인
- CLA 기반 PI 제어기 완성

---

## [0.3.0] - 2025-07-08

### Added
- SCI (RS485) 통신 완료 및 통합 테스트

### Changed
- SysConfig 제거 후 수동 GPIO 설정 구조 전환

---

## [0.2.0] - 2025-07-07

### Added
- SLAVE용 CAN 송수신 루틴 구현 및 완료

---

## [0.1.0] - 2025-07-04

### Added
- 드라이버 레벨 헤더 수정 및 재구성

---

## [0.0.1] - 2025-07-03

### Added
- 프로젝트 초기 설정
- EPWM3 인터럽트 적용 테스트 완료

---

## Changelog 작성 가이드

### 카테고리
- **Added**: 새로운 기능 추가
- **Changed**: 기존 기능 변경
- **Deprecated**: 곧 제거될 기능
- **Removed**: 제거된 기능
- **Fixed**: 버그 수정
- **Security**: 보안 관련 수정

### 버전 번호 규칙 (Semantic Versioning)
```
MAJOR.MINOR.PATCH

MAJOR: 호환되지 않는 API 변경
MINOR: 하위 호환되는 기능 추가
PATCH: 하위 호환되는 버그 수정
```

### 작성 예시
```markdown
## [1.2.3] - 2025-10-20

### Added
- 새로운 기능 설명

### Fixed
- 버그 수정 설명
```
