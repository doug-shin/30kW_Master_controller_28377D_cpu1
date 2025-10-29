# TODO List - 30kW Master Controller

> 마지막 업데이트: 2025-10-28

## ✅ 완료된 CRITICAL 항목

### ~~1. 구조 기반 아키텍처 마이그레이션 (Structure-Based Architecture)~~ ✅ 완료 (2025-10-27)
- **파일**:
  - `HABA_globals.h:113-117, 345-384, 547-548, 713-714`
  - `HABA_globals.c:50-75, 266-267`
  - `HABA_control.c:515-569, 813-967, 1441-1533`
  - `HABA_main.c:231`
- **완료 내용**:
  - **SequenceStep_t** enum 추가 (타입 안전성)
  - **SCADA_Command_t** 구조체: SCADA → Master 명령 (입력)
    - `cmd_ready` (Precharge 시작 명령)
    - `cmd_run` (운전 명령)
    - `control_mode`, `parallel_mode`
    - `V_max_cmd`, `V_min_cmd`, `I_cmd`, `V_cmd`, `I_max_cmd`, `I_min_cmd`
  - **Master_Status_t** 구조체: Master → SCADA 상태 (출력)
    - `ready`, `running`, `precharge_ok`
    - `sequence_step`, `fault_latched`
    - `over_voltage`, `over_current`, `over_temp`
    - `V_out`, `V_batt`, `I_out`
  - 마이그레이션된 함수 7개 (Parse_SCADA_Command, Update_SCADA_Watchdog, Update_System_Sequence, Check_Fault, Sensing_And_Trigger_PI, Apply_PI_And_Convert_DAC, INT_EPWM1_ISR)
  - 레거시 호환성 유지 (동기화 코드)
- **장점**:
  - 명확한 데이터 흐름 (입력 vs 출력 구분)
  - 타입 안전성 향상 (enum 사용)
  - 유지보수성 개선 (관련 변수 그룹화)
  - 확장성 향상 (새 명령/상태 추가 용이)
- **소요 시간**: 3시간

### ~~2. 고장 래칭 메커니즘 추가~~ ✅ 완료 (2025-10-27)
- **파일**: `HABA_control.c:515-569`, `HABA_globals.h:713-714`, `HABA_globals.c:266-267`
- **완료 내용**:
  - 고장 발생 시 `master_status.fault_latched = true`, `run = 0` 강제 설정
  - IDLE 상태 전환 시 자동 리셋 (프리차지 보존)
  - Fault 발생 → STOP 유지 (V_out ≈ V_batt) → IDLE 명령 → 자동 리셋
  - CMD bit[3] Reset 비트 제거 (프로토콜 변경 불필요)
- **동작 시퀀스**:
  1. 정상 운전 → Fault 발생 → run=0, master_status.fault_latched=true
  2. STOP 상태 유지 (프리차지 전압 보존)
  3. SCADA IDLE 명령 (scada_cmd.cmd_ready=0) → master_status.fault_latched=false (자동 리셋)
  4. IDLE → READY → RUN 재시작
- **장점**:
  - 프로토콜 수정 불필요 (CMD bit[3] 향후 확장용으로 보존)
  - 직관적 (IDLE = 초기화 개념)
  - 프리차지 보존 (빠른 재시작)
  - 간헐적 고장 방지 (안전성 확보)
  - 구조체 통합 (Master_Status_t)
- **소요 시간**: 40분

### ~~3. Master-to-Master 프로토콜 Rev 1.0 및 동적 전류 분배 구현~~ ✅ 완료 (2025-10-28)
- **파일**:
  - `HABA_globals.h:21, 496-546, 788-798`
  - `HABA_globals.c:383-410`
  - `HABA_control.c:294-317, 760-771, 938-956, 1095-1195, 1761-1801`
  - `HABA_control.h:57-58, 83`
  - `HABA_main.c:142-143, 199-203`
- **완료 내용**:
  - **Master-to-Master 프로토콜 Rev 1.0** (병렬 모드 전용)
    - CRC-32 검증 (VCU2 하드웨어 가속, ~0.5μs)
    - STX + ETX + 고정 바이트 수 프레임 동기화
    - 5ms 타임아웃 감시 (`Update_MM_Watchdog()`)
    - 스파이크 거부 (±30% 변화량 필터링)
    - 통신 통계 추적 (`MM_Statistics_t`)
  - **양방향 통신**
    - M1 → M2: 전류 지령 (20kHz, Phase 2)
    - M2 → M1: 슬레이브 개수 전송 (100ms, 병렬 모드)
    - `Send_RS485_MM_SlaveCount()` 함수 추가
  - **상태 머신 기반 수신 ISR**
    - `SCIA_RS485_MM_Rx_ISR()` 완전 재작성
    - `MM_RxState_t` enum (WAIT_STX, RECEIVING)
    - 프레임 타입 자동 감지 (전류 vs 슬레이브 개수)
  - **동적 슬레이브 개수 기반 전류 분배**
    - 개별 모드: `I_total / active_slave_list.count`
    - 병렬 모드: 가중 분배 (CH1/CH2 슬레이브 비율 반영)
    - 예시: CH1=6, CH2=4, 480A → 양쪽 모두 48A/모듈 (균등)
  - **Precharge 전류 수정**
    - 기존: 2A 시스템 전류 (모듈당 0.33A)
    - 수정: 2A/모듈 × 슬레이브 개수
    - 개별 모드 6모듈: 12A, 병렬 모드 12모듈: 24A
- **해결한 문제**:
  - 병렬 모드 채널간 전류 불균형 (M2 슬레이브 개수 미전달)
  - Master-to-Master 통신 신뢰성 (CRC 오류, 타임아웃, 프레임 동기화)
  - Precharge 전류 부족 (2A → 2A/모듈)
- **소요 시간**: 4시간
- **버전**: v2.3.0

### ~~4. 과온도 임계값 긴급 수정~~ ✅ 완료 (2025-10-20)
- **파일**: `HABA_globals.h:109`
- **완료 내용**:
  - OVER_TEMP: 120°C → **85°C** (강제 공랭 팬×3 고려)
  - OVER_TEMP_WARNING: **75°C** 추가 (Derating 단계)
  - 하드웨어 손상 위험 제거 (Tj ≈ 720°C → 180°C)
- **소요 시간**: 5분
- **버전**: v2.1.3

---

## 🔴 CRITICAL Priority

---

## 🔴 HIGH Priority

### 1. PI 게인 재튜닝 검증 (DCL 전환 후) ⚠️ **하드웨어 테스트 필수**
- **파일**: `HABA_setup.c:350`, `HABA_cla_tasks.cla`
- **설명**:
  - 첫 커밋: 수동 PI 구현 → 현재: DCL_runPI_L2() (parallel form)
  - 미세한 동작 차이 가능 (반올림, 포화 처리)
  - 제어 안정성 검증 필요
- **검증 사항**:
  - [ ] 스텝 응답 테스트 (V_max_cmd 급변 100V → 300V)
  - [ ] 오버슈트 < 5%
  - [ ] 정착 시간 < 100ms
  - [ ] 보드 선도 측정 (Phase Margin ≥ 45°, Gain Margin ≥ 6dB)
- **현재 게인**: Kp=1.0, Ki=0.15
- **출처**: 첫 커밋 비교 분석, IMPROVEMENT_CHECKLIST.md
- **예상 시간**: 2시간
- **담당**: dougshin
- **마감**: 2025-11-05 (연장)
- **우선순위**: ⚠️ 높음

### 2. NTC 온도 센싱 구현 ⚠️ **하드웨어 테스트 필수**
- **파일**:
  - `HABA_control.c:396` (Phase 3)
  - `HABA_control.c:431` (TODO 주석)
  - `HABA_setup.c` (Init_ADCA 수정 필요)
- **설명**:
  - ADC 기반 NTC 온도 측정 및 과온도 보호 활성화
  - LUT_Resistance 테이블 이미 준비됨 (-40°C ~ +120°C, 161 포인트)
  - 현재 NTC_0_temp, NTC_1_temp 항상 0 (비활성화)
- **구현 내용**:
  1. ADCA SOC 채널 할당 (NTC0, NTC1)
  2. ADC → 저항 변환 함수
  3. LUT 기반 온도 변환 (선형 보간)
  4. Phase 3에서 온도 업데이트
  5. 과온도 보호 로직 활성화 (OVER_TEMP: 85°C)
- **개발 이력 참고**: `[07.11] NTC 온도 측정을 위한 ADCA 채널 추가 적용` (미완성)
- **예상 시간**: 4시간
- **담당**: dougshin
- **마감**: 2025-11-15

### 3. Battery Mode CV 제어 완성 및 튜닝 ⚠️ **하드웨어 테스트 필수**
- **파일**:
  - `HABA_control.c:1252` (TODO 주석)
  - `HABA_cla_tasks.cla` (CLA Task 3)
- **설명**:
  - CLA Task 3 기반 CV 제어 로직 검증 및 최적화
  - 기본 구현 완료, PI 튜닝 및 테스트 필요
- **구현 내용**:
  1. V_cmd, I_max_cmd, I_min_cmd 제어 로직 검증
  2. PI 파라미터 튜닝 (Kp, Ki)
  3. 전류 제한 로직 검증
  4. 실제 배터리 충방전 테스트
- **상태**: 기본 구현 완료 (2025-09-01), 튜닝 필요
- **예상 시간**: 2시간
- **담당**: dougshin
- **마감**: 2025-11-10

### 4. 마스터 고장 정보 SCADA 피드백 추가 ✅ **소프트웨어만으로 해결 가능**
- **파일**: `HABA_control.c` (Send_Slave_Batch_To_SCADA), `HABA_globals.h`
- **설명**:
  - **Rev 5.0 확인 필요**: Slave_Batch_Packet에 Master_Fault 비트 이미 존재 가능성
  - 현재 `master_status.fault_latched`, `over_voltage`, `over_current`, `over_temp` 구조체 있음
  - SCADA 패킷에 Master_Fault 비트 추가만 하면 됨
- **구현 내용**:
  1. Rev 5.0 프로토콜 명세 확인 (Master_Fault 필드)
  2. Send_Slave_Batch_To_SCADA()에서 Master_Fault 비트 설정
  3. SCADA 수신 테스트
- **예상 시간**: 30분
- **담당**: dougshin
- **마감**: 2025-11-05
- **우선순위**: 높음

---

## 🟡 MEDIUM Priority

### 5. RS485 스킵 카운터 모니터링 ✅ **소프트웨어만으로 해결 가능**
- **파일**: `HABA_control.c:344`, `HABA_main.c` (10ms 작업), `HABA_globals.h`
- **설명**:
  - 현재: rs485_mm_skip_cnt, rs485_ms_skip_cnt 카운터만 증가
  - 통신 품질 감시 로직 필요
- **구현 내용**:
  ```c
  #define RS485_SKIP_THRESHOLD  100  // 1초당 100회 = 10%

  // 10ms 또는 1초 주기
  if (rs485_mm_skip_cnt > RS485_SKIP_THRESHOLD) {
      comm_quality_warning = true;  // SCADA 보고 또는 LED 표시
  }

  // 주기적 리셋
  if (flag_1s) {
      rs485_mm_skip_cnt = 0;
      rs485_ms_skip_cnt = 0;
  }
  ```
- **출처**: IMPROVEMENT_CHECKLIST.md
- **예상 시간**: 30분
- **담당**: dougshin
- **마감**: 2025-11-05
- **우선순위**: 중간

### 6. 병렬 모드 채널간 부하 모니터링 ✅ **소프트웨어만으로 해결 가능**
- **파일**: `HABA_control.c` (Phase 4), `HABA_main.c` (10ms 작업)
- **설명**:
  - **구현 완료**: 동적 가중 분배로 슬레이브 개수 불일치 해결 (v2.3.0)
  - **추가 필요**: 실시간 모니터링 및 경고 메시지
    - M2→M1 슬레이브 개수 변동 감지 (고장/복구)
    - 부하 불균형 경고 (5% 초과 시)
    - SCADA 피드백 (채널별 슬레이브 개수 송신)
- **구현 내용**:
  ```c
  // 슬레이브 개수 변동 감지
  static uint8_t ch2_prev_count = 0;
  if (ch2_slave_count != ch2_prev_count) {
      log_slave_count_change(ch2_prev_count, ch2_slave_count);
      ch2_prev_count = ch2_slave_count;
  }

  // 부하 불균형 모니터링
  float32_t ch1_avg = I_total_ch1 / active_slave_list.count;
  float32_t ch2_avg = I_total_ch2 / ch2_slave_count;
  float32_t imbalance = fabsf(ch1_avg - ch2_avg) / ch1_avg;
  if (imbalance > 0.05f)
      warn_load_imbalance = true;
  ```
- **출처**: v2.3.0 개선, POWER_CONTROL_REVIEW.md
- **예상 시간**: 1시간
- **담당**: dougshin
- **마감**: 2025-11-10
- **우선순위**: 중간

### 7. 개별 모드 Master-to-Master 제어 구현 ⚠️ **요구사항 명확화 필요**
- **파일**: `HABA_control.c`, `HABA_main.c`
- **설명**:
  - 현재: 병렬 모드에서만 M1→M2 통신 (전류 지령)
  - 개별 모드 확장: M1을 통해 M2 제어 (통합 HMI)
- **구현 내용**:
  - 개별 모드에서 SCIA M1→M2 통신 활성화
  - M2가 M1 지령 수신 후 독립 PI 제어 실행
  - 제어 메커니즘 구체화 필요 (추후 설계)
- **예상 시간**: 미정
- **담당**: 미정
- **마감**: 미정
- **우선순위**: 중간


---

## 🟢 LOW Priority

### 8. Magic Number 제거 (코드 품질) ✅ **소프트웨어만으로 해결 가능**
- **파일**: `HABA_main.c:219` 및 기타
- **설명**:
  - v2.2.0에서 일부 해결 (SequenceStep_t enum)
  - 나머지 숫자 리터럴 상수화 필요
- **구현 내용**:
  1. 전체 파일 검색 (`grep -n "== [0-9]"`)
  2. 상수로 대체
  3. 가독성 검증
- **출처**: CODE_ANALYSIS_REPORT.md
- **예상 시간**: 15분
- **담당**: 미정
- **마감**: 미정
- **우선순위**: 낮음 - 코드 품질

### 9. 복잡한 함수 리팩토링 ✅ **소프트웨어만으로 해결 가능**
- **파일**: `HABA_control.c:173` (Apply_PI_And_Convert_DAC)
- **설명**:
  - 중첩 if-else 구조 (최대 3단계), 150라인
  - 사이클로매틱 복잡도 높음
  - **주의**: 현재 안정적 동작 중, 리팩토링 후 충분한 테스트 필요
- **구현 내용**:
  1. 헬퍼 함수 분리 (Calculate_Battery_Mode_Current, Calculate_ChargeDischarge_Mode_Current)
  2. 전략 패턴 적용
  3. 가독성 향상
  4. 하드웨어 검증
- **출처**: CODE_ANALYSIS_REPORT.md
- **예상 시간**: 2시간 (코딩) + 1시간 (테스트)
- **담당**: 미정
- **마감**: 미정
- **우선순위**: 낮음 - 유지보수성

### 10. 전역 변수 구조체 그룹화 ✅ **소프트웨어만으로 해결 가능**
- **파일**: `HABA_globals.h` (685라인, 113개 변수)
- **설명**:
  - v2.2.0에서 시작 (SCADA_Command_t, Master_Status_t)
  - 나머지 전역 변수 구조체로 그룹화
- **구현 내용**:
  ```c
  typedef struct {
      float32_t V_out;
      float32_t V_batt;
      float32_t I_cmd;
      // ...
  } SystemState_t;

  extern SystemState_t g_system_state;
  ```
- **출처**: CODE_ANALYSIS_REPORT.md
- **예상 시간**: 4시간 (리팩토링 + 검증)
- **담당**: 미정
- **마감**: 미정
- **우선순위**: 낮음 - 장기 유지보수

---

## ✅ Completed

### 2025-10-27 (Latest)
- **구조 기반 아키텍처 마이그레이션 (Structure-Based Architecture)**
  - SequenceStep_t enum, SCADA_Command_t, Master_Status_t 구조체 추가
  - 7개 함수 마이그레이션 완료 (레거시 호환성 유지)
  - 명확한 데이터 흐름 (입력 vs 출력 분리)
  - 파일: `HABA_globals.h/c`, `HABA_control.c`, `HABA_main.c`

- **고장 래칭 메커니즘 추가**
  - `master_status.fault_latched` 변수 추가
  - 고장 시 즉시 정지, IDLE 상태에서 자동 리셋
  - STOP 상태 유지로 프리차지 보존
  - 파일: `HABA_control.c:515-569`

- **문서 정리**
  - `IMPROVEMENT_CHECKLIST.md` 삭제 (TODO.md와 중복)
  - `SAFETY_AND_SEQUENCE_ANALYSIS.md` 삭제 (모든 이슈 해결)
  - `REV5_IMPLEMENTATION_PLAN.md` 삭제 (구현 완료)
  - 최종 문서 9개로 정리

### 2025-10-20
- **첫 커밋 대비 오작동 가능성 심층 분석**
  - 첫 커밋(ec95937) vs 현재(b7dfcca) 비교 분석 완료
  - 종합 점수: 6.4/10 → 9.0/10 (+2.6점 향상)
  - 주요 개선: PI 제어기 DCL 전환, 적분기 초기화, TX FIFO 블로킹 방지
  - 신규 발견: CH2 수신 타임아웃 체크 부재, PI 게인 재튜닝 필요성

### 2025-10-20
- **RS485 TX FIFO 블로킹 방지 로직 추가**
  - `HABA_control.c:344-384` (Transmit_Current_Command)
  - TX FIFO 상태 확인 후 송신 (SCI_FIFO_TX4 임계값)
  - 스킵 카운터 모니터링 (rs485_mm_skip_cnt, rs485_ms_skip_cnt)
  - ISR 실행 시간 안정화 (~1.5μs 유지)

- **타이밍 디버그 모드 활성화**
  - `HABA_globals.h:32` (ENABLE_TIMING_DEBUG = 1)
  - GPIO90/91/92 타이밍 측정 가능
  - ISR/1ms/10ms 실행 시간 실측 검증용

- **모니터링 변수 추가**
  - `HABA_globals.h:562-564`, `HABA_globals.c:212-214`
  - rs485_mm_skip_cnt, rs485_ms_skip_cnt
  - 통신 품질 모니터링 및 하드웨어 장애 감지

### 2025-09-01
- **DCL PI 제어기 전환 (L1 → L2)**
  - Ideal form → Parallel form
  - CLA Task 1, 2, 3 DCL_runPI_L2() 사용

- **Battery 모드 CV 제어 추가**
  - CLA Task 3 구현
  - V_cmd, I_max_cmd, I_min_cmd 제어 로직

### 2025-09-01 이전
- RS232 프로토콜 Rev 2.1 적용
- 병렬 모드 전류 분배 로직 개선
- 5-Phase 파이프라인 구조 (100kHz ISR, 20kHz effective)
- CLA 기반 PI 제어기 (0.1~0.2μs 실행)

---

## 📝 작업 우선순위 가이드

### ✅ 소프트웨어만으로 해결 가능 (하드웨어 테스트 불필요)
**즉시 실행 가능 항목 (Week 1)**:
1. HIGH #4: 마스터 고장 정보 SCADA 피드백 (30분)
2. MEDIUM #5: RS485 스킵 카운터 모니터링 (30분)
3. MEDIUM #6: 병렬 모드 부하 모니터링 (1시간)
4. LOW #8: Magic Number 제거 (15분)

**총 예상 시간**: 2시간 15분

**장기 품질 개선 (Week 2-3)**:
1. LOW #9: 복잡한 함수 리팩토링 (3시간 - 코딩+테스트)
2. LOW #10: 전역 변수 구조체 그룹화 (4시간)

**총 예상 시간**: 7시간

### ⚠️ 하드웨어 테스트 필수 항목
**제어 검증 (하드웨어 준비 후)**:
1. HIGH #1: PI 게인 재튜닝 검증 (2시간)
2. HIGH #2: NTC 온도 센싱 구현 (4시간)
3. HIGH #3: Battery Mode CV 제어 튜닝 (2시간)

**총 예상 시간**: 8시간

### ⚠️ 요구사항 명확화 필요
1. MEDIUM #7: 개별 모드 Master-to-Master 제어 (사용 시나리오 확인 필요)

### ❌ 삭제된 항목 (불필요 또는 구현 불가)
1. **과전류 임계값 마진 확보** (88A → 96A)
   - 실제 운전 전류 프로파일 확인 후 재검토
   - 현재 88A로 충분할 가능성
2. **슬레이브 과전력(OP) 보호**
   - 슬레이브 전압 데이터 없음 (전류+온도만)
   - 슬레이브 펌웨어 수정 선행 필요
3. **RS485 DMA 전환**
   - v2.1.1에서 TX FIFO 블로킹 방지 완료
   - ISR 여유 82%, 성능 병목 없음
4. **프리차징 릴레이 복구**
   - 현재 시스템 정상 동작 중
   - 하드웨어 구성 확인 필요 (릴레이 존재 여부)
5. **CAN 슬레이브 16~31 지원**
   - Rev 5.0 Active Slave List 최대 6개
   - 확장 필요 없음
6. **Phase 분리 재검토 (6-Phase)**
   - 현재 5-Phase 안정 동작 (82% 여유)
   - PI 재튜닝 위험 대비 효과 낮음

---

## 🔗 관련 문서

- `CLAUDE.md`: 프로젝트 개요 및 아키텍처
- `docs/QUICK_REFERENCE.md`: 빠른 참조
- `docs/PROJECT_INDEX.md`: API 레퍼런스
- `docs/POWER_CONTROL_REVIEW.md`: PI 튜닝 가이드
- `docs/CODE_ANALYSIS_REPORT.md`: 코드 품질/보안 분석
- `docs/RS232_SCADA_protocol_rev5.md`: SCADA 프로토콜 최신 버전
- `docs/RS232_SCADA_protocol_rev4.md`: SCADA 프로토콜 이전 버전
- `CHANGELOG.md`: 버전 이력

---

**작업 추가 방법**:
```markdown
## [Priority]

### N. 작업 제목
- **파일**: 관련 파일 목록
- **설명**: 작업 설명
- **구현 내용**: 구체적인 작업 항목
- **예상 시간**: 시간
- **담당**: 담당자
- **마감**: 날짜
```
