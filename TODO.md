# TODO List - 30kW Master Controller

> 마지막 업데이트: 2025-10-20

## ✅ 완료된 CRITICAL 항목

### ~~1. 과온도 임계값 긴급 수정~~ ✅ 완료 (2025-10-20)
- **파일**: `HABA_globals.h:109`
- **완료 내용**:
  - OVER_TEMP: 120°C → **85°C** (강제 공랭 팬×3 고려)
  - OVER_TEMP_WARNING: **75°C** 추가 (Derating 단계)
  - 하드웨어 손상 위험 제거 (Tj ≈ 720°C → 180°C)
- **소요 시간**: 5분
- **버전**: v2.1.3

---

## 🔴 CRITICAL Priority

### 1. 고장 래칭 메커니즘 추가
- **파일**: `HABA_control.c:508-541`, `HABA_globals.h`
- **설명**:
  - 현재: 고장 조건 해제 시 즉시 복구 → 간헐적 고장 시 불안정
  - 필요: 고장 발생 시 래칭, SCADA 명령으로만 수동 리셋
- **구현 내용**:
  ```c
  // HABA_globals.h
  extern volatile bool fault_latched;

  // HABA_control.c
  static bool fault_latched = false;
  if (is_fault_active && !fault_latched) {
      fault_latched = true;
      run = 0;
  }
  // SCADA 리셋 명령 처리
  ```
- **출처**: POWER_CONTROL_REVIEW.md, IMPROVEMENT_CHECKLIST.md
- **예상 시간**: 30분
- **담당**: dougshin
- **마감**: 2025-10-22
- **우선순위**: ⚠️⚠️ 높음

---

## 🔴 HIGH Priority

### 3. PI 게인 재튜닝 검증 (DCL 전환 후)
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
- **마감**: 2025-10-25
- **우선순위**: ⚠️ 높음

### 4. CH2 전류 지령 수신 타임아웃 체크
- **파일**: `HABA_control.c:929` (SCIA_RS485_MM_Rx_ISR), `HABA_globals.h`
- **설명**:
  - 병렬 모드에서 RS485 통신 끊김 시 `I_cmd_from_master`가 오래된 값 유지
  - 안전값(0A) 적용 필요
- **구현 내용**:
  ```c
  static uint16_t ch2_rx_timeout_cnt = 0;
  if (rxBuffer[0] == STX) {
      I_cmd_from_master = current;
      ch2_rx_timeout_cnt = 0;  // 리셋
  }
  // Phase 1에서 타임아웃 체크
  if (++ch2_rx_timeout_cnt > 100) {  // 5ms
      I_cmd_from_master = 32768;  // 안전값 (0A)
  }
  ```
- **출처**: 첫 커밋 비교 분석, IMPROVEMENT_CHECKLIST.md
- **예상 시간**: 1시간
- **담당**: dougshin
- **마감**: 2025-10-25
- **우선순위**: ⚠️ 높음

### 5. NTC 온도 센싱 구현
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
  5. 과온도 보호 로직 활성화 (OVER_TEMP: 120°C)
- **개발 이력 참고**: `[07.11] NTC 온도 측정을 위한 ADCA 채널 추가 적용` (미완성)
- **예상 시간**: 4시간
- **담당**: dougshin
- **마감**: 2025-11-15

### 6. 과전류 임계값 마진 확보
- **파일**: `HABA_globals.h:108`
- **설명**:
  - 현재: OVER_CURRENT = 88.0A (정격 80A + 10%)
  - 산업 기준: 정격 + 20% 권장
- **수정**:
  ```c
  #define OVER_CURRENT (96.0f)  // 정격 80A + 20%
  ```
- **출처**: POWER_CONTROL_REVIEW.md, IMPROVEMENT_CHECKLIST.md
- **예상 시간**: 5분
- **담당**: dougshin
- **마감**: 2025-10-22
- **우선순위**: 높음

### 7. Battery Mode CV 제어 완성 및 튜닝
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

### 7-1. 슬레이브 과전력(OP) 보호 구현 (RS-232 Rev5 반영)
- **파일**:
  - `HABA_control.c` (Phase 3: Check_System_Safety)
  - `HABA_control.c` (Send_Slave_Batch_To_SCADA)
  - `HABA_globals.h`, `HABA_globals.c`
- **설명**:
  - RS-232 SCADA 프로토콜 Rev5에서 슬레이브 Fault 비트맵에 OP(Over Power) 추가
  - 슬레이브는 자체 보호 장치가 있지만, 마스터에서도 전력 모니터링 필요
  - V×I 곱으로 전력 계산하여 정격(30kW) 초과 감지
- **구현 내용**:
  ```c
  // HABA_globals.h
  #define OVER_POWER_SLAVE (35.0f)  // 30kW + 15% 여유 [kW]
  extern float32_t V_out_slave[16];  // 슬레이브 전압 (추가 필요 시)
  extern uint8_t over_power_slave[16];  // OP 플래그

  // HABA_control.c - Phase 3 또는 슬레이브 데이터 수신 시
  for (uint8_t id = 1; id <= 15; id++) {
      if (!DAB_ok_slave[id]) continue;

      // 전력 계산 (kW)
      float power = fabsf(V_out_slave[id] * I_out_slave[id]) / 1000.0f;

      if (power > OVER_POWER_SLAVE) {
          over_power_slave[id] = 1;
      } else {
          over_power_slave[id] = 0;
      }
  }

  // Send_Slave_Batch_To_SCADA() - OP 비트 추가
  if (over_power_slave[slave_id]) fault |= 0x80;  // bit7
  ```
- **참고**:
  - 슬레이브 전압(`V_out_slave[]`)을 CAN으로 수신하는지 확인 필요
  - 현재는 전류(`I_out_slave[]`)와 온도만 수신
  - V_out_slave[] 추가 또는 마스터 V_out로 근사 (병렬 모드)
- **프로토콜 명세**: `docs/RS232_SCADA_protocol_rev5.md`
- **예상 시간**: 2시간 (전압 데이터 확인 + 구현)
- **담당**: dougshin
- **마감**: 2025-11-15
- **우선순위**: 중간 (슬레이브 자체 보호 존재, 모니터링 목적)

---

## 🟡 MEDIUM Priority

### 8. RS485 스킵 카운터 모니터링
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
- **마감**: 2025-10-30
- **우선순위**: 중간

### 9. 병렬 모드 채널간 전류 불균형 모니터링
- **파일**: `HABA_control.c:548` (Apply_Current_Reference_Limit)
- **설명**:
  - CH1-CH2 간 전류 불균형 감시 (5% 초과 시 경고)
  - 병렬 모드 품질 향상
- **구현 내용**:
  ```c
  if (operation_mode == MODE_PARALLEL) {
      float32_t ch1_total = 0, ch2_total = 0;
      for (int i=1; i<=6; i++)  ch1_total += I_out_slave[i];
      for (int i=7; i<=12; i++) ch2_total += I_out_slave[i];

      float32_t imbalance = fabsf(ch1_total - ch2_total) / (ch1_total + ch2_total);
      if (imbalance > 0.05f)
          warn_channel_imbalance = true;
  }
  ```
- **출처**: POWER_CONTROL_REVIEW.md, IMPROVEMENT_CHECKLIST.md
- **예상 시간**: 1시간
- **담당**: dougshin
- **마감**: 2025-10-30
- **우선순위**: 중간

### 10. RS485 DMA 전환 (성능 최적화)
- **파일**:
  - `HABA_control.c:344` (Transmit_Current_Command)
  - `HABA_setup.c` (Init_DMA 추가)
- **설명**:
  - Phase 2 RS485 송신을 DMA로 오프로드
  - ISR 실행 시간 단축 (1.5μs → 0.5μs, -1μs)
  - CPU 부하 감소
- **구현 내용**:
  1. DMA 채널 설정 (SCIA, SCIB)
  2. RS485 송신 함수 DMA 방식으로 변경
  3. DMA 완료 인터럽트 처리 (선택적)
  4. 타이밍 검증 (GPIO 디버그)
- **현재 상태**: TX FIFO 블로킹 방지로 안정성 확보 (2025-10-20)
- **우선순위 이유**: 성능 최적화, 필수는 아님
- **예상 시간**: 8시간
- **담당**: 미정
- **마감**: 미정

### 11. 프리차징 릴레이 복구
- **파일**: `HABA_main.c:51` (개발 이력 주석)
- **설명**:
  - ※ 프리차징 Relay 소스코드 주석처리 추후 코드 복귀작업 필요(20250918)
  - 현재 메인 릴레이만 사용 (GPIO8)
- **구현 내용**:
  1. 프리차징 릴레이 GPIO 매핑 확인
  2. 시퀀스 제어 로직에 프리차징 단계 추가
  3. V_out ≈ V_batt 조건 확인 후 메인 릴레이 ON
  4. 안전성 검증
- **개발 이력**: `[09.18] 개별운전, 병렬운전 릴레이 수정`
- **예상 시간**: 1시간
- **담당**: 미정
- **마감**: 미정

---

## 🟢 LOW Priority

### 12. Magic Number 제거 (코드 품질)
- **파일**: `HABA_main.c:219` 및 기타
- **설명**:
  - 현재: 일부 코드에서 상수 대신 숫자 직접 사용
  - 예: `if (sequence_step == 20)` → `if (sequence_step == SEQ_STEP_NORMAL_RUN)`
- **구현 내용**:
  1. 전체 파일 검색 (`grep -n "== [0-9]"`)
  2. 상수로 대체
  3. 가독성 검증
- **출처**: CODE_ANALYSIS_REPORT.md, IMPROVEMENT_CHECKLIST.md
- **예상 시간**: 15분
- **담당**: 미정
- **마감**: 미정
- **우선순위**: 낮음 - 코드 품질

### 13. 복잡한 함수 리팩토링
- **파일**: `HABA_control.c:173` (Apply_PI_And_Convert_DAC)
- **설명**:
  - 중첩 if-else 구조 (최대 3단계), 150라인
  - 사이클로매틱 복잡도 높음
- **구현 내용**:
  1. 헬퍼 함수 분리 (Calculate_Battery_Mode_Current, Calculate_ChargeDischarge_Mode_Current)
  2. 전략 패턴 적용
  3. 가독성 향상
- **출처**: CODE_ANALYSIS_REPORT.md, IMPROVEMENT_CHECKLIST.md
- **예상 시간**: 2시간
- **담당**: 미정
- **마감**: 미정
- **우선순위**: 낮음 - 유지보수성

### 14. 전역 변수 구조체 그룹화
- **파일**: `HABA_globals.h` (685라인, 113개 변수)
- **설명**:
  - 현재: 개별 변수로 선언 (volatile extern)
  - 권장: 구조체로 그룹화 (SystemState_t, ControlParams_t)
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
- **출처**: CODE_ANALYSIS_REPORT.md, TODO 중기 계획
- **예상 시간**: 4시간 (리팩토링 + 검증)
- **담당**: 미정
- **마감**: 미정
- **우선순위**: 낮음 - 장기 유지보수

### 15. CAN 슬레이브 16~31 지원 확장
- **파일**:
  - `HABA_main.c:58` (개발 이력 주석)
  - `HABA_main.c:166` (Main Loop, for문 수정)
- **설명**:
  - 16~31 추가 시 while문 검증 필요
  - 현재 1~16만 지원 (실제 사용은 1~6)
- **구현 내용**:
  1. CAN 메일박스 16~31 초기화
  2. Main Loop 슬레이브 읽기 범위 확장
  3. 배열 크기 검증 (I_out_slave[31])
  4. 성능 영향 확인 (10ms 작업 시간)
- **현재 상태**: 실사용 슬레이브 6개, 확장 필요성 낮음
- **예상 시간**: 2시간
- **담당**: 미정
- **마감**: 미정

### 16. Phase 분리 재검토 (6-Phase 구조)
- **파일**:
  - `HABA_main.c:91` (control_phase_table)
  - `HABA_control.c` (Phase 함수들)
- **설명**:
  - 현재 5-Phase (20kHz) → 6-Phase (16.67kHz) 전환 검토
  - Phase 2 부하 분산 (SCIA, SCIB 분리)
- **구현 내용**:
  1. Phase 2a: SCIA RS485 (Master-to-Master)
  2. Phase 2b: SCIB RS485 (Master-to-Slave)
  3. 제어 주기 변경에 따른 PI 튜닝 재검토
  4. 타이밍 검증
- **우선순위 이유**: RS485 DMA 전환이 더 효과적
- **예상 시간**: 6시간
- **담당**: 미정
- **마감**: 미정

---

## ✅ Completed

### 2025-10-20 (Latest)
- **첫 커밋 대비 오작동 가능성 심층 분석**
  - 첫 커밋(ec95937) vs 현재(b7dfcca) 비교 분석 완료
  - 종합 점수: 6.4/10 → 9.0/10 (+2.6점 향상)
  - 주요 개선: PI 제어기 DCL 전환, 적분기 초기화, TX FIFO 블로킹 방지
  - 신규 발견: CH2 수신 타임아웃 체크 부재, PI 게인 재튜닝 필요성
  - 문서: `docs/IMPROVEMENT_CHECKLIST.md` 생성

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

### 🔴 즉시 조치 (Week 1)
1. 과온도 임계값 수정 (10분) ⚠️⚠️⚠️
2. 과전류 마진 확보 (5분) ⚠️
3. 고장 래칭 메커니즘 (30분) ⚠️
4. CH2 수신 타임아웃 (1시간) ⚠️

**총 예상 시간**: 1.75시간

### 🟡 검증 작업 (Week 2)
1. PI 게인 재튜닝 검증 (2시간)
2. NTC 온도 센싱 구현 (4시간)

**총 예상 시간**: 6시간

### 🟢 품질 개선 (Week 3)
1. RS485 스킵 모니터링 (30분)
2. 채널 불균형 모니터링 (1시간)
3. Battery Mode CV 튜닝 (2시간)

**총 예상 시간**: 3.5시간

### 장기 개선
1. RS485 DMA 전환 (성능)
2. Magic Number 제거 (코드 품질)
3. 복잡한 함수 리팩토링 (유지보수성)
4. 전역 변수 구조체 그룹화 (아키텍처)

---

## 🔗 관련 문서

- `CLAUDE.md`: 프로젝트 개요 및 아키텍처
- `docs/QUICK_REFERENCE.md`: 빠른 참조
- `docs/PROJECT_INDEX.md`: API 레퍼런스
- `docs/POWER_CONTROL_REVIEW.md`: PI 튜닝 가이드
- `docs/CODE_ANALYSIS_REPORT.md`: 코드 품질/보안 분석
- `docs/IMPROVEMENT_CHECKLIST.md`: 개선사항 체크리스트 (NEW!)
- `docs/RS232_interface_protocol_rev4.md`: SCADA 프로토콜
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
