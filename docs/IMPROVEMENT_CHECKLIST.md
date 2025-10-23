# 개선사항 체크리스트 - 30kW Master Controller

**작성일**: 2025-10-20
**기준**: 첫 커밋(ec95937) vs 현재(b7dfcca) 비교 분석 + 기존 분석 문서 통합

---

## 🔴 CRITICAL (즉시 조치 필요)

### ~~1. 과온도 임계값 위험~~ ✅ 완료 (2025-10-20)
**출처**: POWER_CONTROL_REVIEW.md
**파일**: `HABA_globals.h:109`

**완료 내용**:
```c
#define OVER_TEMP         (85)   // 방열판 85°C → Tj ≈ 180°C (강제 공랭 팬×3 고려)
#define OVER_TEMP_WARNING (75)   // 경고 레벨 (Derating 시작)
```

**결과**:
- 하드웨어 손상 위험 제거 (Tj ≈ 720°C → 180°C)
- 강제 공랭 시스템(팬 3개) 특성 반영
- Derating 단계 구현으로 안전성 향상

**소요 시간**: 5분
**우선순위**: **최우선** - 하드웨어 손상 방지
**상태**: ✅ 완료

---

### 2. 고장 래칭 메커니즘 부재 ⚠️
**출처**: POWER_CONTROL_REVIEW.md
**파일**: `HABA_control.c:508-541`

**현재**: 고장 조건 해제 시 즉시 복구 → 간헐적 고장 시 불안정

**수정**:
```c
// HABA_globals.h
extern volatile bool fault_latched;

// HABA_control.c:508
static bool fault_latched = false;

if (is_fault_active && !fault_latched) {
    fault_latched = true;  // 래칭
    run = 0;
}

// 수동 리셋: SCADA 명령 또는 전원 재부팅으로만 해제
if (scada_fault_reset_cmd) {
    fault_latched = false;
    over_voltage_flag = FAULT_FLAG_INACTIVE;
    over_current_flag = FAULT_FLAG_INACTIVE;
    over_temp_flag = FAULT_FLAG_INACTIVE;
}
```

**예상 시간**: 30분
**우선순위**: 높음 - 안전성 향상
**상태**: ❌ 미완료

---

## 🟡 HIGH (검증 필요)

### 3. PI 게인 재튜닝 검증 (DCL 전환 후)
**출처**: 첫 커밋 비교 분석 (신규 발견)
**파일**: `HABA_setup.c:350`, `HABA_cla_tasks.cla`

**배경**:
- 첫 커밋: 수동 PI 구현 (ideal 유사)
- 현재: DCL_runPI_L2() (parallel form)
- 미세한 동작 차이 가능 (반올림, 포화 처리)

**검증 사항**:
- [ ] 스텝 응답 테스트 (V_max_cmd 급변 100V → 300V)
- [ ] 오버슈트 < 5%
- [ ] 정착 시간 < 100ms
- [ ] 보드 선도 측정 (Phase Margin ≥ 45°, Gain Margin ≥ 6dB)

**현재 게인**:
```c
pi_charge.Kp = 1.0f;
pi_charge.Ki = 3000.0f * 50e-6f;  // = 0.15
```

**예상 시간**: 2시간 (측정 + 분석)
**우선순위**: 높음 - 제어 안정성 검증
**상태**: ❌ 미완료

---

### 4. CH2 전류 지령 수신 타임아웃 체크 부재
**출처**: 첫 커밋 비교 분석 (신규 발견)
**파일**: `HABA_control.c:929` (SCIA_RS485_MM_Rx_ISR)

**현재**: RS485 통신 끊김 시 `I_cmd_from_master`가 오래된 값 유지

**수정**:
```c
// HABA_globals.h
extern volatile uint16_t ch2_rx_timeout_cnt;

// HABA_control.c:929
static uint16_t ch2_rx_timeout_cnt = 0;

if (IS_CH2) {
    rxBuffer[rxIndex++] = rx;

    if (rxIndex >= 4) {
        if (rxBuffer[0] == STX) {
            uint16_t current = ((uint16_t)rxBuffer[2] << 8) | rxBuffer[1];
            I_cmd_from_master = current;
            ch2_rx_timeout_cnt = 0;  // 리셋
        }
        rxIndex = 0;
    }
}

// Phase 1 또는 1ms 작업에서 타임아웃 체크
if (IS_CH2 && operation_mode == MODE_PARALLEL) {
    if (++ch2_rx_timeout_cnt > 100) {  // 100회 = 5ms @ 20kHz
        I_cmd_from_master = 32768;  // 안전값 (0A)
        // 경고 플래그 설정
    }
}
```

**예상 시간**: 1시간
**우선순위**: 높음 - 병렬 모드 안전성
**상태**: ❌ 미완료

---

### 5. 과전류 임계값 마진 부족
**출처**: POWER_CONTROL_REVIEW.md
**파일**: `HABA_globals.h:108`

**현재**:
```c
#define OVER_CURRENT (88.0f)  // 정격 80A + 10%
```

**권고**:
```c
#define OVER_CURRENT (96.0f)  // 정격 80A + 20% (산업 기준)
```

**예상 시간**: 5분
**우선순위**: 중간 - 안전 마진 확보
**상태**: ❌ 미완료

---

## 🟢 MEDIUM (개선 권장)

### 6. RS485 스킵 카운터 모니터링
**출처**: 첫 커밋 비교 분석 (신규 발견)
**파일**: `HABA_control.c:344`, `HABA_main.c` (10ms 작업)

**현재**: 스킵 카운터만 증가, 감시 로직 없음

**추가**:
```c
// HABA_globals.h
#define RS485_SKIP_THRESHOLD  100  // 1초당 100회 = 10% at 1kHz

// HABA_main.c - 10ms 작업 또는 1초 주기
if (rs485_mm_skip_cnt > RS485_SKIP_THRESHOLD) {
    // 통신 품질 경고 (SCADA에 보고 또는 LED 표시)
    comm_quality_warning = true;
}

// 주기적 리셋 (1초마다)
if (flag_1s) {
    rs485_mm_skip_cnt = 0;
    rs485_ms_skip_cnt = 0;
}
```

**예상 시간**: 30분
**우선순위**: 중간 - 통신 품질 추적
**상태**: ❌ 미완료

---

### 7. 병렬 모드 채널간 전류 불균형 모니터링
**출처**: POWER_CONTROL_REVIEW.md
**파일**: `HABA_control.c:548` (Apply_Current_Reference_Limit)

**현재**: CH1-CH2 간 1 사이클(50μs) 지연 존재, 소프트스타트로 완화

**추가**:
```c
// Apply_Current_Reference_Limit() 내 추가
if (operation_mode == MODE_PARALLEL) {
    float32_t ch1_total = 0, ch2_total = 0;
    for (int i=1; i<=6; i++)  ch1_total += I_out_slave[i];
    for (int i=7; i<=12; i++) ch2_total += I_out_slave[i];

    float32_t total = ch1_total + ch2_total;
    if (total > 10.0f) {  // 최소 전류 조건
        float32_t imbalance = fabsf(ch1_total - ch2_total) / total;
        if (imbalance > 0.05f) {  // 5% 초과 시 경고
            warn_channel_imbalance = true;
        }
    }
}
```

**예상 시간**: 1시간
**우선순위**: 중간 - 병렬 모드 품질
**상태**: ❌ 미완료

---

## 🔵 LOW (코드 품질 개선)

### 8. Magic Number 제거
**출처**: CODE_ANALYSIS_REPORT.md
**파일**: `HABA_main.c:219`

**현재**:
```c
if (sequence_step == 20)  // SEQ_STEP_NORMAL_RUN 상수 대신 숫자
    V_fb = V_batt;
```

**수정**:
```c
if (sequence_step == SEQ_STEP_NORMAL_RUN)
    V_fb = V_batt;
```

**영향**: 가독성 향상, 유지보수 용이
**예상 시간**: 15분 (전체 파일 검색 및 수정)
**우선순위**: 낮음
**상태**: ❌ 미완료

---

### 9. 복잡한 함수 리팩토링
**출처**: CODE_ANALYSIS_REPORT.md
**파일**: `HABA_control.c:173` (Apply_PI_And_Convert_DAC)

**문제**: 중첩 if-else 구조 (최대 3단계), 150라인, 사이클로매틱 복잡도 높음

**권장**: 전략 패턴 또는 함수 분리
```c
// 헬퍼 함수 분리
static inline float32_t Calculate_Battery_Mode_Current(void);
static inline float32_t Calculate_ChargeDischarge_Mode_Current(void);
static inline uint16_t Convert_Current_To_DAC(float32_t I_per_slave);

void Apply_PI_And_Convert_DAC(void) {
    float32_t I_final;

    if (control_mode == CONTROL_MODE_BATTERY)
        I_final = Calculate_Battery_Mode_Current();
    else
        I_final = Calculate_ChargeDischarge_Mode_Current();

    I_cmd_to_slave = Convert_Current_To_DAC(I_final / get_slaves_count());

    // ... 정지 상태 처리
}
```

**예상 시간**: 2시간
**우선순위**: 낮음 - 유지보수성
**상태**: ❌ 미완료

---

### 10. 전역 변수 구조체 그룹화
**출처**: CODE_ANALYSIS_REPORT.md, TODO.md (중기 계획)
**파일**: `HABA_globals.h` (685라인, 113개 변수)

**권장**:
```c
typedef struct {
    float32_t V_out;
    float32_t V_batt;
    float32_t I_cmd;
    float32_t I_cmd_filtered;
    // ...
} SystemState_t;

typedef struct {
    float32_t Kp;
    float32_t Ki;
    float32_t I_max;
    // ...
} ControlParams_t;

extern SystemState_t g_system_state;
extern ControlParams_t g_control_params;
```

**예상 시간**: 4시간 (리팩토링 + 검증)
**우선순위**: 낮음 - 장기 유지보수
**상태**: ❌ 미완료 (TODO 중기 계획)

---

## 📋 기존 TODO 연계

### 이미 TODO에 있는 항목 (중복 제외)
- ✅ **NTC 온도 센싱 구현** → TODO.md #1 (HIGH)
- ✅ **Battery Mode CV 튜닝** → TODO.md #2 (HIGH)
- ✅ **RS485 DMA 전환** → TODO.md #3 (MEDIUM)
- ✅ **프리차징 릴레이 복구** → TODO.md #4 (MEDIUM)

### 새로 추가 필요한 항목
- ❌ **과온도 임계값 수정** (CRITICAL) → TODO에 추가 필요!
- ❌ **고장 래칭 메커니즘** (HIGH) → TODO에 추가 필요!
- ❌ **PI 게인 재튜닝 검증** (HIGH) → TODO에 추가 필요!
- ❌ **CH2 수신 타임아웃** (HIGH) → TODO에 추가 필요!

---

## 📊 우선순위 매트릭스

| 우선순위 | 항목 | 예상 시간 | 영향도 | 긴급도 |
|---------|------|-----------|--------|--------|
| 1️⃣ | 과온도 임계값 수정 | 10분 | 🔴 매우 높음 | 🔴 즉시 |
| 2️⃣ | 고장 래칭 메커니즘 | 30분 | 🟠 높음 | 🟠 높음 |
| 3️⃣ | PI 게인 재튜닝 검증 | 2시간 | 🟠 높음 | 🟡 중간 |
| 4️⃣ | CH2 수신 타임아웃 | 1시간 | 🟠 높음 | 🟡 중간 |
| 5️⃣ | 과전류 마진 확보 | 5분 | 🟡 중간 | 🟡 중간 |
| 6️⃣ | RS485 스킵 모니터링 | 30분 | 🟡 중간 | 🟢 낮음 |
| 7️⃣ | 채널 불균형 모니터링 | 1시간 | 🟡 중간 | 🟢 낮음 |

**총 예상 시간**: 약 5.5시간 (우선순위 1-7)

---

## 🎯 추천 작업 순서

### Week 1 (즉시 조치)
1. 과온도 임계값 수정 (10분)
2. 과전류 마진 확보 (5분)
3. 고장 래칭 메커니즘 (30분)
4. CH2 수신 타임아웃 (1시간)

**총**: 1.75시간

### Week 2 (검증)
5. PI 게인 재튜닝 검증 (2시간)
6. NTC 온도 센싱 구현 (4시간, TODO #1)

**총**: 6시간

### Week 3 (품질 개선)
7. RS485 스킵 모니터링 (30분)
8. 채널 불균형 모니터링 (1시간)
9. Battery Mode CV 튜닝 (2시간, TODO #2)

**총**: 3.5시간

---

**작성자**: Claude Code (Analyzer Persona)
**검토 기준**: 첫 커밋 vs 현재 + CODE_ANALYSIS + POWER_CONTROL + TODO 통합
**다음 업데이트**: 2025-11-01
