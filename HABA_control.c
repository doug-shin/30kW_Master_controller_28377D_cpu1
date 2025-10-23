//==================================================
// HABA_control.c - Control Logic and Communication Implementation
//==================================================
// TI F28377D 30kW Master Controller
// 제어 알고리즘, 통신 핸들러, ISR 구현
//==================================================

#include "HABA_control.h"
#include <string.h>
#include "vcu2/vcu2_crc.h"  // TI VCU2 CRC-32 라이브러리

//==================================================
// 디버그 모드 설정
//==================================================

#ifndef _DEBUG_CLK_STATUS_ENABLE_
#define _DEBUG_CLK_STATUS_ENABLE_      (0)
#endif

#ifndef _DEBUG_CAN_STATUS_ENABLE_
#define _DEBUG_CAN_STATUS_ENABLE_      (0)
#endif

#ifndef _DEBUG_SCI_STATUS_ENABLE_
#define _DEBUG_SCI_STATUS_ENABLE_      (0)
#endif

#ifndef _DEBUG_SPI_STATUS_ENABLE_
#define _DEBUG_SPI_STATUS_ENABLE_      (0)
#endif

//==================================================
// 파일 내부 변수
//==================================================

volatile uint16_t rxIndex = 0;          // SCIA 수신 버퍼 인덱스
volatile uint8_t  rxBuffer[4];          // SCIA 수신 버퍼 (4바이트)

// VCU2 CRC-32 객체 (SCADA 프로토콜용)
CRC_Obj    crcObj_SCADA;                // CRC 객체
CRC_Handle handleCRC_SCADA = &crcObj_SCADA;  // CRC 핸들

//==================================================
// [1] 5-Phase 제어 함수 (20kHz @ RAMFUNC)
//==================================================
// 100kHz ISR 내에서 순환 호출되는 실시간 제어 태스크
// RAM에 배치하여 Flash 대기 시간 제거 → 결정론적 실행
//==================================================

//--------------------------------------------------
// 헬퍼 함수: 전류 제한 (PI 출력 + 최종 제한)
//--------------------------------------------------
/**
 * @brief 전류 지령에 PI 출력 범위와 시스템 레벨 제한을 적용
 *
 * @details 2단계 제한 전략:
 *          1. PI 제어기 출력 범위 (I_PI_charge_out ~ I_PI_discharge_out)
 *          2. 시스템 레벨 전류 제한 (운전 모드별: 개별 480A, 병렬 960A)
 *
 * @param[in] cmd_filtered  필터링된 전류 지령 (A)
 * @return    제한 적용된 전류 지령 (A)
 *
 * @note 호출 위치: Apply_PI_And_Convert_DAC() (Phase 1, 20kHz)
 * @note 실행 시간: ~0.5μs (inline 최적화)
 */
static inline float32_t Apply_Current_Limits(float32_t cmd_filtered)
{
    float32_t limited;

    // 운전 모드별 전류 제한 선택
    float32_t current_limit = (operation_mode == MODE_PARALLEL) ?
                               CURRENT_LIMIT_PARALLEL : CURRENT_LIMIT_INDIVIDUAL;

    // 1단계: PI 출력 범위 제한 (충전/방전 전류)
    if      (cmd_filtered >  I_PI_charge_out)    limited = I_PI_charge_out;
    else if (cmd_filtered <  I_PI_discharge_out) limited = I_PI_discharge_out;
    else                                          limited = cmd_filtered;

    // 2단계: 최종 전류 제한 (시스템 레벨, 모드별)
    if (limited >  current_limit) limited =  current_limit;
    if (limited < -current_limit) limited = -current_limit;

    return limited;
}

//--------------------------------------------------
// Phase 0: 전압 센싱 + CLA PI 제어기 트리거
//--------------------------------------------------
/**
 * @brief Phase 0 제어 태스크 (전압 센싱 및 CLA PI 제어기 트리거)
 *
 * @details 파이프라인 제어 구조:
 *          - V_out, V_batt 평균 계산 (5샘플 이동평균)
 *          - 2단계 캘리브레이션 (물리값 변환 → 실측 보정)
 *          - 제어 모드별 CLA Task 트리거 (비블로킹 Force)
 *          - CLA는 백그라운드 실행, 결과는 Phase 1(10μs 후) 사용
 *
 * @note 호출 주기: 20kHz (100kHz ISR, 5-phase rotation)
 * @note 실행 시간: ~2μs (CLA Force 포함)
 * @note RAM 배치: Flash 대기 시간 제거 (.TI.ramfunc)
 *
 * @see Apply_PI_And_Convert_DAC() - Phase 1에서 CLA 결과 사용
 */
#pragma CODE_SECTION(Sensing_And_Trigger_PI, ".TI.ramfunc");
void Sensing_And_Trigger_PI(void)
{
    // --- 전압 센싱 및 캘리브레이션 ---

    // 출력 전압 (V_out) 평균 계산 (5회 누적합 / 5)
    V_out_raw_avg = (float32_t)(V_out_raw_sum * AVG_5_SAMPLES_COEFF);
    V_out_raw_sum = 0;

    // Step 1: ADC 원시값 → 물리 전압 변환
    V_out_uncal = V_out_raw_avg * VOLTAGE_ADC_SCALE - VOLTAGE_ADC_OFFSET;
    // Step 2: 실측 캘리브레이션 적용
    V_out = (V_out_uncal + VOUT_CALIB_OFFSET) * VOUT_CALIB_GAIN;

    // 배터리 전압 (V_batt) 평균 계산
    V_batt_raw_avg = (float32_t)(V_batt_raw_sum * AVG_5_SAMPLES_COEFF);
    V_batt_raw_sum = 0;

    // Step 1: ADC 원시값 → 물리 전압 변환
    V_batt_uncal = V_batt_raw_avg * VOLTAGE_ADC_SCALE - VOLTAGE_ADC_OFFSET;
    // Step 2: 실측 캘리브레이션 적용
    V_batt = (V_batt_uncal + VBATT_CALIB_OFFSET) * VBATT_CALIB_GAIN;

    // --- CLA PI 제어기 트리거 (백그라운드 실행) ---
    // Force만 사용, Wait 없음 → CPU와 CLA 병렬 실행
    // 결과는 Phase 1(10us 후)에서 사용

    // 제어 모드별 CLA Task 선택
    if (control_mode == CONTROL_MODE_CHARGE_DISCHARGE)
    {
        // Charge/Discharge 모드: Task 1 & 2 실행
        Cla1ForceTask1();       // pi_charge 제어기 (충전 모드, V_max_cmd 기준)
        Cla1ForceTask2();       // pi_discharge 제어기 (방전 모드, V_min_cmd 기준)
    }
    else if (control_mode == CONTROL_MODE_BATTERY)
    {
        // Battery 모드: Task 3 실행
        Cla1ForceTask3();       // pi_cv 제어기 (CV 제어, V_cmd 기준)
    }
    else
    {
        // 예상치 못한 control_mode: 안전을 위해 아무 Task도 실행하지 않음
        // (향후 추가 모드를 위한 확장 포인트)
    }

}

//--------------------------------------------------
// Phase 1: PI 결과 적용 및 DAC 변환
//--------------------------------------------------
/**
 * @brief Phase 1 제어 태스크 (PI 제어 결과 적용 및 슬레이브 DAC 변환)
 *
 * @details 처리 흐름:
 *          1. CLA 결과 수신 (Phase 0에서 10μs 경과 → 완료 보장)
 *          2. 제어 모드 분기 (Charge/Discharge vs Battery)
 *          3. 운전 모드 분기 (Individual vs Parallel)
 *          4. 전류 제한 적용 (PI 범위 → 시스템 레벨)
 *          5. 슬레이브 전류 지령 변환 (±100A → 0~65535 DAC 코드)
 *
 * @note 호출 주기: 20kHz (100kHz ISR, 5-phase rotation)
 * @note 실행 시간: ~3μs
 * @note CLA 완료 보장: Phase 0 트리거 후 10μs 경과 (CLA 실행은 0.1~0.2μs)
 * @note RAM 배치: Flash 대기 시간 제거 (.TI.ramfunc)
 *
 * @see Sensing_And_Trigger_PI() - Phase 0에서 CLA 트리거
 * @see Transmit_Current_Command() - Phase 2에서 DAC 값 송신
 */
#pragma CODE_SECTION(Apply_PI_And_Convert_DAC, ".TI.ramfunc");
void Apply_PI_And_Convert_DAC(void)
{
    // CLA Task 결과 준비 완료 (Phase 0에서 Force, 10us 경과)
    // Charge/Discharge 모드: I_PI_charge_out, I_PI_discharge_out 사용
    // Battery 모드: I_PI_cv_out 사용

    // --- 1. 제어 모드별 최종 전류 지령 계산 ---
    if (control_mode == CONTROL_MODE_BATTERY)
    {
        // Battery 모드 CV 제어: CLA Task 3 결과 사용
        // I_PI_cv_out는 CLA에서 V_cmd 기준 CV 제어로 계산된 전류 출력

        // 운전 모드별 처리
        if (operation_mode == MODE_INDIVIDUAL)
        {
            master_mode = 1;
            I_cmd_PI_limited = I_PI_cv_out;  // CLA Task 3 결과 사용
        }
        else if (operation_mode == MODE_PARALLEL)
        {
            if (IS_CH1)
            {
                master_mode = 1;
                I_cmd_PI_limited = I_PI_cv_out;  // CLA Task 3 결과 사용
            }
            else  // IS_CH2
            {
                master_mode = 2;
                I_cmd_PI_limited = 0.0f;  // CH2는 PI 제어 안 함
            }
        }
        else
        {
            master_mode = 0;
            I_cmd_PI_limited = 0.0f;
        }

        // Battery 모드: 전류 제한 적용 (I_max_cmd, I_min_cmd)
        float32_t current_limit = (operation_mode == MODE_PARALLEL) ?
                                   CURRENT_LIMIT_PARALLEL : CURRENT_LIMIT_INDIVIDUAL;

        // 전류 한계값 적용
        if      (I_cmd_PI_limited >  I_max_cmd)       I_cmd_PI_limited =  I_max_cmd;
        else if (I_cmd_PI_limited <  I_min_cmd)       I_cmd_PI_limited =  I_min_cmd;

        // 시스템 레벨 전류 제한
        if      (I_cmd_PI_limited >  current_limit)   I_cmd_PI_limited =  current_limit;
        else if (I_cmd_PI_limited < -current_limit)   I_cmd_PI_limited = -current_limit;
    }
    else if (control_mode == CONTROL_MODE_CHARGE_DISCHARGE)
    {
        // Charge/Discharge 모드 (기존 로직)
        if (operation_mode == MODE_INDIVIDUAL)
        {
            // 개별 모드: PI 제어 사용
            master_mode = 1;
            I_cmd_PI_limited = Apply_Current_Limits(I_cmd_filtered);
        }
        else if (operation_mode == MODE_PARALLEL)
        {
            if (IS_CH1)
            {
                // 병렬 모드 - CH1 마스터: PI 제어 사용
                master_mode = 1;
                I_cmd_PI_limited = Apply_Current_Limits(I_cmd_filtered);
            }
            else  // IS_CH2
            {
                // 병렬 모드 - CH2 마스터: CH1로부터 DAC 값 수신
                // I_cmd_from_master는 이미 DAC 변환된 값이므로 여기서는 처리 안 함
                master_mode = 2;
                I_cmd_PI_limited = 0.0f;  // CH2는 PI 제어 안 함
            }
        }
        else
        {
            // 정지 모드
            master_mode = 0;
            I_cmd_PI_limited = 0.0f;
        }
    }
    else
    {
        // 예상치 못한 control_mode: 안전 모드
        master_mode = 0;
        I_cmd_PI_limited = 0.0f;
    }

    // --- 2. 슬레이브 전류 지령 변환 (A → DAC 코드) ---
    // 변환: ±100A 전류 → 0~65535 DAC 값 (중앙값 32768)
    //
    // 개별 모드: 1채널(6개 슬레이브) 운전, I_per_slave = I_total / 6
    // 병렬 모드: 2채널(12개 슬레이브) 운전
    //            각 마스터는 자기 채널(6개 슬레이브)만 제어
    //            CH1은 I_total/2를 6개로 분배, CH2는 CH1에서 받은 DAC 값 사용
    if (operation_mode == MODE_PARALLEL && IS_CH2)
    {
        // 병렬 모드 - CH2 마스터: CH1로부터 받은 DAC 값 그대로 사용
        I_cmd_to_slave = I_cmd_from_master;
    }
    else
    {
        // 개별 모드 또는 병렬 모드 CH1: DAC 변환 수행
        float32_t I_cmd_to_slave_tmp;
        float32_t I_per_slave;  // 슬레이브당 전류 지령
        float32_t I_cmd_PI_limited_per_channel;

        if (operation_mode == MODE_INDIVIDUAL)
        {
            // 개별 모드: 전체 전류를 6개 슬레이브로 분배
            I_cmd_PI_limited_per_channel = I_cmd_PI_limited;
            I_per_slave = I_cmd_PI_limited_per_channel / SLAVES_PER_CHANNEL;
        }
        else if (operation_mode == MODE_PARALLEL && IS_CH1)
        {
            // 병렬 모드 CH1: 전체 전류를 12개 슬레이브로 분배 (각 마스터는 6개 담당)
            I_cmd_PI_limited_per_channel = I_cmd_PI_limited / TOTAL_CHANNELS;
            I_per_slave = I_cmd_PI_limited_per_channel / SLAVES_PER_CHANNEL;
        }
        else
        {
            I_per_slave = 0.0f;
        }

        // Slave 전류 지령 변환: ±100A → 0~65535 (중앙값 32768)
        I_cmd_to_slave_tmp = I_per_slave * SLAVE_CURRENT_TO_DAC_SCALE + SLAVE_CURRENT_TO_DAC_OFFSET;

        // 포화 방지
        if (I_cmd_to_slave_tmp < DAC_MIN_VALUE)     I_cmd_to_slave_tmp = DAC_MIN_VALUE;
        if (I_cmd_to_slave_tmp > DAC_MAX_VALUE)     I_cmd_to_slave_tmp = DAC_MAX_VALUE;

        I_cmd_to_slave = (uint16_t)I_cmd_to_slave_tmp;
    }

    // --- 3. 정지 상태 변수 초기화 ---
    if (run == 0)
    {
        MAIN_RELAY_OFF();               // 메인 릴레이 OFF (긴급 안전 차단)
        I_ss_ramp        = 0.0f;
        I_cmd_ramped     = 0.0f;
        I_out_ref        = 0.0f;
        
        // DCL PI 제어기 내부 상태 초기화
        pi_charge.i10 = 0.0f;           // 적분기 값
        pi_charge.i6  = 1.0f;           // Saturation flag
        pi_charge.i11 = 0.0f;           // Tustin integrator
        pi_discharge.i10  = 0.0f;
        pi_discharge.i6   = 1.0f;
        pi_discharge.i11  = 0.0f;
    }
}

//--------------------------------------------------
// Phase 2: 전류 지령 전송 (RS485)
//--------------------------------------------------
/**
 * @brief Phase 2 제어 태스크 (RS485 전류 지령 전송 with TX FIFO 블로킹 방지)
 *
 * @details 처리 흐름:
 *          1. TX FIFO 상태 확인 (최소 4레벨 여유 필요)
 *          2. FIFO 여유 있으면 송신, 없으면 스킵 (카운터 증가)
 *          3. 운전 모드별 송신 대상 선택 (Master-to-Master, Master-to-Slave)
 *
 * @note 호출 주기: 20kHz (100kHz ISR, 5-phase rotation)
 * @note 실행 시간: ~1.5μs (FIFO 여유 있음), 블로킹 없음
 * @note RAM 배치: Flash 대기 시간 제거 (.TI.ramfunc)
 *
 * @warning TX FIFO 블로킹 방지를 위해 FIFO 상태 확인 필수
 *          스킵 카운터(rs485_mm_skip_cnt, rs485_ms_skip_cnt) 모니터링 권장
 */
#pragma CODE_SECTION(Transmit_Current_Command, ".TI.ramfunc");
void Transmit_Current_Command(void)
{
    // TX FIFO 상태 확인 (최소 4레벨 여유 필요)
    // FIFO 깊이: 16레벨, 4레벨 미만 = 여유 있음
    bool tx_ready_mm = (SCI_getTxFIFOStatus(SCIA_BASE) < SCI_FIFO_TX4);
    bool tx_ready_ms = (SCI_getTxFIFOStatus(SCIB_BASE) < SCI_FIFO_TX4);

    if (operation_mode == MODE_PARALLEL)        // 병렬 운전
    {
        if (IS_CH1)                             // CH1 마스터
        {
            // Master-to-Master 전송 (CH1 → CH2)
            if (tx_ready_mm)
                Send_RS485_MM_Current(I_cmd_to_slave);
            else
                rs485_mm_skip_cnt++;            // FIFO 풀, 스킵 카운터 증가

            // Master-to-Slave 전송 (CH1 → Slaves)
            if (tx_ready_ms)
                Send_RS485_MS_Current(I_cmd_to_slave);
            else
                rs485_ms_skip_cnt++;            // FIFO 풀, 스킵 카운터 증가
        }
        else  // IS_CH2                         // CH2 마스터
        {
            // Master-to-Slave 전송만 (CH2 → Slaves)
            if (tx_ready_ms)
                Send_RS485_MS_Current(I_cmd_to_slave);
            else
                rs485_ms_skip_cnt++;            // FIFO 풀, 스킵 카운터 증가
        }
    }
    else                                        // 개별 운전 또는 정지 모드
    {
        // Master-to-Slave 전송만
        if (tx_ready_ms)
            Send_RS485_MS_Current(I_cmd_to_slave);
        else
            rs485_ms_skip_cnt++;                // FIFO 풀, 스킵 카운터 증가
    }
}

//--------------------------------------------------
// Phase 3: 시스템 안전 체크
//--------------------------------------------------
/**
 * @brief Phase 3 제어 태스크 (안전 체크 및 릴레이 제어)
 *
 * @details 안전 기능:
 *          - 과전압/과전류/과온 감지 (Check_Fault)
 *          - 비상 정지 스위치 감시 (GPIO11)
 *          - 상태 기반 릴레이 제어 (Direct Register Access)
 *
 * @details 릴레이 제어 전략:
 *          - 메인 릴레이 (GPIO8): 프리차지 완료 후 ON (개별/병렬 무관)
 *          - 병렬 연결 릴레이 (GPIO9): 병렬 모드일 때만 ON
 *          - 안전 우선: run == 0 시 모든 릴레이 강제 OFF
 *
 * @note 호출 주기: 20kHz (100kHz ISR, 5-phase rotation)
 * @note 실행 시간: ~1.5μs (GPIO 직접 액세스 사용)
 * @note RAM 배치: Flash 대기 시간 제거 (.TI.ramfunc)
 *
 * @see Check_Fault() - 고장 감지 및 LED 제어
 */
#pragma CODE_SECTION(Check_System_Safety, ".TI.ramfunc");
void Check_System_Safety(void)
{
    Check_Fault();                      // 과전압/과전류/과온 체크

    // 비상 정지 스위치 읽기 (GPIO11)
    run_switch = GPIO_readPin(11);

    // --- 릴레이 제어 (상태 기반, Direct Register Access) ---

    // 메인 릴레이: 시퀀스 단계 기반 제어 (개별/병렬 공통)
    if (run == 1 && sequence_step >= SEQ_STEP_PRECHARGE_DONE)
        MAIN_RELAY_ON();                // 메인 릴레이 ON (배터리 연결)
    else
        MAIN_RELAY_OFF();               // 안전 상태 (배터리 차단)

    // 병렬 연결 릴레이: 운전 모드 기반 제어
    if (run == 1 && operation_mode == MODE_PARALLEL)
        PARALLEL_LINK_ON();             // CH1↔CH2 병렬 연결
    else
        PARALLEL_LINK_OFF();            // 개별 운전 (연결 해제)

    // NTC 온도 센싱 (현재 비활성화)
    // TODO: ADC 기반 NTC 온도 측정 구현 시 활성화
}

//--------------------------------------------------
// Phase 4: 모니터링 데이터 갱신 및 시퀀스 실행
//--------------------------------------------------
// 장기 평균 계산 (200회 = 10ms) + 시퀀스 제어
//--------------------------------------------------
#pragma CODE_SECTION(Update_Monitoring_And_Sequence, ".TI.ramfunc");
void Update_Monitoring_And_Sequence(void)
{
    // 장기 평균 계산 (200회 = 10ms @ 20kHz)
    V_out_display_sum  += V_out;
    V_batt_display_sum += V_batt;

    if (++V_display_calc_cnt >= TIMING_200SAMPLES)
    {
        V_out_display  = V_out_display_sum * AVG_200_SAMPLES_COEFF;    // = sum / 200
        V_batt_display = V_batt_display_sum * AVG_200_SAMPLES_COEFF;   // = sum / 200

        V_out_display_sum  = 0.0f;
        V_batt_display_sum = 0.0f;
        V_display_calc_cnt = 0;
    }

    // --- 1. 소프트 스타트 램프 (돌입 전류 방지) ---
    // 운전 모드별 전류 제한 적용
    float32_t current_limit_ss = (operation_mode == MODE_PARALLEL) ?
                                  CURRENT_LIMIT_PARALLEL : CURRENT_LIMIT_INDIVIDUAL;

    I_ss_ramp += current_limit_ss * SOFTSTART_RAMP_COEFF;   // 20kHz → 개별 480A/9.6s, 병렬 960A/19.2s
    if (I_ss_ramp > current_limit_ss)
        I_ss_ramp = current_limit_ss;

    // --- 2. 소프트 스타트 제한 적용 ---
    if      (I_cmd >  I_ss_ramp) I_cmd_ramped =  I_ss_ramp;
    else if (I_cmd < -I_ss_ramp) I_cmd_ramped = -I_ss_ramp;
    else                         I_cmd_ramped =  I_cmd;

    // --- 3. 저역통과 필터 (LPF: fc=1kHz, Ts=50us) ---
    I_cmd_filtered = lpf_coeff_a * (I_cmd_ramped + I_cmd_prev) 
    + lpf_coeff_b * I_cmd_filtered;
    I_cmd_prev = I_cmd_ramped;
    
    // --- 4. DCL PI 제어기 Integrator 한계값 동적 업데이트 ---
    if (I_cmd_filtered > 0) pi_charge.Imax    = I_cmd_filtered;
    else                    pi_charge.Imax    = 0;              // 충전 모드 적분기 상한 설정
    if (I_cmd_filtered < 0) pi_discharge.Imin = I_cmd_filtered;
    else                    pi_discharge.Imin = 0;              // 방전 모드 적분기 하한 설정

    // 시퀀스 제어 모듈 실행
    Update_System_Sequence();
}

//==================================================
// [2] 제어 알고리즘
//==================================================

// [Execute_PI_Controller 함수 삭제됨 - Execute_Current_Control에 인라인 통합]

//--------------------------------------------------
// 고장 체크 및 LED 제어
//--------------------------------------------------
/**
 * @brief 시스템 고장 감지 및 LED 상태 표시
 *
 * @details 보호 기능:
 *          - 과전압 감지: V_out ≥ 1400V
 *          - 과전류 감지: |I_out| ≥ 88A
 *          - 과온 감지: NTC0 또는 NTC1 ≥ 120°C
 *          - 비상 정지 스위치 상태 반영
 *
 * @note 호출 위치: Check_System_Safety() (Phase 3, 20kHz)
 * @note 고장 발생 시: 모든 운전 LED OFF, 고장 LED ON
 *
 * @warning 고장 플래그는 래치(Latch) 방식 - 수동 리셋 필요
 */
void Check_Fault(void)
{
    // 고장 조건 감지
    bool is_overvoltage  = (V_out_display >= OVER_VOLTAGE);
    bool is_overcurrent  = (fabs(I_out_avg) >= OVER_CURRENT);
    bool is_overtemp     = (NTC_0_temp >= OVER_TEMP || NTC_1_temp >= OVER_TEMP);
    bool is_fault_active = (is_overvoltage || is_overcurrent || is_overtemp);

    // 고장 플래그 설정 (래치 방식)
    if (is_overvoltage)  over_voltage_flag = FAULT_FLAG_ACTIVE;
    if (is_overcurrent)  over_current_flag = FAULT_FLAG_ACTIVE;
    if (is_overtemp)     over_temp_flag    = FAULT_FLAG_ACTIVE;

    // 비상 정지 스위치 상태 반영
    run = run_switch ? 1 : 0;

    // LED 상태 표시
    if (is_fault_active)
    {
        // 고장 발생: 운전 정지 및 고장 표시
        master_fault_flag = FAULT_FLAG_INACTIVE;
        GPIO_writePin(LED_FAULT,     LED_ON);   // F_LED3 FAULT ON
        GPIO_writePin(LED_CHARGE,    LED_OFF);  // F_LED5 CHARGE OFF
        GPIO_writePin(LED_DISCHARGE, LED_OFF);  // F_LED4 DISCHARGE OFF
        GPIO_writePin(LED_SINGLE,    LED_OFF);  // F_LED2 SINGLE OFF
        GPIO_writePin(LED_DUAL,      LED_OFF);  // F_LED1 DUAL OFF
    }
    else
    {
        // 정상 상태: 고장 LED OFF
        master_fault_flag = FAULT_FLAG_ACTIVE;
        GPIO_writePin(LED_FAULT, LED_OFF);      // F_LED3 FAULT OFF
    }
}

//--------------------------------------------------
// 시스템 상태 업데이트 (슬레이브 모니터링 + 전류 지령 적용)
//--------------------------------------------------
// 슬레이브 출력 전류 합산 및 SCADA 전류 지령에 리미터 적용
//--------------------------------------------------
void Apply_Current_Reference_Limit(void)
{
    // 슬레이브 출력 전류 합산 (모니터링용, 슬레이브 1~10)
    I_out_slave_total =
        I_out_slave[1] + I_out_slave[2] + I_out_slave[3] +
        I_out_slave[4] + I_out_slave[5] + I_out_slave[6];
//      + I_out_slave[7] + I_out_slave[8] + I_out_slave[9] + I_out_slave[10];

    // SCADA 전류 지령에 리미터 적용 (운전 모드별 전류 제한)
    float32_t current_limit = (operation_mode == MODE_PARALLEL) ?
                               CURRENT_LIMIT_PARALLEL : CURRENT_LIMIT_INDIVIDUAL;

    if      (I_out_ref >  current_limit) I_cmd =  current_limit;
    else if (I_out_ref < -current_limit) I_cmd = -current_limit;
    else                                 I_cmd =  I_out_ref;
}

// [Debug_Communication_Status 함수 삭제됨 - 사용되지 않음]
// 필요 시 Debug_CAN_Status(), Debug_SCI_Status(), Debug_SPI_Status()를 직접 호출

//==================================================
// [3] CAN 통신 (슬레이브 모듈 제어)
//==================================================

//--------------------------------------------------
// CAN 메시지 송신
//--------------------------------------------------
// Master → Slave 제어 명령 송신 (4바이트)
//--------------------------------------------------
#pragma CODE_SECTION(Send_CANA_Message, ".TI.ramfunc");
void Send_CANA_Message(int8_t CAN_CMD)
{
    CANA_txData[0] = CAN_CMD;
    CANA_txData[1] = 0x00;
    CANA_txData[2] = 0x00;
    CANA_txData[3] = 0x00;

    CAN_sendMessage(CANA_BASE, TX_MSG_OBJ_ID1, MSG_DATA_LENGTH, CANA_txData);
}

//--------------------------------------------------
// CAN 메시지 일괄 수신 (현재 사용 안 함)
//--------------------------------------------------
#pragma CODE_SECTION(Read_CANA_Messages, ".TI.ramfunc");
void Read_CANA_Messages(void)
{
    int i;
    for (i = 0; i < RX_MSG_OBJ_COUNT; i++)
    {
        CAN_readMessage(CANA_BASE,
                        RX_MSG_OBJ_BASE_ID + i,
                        CANA_rxDataArray[i]);
    }
}

//--------------------------------------------------
// 슬레이브 초기화 (초기 탐색)
//--------------------------------------------------
// 모든 슬레이브 변수 초기화 (시스템 부팅 시 호출)
//--------------------------------------------------
void Init_Slave_Variables(void)
{
    for (int i = 0; i < CAN_MAILBOX_MAX; i++)
    {
        slave_enabled[i]        = false;
        I_out_slave_raw[i]      = 0;
        I_out_slave[i]          = 0.0f;
        temp_slave_raw[i]       = 0;
        DAB_ok_slave[i]         = 0;
        can_rx_fault_cnt[i]     = 0;
    }
}

//--------------------------------------------------
// 슬레이브 데이터 읽기 (개별 메일박스)
//--------------------------------------------------
// Slave → Master 피드백 데이터 수신 (전류, 온도, 상태)
// 프레임 형식: [I_out_H][I_out_L][Temp_11:0][DAB_OK]
//--------------------------------------------------
#pragma CODE_SECTION(Read_CAN_Slave, ".TI.ramfunc");
bool Read_CAN_Slave(uint16_t mbox)
{
    bool status;
    if (mbox < CAN_MAILBOX_MIN || mbox > CAN_MAILBOX_MAX) return false;

    status = CAN_readMessage(CANA_BASE, mbox, rxData);

    if (status)
    {
        // 수신 데이터 파싱 (4바이트)
        uint8_t byte0 = rxData[0];
        uint8_t byte1 = rxData[1];
        uint8_t byte2 = rxData[2];
        uint8_t byte3 = rxData[3];

        uint16_t slv_idx = mbox - 1;    // slave index: 1~31

        // 전류 (16bit), 온도 (12bit), DAB_OK (1bit)
        I_out_slave_raw[slv_idx]    = ((uint16_t)byte0 << 8) | byte1;
        temp_slave_raw[slv_idx]     = (byte2 << 4) | ((byte3 & 0xF0) >> 4);
        DAB_ok_slave[slv_idx]       = byte3 & 0x01;

        // 전류 정규화: 0~65535 → -100~100A
        float32_t normalized = ((float32_t)I_out_slave_raw[slv_idx] / (float32_t)DAC_MAX_VALUE) * CURRENT_RAW_TO_NORM_SCALE - CURRENT_RAW_TO_NORM_OFFSET;
        I_out_slave[slv_idx] = normalized * CURRENT_NORM_TO_AMP_SCALE;

        can_rx_fault_cnt[slv_idx] = 0;
        return true;
    }
    else
    {
        // 수신 실패 시 카운트 증가
        uint16_t slv_idx = mbox - 1;
        if (can_rx_fault_cnt[slv_idx]++ >= CAN_RX_FAULT_THRESHOLD)
        {
            I_out_slave[slv_idx]        = 0.0f;
            temp_slave_raw[slv_idx]     = 0;
            DAB_ok_slave[slv_idx]       = 0;
            I_out_slave_raw[slv_idx]    = 0;
            can_rx_fault_cnt[slv_idx]   = CAN_RX_FAULT_THRESHOLD;
        }
        return false;
    }
}

//==================================================
// [4] SPI 통신 (DAC 및 FPGA)
//==================================================

//--------------------------------------------------
// DAC80502 SPI 제어 (SPIA)
//--------------------------------------------------
// 아날로그 출력 제어 (3바이트 전송)
//--------------------------------------------------
#pragma CODE_SECTION(Write_SPI_DAC1, ".TI.ramfunc");
void Write_SPI_DAC1(uint8_t cmd_byte, uint16_t dac_data)
{
    SPI_writeDataBlockingFIFO(SPIA_BASE, ((uint16_t)cmd_byte) << 8);
    SPI_writeDataBlockingFIFO(SPIA_BASE, ((uint16_t)(dac_data >> 8)) << 8);
    SPI_writeDataBlockingFIFO(SPIA_BASE, ((uint16_t)(dac_data & 0xFF)) << 8);
}

//--------------------------------------------------
// FPGA 데이터 읽기 (SPIC)
//--------------------------------------------------
// Dummy Write로 FPGA 데이터 트리거 (실제 읽기는 ISR에서 수행)
//--------------------------------------------------
#pragma CODE_SECTION(Read_FPGA_Data, ".TI.ramfunc");
void Read_FPGA_Data(void)
{
    // Dummy Write (FPGA 데이터 클럭 생성)
    HWREGH(SPIC_BASE + SPI_O_TXBUF) = 0x0000;
    HWREGH(SPIC_BASE + SPI_O_TXBUF) = 0x0000;
    HWREGH(SPIC_BASE + SPI_O_TXBUF) = 0x0000;

    // 실제 읽기는 SPIC_FPGA_Rx_ISR에서 수행
}

//==================================================
// [5] RS485 통신 (전류 지령 전송)
//==================================================

//--------------------------------------------------
// SCIA RS485 전류 지령 송신 (Master-to-Master)
//--------------------------------------------------
// 프레임 형식: [STX][Current_L][Current_H][ETX]
//--------------------------------------------------
#pragma CODE_SECTION(Send_RS485_MM_Current, ".TI.ramfunc");
void Send_RS485_MM_Current(uint16_t current)
{
    if (IS_CH1)     // CH1 마스터만 송신
    {
        scia_rs485_mm_tx_buf[0] = STX;
        scia_rs485_mm_tx_buf[1] = current & 0xFF;
        scia_rs485_mm_tx_buf[2] = (current >> 8) & 0xFF;
        scia_rs485_mm_tx_buf[3] = ETX;

        for (int i = 0; i < 4; i++)
        {
            HWREGH(SCIA_BASE + SCI_O_TXBUF) = scia_rs485_mm_tx_buf[i];
        }
    }
}

//--------------------------------------------------
// SCIB RS485 전류 지령 송신 (Master-to-Slave)
//--------------------------------------------------
// 프레임 형식: [STX][Current_L][Current_H][ETX]
//--------------------------------------------------
#pragma CODE_SECTION(Send_RS485_MS_Current, ".TI.ramfunc");
void Send_RS485_MS_Current(uint16_t current)
{
    int i;
    scib_rs485_ms_tx_buf[0] = STX;
    scib_rs485_ms_tx_buf[1] = current & 0xFF;
    scib_rs485_ms_tx_buf[2] = (current >> 8) & 0xFF;
    scib_rs485_ms_tx_buf[3] = ETX;

    for (i = 0; i < 4; i++)
    {
        HWREGH(SCIB_BASE + SCI_O_TXBUF) = scib_rs485_ms_tx_buf[i];
    }
}

//==================================================
// [6] GPIO 제어 및 유틸리티
//==================================================

// [Control_Relay, Read_Emergency_Stop_Switch 함수 삭제됨 - Check_System_Safety에 인라인 통합]

//--------------------------------------------------
// Master ID 읽기 (GPIO36~39 DIP 스위치)
//--------------------------------------------------
// 4비트 DIP 스위치로 Master ID 결정 (0=상위, 1=하위)
//--------------------------------------------------
void Read_Master_ID_From_DIP(void)
{
    // GPIO36~39를 직접 읽어서 4비트 Master ID 구성
    master_id = (GPIO_readPin(39) << 3) | 
                (GPIO_readPin(38) << 2) | 
                (GPIO_readPin(37) << 1) | 
                GPIO_readPin(36);
}

//==================================================
// [7] 시퀀스 제어 모듈
//==================================================

//--------------------------------------------------
// 시퀀스 제어 (Precharge → Main Relay ON → 정상 운전)
//--------------------------------------------------
// sequence_step 진행:
//   SEQ_STEP_IDLE (0)              → 대기 (프리차지 진행)
//   SEQ_STEP_PRECHARGE_DONE (10)  → 프리차지 완료, 1초 대기
//   SEQ_STEP_NORMAL_RUN (20)      → 메인 릴레이 ON, 정상 운전
//
// 고장 시: 자동 차단 (과전압/과전류/과온)
// 릴레이 제어: Phase 3에서 sequence_step 기반 자동 처리
//--------------------------------------------------
void Update_System_Sequence(void)
{
    // ===== 시퀀스 제어 (sequence_step 기반) =====
    // Rev 2.1 state machine 코드 삭제됨 (SCADA_cmd, scada_rx_data 의존성 제거)

    if (run == 1)
    {
        switch (sequence_step)
        {
            case SEQ_STEP_IDLE:     // Precharge 단계 (IDLE 상태)
                if (start_stop == START)
                {
                    V_max_cmd = V_batt_display;
                    V_min_cmd = 0;
                    I_out_ref = 2;

                    // 프리차지 완료 조건 판정: V_out ≈ V_batt (±2V 이내)
                    float32_t voltage_diff = V_out_display - V_batt_display;
                    bool is_precharge_complete = (fabs(voltage_diff) < PRECHARGE_VOLTAGE_DIFF_OK);

                    if (is_precharge_complete)
                    {
                        pre_chg_ok = 1;
                        ready_state = 1;    // IDLE → READY 전환
                        sequence_step = SEQ_STEP_PRECHARGE_DONE;
                    }
                    else
                    {
                        pre_chg_ok = 0;
                        ready_state = 0;    // IDLE 상태 유지
                    }
                }
                else    // Pre-charge 해제
                {
                    V_max_cmd = 0;
                    V_min_cmd = 0;
                    I_out_ref = 0;
                    pre_chg_ok = 0;
                    ready_state = 0;        // IDLE 상태
                    // 릴레이 제어: Phase 3에서 sequence_step 기반 자동 처리
                }
            break;

            case SEQ_STEP_PRECHARGE_DONE:    // 프리차지 완료 후 전류를 0으로 변경 (READY+STOP)
                ready_state = 1;    // READY 상태 유지
                V_max_cmd = V_batt_display;
                V_min_cmd = 0;
                I_out_ref = 0;
                pre_chg_cnt++;

                // 1초 대기 후 정상 운전으로 전환
                bool is_delay_complete = (pre_chg_cnt >= SEQ_PRECHARGE_DELAY_COUNT);
                if (is_delay_complete)
                {
                    pre_chg_cnt = SEQ_PRECHARGE_DELAY_COUNT;  // 카운터 포화 방지
                    // 메인 릴레이 ON: Phase 3에서 sequence_step 기반 자동 처리
                    sequence_step = SEQ_STEP_NORMAL_RUN;
                }
            break;

            case SEQ_STEP_NORMAL_RUN:    // 정상 운전 (UI 전류 지령에 따라 동작, READY+RUN)
                ready_state = 1;    // READY 상태 유지

                // 제어 모드별 데이터 처리
                if (control_mode == CONTROL_MODE_CHARGE_DISCHARGE)
                {
                    // Charge/Discharge 모드: V_max_cmd, V_min_cmd, I_out_ref는
                    // Parse_SCADA_Command()에서 이미 설정됨 (추가 처리 불필요)
                }
                else if (control_mode == CONTROL_MODE_BATTERY)
                {
                    // Battery 모드: V_cmd, I_max_cmd, I_min_cmd 사용
                    V_max_cmd = V_cmd;      // CV 제어 목표 전압
                    V_min_cmd = 0;
                    I_out_ref = 0;          // Battery 모드에서는 전류 지령 사용 안 함
                }
                else
                {
                    // 예상치 못한 control_mode: 안전 상태
                    V_max_cmd = 0;
                    V_min_cmd = 0;
                    I_out_ref = 0;
                }

                // UI에서 STOP 명령 시
                if (start_stop == STOP)
                {
                    sequence_step = SEQ_STEP_IDLE;
                    ready_state = 0;    // IDLE 상태로 복귀
                    pre_chg_cnt = 0;    // 카운터 리셋
                }
            break;
        }
    }
    else    // run == 0 (비상 정지 또는 고장)
    {
        V_max_cmd = 0;
        V_min_cmd = 0;
        I_out_ref = 0;
        sequence_step = SEQ_STEP_IDLE;
        start_stop = STOP;
        ready_state = 0;    // IDLE 상태로 복귀
        pre_chg_cnt = 0;    // 카운터 리셋
        // 릴레이 OFF: Phase 3에서 run == 0 조건으로 자동 처리
    }
}

//==================================================
// [8] 인터럽트 서비스 루틴 (ISR)
//==================================================

//--------------------------------------------------
// ADCA1 인터럽트 (현재 비활성화)
//--------------------------------------------------
__interrupt void INT_ADCA1_ISR(void)
{
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

//--------------------------------------------------
// CLA 인터럽트 (CLA Task 완료 시 호출)
//--------------------------------------------------
// CLA Task1/Task2 완료 시 호출되는 ISR (현재 별도 처리 없음)
//--------------------------------------------------
__interrupt void CLA1_ISR1()
{
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP11);
    // asm(" ESTOP0");      // 디버그용 브레이크포인트 (비활성화)
}

__interrupt void CLA1_ISR2()
{
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP11);
}

//--------------------------------------------------
// SCIA RS485 Master-to-Master RX 인터럽트
//--------------------------------------------------
// 상위 마스터(CH1)로부터 전류 지령 수신 (하위 마스터 전용)
// 프레임 형식: [STX][Current_L][Current_H][ETX]
//--------------------------------------------------
__interrupt void SCIA_RS485_MM_Rx_ISR(void)
{
    uint8_t rx = SCI_readCharNonBlocking(SCIA_BASE);

    if (IS_CH2)     // CH2 마스터일 때만 수신
    {
        rxBuffer[rxIndex++] = rx;

        if (rxIndex >= 4)
        {
            // 4바이트 프레임 수신 완료: STX 체크 후 전류 지령 추출
            if (rxBuffer[0] == 0x02)    // STX
            {
                uint16_t current = ((uint16_t)rxBuffer[2] << 8) | rxBuffer[1];
                I_cmd_from_master = current;    // CH1에서 받은 전류 지령 저장
            }
            rxIndex = 0;
        }
    }

    SCI_clearOverflowStatus(SCIA_BASE);
    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_RXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}

//--------------------------------------------------
// SPIC FPGA RX 인터럽트
//--------------------------------------------------
// FPGA로부터 I_out, V_out, V_batt 원시값 수신 (3 워드)
//--------------------------------------------------
__interrupt void SPIC_FPGA_Rx_ISR(void)
{
    I_out_raw   = HWREGH(SPIC_BASE + SPI_O_RXBUF);
    V_out_raw   = HWREGH(SPIC_BASE + SPI_O_RXBUF);
    V_batt_raw  = HWREGH(SPIC_BASE + SPI_O_RXBUF);

    SPI_clearInterruptStatus(SPIC_BASE, SPI_INT_RXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP6);
}

//==================================================
// [9] SCADA 통신 (SCID 프로토콜)
//==================================================

//--------------------------------------------------
// Slave 데이터 → SCADA 송신 (10ms 주기)
//--------------------------------------------------
// 11바이트 패킷: [STX][ID+Status][Current_H][Current_L][Temp][Reserved][Reserved][CRC32_3][CRC32_2][CRC32_1][CRC32_0][ETX]
//--------------------------------------------------
void Send_Slave_Status_To_SCADA(void)
{
    static uint8_t slave_index = 0;                     // 현재 송신할 슬레이브 인덱스 (0~14)
    static uint8_t max_slaves = SCADA_MAX_SLAVES;       // 최대 슬레이브 수 (SCADA 송신용)
    
    uint8_t slave_id = slave_index + 1;
    int current_cA = 0;
    uint16_t current_code = 0;
    uint8_t temp_data = 0;
    uint8_t checksum = 0;
    uint16_t i;

    // 테스트 모드 제어 스위치
    #define USE_TEST_DATA 0     // 0: 실제 CAN 데이터, 1: 테스트 데이터

    memset(slave_tx_buffer, 0, sizeof(slave_tx_buffer));

    if (USE_TEST_DATA == 0)
    {
        // --- 실제 CAN 데이터 사용 ---
        current_cA = (int)(I_out_slave[slave_index] * 100);    // 0.01A 단위
        if (current_cA > INT16_MAX_VALUE) current_cA = INT16_MAX_VALUE;
        if (current_cA < INT16_MIN_VALUE) current_cA = INT16_MIN_VALUE;

        // 전류를 uint16 코드로 변환 (Center=32768)
        current_code = (uint16_t)(current_cA + DAC_CENTER_VALUE);

        // 온도 (0.5℃ 단위 스케일링)
        temp_data = (uint8_t)(temp_slave_raw[slave_index] * 2);
        if (temp_data > UINT8_MAX_VALUE) temp_data = UINT8_MAX_VALUE;
        slave_tx_buffer[4] = temp_data;

        // ID + Status
        slave_tx_buffer[1] = (slave_id << 3) | (DAB_ok_slave[slave_index] & 0x01);
    }
    else
    {
        // --- 테스트 데이터 ---
        if (slave_index == 0) { current_cA = current_cA_test; slave_tx_buffer[4] = 40; }
        else if (slave_index == 1) { current_cA = current_cA_test; slave_tx_buffer[4] = 40; }
        else if (slave_index == 2) { current_cA = current_cA_test; slave_tx_buffer[4] = 40; }
        else if (slave_index == 3) { current_cA = current_cA_test; slave_tx_buffer[4] = 40; }
        else if (slave_index == 4) { current_cA = current_cA_test; slave_tx_buffer[4] = 40; }
        else if (slave_index == 5) { current_cA = current_cA_test; slave_tx_buffer[4] = 40; }
        else if (slave_index == 6) { current_cA = current_cA_test; slave_tx_buffer[4] = 40; }
        else if (slave_index == 7) { current_cA = current_cA_test; slave_tx_buffer[4] = 40; }
        else if (slave_index == 8) { current_cA = current_cA_test; slave_tx_buffer[4] = 40; }
        else if (slave_index == 9) { current_cA = current_cA_test; slave_tx_buffer[4] = 40; }
        else { current_cA = 0; slave_tx_buffer[4] = 0; }

        current_code = (uint16_t)(current_cA + DAC_CENTER_VALUE);
        slave_tx_buffer[1] = (slave_id << 3) | 0x01;
    }

    // 패킷 구성
    slave_tx_buffer[0] = 0x02;                              // STX
    // slave_tx_buffer[1] = ID+Status (위에서 설정함)
    slave_tx_buffer[2] = (uint8_t)(current_code >> 8);     // 전류 상위 바이트
    slave_tx_buffer[3] = (uint8_t)(current_code & 0xFF);   // 전류 하위 바이트
    // slave_tx_buffer[4] = 온도 (이미 설정함)

    // 체크섬 (Byte1~4 합)
    checksum = 0;
    for (i = 1; i <= 4; i++) checksum += slave_tx_buffer[i];
    slave_tx_buffer[5] = checksum & 0xFF;
    slave_tx_buffer[6] = 0x03;                              // ETX

    // SCI-D 송신 (blocking 방식)
    for (i = 0; i < 7; i++)
    {
        SCI_writeCharBlockingFIFO(SCID_BASE, slave_tx_buffer[i]);
    }

    // 다음 슬레이브로 순환
    slave_index++;
    if (slave_index >= max_slaves) slave_index = 0;
}

//--------------------------------------------------
// System Voltage → SCADA 송신 (50ms 주기)
//--------------------------------------------------
// 7바이트 패킷: [STX][master_id+Rack_Ch][Voltage_H][Voltage_L][Reserved][Checksum][ETX]
//
// RS232 프로토콜 Rev 2.1:
//   SCADA 수신 공식: (voltage_scaled - 300) / 10 = 실제 전압 (V)
//   Master 송신 공식: voltage_scaled = V_out_display * 10 + 300
//   예시: V_out_display=300.0V → voltage_scaled=3300
//         → SCADA 수신: (3300-300)/10 = 300.0V ✓
//--------------------------------------------------
void Send_System_Voltage_To_SCADA(void)
{
    uint16_t voltage_scaled = 0;

    // 테스트 모드 제어 스위치
    #define USE_TEST_DATA 0     // 0: 실제 V_out_display, 1: 테스트 값

#if (USE_TEST_DATA == 0)
    // --- 실제 데이터 사용 (Rev 2.1 프로토콜) ---
    // 음수 전압 지원: -30.0V ~ +6523.5V
    voltage_scaled = (uint16_t)(V_out_display * 10.0f + 300);
    if (voltage_scaled > DAC_MAX_VALUE) voltage_scaled = DAC_MAX_VALUE;

#else
    // --- 테스트 데이터 ---
    test_voltage += VOLTAGE_TEST_INCREMENT;                         // 호출할 때마다 +5.0V 증가
    if (test_voltage > VOLTAGE_TEST_MAX) test_voltage = 0;         // 500.0V 도달 시 0으로 리셋

    voltage_scaled = test_voltage;
#endif

    // 패킷 구성
    system_tx_buffer[0] = 0x02;     // STX
    system_tx_buffer[1] = ((master_id & 0x1F) << 3) | (Rack_Channel & 0x07);  // ID + 렉채널
    system_tx_buffer[2] = (uint8_t)(voltage_scaled >> 8);      // 전압 상위 바이트
    system_tx_buffer[3] = (uint8_t)(voltage_scaled & 0xFF);    // 전압 하위 바이트
    system_tx_buffer[4] = 0x00;                                 // Reserved

    uint8_t checksum = 0;
    for (uint8_t i = 1; i <= 4; i++)
    {
        checksum += system_tx_buffer[i];
    }
    system_tx_buffer[5] = checksum & 0xFF;
    system_tx_buffer[6] = 0x03;     // ETX

    SCI_writeCharArray(SCID_BASE, system_tx_buffer, 7);
}

//--------------------------------------------------
// SCID SCADA RX 인터럽트
//--------------------------------------------------
// SCADA 패킷 수신 (13바이트):
// [STX][CMD][Param1_H][Param1_L][Param2_H][Param2_L][Param3_H][Param3_L][CRC32_3][CRC32_2][CRC32_1][CRC32_0][ETX]
//
// ISR 역할:
//   - 13바이트 프레임 수신 (STX/ETX 검증)
//   - 에러 검사 (FE/OE/PE/BRKDT)
//   - scada_packet_ready 플래그 설정 (파싱은 Main Loop에서 수행)
//
// 처리 최적화:
//   - ISR 실행 시간: ~2μs (CRC 계산 제거로 10μs → 2μs 단축)
//   - Main Loop에서 Parse_SCADA_Command() 호출 (CRC-32 검증 + 제어 변수 업데이트)
//--------------------------------------------------
__interrupt void SCID_SCADA_Rx_ISR(void)
{
    // 1) RX 에러 상태 확인
    uint16_t rxst = HWREGH(SCID_BASE + SCI_O_RXST);
    if (rxst & (SCI_RXST_FE | SCI_RXST_OE | SCI_RXST_PE | SCI_RXST_BRKDT))
    {
        SCI_resetRxFIFO(SCID_BASE);
        SCI_clearInterruptStatus(SCID_BASE, SCI_INT_RXFF | SCI_INT_RXERR);
        scada_rx_index = 0;
        Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP8);
        return;
    }

    // 2) FIFO 카운트만큼 읽기
    uint16_t n = (uint16_t)SCI_getRxFIFOStatus(SCID_BASE);
    while (n--)
    {
        uint8_t b = (uint8_t)(SCI_readCharBlockingFIFO(SCID_BASE) & 0xFF);

        if (scada_rx_index == 0)            // 첫 바이트는 반드시 STX
        {
            if (b == 0x02) scada_rx_buffer[scada_rx_index++] = b;
            // STX 아닐 경우 무시
        }
        else
        {
            // 나머지 바이트 저장
            scada_rx_buffer[scada_rx_index++] = b;

            if (scada_rx_index == SCADA_PACKET_SIZE)  // 13바이트 수신 완료
            {
                if (scada_rx_buffer[0] == 0x02 && scada_rx_buffer[12] == 0x03)
                {
                    scada_packet_ready = 1;     // 유효 프레임 수신, Main Loop에서 파싱
                }
                scada_rx_index = 0;         // 다음 패킷 준비
            }
        }
    }

    // 3) 인터럽트 플래그 클리어
    SCI_clearInterruptStatus(SCID_BASE, SCI_INT_RXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP8);
}

// [Rev 2.1 파싱 함수 삭제됨 - 현재 프로토콜로 완전 전환]

//--------------------------------------------------
// SCADA 패킷 파싱 (제어 변수 반영)
//--------------------------------------------------
// 수신된 13바이트 패킷 해석 → CRC-32 검증 → 제어 모드별 변수 갱신
//
// 호출 위치: Main Loop (HABA_main.c)
//   - ISR에서 scada_packet_ready 플래그 설정 시 즉시 호출
//   - Main Loop 실행 주기: 100kHz ISR 종료 후 즉시 (< 1μs 지연)
//
// Command bit 구조:
//   bit[7]: 제어 모드 (0=Charge/Discharge, 1=Battery)
//   bit[6]: 준비 상태 (0=IDLE, 1=READY)
//   bit[5]: 운전 상태 (0=STOP, 1=RUN)
//   bit[4]: 병렬 상태 (0=Individual, 1=Parallel)
//
// 처리 시간: ~3-5μs (VCU2 CRC-32 하드웨어 가속)
//--------------------------------------------------
void Parse_SCADA_Command(void)
{
    // VCU2 CRC-32 계산 (Byte 1~7, Polynomial 0x04C11DB7)
    crcObj_SCADA.seedValue   = 0x00000000;
    crcObj_SCADA.nMsgBytes   = 7;               // Command + Data (7 bytes)
    crcObj_SCADA.parity      = CRC_parity_even; // Start from low byte
    crcObj_SCADA.crcResult   = 0;
    crcObj_SCADA.pMsgBuffer  = (void *)(&scada_rx_buffer[1]);  // Byte 1부터 시작

    // VCU2 하드웨어 가속 CRC-32 계산 (Polynomial 0x04C11DB7)
    CRC_run32BitPoly2(handleCRC_SCADA);

    // 수신된 CRC-32 (Big-endian, Byte 8~11)
    uint32_t received_crc = ((uint32_t)scada_rx_buffer[8] << 24) |
                            ((uint32_t)scada_rx_buffer[9] << 16) |
                            ((uint32_t)scada_rx_buffer[10] << 8) |
                            ((uint32_t)scada_rx_buffer[11]);

    // CRC 검증
    if (crcObj_SCADA.crcResult != received_crc)
    {
        scada_crc_error_cnt++;
        scada_packet_ready = 0;
        return;  // CRC 오류 시 패킷 폐기
    }

    // --- 패킷 파싱 (CRC 검증 통과) ---

    uint8_t cmd_byte = scada_rx_buffer[1];

    // Command 비트 파싱
    control_mode  = (ControlMode_t)((cmd_byte >> 7) & 0x01);  // bit[7]
    ready_state   = (cmd_byte >> 6) & 0x01;                    // bit[6]
    run_state     = (cmd_byte >> 5) & 0x01;                    // bit[5]
    parallel_mode = (cmd_byte >> 4) & 0x01;                    // bit[4]

    // IDLE+RUN 비정상 조합 검증 (프로토콜 라인 199-209)
    if (run_state && !ready_state)
    {
        // IDLE+RUN 무시, 이전 상태 유지
        scada_packet_ready = 0;
        return;
    }

    // 제어 모드별 데이터 파싱 (int16, ÷10 스케일)
    if (control_mode == CONTROL_MODE_CHARGE_DISCHARGE)
    {
        // Charge/Discharge Mode (bit[7]=0)
        int16_t v_max_raw = (int16_t)((scada_rx_buffer[2] << 8) | scada_rx_buffer[3]);
        int16_t v_min_raw = (int16_t)((scada_rx_buffer[4] << 8) | scada_rx_buffer[5]);
        int16_t i_cmd_raw = (int16_t)((scada_rx_buffer[6] << 8) | scada_rx_buffer[7]);

        V_max_cmd = v_max_raw / 10.0f;  // V 단위
        V_min_cmd = v_min_raw / 10.0f;  // V 단위
        I_out_ref = i_cmd_raw / 10.0f;  // A 단위
    }
    else  // CONTROL_MODE_BATTERY
    {
        // Battery Mode (bit[7]=1)
        int16_t v_cmd_raw   = (int16_t)((scada_rx_buffer[2] << 8) | scada_rx_buffer[3]);
        int16_t i_max_raw   = (int16_t)((scada_rx_buffer[4] << 8) | scada_rx_buffer[5]);
        int16_t i_min_raw   = (int16_t)((scada_rx_buffer[6] << 8) | scada_rx_buffer[7]);

        V_cmd     = v_cmd_raw / 10.0f;  // 목표 전압 (CV 제어)
        I_max_cmd = i_max_raw / 10.0f;  // 최대 전류 제한
        I_min_cmd = i_min_raw / 10.0f;  // 최소 전류 제한

        // TODO: Battery Mode CV 제어 로직 구현 (CLA Task 3 사용)
    }

    // 기존 변수 매핑 (호환성)
    start_stop = run_state;
    operation_mode = parallel_mode ? MODE_PARALLEL : MODE_INDIVIDUAL;

    scada_packet_ready = 0;  // 다음 패킷 대기
}

//==================================================
// [10] 디버그 함수 (조건부 컴파일)
//==================================================

//--------------------------------------------------
// 클럭 상태 디버그
//--------------------------------------------------
#if _DEBUG_CLK_STATUS_ENABLE_
void Debug_CLK_Status(void)
{
    // 클럭 정보
    debug_sysclk_freq         = SysCtl_getClock(DEVICE_OSCSRC_FREQ);
    debug_device_sysclk_freq  = DEVICE_SYSCLK_FREQ;
    SPIclk                    = SysCtl_getClock(SYSCTL_PERIPH_CLK_SPIA);
    epwmclk                   = debug_sysclk_freq / (1 << ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV);
    lspclk                    = SysCtl_getLowSpeedClock(sysclk);
    Epwm1CLK                  = SysCtl_getClock(SYSCTL_PERIPH_CLK_EPWM1);
    Epwm3CLK                  = SysCtl_getClock(SYSCTL_PERIPH_CLK_EPWM3);

    // CAN 클럭 소스 확인
    debug_can_clock_source =
        (HWREGH(CLKCFG_BASE + SYSCTL_O_CLKSRCCTL2) & SYSCTL_CLKSRCCTL2_CANABCLKSEL_M) >>
        SYSCTL_CLKSRCCTL2_CANABCLKSEL_S;
}
#endif

//--------------------------------------------------
// CAN 상태 디버그
//--------------------------------------------------
#if _DEBUG_CAN_STATUS_ENABLE_
void Debug_CAN_Status(void)
{
    // CAN 에러 상태 및 에러 카운트 레지스터 읽기
    uint16_t errorStatus = HWREGH(CANA_BASE + CAN_O_ES);
    uint16_t errorCount  = HWREGH(CANA_BASE + CAN_O_ERRC);

    // 에러 카운트 추출
    uint16_t txErrCnt = errorCount & 0x00FF;
    uint16_t rxErrCnt = (errorCount >> 8) & 0x00FF;

    // 에러 상태 플래그 추출
    bool can_bus_off  = (errorStatus & CAN_ES_BOFF)  != 0;
    bool can_warn     = (errorStatus & CAN_ES_EWARN) != 0;
    bool can_passive  = (errorStatus & CAN_ES_EPASS) != 0;

    // 디버깅용 LED 표시
    if (can_bus_off)  GPIO_togglePin(LED31);
    if (can_warn)     GPIO_togglePin(LED32);
    if (can_passive)  GPIO_togglePin(LED33);

    // 디버깅 전역 변수 저장
    debug_can_bus_off  = can_bus_off;
    debug_can_warn     = can_warn;
    debug_can_passive  = can_passive;
    debug_tx_err_cnt = txErrCnt;
    debug_rx_err_cnt = rxErrCnt;
    debug_can_es       = errorStatus;

    // 비트 타이밍 파라미터 계산
    debug_btr_value = HWREG(CANA_BASE + CAN_O_BTR);

    uint16_t prescaler = (debug_btr_value & CAN_BTR_BRP_M);
    uint16_t brpe      = (debug_btr_value & CAN_BTR_BRPE_M) >> CAN_BTR_BRPE_S;
    uint16_t tseg1     = (debug_btr_value & CAN_BTR_TSEG1_M) >> CAN_BTR_TSEG1_S;
    uint16_t tseg2     = (debug_btr_value & CAN_BTR_TSEG2_M) >> CAN_BTR_TSEG2_S;

    debug_actual_prescaler     = prescaler + (brpe * 64) + 1;
    debug_tseg1_tq             = tseg1 + 1;
    debug_tseg2_tq             = tseg2 + 1;
    debug_total_tq             = 1 + debug_tseg1_tq + debug_tseg2_tq;
    debug_calculated_bitrate   = debug_sysclk_freq / (debug_actual_prescaler * debug_total_tq);
}
#endif

//--------------------------------------------------
// SCI 상태 디버그
//--------------------------------------------------
#if _DEBUG_SCI_STATUS_ENABLE_
void Debug_SCI_Status(void)
{
    uint16_t rxStatus = SCI_getRxStatus(SCIA_BASE);

    // 에러 상태 읽기
    bool isOverrunError = (rxStatus & SCI_RXST_RXERROR) != 0;
    bool isFramingError = (rxStatus & SCI_RXST_FE) != 0;
    bool isParityError  = (rxStatus & SCI_RXST_PE) != 0;
    bool isBreakDetected = (rxStatus & SCI_RXST_BRKDT) != 0;

    // 송수신 준비 상태
    bool txReady = (HWREGH(SCIA_BASE + SCI_O_CTL1) & SCI_CTL2_TXRDY) != 0;
    bool rxReady = (HWREGH(SCIA_BASE + SCI_O_CTL1) & SCI_RXST_RXRDY) != 0;

    // LED 디버깅 출력
    if (isOverrunError)   GPIO_togglePin(LED31);
    if (isFramingError)   GPIO_togglePin(LED32);
    if (isParityError)    GPIO_togglePin(LED33);

    // 디버깅 변수 기록
    debug_sci_rx_error   = isOverrunError;
    debug_sci_frame_err  = isFramingError;
    debug_sci_parity_err = isParityError;
    debug_sci_break      = isBreakDetected;
    debug_sci_tx_ready   = txReady;
    debug_sci_rx_ready   = rxReady;
}
#endif

//--------------------------------------------------
// SPI 상태 디버그
//--------------------------------------------------
#if _DEBUG_SPI_STATUS_ENABLE_
void Debug_SPI_Status(void)
{
    uint16_t sts   = HWREGH(SPIA_BASE + SPI_O_STS);
    uint16_t fftx  = HWREGH(SPIA_BASE + SPI_O_FFTX);
    uint16_t ffrx  = HWREGH(SPIA_BASE + SPI_O_FFRX);

    // 상태 추출
    bool tx_buf_full     = (sts & SPI_STS_BUFFULL_FLAG) != 0;
    bool int_flag        = (sts & SPI_STS_INT_FLAG) != 0;
    bool overrun_error   = (sts & SPI_STS_OVERRUN_FLAG) != 0;

    bool tx_fifo_ready   = ((fftx & SPI_FFTX_TXFFST_M) >> SPI_FFTX_TXFFST_S) < 2;
    bool rx_fifo_data    = ((ffrx & SPI_FFRX_RXFFST_M) >> SPI_FFRX_RXFFST_S) > 0;
    bool rx_fifo_ovf     = (ffrx & SPI_FFRX_RXFFOVF) != 0;

    // 디버그 변수 설정
    debug_spi_tx_full     = tx_buf_full;
    debug_spi_int         = int_flag;
    debug_spi_overrun     = overrun_error;
    debug_spi_tx_ready    = tx_fifo_ready;
    debug_spi_rx_hasdata  = rx_fifo_data;
    debug_spi_rx_overflow = rx_fifo_ovf;
}
#endif

//==================================================
// End of HABA_control.c
//==================================================
