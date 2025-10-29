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

    // 제어 모드별 CLA Task 선택 (SCADA 명령 기반)
    if (scada_cmd.control_mode == CONTROL_MODE_CHARGE_DISCHARGE)
    {
        // Charge/Discharge 모드: Task 1 & 2 실행
        Cla1ForceTask1();       // pi_charge 제어기 (충전 모드, V_max_cmd 기준)
        Cla1ForceTask2();       // pi_discharge 제어기 (방전 모드, V_min_cmd 기준)
    }
    else if (scada_cmd.control_mode == CONTROL_MODE_BATTERY)
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

    // --- 1. 제어 모드별 최종 전류 지령 계산 (SCADA 명령 기반) ---
    if (scada_cmd.control_mode == CONTROL_MODE_BATTERY)
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

        // 전류 한계값 적용 (SCADA 명령)
        if      (I_cmd_PI_limited >  scada_cmd.I_max_cmd)  I_cmd_PI_limited =  scada_cmd.I_max_cmd;
        else if (I_cmd_PI_limited <  scada_cmd.I_min_cmd)  I_cmd_PI_limited =  scada_cmd.I_min_cmd;

        // 시스템 레벨 전류 제한
        if      (I_cmd_PI_limited >  current_limit)        I_cmd_PI_limited =  current_limit;
        else if (I_cmd_PI_limited < -current_limit)        I_cmd_PI_limited = -current_limit;
    }
    else if (scada_cmd.control_mode == CONTROL_MODE_CHARGE_DISCHARGE)
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
        uint8_t   active_count;  // 실제 연결된 슬레이브 개수

        // Active Slave List 개수 확인 (동적 감지)
        active_count = active_slave_list.count;

        // Fallback: 감지된 슬레이브 없으면 기본값 사용
        if (active_count == 0)
            active_count = SLAVES_PER_CHANNEL;  // 기본 6개

        if (operation_mode == MODE_INDIVIDUAL)
        {
            // 개별 모드: 전체 전류를 실제 연결된 슬레이브로 동적 분배
            I_cmd_PI_limited_per_channel = I_cmd_PI_limited;
            I_per_slave = I_cmd_PI_limited_per_channel / active_count;
        }
        else if (operation_mode == MODE_PARALLEL && IS_CH1)
        {
            // 병렬 모드 CH1: 가중 분배 (CH1, CH2 슬레이브 개수 고려)
            uint8_t ch1_count = active_count;                       // CH1 슬레이브 개수
            uint8_t ch2_count = ch2_slave_count;                    // CH2 슬레이브 개수 (M2에서 수신)

            // Fallback: CH2 개수 정보 없으면 CH1과 동일하다고 가정
            if (ch2_count == 0)
                ch2_count = ch1_count;

            uint8_t total_slaves = ch1_count + ch2_count;          // 전체 슬레이브 개수

            // 가중 분배: CH1 비율에 따른 전류 할당
            float32_t ch1_ratio = (float32_t)ch1_count / total_slaves;
            I_cmd_PI_limited_per_channel = I_cmd_PI_limited * ch1_ratio;
            I_per_slave = I_cmd_PI_limited_per_channel / ch1_count;

            // 예시: CH1=6개, CH2=4개, 시스템 전류=480A
            //   ch1_ratio = 6/(6+4) = 0.6
            //   CH1 전류 = 480 × 0.6 = 288A
            //   CH1 모듈당 = 288A ÷ 6 = 48A ✅
            //   CH2 전류 = 480 × 0.4 = 192A (M1이 M2에 전송)
            //   CH2 모듈당 = 192A ÷ 4 = 48A ✅ (균등 분배!)
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
        I_cmd_scada        = 0.0f;
        
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
 *          - 과온 감지: NTC0 또는 NTC1 ≥ 85°C
 *          - 비상 정지 스위치 상태 반영
 *
 * @details 래칭 메커니즘:
 *          - 고장 발생 시 fault_latched = true 설정
 *          - run = 0 강제 설정 → 릴레이 자동 차단
 *          - IDLE 상태 전환 시 자동 리셋
 *          - 간헐적 고장에도 안정적 보호
 *
 * @note 호출 위치: Check_System_Safety() (Phase 3, 20kHz)
 * @note 고장 발생 시: 모든 운전 LED OFF, 고장 LED ON
 *
 * @warning 고장 플래그는 래치(Latch) 방식 - IDLE 상태 전환 시 자동 해제
 *          (Fault 발생 → STOP 유지 → SCADA IDLE 명령 → 고장 리셋)
 */
void Check_Fault(void)
{
    // 고장 조건 감지
    bool is_overvoltage  = (V_out_display >= OVER_VOLTAGE);
    bool is_overcurrent  = (fabs(I_out_avg) >= OVER_CURRENT);
    bool is_overtemp     = (NTC_0_temp >= OVER_TEMP || NTC_1_temp >= OVER_TEMP);
    bool is_fault_active = (is_overvoltage || is_overcurrent || is_overtemp);

    // 고장 플래그 설정 (master_status 구조체에 저장)
    if (is_overvoltage)  master_status.over_voltage = FAULT_FLAG_ACTIVE;
    if (is_overcurrent)  master_status.over_current = FAULT_FLAG_ACTIVE;
    if (is_overtemp)     master_status.over_temp    = FAULT_FLAG_ACTIVE;

    // 고장 래칭 메커니즘
    if (is_fault_active && !master_status.fault_latched)
    {
        master_status.fault_latched = true;  // 고장 래치 설정
        run = 0;                             // 즉시 운전 정지 (릴레이 자동 차단)
    }

    // 래치된 고장이 있으면 run 강제 0 유지
    if (master_status.fault_latched)
    {
        run = 0;
    }
    else
    {
        // 고장 없을 때만 비상 정지 스위치 상태 반영
        run = run_switch ? 1 : 0;
    }

    // 레거시 변수 동기화 (하위 호환성)
    over_voltage_flag = master_status.over_voltage;
    over_current_flag = master_status.over_current;
    over_temp_flag    = master_status.over_temp;
    fault_latched     = master_status.fault_latched;

    // LED 상태 표시
    if (is_fault_active || master_status.fault_latched)
    {
        // 고장 발생 또는 래치됨: 운전 정지 및 고장 표시
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

    if      (I_cmd_scada >  current_limit) I_cmd =  current_limit;
    else if (I_cmd_scada < -current_limit) I_cmd = -current_limit;
    else                                 I_cmd =  I_cmd_scada;
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

        // Rev 5: Active Slave List 업데이트
        Update_Active_Slave_List((uint8_t)(slv_idx + 1));  // slv_idx는 0-based, slave_id는 1-based

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
// Master-to-Master CRC-32 계산 함수 (VCU2 하드웨어 가속)
//--------------------------------------------------
// @brief    VCU2 CRC 모듈을 사용한 CRC-32 계산
// @param    data      데이터 버퍼 포인터
// @param    length    데이터 길이 (바이트)
// @return   CRC-32 값 (Little-endian)
// @note     실행 시간: ~0.5μs (하드웨어 가속)
//--------------------------------------------------
static inline uint32_t Calculate_MM_CRC32(uint8_t* data, uint16_t length)
{
    crcObj_MM.seedValue  = 0x00000000;
    crcObj_MM.nMsgBytes  = length;
    crcObj_MM.parity     = CRC_parity_even;
    crcObj_MM.crcResult  = 0;
    crcObj_MM.pMsgBuffer = (void*)data;

    CRC_run32BitPoly2(handleCRC_MM);  // VCU2 하드웨어 가속 CRC-32 계산

    return crcObj_MM.crcResult;
}

//--------------------------------------------------
// SCIA RS485 전류 지령 송신 (Master-to-Master, Rev 1.0)
//--------------------------------------------------
// @brief    M1 → M2 전류 지령 전송 (병렬 모드)
// @param    current   전류 지령 (DAC 코드, 0~65535)
// @note     프레임: [STX][CMD][Current_L][Current_H][CRC32(4)][ETX] (9바이트)
// @note     실행 시간: ~2.5μs (CRC-32 계산 + 9바이트 송신)
//--------------------------------------------------
#pragma CODE_SECTION(Send_RS485_MM_Current, ".TI.ramfunc");
void Send_RS485_MM_Current(uint16_t current)
{
    if (IS_CH1)     // CH1 마스터만 송신
    {
        uint8_t tx_buf[MM_FRAME_SIZE_CURRENT];

        // 프레임 구성
        tx_buf[0] = STX;                        // 0x02
        tx_buf[1] = MM_CMD_CURRENT;             // 0x01
        tx_buf[2] = current & 0xFF;             // Current_L
        tx_buf[3] = (current >> 8) & 0xFF;      // Current_H

        // CRC-32 계산 (Byte 1~3: CMD + Current)
        uint32_t crc = Calculate_MM_CRC32(&tx_buf[1], 3);

        tx_buf[4] = (crc >> 0) & 0xFF;          // CRC32_0
        tx_buf[5] = (crc >> 8) & 0xFF;          // CRC32_1
        tx_buf[6] = (crc >> 16) & 0xFF;         // CRC32_2
        tx_buf[7] = (crc >> 24) & 0xFF;         // CRC32_3
        tx_buf[8] = ETX;                        // 0x03

        // SCIA TX FIFO로 전송
        for (int i = 0; i < MM_FRAME_SIZE_CURRENT; i++)
        {
            HWREGH(SCIA_BASE + SCI_O_TXBUF) = tx_buf[i];
        }

        mm_statistics.tx_count++;
    }
}

//--------------------------------------------------
// SCIA RS485 슬레이브 개수 송신 (Master-to-Master, Rev 1.0)
//--------------------------------------------------
// @brief    M2 → M1 슬레이브 개수 전송 (병렬 모드)
// @param    count     슬레이브 개수 (0~6)
// @note     프레임: [STX][CMD][Count][CRC32(4)][ETX] (8바이트)
// @note     호출 위치: Main Loop (100ms 주기)
//--------------------------------------------------
void Send_RS485_MM_SlaveCount(uint8_t count)
{
    if (IS_CH2)     // CH2 마스터만 송신
    {
        uint8_t tx_buf[MM_FRAME_SIZE_SLAVE_COUNT];

        // 프레임 구성
        tx_buf[0] = STX;                        // 0x02
        tx_buf[1] = MM_CMD_SLAVE_COUNT;         // 0x02
        tx_buf[2] = count;                      // Slave Count

        // CRC-32 계산 (Byte 1~2: CMD + Count)
        uint32_t crc = Calculate_MM_CRC32(&tx_buf[1], 2);

        tx_buf[3] = (crc >> 0) & 0xFF;          // CRC32_0
        tx_buf[4] = (crc >> 8) & 0xFF;          // CRC32_1
        tx_buf[5] = (crc >> 16) & 0xFF;         // CRC32_2
        tx_buf[6] = (crc >> 24) & 0xFF;         // CRC32_3
        tx_buf[7] = ETX;                        // 0x03

        // SCIA TX FIFO로 전송
        for (int i = 0; i < MM_FRAME_SIZE_SLAVE_COUNT; i++)
        {
            HWREGH(SCIA_BASE + SCI_O_TXBUF) = tx_buf[i];
        }

        mm_statistics.tx_count++;
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
        switch (master_status.sequence_step)
        {
            case SEQ_STEP_IDLE:     // Precharge 단계 (SCADA cmd_ready=1 명령 대기)
                if (scada_cmd.cmd_ready == 1)   // SCADA가 READY 명령 (Precharge 시작 지시)
                {
                    V_max_cmd = V_batt_display;
                    V_min_cmd = 0;

                    // Precharge 전류: 각 모듈당 2A
                    // 병렬 모드: CH1+CH2 슬레이브 총 개수, 개별 모드: CH1 슬레이브 개수만
                    uint8_t precharge_slave_count = active_slave_list.count;

                    if (operation_mode == MODE_PARALLEL && IS_CH1)
                    {
                        // 병렬 모드: CH1 + CH2 슬레이브 총 개수
                        uint8_t ch2_count = (ch2_slave_count > 0) ? ch2_slave_count : active_slave_list.count;
                        precharge_slave_count = active_slave_list.count + ch2_count;
                    }

                    // Fallback: 슬레이브 개수 정보 없으면 기본값
                    if (precharge_slave_count == 0)
                    {
                        precharge_slave_count = (operation_mode == MODE_PARALLEL) ? 12 : 6;
                    }

                    I_cmd_scada = 2.0f * precharge_slave_count;  // 모듈당 2A × 슬레이브 개수
                    // 예시: 개별 모드 6개 → 12A, 병렬 모드 12개 → 24A

                    // 프리차지 완료 조건 판정: V_out ≈ V_batt (±2V 이내)
                    float32_t voltage_diff = V_out_display - V_batt_display;
                    bool is_precharge_complete = (fabs(voltage_diff) < PRECHARGE_VOLTAGE_DIFF_OK);

                    if (is_precharge_complete)
                    {
                        master_status.precharge_ok = 1;     // Precharge 완료 (SCADA 피드백)
                        master_status.ready = 1;            // READY 상태 설정
                        master_status.sequence_step = SEQ_STEP_PRECHARGE_DONE;

                        // 레거시 변수 동기화
                        pre_chg_ok = 1;
                        sequence_step = SEQ_STEP_PRECHARGE_DONE;
                    }
                    else
                    {
                        master_status.precharge_ok = 0;     // Precharge 진행 중
                        master_status.ready = 0;            // IDLE 상태

                        // 레거시 변수 동기화
                        pre_chg_ok = 0;
                    }
                }
                else    // SCADA cmd_ready=0 (IDLE 유지)
                {
                    V_max_cmd = 0;
                    V_min_cmd = 0;
                    I_cmd_scada = 0;
                    master_status.precharge_ok = 0;
                    master_status.ready = 0;

                    // 레거시 변수 동기화
                    pre_chg_ok = 0;
                    // 릴레이 제어: Phase 3에서 sequence_step 기반 자동 처리
                }
            break;

            case SEQ_STEP_PRECHARGE_DONE:    // 프리차지 완료 후 전류를 0으로 변경 (READY+STOP)
                V_max_cmd = V_batt_display;
                V_min_cmd = 0;
                I_cmd_scada = 0;
                pre_chg_cnt++;

                master_status.ready = 1;    // READY 상태 유지
                master_status.running = 0;  // 아직 RUN 아님

                // 1초 대기 후 정상 운전으로 전환
                bool is_delay_complete = (pre_chg_cnt >= SEQ_PRECHARGE_DELAY_COUNT);
                if (is_delay_complete)
                {
                    pre_chg_cnt = SEQ_PRECHARGE_DELAY_COUNT;  // 카운터 포화 방지
                    // 메인 릴레이 ON: Phase 3에서 sequence_step 기반 자동 처리
                    master_status.sequence_step = SEQ_STEP_NORMAL_RUN;

                    // 레거시 변수 동기화
                    sequence_step = SEQ_STEP_NORMAL_RUN;
                }
            break;

            case SEQ_STEP_NORMAL_RUN:    // 정상 운전 (UI 전류 지령에 따라 동작, READY+RUN)
                master_status.ready = 1;    // READY 상태 유지
                master_status.running = 1;  // RUN 상태

                // 제어 모드별 데이터 처리
                if (scada_cmd.control_mode == CONTROL_MODE_CHARGE_DISCHARGE)
                {
                    // Charge/Discharge 모드: scada_cmd.V_max_cmd, V_min_cmd, I_cmd는
                    // Parse_SCADA_Command()에서 이미 설정됨 (추가 처리 불필요)
                }
                else if (scada_cmd.control_mode == CONTROL_MODE_BATTERY)
                {
                    // Battery 모드: scada_cmd.V_cmd, I_max_cmd, I_min_cmd 사용
                    V_max_cmd = scada_cmd.V_cmd;      // CV 제어 목표 전압
                    V_min_cmd = 0;
                    I_cmd_scada = 0;                  // Battery 모드에서는 전류 지령 사용 안 함
                }
                else
                {
                    // 예상치 못한 control_mode: 안전 상태
                    V_max_cmd = 0;
                    V_min_cmd = 0;
                    I_cmd_scada = 0;
                }

                // SCADA STOP 명령 시 (cmd_run=0)
                if (scada_cmd.cmd_run == 0)
                {
                    master_status.sequence_step = SEQ_STEP_IDLE;
                    master_status.ready = 0;
                    master_status.running = 0;
                    master_status.precharge_ok = 0;

                    // 레거시 변수 동기화
                    sequence_step = SEQ_STEP_IDLE;
                    pre_chg_cnt = 0;    // 카운터 리셋
                }
            break;
        }
    }
    else    // run == 0 (비상 정지 또는 고장)
    {
        V_max_cmd = 0;
        V_min_cmd = 0;
        I_cmd_scada = 0;
        start_stop = STOP;

        // 마스터 상태 업데이트: 정지 상태
        master_status.running = 0;  // RUN 정지

        // IDLE 상태에서는 프리차지 리셋 (고장 자동 해제)
        if (master_status.sequence_step == SEQ_STEP_IDLE)
        {
            // IDLE 상태에서 고장 래치 자동 해제
            master_status.fault_latched   = false;
            master_status.over_voltage    = 0;
            master_status.over_current    = 0;
            master_status.over_temp       = 0;
            master_status.ready           = 0;
            master_status.precharge_ok    = 0;

            // 레거시 변수 동기화
            fault_latched      = false;
            over_voltage_flag  = 0;
            over_current_flag  = 0;
            over_temp_flag     = 0;
            pre_chg_cnt = 0;    // 카운터 리셋
        }
        // IDLE이 아닌 경우 (READY, RUN) → STOP 상태 유지 (프리차지 보존)
        // sequence_step는 변경하지 않음

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
// SCIA RS485 Master-to-Master RX 인터럽트 (Rev 1.0)
//--------------------------------------------------
// @brief    M1 ↔ M2 양방향 통신 수신
//           - M2: M1에서 전류 지령 수신 (MM_CMD_CURRENT)
//           - M1: M2에서 슬레이브 개수 수신 (MM_CMD_SLAVE_COUNT)
// @note     State Machine 방식 프레임 동기화
//           STX + ETX + CRC-32 3중 검증
//           급격한 변화 감지 (±30% 이상 거부)
//--------------------------------------------------
__interrupt void SCIA_RS485_MM_Rx_ISR(void)
{
    uint8_t rx = SCI_readCharNonBlocking(SCIA_BASE);

    // State Machine 방식 프레임 동기화
    switch (mm_rx_state)
    {
        case MM_RX_WAIT_STX:
            if (rx == STX)  // 0x02
            {
                mm_rx_buffer[0] = rx;
                mm_rx_index = 1;
                mm_rx_state = MM_RX_RECEIVING;
            }
            break;

        case MM_RX_RECEIVING:
            mm_rx_buffer[mm_rx_index++] = rx;

            // CMD 바이트 확인 후 프레임 크기 결정
            if (mm_rx_index == 2)  // CMD 수신 완료
            {
                uint8_t cmd = mm_rx_buffer[1];

                // 유효하지 않은 CMD → 리셋
                if (cmd != MM_CMD_CURRENT && cmd != MM_CMD_SLAVE_COUNT)
                {
                    mm_rx_state = MM_RX_WAIT_STX;
                    mm_rx_index = 0;
                    mm_statistics.frame_error_count++;
                    break;
                }
            }

            // 프레임 크기별 수신 완료 체크
            if ((mm_rx_buffer[1] == MM_CMD_CURRENT && mm_rx_index >= MM_FRAME_SIZE_CURRENT) ||
                (mm_rx_buffer[1] == MM_CMD_SLAVE_COUNT && mm_rx_index >= MM_FRAME_SIZE_SLAVE_COUNT))
            {
                uint8_t frame_size = (mm_rx_buffer[1] == MM_CMD_CURRENT) ? MM_FRAME_SIZE_CURRENT : MM_FRAME_SIZE_SLAVE_COUNT;

                // STX + ETX 동시 체크
                if (mm_rx_buffer[0] == STX && mm_rx_buffer[frame_size - 1] == ETX)
                {
                    // CRC-32 검증
                    uint8_t data_len = (mm_rx_buffer[1] == MM_CMD_CURRENT) ? 3 : 2;  // CMD + Data
                    uint32_t calc_crc = Calculate_MM_CRC32(&mm_rx_buffer[1], data_len);
                    uint32_t recv_crc = ((uint32_t)mm_rx_buffer[frame_size - 5] << 0)  |
                                        ((uint32_t)mm_rx_buffer[frame_size - 4] << 8)  |
                                        ((uint32_t)mm_rx_buffer[frame_size - 3] << 16) |
                                        ((uint32_t)mm_rx_buffer[frame_size - 2] << 24);

                    if (calc_crc == recv_crc)
                    {
                        // 정상 프레임: 명령 타입별 처리
                        if (mm_rx_buffer[1] == MM_CMD_CURRENT && IS_CH2)
                        {
                            // M2: M1에서 전류 지령 수신
                            uint16_t current = mm_rx_buffer[2] | ((uint16_t)mm_rx_buffer[3] << 8);

                            // 급격한 변화 감지 (±30% 이상 변화 시 거부)
                            if (I_cmd_from_master_prev == 0 ||
                                (current >= I_cmd_from_master_prev * 0.7f && current <= I_cmd_from_master_prev * 1.3f))
                            {
                                I_cmd_from_master = current;
                                I_cmd_from_master_prev = current;
                                mm_watchdog.last_rx_time_ms = cnt_1ms;  // 타임스탬프 업데이트
                                mm_statistics.rx_count++;
                            }
                            else
                            {
                                mm_statistics.spike_reject_count++;  // 급격한 변화 거부
                            }
                        }
                        else if (mm_rx_buffer[1] == MM_CMD_SLAVE_COUNT && IS_CH1)
                        {
                            // M1: M2에서 슬레이브 개수 수신
                            ch2_slave_count = mm_rx_buffer[2];
                            mm_statistics.rx_count++;
                        }
                    }
                    else
                    {
                        mm_statistics.crc_error_count++;  // CRC 에러
                    }
                }
                else
                {
                    mm_statistics.frame_error_count++;  // 프레임 에러 (STX/ETX 불일치)
                }

                // 다음 프레임 대기
                mm_rx_state = MM_RX_WAIT_STX;
                mm_rx_index = 0;
            }
            break;
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

//--------------------------------------------------
// SCID SCADA RX 인터럽트 (Rev 5)
//--------------------------------------------------
// SCADA 패킷 수신 (16바이트):
// [STX][CMD][Param1_H][Param1_L][Param2_H][Param2_L][Param3_H][Param3_L]
// [Reserved_1][Reserved_2][Reserved_3]
// [CRC32_3][CRC32_2][CRC32_1][CRC32_0][ETX]
//
// ISR 역할:
//   - 16바이트 프레임 수신 (STX/ETX 검증)
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
            // 나머지 바이트 저장 (Rev 5: 16 bytes)
            scada_rx_buffer[scada_rx_index++] = b;

            if (scada_rx_index == SCADA_PACKET_SIZE)  // 16바이트 수신 완료
            {
                if (scada_rx_buffer[0] == 0x02 && scada_rx_buffer[15] == 0x03)
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
// [11] Rev 5 프로토콜 함수 구현
//==================================================

//--------------------------------------------------
// Active Slave List 업데이트
//--------------------------------------------------
// CAN 수신 시 호출하여 슬레이브 ID를 Active List에 추가
//
// 입력:
//   - slave_id: 수신된 슬레이브 ID (1~15)
//
// 동작:
//   - ID가 이미 리스트에 있으면 타임스탬프만 업데이트
//   - 없으면 리스트에 추가 (최대 6개)
//   - 리스트 초과 시 경고 (고장 비트 설정)
//--------------------------------------------------
void Update_Active_Slave_List(uint8_t slave_id)
{
    uint8_t i;
    uint8_t found = 0;

    // 유효성 검사
    if (slave_id == 0 || slave_id > 15)
        return;

    // 이미 리스트에 있는지 확인
    for (i = 0; i < active_slave_list.count; i++)
    {
        if (active_slave_list.slave_ids[i] == slave_id)
        {
            found = 1;
            break;
        }
    }

    // 새 슬레이브 추가
    if (!found)
    {
        if (active_slave_list.count < SCADA_MAX_SLAVES)
        {
            active_slave_list.slave_ids[active_slave_list.count] = slave_id;
            active_slave_list.count++;
        }
        else
        {
            // 리스트 초과: Warning 비트 설정
            slave_status_bitmap.warning_bitmap |= (1 << (slave_id - 1));
        }
    }

    // 타임스탬프 업데이트 (1ms 카운터 사용)
    active_slave_list.last_updated_ms = cnt_1ms;
}

//--------------------------------------------------
// 슬레이브 배치 패킷 송신 (Rev 5)
//--------------------------------------------------
// Active Slave List 기반으로 3개씩 묶어 송신
//
// 호출 주기: 10ms (Main Loop)
// 동작:
//   - batch_index=0: 슬레이브 1-3 송신
//   - batch_index=1: 슬레이브 4-6 송신
//   - 토글 방식으로 2패킷 번갈아 전송
//
// 성능:
//   - 전송 시간: 10ms × 2패킷 = 20ms (Rev 4는 150ms)
//   - 전송 효율: 87% 개선
//--------------------------------------------------
void Send_Slave_Batch_To_SCADA(void)
{
    uint8_t i;
    uint8_t start_idx = batch_index * SCADA_SLAVES_PER_PACKET;
    uint8_t end_idx = start_idx + SCADA_SLAVES_PER_PACKET;
    Slave_Batch_Packet_t* packet = &slave_batch_packets[batch_index];

    // 패킷 초기화
    memset(packet, 0, sizeof(Slave_Batch_Packet_t));
    packet->stx = 0x02;
    packet->etx = 0x03;

    // 슬레이브 3개 데이터 채우기
    uint8_t slave_count = 0;
    for (i = start_idx; i < end_idx && i < active_slave_list.count; i++)
    {
        uint8_t slave_id = active_slave_list.slave_ids[i];
        uint8_t slave_idx = slave_id - 1;  // 배열 인덱스 (0~14)

        // 전류 (0.01A 단위, Center=0)
        int16_t current_cA = (int16_t)(I_out_slave[slave_idx] * 100.0f);
        if (current_cA > 3200) current_cA = 3200;    // +32.00A 상한
        if (current_cA < -3200) current_cA = -3200;  // -32.00A 하한

        // 온도 (0.5℃ 단위)
        uint8_t temp_data = (uint8_t)(temp_slave_raw[slave_idx] * 2.0f);
        if (temp_data > 255) temp_data = 255;

        // Status 비트 (bit[2:0])
        uint8_t status = 0;
        status |= (DAB_ok_slave[slave_idx] & 0x01);                                        // bit[0]: DAB_OK
        status |= ((slave_status_bitmap.fault_bitmap >> slave_idx) & 0x01) << 1;           // bit[1]: Fault
        status |= ((slave_status_bitmap.warning_bitmap >> slave_idx) & 0x01) << 2;         // bit[2]: Warning

        // 패킷에 데이터 저장 (Big-endian)
        switch (slave_count) {
            case 0:  // Slave 1
                packet->id1_status = (slave_id << 3) | status;
                packet->current1 = __byte((uint32_t)current_cA, 1) | ((uint16_t)__byte((uint32_t)current_cA, 0) << 8);  // Big-endian
                packet->temp1 = temp_data;
                break;
            case 1:  // Slave 2
                packet->id2_status = (slave_id << 3) | status;
                packet->current2 = __byte((uint32_t)current_cA, 1) | ((uint16_t)__byte((uint32_t)current_cA, 0) << 8);  // Big-endian
                packet->temp2 = temp_data;
                break;
            case 2:  // Slave 3
                packet->id3_status = (slave_id << 3) | status;
                packet->current3 = __byte((uint32_t)current_cA, 1) | ((uint16_t)__byte((uint32_t)current_cA, 0) << 8);  // Big-endian
                packet->temp3 = temp_data;
                break;
        }

        slave_count++;
    }

    // Checksum 계산 (Byte 1~13)
    uint8_t checksum = 0;
    uint8_t* buf = (uint8_t*)packet;
    for (i = 1; i <= 13; i++)
    {
        checksum += buf[i];
    }
    packet->checksum = checksum;

    // SCI-D 송신 (blocking)
    SCI_writeCharArray(SCID_BASE, (uint16_t*)packet, SCADA_PACKET_SIZE);

    // 다음 배치로 토글
    batch_index++;
    if (batch_index >= 2 || (start_idx + SCADA_SLAVES_PER_PACKET) >= active_slave_list.count)
    {
        batch_index = 0;  // 배치 순환
    }
}

//--------------------------------------------------
// System Voltage → SCADA 송신 (Rev 5, 16 bytes)
//--------------------------------------------------
void Send_System_Voltage_To_SCADA(void)
{
    uint8_t i;
    System_Voltage_Packet_t* packet = &system_voltage_packet;

    // 패킷 초기화
    memset(packet, 0, sizeof(System_Voltage_Packet_t));
    packet->stx = 0x02;
    packet->etx = 0x03;

    // Master ID + Rack Channel
    packet->id_channel = ((master_id & 0x1F) << 3) | (Rack_Channel & 0x07);

    // 전압 (0.1V 단위, Big-endian)
    int16_t voltage_dV = (int16_t)(V_out_display * 10.0f);
    packet->voltage = __byte((uint32_t)voltage_dV, 1) | ((uint16_t)__byte((uint32_t)voltage_dV, 0) << 8);  // Big-endian

    // Checksum 계산 (Byte 1~13)
    uint8_t checksum = 0;
    uint8_t* buf = (uint8_t*)packet;
    for (i = 1; i <= 13; i++)
    {
        checksum += buf[i];
    }
    packet->checksum = checksum;

    // SCI-D 송신
    SCI_writeCharArray(SCID_BASE, (uint16_t*)packet, SCADA_PACKET_SIZE);
}

//--------------------------------------------------
// SCADA 패킷 파싱 (Rev 5, 16 bytes)
//--------------------------------------------------
// SCADA → Master 수신 패킷 파싱 및 제어 변수 업데이트
//
// 호출 주기: 10ms (Main Loop)
// 동작:
//   - CRC-32 검증 (VCU2 하드웨어 가속)
//   - Command 비트 파싱 (ready_state: SCADA 명령, Precharge 시작 지시)
//   - Parameter Big-endian → Little-endian 변환
//   - 제어 모드별 변수 할당
//   - 연결 감시 타임스탬프 업데이트
//
// 제어 흐름:
//   1. SCADA: ready_state=1 → 마스터: Precharge 시작
//   2. 마스터: V_out ≈ V_batt 확인 → pre_chg_ok=1 피드백
//   3. SCADA: run_state=1 → 마스터: 정상 운전 시작
//--------------------------------------------------
void Parse_SCADA_Command(void)
{
    if (!scada_packet_ready)
        return;

    uint8_t i;
    SCADA_RxPacket_t* packet = (SCADA_RxPacket_t*)scada_rx_buffer;

    // VCU2 CRC-32 계산 (Byte 1~10, Polynomial 0x04C11DB7)
    crcObj_SCADA.seedValue   = 0x00000000;
    crcObj_SCADA.nMsgBytes   = 10;              // Command + Param1~3 + Reserved (10 bytes)
    crcObj_SCADA.parity      = CRC_parity_even;
    crcObj_SCADA.crcResult   = 0;
    crcObj_SCADA.pMsgBuffer  = (void *)(&scada_rx_buffer[1]);  // Byte 1부터 시작

    // VCU2 하드웨어 가속 CRC-32 계산
    CRC_run32BitPoly2(handleCRC_SCADA);

    // 수신된 CRC-32 (Big-endian, Byte 11~14)
    uint32_t received_crc_be = packet->crc32;
    uint32_t received_crc = __byte((uint32_t)received_crc_be, 3)
                          | ((uint32_t)__byte((uint32_t)received_crc_be, 2) << 8)
                          | ((uint32_t)__byte((uint32_t)received_crc_be, 1) << 16)
                          | ((uint32_t)__byte((uint32_t)received_crc_be, 0) << 24);

    // CRC 검증
    if (crcObj_SCADA.crcResult != received_crc)
    {
        scada_crc_error_cnt++;
        scada_packet_ready = 0;
        return;
    }

    // 연결 감시 타임스탬프 업데이트
    scada_watchdog.last_rx_time_ms = cnt_1ms;

    // Command 비트 파싱 → scada_cmd 구조체에 저장
    uint8_t cmd_byte = packet->cmd;
    scada_cmd.control_mode  = (ControlMode_t)((cmd_byte >> 7) & 0x01);  // bit[7]: 제어 모드
    scada_cmd.cmd_ready     = (cmd_byte >> 6) & 0x01;                    // bit[6]: Ready 명령
    scada_cmd.cmd_run       = (cmd_byte >> 5) & 0x01;                    // bit[5]: Run 명령
    scada_cmd.parallel_mode = (cmd_byte >> 4) & 0x01;                    // bit[4]: Parallel
    // bit[3:0]: Reserved (향후 확장용)

    // IDLE+RUN 검증 (cmd_ready=0이면 cmd_run=1 불가)
    if (scada_cmd.cmd_run && !scada_cmd.cmd_ready)
    {
        scada_packet_ready = 0;
        return;
    }

    // Parameter 파싱 (Big-endian → Little-endian 변환)
    int16_t param1_be = packet->param1;
    int16_t param2_be = packet->param2;
    int16_t param3_be = packet->param3;

    int16_t param1 = __byte((uint32_t)param1_be, 1) | ((int16_t)__byte((uint32_t)param1_be, 0) << 8);
    int16_t param2 = __byte((uint32_t)param2_be, 1) | ((int16_t)__byte((uint32_t)param2_be, 0) << 8);
    int16_t param3 = __byte((uint32_t)param3_be, 1) | ((int16_t)__byte((uint32_t)param3_be, 0) << 8);

    // 제어 모드별 변수 할당 → scada_cmd 구조체에 저장
    if (scada_cmd.control_mode == CONTROL_MODE_CHARGE_DISCHARGE)
    {
        scada_cmd.V_max_cmd = param1 / 10.0f;
        scada_cmd.V_min_cmd = param2 / 10.0f;
        scada_cmd.I_cmd     = param3 / 10.0f;
    }
    else  // CONTROL_MODE_BATTERY
    {
        scada_cmd.V_cmd     = param1 / 10.0f;
        scada_cmd.I_max_cmd = param2 / 10.0f;
        scada_cmd.I_min_cmd = param3 / 10.0f;
    }

    // 레거시 변수 매핑 (하위 호환성)
    control_mode   = scada_cmd.control_mode;
    ready_state    = scada_cmd.cmd_ready;
    run_state      = scada_cmd.cmd_run;
    start_stop     = scada_cmd.cmd_run;
    parallel_mode  = scada_cmd.parallel_mode;
    operation_mode = scada_cmd.parallel_mode ? MODE_PARALLEL : MODE_INDIVIDUAL;

    scada_packet_ready = 0;
}

//--------------------------------------------------
// SCADA 연결 감시 업데이트
//--------------------------------------------------
// SCADA 통신 타임아웃 감지 및 안전 동작
//
// 호출 주기: 1ms (Main Loop)
// 동작:
//   - 마지막 수신 후 200ms 경과 시 타임아웃 플래그 설정
//   - 타임아웃 시 안전 동작: Run → Stop, 릴레이 OFF
//   - 재연결 시 타임스탬프 기록
//--------------------------------------------------
void Update_SCADA_Watchdog(void)
{
    uint32_t current_time_ms = cnt_1ms;
    uint32_t elapsed_ms = current_time_ms - scada_watchdog.last_rx_time_ms;

    // 타임아웃 검사 (200ms)
    if (elapsed_ms > scada_watchdog.timeout_threshold_ms)
    {
        if (!scada_watchdog.connection_lost)
        {
            // 타임아웃 발생 (첫 감지)
            scada_watchdog.connection_lost = 1;

            // 안전 동작: SCADA 명령 초기화
            scada_cmd.cmd_ready = 0;
            scada_cmd.cmd_run = 0;

            // 레거시 변수 동기화
            start_stop = 0;
            run_state = 0;
            ready_state = 0;
            // operation_mode은 유지 (채널 구성, Run/Stop과 독립적)

            // 릴레이 OFF
            GPIO8_CLEAR();  // 메인 릴레이
            GPIO9_CLEAR();  // 병렬 릴레이
        }
    }
    else
    {
        // 정상 수신 중
        if (scada_watchdog.connection_lost)
        {
            // 재연결 감지
            scada_watchdog.connection_lost = 0;
            scada_watchdog.reconnect_time_ms = current_time_ms;
        }
    }
}

//--------------------------------------------------
// Master-to-Master 연결 감시 업데이트 (Rev 1.0)
//--------------------------------------------------
// @brief    M1 ↔ M2 통신 타임아웃 감지 및 안전 동작 (병렬 모드 전용)
// @note     호출 주기: 1ms (Main Loop)
// @note     타임아웃: 5ms (20kHz 전송, 100배 여유)
// @note     안전 동작: I_cmd_from_master = 0, IDLE 전환
//--------------------------------------------------
void Update_MM_Watchdog(void)
{
    // 병렬 모드가 아니거나 M1이면 감시 불필요
    if (operation_mode != MODE_PARALLEL || IS_CH1)
        return;

    uint32_t current_time_ms = cnt_1ms;
    uint32_t elapsed_ms = current_time_ms - mm_watchdog.last_rx_time_ms;

    // 타임아웃 검사 (5ms)
    if (elapsed_ms > mm_watchdog.timeout_threshold_ms)
    {
        if (!mm_watchdog.connection_lost)
        {
            // 타임아웃 발생 (첫 감지)
            mm_watchdog.connection_lost = 1;
            mm_statistics.timeout_count++;

            // 안전 동작: 전류 지령 0으로 초기화
            I_cmd_from_master = 0;
            I_cmd_from_master_prev = 0;

            // 시퀀스 강제 IDLE (안전)
            sequence_step = SEQ_STEP_IDLE;
            master_status.sequence_step = SEQ_STEP_IDLE;
            master_status.running = 0;

            // 릴레이 OFF (Phase 3에서 run == 0 조건으로 자동 처리)
        }
    }
    else
    {
        // 정상 수신 중
        if (mm_watchdog.connection_lost)
        {
            // 재연결 감지
            mm_watchdog.connection_lost = 0;
            mm_watchdog.reconnect_time_ms = current_time_ms;
        }
    }
}

//--------------------------------------------------
// Keep-Alive 토글 전송
//--------------------------------------------------
// 주기적인 패킷 전송으로 연결 유지
//
// 호출 주기: 100ms (Main Loop)
// 동작:
//   - 100ms마다 시스템 전압 패킷 재전송
//   - 토글 플래그 반전 (Reserved 필드 활용 가능)
//   - SCADA에서 토글 감지로 연결 유지 확인
//--------------------------------------------------
void Send_KeepAlive_Packet(void)
{
    // 토글 플래그 반전
    keepalive_toggle = !keepalive_toggle;

    // 시스템 전압 패킷 재전송 (Keep-Alive 역할)
    Send_System_Voltage_To_SCADA();
}

//--------------------------------------------------
// Fault/Warning 비트맵 클리어
//--------------------------------------------------
// SCADA 명령 또는 자동 복구 시 호출
//
// 동작:
//   - Fault 비트맵 초기화
//   - Warning 비트맵 초기화
//   - 타임스탬프는 유지 (이력 추적용)
//--------------------------------------------------
void Clear_Slave_Status_Bitmap(void)
{
    slave_status_bitmap.fault_bitmap = 0;
    slave_status_bitmap.warning_bitmap = 0;
}

//==================================================
// End of HABA_control.c
//==================================================
