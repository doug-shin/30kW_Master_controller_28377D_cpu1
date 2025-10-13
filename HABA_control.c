//==================================================
// HABA_control.c - Control Logic and Communication Implementation
//==================================================
// TI F28377D 30kW Master Controller
// 제어 알고리즘, 통신 핸들러, ISR 구현
//==================================================

#include "HABA_control.h"
#include <string.h>

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

//==================================================
// [1] 5-Phase 제어 함수 (20kHz @ RAMFUNC)
//==================================================
// 100kHz ISR 내에서 순환 호출되는 실시간 제어 태스크
// RAM에 배치하여 Flash 대기 시간 제거 → 결정론적 실행
//==================================================

//--------------------------------------------------
// 헬퍼 함수: 전류 제한 (PI 출력 + 최종 제한)
//--------------------------------------------------
// PI 출력 범위와 최종 전류 제한을 단계적으로 적용
//--------------------------------------------------
static inline float32_t Apply_Current_Limits(float32_t cmd_filtered)
{
    float32_t limited;
    
    // 1단계: PI 출력 범위 제한 (충전/방전 전류)
    if      (cmd_filtered >  I_PI_charge_out)    limited = I_PI_charge_out;
    else if (cmd_filtered <  I_PI_discharge_out) limited = I_PI_discharge_out;
    else                                          limited = cmd_filtered;
    
    // 2단계: 최종 전류 제한 (시스템 레벨)
    if (limited >  CURRENT_LIMIT) limited =  CURRENT_LIMIT;
    if (limited < -CURRENT_LIMIT) limited = -CURRENT_LIMIT;
    
    return limited;
}

//--------------------------------------------------
// Phase 0: 전압 센싱 + PI 준비 + CLA Force
//--------------------------------------------------
// 전압 센싱 → 소프트스타트 → LPF → CLA Task 트리거
// CLA는 백그라운드에서 실행, 결과는 Phase 1에서 사용
//--------------------------------------------------
#pragma CODE_SECTION(Sensing_And_Trigger_PI, ".TI.ramfunc");
void Sensing_And_Trigger_PI(void)
{
    // --- 1. 전압 센싱 및 캘리브레이션 ---
    
    // 출력 전압 (V_out) 평균 계산 (5회 누적합 / 5)
    V_out_raw_avg = (float32_t)(V_out_raw_sum * 0.2f);
    V_out_raw_sum = 0;

    // 캘리브레이션: 원시값 → 물리값 변환
    V_out_uncal = V_out_raw_avg * 0.019856f - 50.573f;
    V_out = (V_out_uncal + 0.091694057f) * 0.9926000888f;

    // 배터리 전압 (V_batt) 평균 계산
    V_batt_raw_avg = (float32_t)(V_batt_raw_sum * 0.2f);
    V_batt_raw_sum = 0;

    // 캘리브레이션
    V_batt_uncal = V_batt_raw_avg * 0.019856f - 50.573f;
    V_batt = (V_batt_uncal - 0.3058461657f) * 0.9945009708f;

    // --- 2. 소프트 스타트 램프 (돌입 전류 방지) ---
    I_ss_ramp += CURRENT_LIMIT * 0.00005f;      // 20kHz → 80A/1.6s
    if (I_ss_ramp > CURRENT_LIMIT)
        I_ss_ramp = CURRENT_LIMIT;

    // --- 3. 소프트 스타트 제한 적용 ---
    if      (I_cmd >  I_ss_ramp) I_cmd_ramped =  I_ss_ramp;
    else if (I_cmd < -I_ss_ramp) I_cmd_ramped = -I_ss_ramp;
    else                         I_cmd_ramped =  I_cmd;

    // --- 4. 저역통과 필터 (LPF: fc=1kHz, Ts=50us) ---
    I_cmd_filtered = lpf_coeff_a * (I_cmd_ramped + I_cmd_prev) 
                   + lpf_coeff_b * I_cmd_filtered;
    I_cmd_prev = I_cmd_ramped;

    // --- 5. DCL PI 제어기 Integrator 한계값 동적 업데이트 ---
    pi_charge.Imax    = I_cmd_filtered;      // 충전 모드 적분기 상한
    pi_discharge.Imin = I_cmd_filtered;      // 방전 모드 적분기 하한

    // --- 6. CLA PI 제어기 트리거 (백그라운드 실행) ---
    // Force만 사용, Wait 없음 → CPU와 CLA 병렬 실행
    // 결과는 Phase 1(10us 후)에서 사용
    Cla1ForceTask1();       // pi_charge 제어기 (충전 모드, V_max_cmd 기준)
    Cla1ForceTask2();       // pi_discharge 제어기 (방전 모드, V_min_cmd 기준)
}

//--------------------------------------------------
// Phase 1: PI 결과 적용 및 DAC 변환
//--------------------------------------------------
// CLA 결과(Phase 0에서 트리거) 사용 → 운전 모드별 분기 → DAC 변환
// Phase 0에서 10us 경과 → CLA 완료 보장 (CLA는 0.1-0.2us만 소요)
//--------------------------------------------------
#pragma CODE_SECTION(Apply_PI_And_Convert_DAC, ".TI.ramfunc");
void Apply_PI_And_Convert_DAC(void)
{
    // CLA Task 결과 준비 완료 (Phase 0에서 Force, 10us 경과)
    // I_PI_charge_out, I_PI_discharge_out 사용 가능
    
    // --- 1. 운전 모드별 최종 전류 지령 계산 ---
    // 병렬 운전 모드에서 하위 마스터만 상위 지령 사용, 나머지는 PI 제어 사용
    if (operation_mode == MODE_PARALLEL && master_id != 0)
    {
        // 병렬 운전 - 하위 마스터: 상위로부터 전류 지령 수신
        master_mode = 2;
        I_cmd_PI_limited = I_cmd_from_master;
    }
    else
    {
        // 병렬 운전 - 상위 마스터 / 독립 운전 / 정지 모드: PI 제어 사용
        master_mode = 1;
        I_cmd_PI_limited = Apply_Current_Limits(I_cmd_filtered);
    }

    // --- 2. DAC 출력 변환 (±100A → 0~65535) ---
    I_cmd_DAC = I_cmd_PI_limited * 327.68f + 32768.0f;
    if (I_cmd_DAC > 65535) I_cmd_DAC = 65535;

    // --- 3. 정지 상태 변수 초기화 ---
    if (run == 0)
    {
        GPIO8_CLEAR();                  // Relay 8 OFF
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
// 운전 모드에 따라 SCIA(마스터간), SCIB(슬레이브) 전류 지령 송신
//--------------------------------------------------
#pragma CODE_SECTION(Transmit_Current_Command, ".TI.ramfunc");
void Transmit_Current_Command(void)
{
    if (operation_mode == MODE_PARALLEL)        // 병렬 운전
    {
        if (master_id == 0)                     // 상위 마스터
        {
            Send_485B_Current_Command(I_cmd_DAC);   // 자기 슬레이브에 전송
            Send_485A_Current_Command(I_cmd_DAC);   // 하위 마스터에 전송
        }
        else                                    // 하위 마스터
        {
            I_cmd_PI_limited = I_cmd_from_master;
            Send_485B_Current_Command(I_cmd_from_master);  // 자기 슬레이브에만 전송
        }
    }
    else                                        // 독립 운전 또는 정지 모드
    {
        Send_485A_Current_Command(I_cmd_DAC);
        Send_485B_Current_Command(I_cmd_DAC);
    }
}

//--------------------------------------------------
// Phase 3: 시스템 안전 체크
//--------------------------------------------------
// 고장 감지, 비상 정지, 릴레이 제어
//--------------------------------------------------
#pragma CODE_SECTION(Check_System_Safety, ".TI.ramfunc");
void Check_System_Safety(void)
{
    Check_Fault();                      // 과전압/과전류/과온 체크
    
    // 비상 정지 스위치 읽기 (GPIO11)
    run_switch = GPIO_readPin(11);
    
    // 릴레이 제어 (Direct Register Access)
    if (relay_8_on_off == 1)
        GPIO8_SET();                    // Relay 8 ON (독립 운전)
    else
        GPIO8_CLEAR();                  // Relay 8 OFF
    
    if (relay_7_on_off == 1)
        GPIO9_SET();                    // Relay 7 ON (병렬 운전)
    else
        GPIO9_CLEAR();                  // Relay 7 OFF

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

    if (++V_display_calc_cnt >= 200)
    {
        V_out_display  = V_out_display_sum * 0.005f;    // = sum / 200
        V_batt_display = V_batt_display_sum * 0.005f;   // = sum / 200

        V_out_display_sum  = 0.0f;
        V_batt_display_sum = 0.0f;
        V_display_calc_cnt = 0;
    }

    // 시퀀스 제어 모듈 실행
    Execute_Sequence_Module();
}

//==================================================
// [2] 제어 알고리즘
//==================================================

// [Execute_PI_Controller 함수 삭제됨 - Execute_Current_Control에 인라인 통합]

//--------------------------------------------------
// 고장 체크 및 LED 제어
//--------------------------------------------------
// 과전압/과전류/과온 감지 → run 플래그 및 LED 상태 갱신
//--------------------------------------------------
void Check_Fault(void)
{
    // 고장 플래그 설정
    if (V_out_display >= OVER_VOLTAGE)                          over_voltage_flag = 1;
    if (fabs(I_out_avg) >= OVER_CURRENT)                        over_current_flag = 1;
    if (NTC_0_temp >= OVER_TEMP || NTC_1_temp >= OVER_TEMP)    over_temp_flag = 1;

    // run 조건 판정
    if (run_switch) {
        run = 1;
    } else {
        run = 0;
    }

    // LED 상태 표시
    if ((over_voltage_flag || over_current_flag || over_temp_flag) == 1)
    {
        master_fault_flag = 0;
        GPIO_writePin(LED_FAULT, 1);        // F_LED3 FAULT ON
        GPIO_writePin(LED_CHARGE, 0);       // F_LED5 CHARGE OFF
        GPIO_writePin(LED_DISCHARGE, 0);    // F_LED4 DISCHARGE OFF
        GPIO_writePin(LED_SINGLE, 0);       // F_LED2 SINGLE OFF
        GPIO_writePin(LED_DUAL, 0);         // F_LED1 DUAL OFF
    }
    else
    {
        master_fault_flag = 1;
        GPIO_writePin(LED_FAULT, 0);        // F_LED3 FAULT OFF
    }
}

//--------------------------------------------------
// 시스템 상태 업데이트 (슬레이브 모니터링 + 전류 지령 적용)
//--------------------------------------------------
// 슬레이브 출력 전류 합산 및 SCADA 전류 지령에 리미터 적용
//--------------------------------------------------
void Update_System_Status(void)
{
    // 슬레이브 출력 전류 합산 (모니터링용, 슬레이브 1~10)
    I_out_slave_total =
        I_out_slave[1] + I_out_slave[2] + I_out_slave[3] +
        I_out_slave[4] + I_out_slave[5] + I_out_slave[6] + I_out_slave[7] +
        I_out_slave[8] + I_out_slave[9] + I_out_slave[10];

    // SCADA 전류 지령에 리미터 적용
    if      (I_out_ref >  CURRENT_LIMIT) I_cmd =  CURRENT_LIMIT;
    else if (I_out_ref < -CURRENT_LIMIT) I_cmd = -CURRENT_LIMIT;
    else                         I_cmd =  I_out_ref;
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
void Detect_Active_Slaves(void)
{
    for (int i = 0; i < 32; i++)
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
    if (mbox < 2 || mbox > 32) return false;

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
        I_out_slave[slv_idx] = (((float)I_out_slave_raw[slv_idx] / 65535.0f) * 2.0f - 1.0f) * 100.0f;

        can_rx_fault_cnt[slv_idx] = 0;
        return true;
    }
    else
    {
        // 수신 실패 시 카운트 증가
        uint16_t slv_idx = mbox - 1;
        if (can_rx_fault_cnt[slv_idx]++ >= 10000)
        {
            I_out_slave[slv_idx]        = 0.0f;
            temp_slave_raw[slv_idx]     = 0;
            DAB_ok_slave[slv_idx]       = 0;
            I_out_slave_raw[slv_idx]    = 0;
            can_rx_fault_cnt[slv_idx]   = 10000;
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

    // 실제 읽기는 SPIC_Rx_ISR에서 수행
}

//==================================================
// [5] RS485 통신 (전류 지령 전송)
//==================================================

//--------------------------------------------------
// RS485-A 전류 지령 송신 (상위 → 하위 마스터)
//--------------------------------------------------
// 프레임 형식: [STX][Current_L][Current_H][ETX]
//--------------------------------------------------
#pragma CODE_SECTION(Send_485A_Current_Command, ".TI.ramfunc");
void Send_485A_Current_Command(uint16_t current)
{
    if (master_id == 0)     // 상위 마스터만 송신
    {
        gSciATxBuf[0] = STX;
        gSciATxBuf[1] = current & 0xFF;
        gSciATxBuf[2] = (current >> 8) & 0xFF;
        gSciATxBuf[3] = ETX;

        for (int i = 0; i < 4; i++)
        {
            HWREGH(SCIA_BASE + SCI_O_TXBUF) = gSciATxBuf[i];
        }
    }
}

//--------------------------------------------------
// RS485-B 전류 지령 송신 (마스터 → 슬레이브)
//--------------------------------------------------
// 프레임 형식: [STX][Current_L][Current_H][ETX]
//--------------------------------------------------
#pragma CODE_SECTION(Send_485B_Current_Command, ".TI.ramfunc");
void Send_485B_Current_Command(uint16_t current)
{
    int i;
    gSciBTxBuf[0] = STX;
    gSciBTxBuf[1] = current & 0xFF;
    gSciBTxBuf[2] = (current >> 8) & 0xFF;
    gSciBTxBuf[3] = ETX;

    for (i = 0; i < 4; i++)
    {
        HWREGH(SCIB_BASE + SCI_O_TXBUF) = gSciBTxBuf[i];
    }
}

//==================================================
// [6] GPIO 제어 및 유틸리티
//==================================================

// [Control_Relay, Read_Emergency_Stop_Switch 함수 삭제됨 - Check_System_Safety에 인라인 통합]

//--------------------------------------------------
// Master ID 선택 (GPIO36~39 DIP 스위치)
//--------------------------------------------------
// 4비트 DIP 스위치로 Master ID 결정 (0=상위, 1=하위)
//--------------------------------------------------
void Select_master_id(void)
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
// 시퀀스 제어 (Precharge → Relay ON → Run)
//--------------------------------------------------
// sequence_step 진행:
//   0   → Idle (대기)
//   10  → Precharge 완료 후 대기 (1초)
//   20  → Main relay ON (정상 운전)
//   Fault → 자동 차단 (과전압/과전류/과온)
//--------------------------------------------------
void Execute_Sequence_Module(void)
{
#if 0  // ===== 구 버전 (State Machine, 현재 비활성화) =====
    switch (state)
    {
        case STATE_NO_OP:
            pre_chg_ok = 0;
            pre_chg_fail = 0;
            V_max_cmd = 0;
            V_min_cmd = 0;
            I_out_ref = 0;
            start_stop = STOP;

            if (SCADA_cmd == 1)
            {
                state = STATE_READY;
            }

            if (run == 0 || master_fault_flag == 1)
            {
                state = STATE_FAULT;
            }
        break;

        case STATE_READY:
            start_stop = START;
            V_max_cmd = V_batt_display;
            V_min_cmd = 0;
            I_out_ref = 2;
            pre_chg_cnt++;
            if (pre_chg_cnt >= 20000) // 1s
            {
                if ((V_out_display - V_batt_display) < 2 && (V_out_display - V_batt_display) > -2)
                {
                    pre_chg_ok = 1;
                    state = STATE_PRE_CHG_OK;
                }
                else
                {
                    pre_chg_fail = 1;
                }
            }

            if (run == 0 || master_fault_flag == 1)
            {
                state = STATE_FAULT;
            }
        break;

        case STATE_PRE_CHG_OK:
            V_max_cmd = V_batt_display;
            V_min_cmd = 0;
            I_out_ref = 2;
            pre_chg_cnt++;
            if (pre_chg_cnt >= 40000) // 총 2s
            {
                pre_chg_cnt = 0;
                relay_8_on_off = 1;  // Relay 8번 ON
                state = STATE_STAND_BY;
            }

            if (run == 0 || master_fault_flag == 1)
            {
                state = STATE_FAULT;
            }
        break;

        case STATE_STAND_BY:
            V_max_cmd = V_batt_display;
            V_min_cmd = 0;
            I_out_ref = 0;

            if (SCADA_cmd == 2)
            {
                state = STATE_RUN;
            }
            else if (SCADA_cmd == 0)
            {
                state = STATE_RL_OFF;
            }

            if (run == 0 || master_fault_flag == 1)
            {
                state = STATE_FAULT;
            }
        break;

        case STATE_RUN:
            V_max_cmd = (float)scada_rx_data.max_voltage;
            V_min_cmd = (float)scada_rx_data.min_voltage;
            I_out_ref = (float)scada_rx_data.current_cmd / 10.0f; // A 단위 변환

            if (SCADA_cmd == 3)
            {
                state = STATE_STAND_BY;
            }
            else if (SCADA_cmd == 0)
            {
                state = STATE_RL_OFF;
            }

            if (run == 0 || master_fault_flag == 1)
            {
                state = STATE_FAULT;
            }
        break;

        case STATE_RL_OFF:
            V_max_cmd = V_batt_display;
            V_min_cmd = 0;
            I_out_ref = 0;

            relay_off_delay_cnt++;
            if (relay_off_delay_cnt >= 10000)    // 0.5s 경과
            {
                relay_8_on_off = 0;  // Relay8 OFF

                if (relay_8_on_off == 0)
                {
                    state = STATE_NO_OP;
                }
                relay_off_delay_cnt = 0;
            }

            if (run == 0 || master_fault_flag == 1)
            {
                state = STATE_FAULT;
            }
        break;

        case STATE_FAULT:
            V_max_cmd = 0;
            V_min_cmd = 0;
            I_out_ref = 0;
            start_stop = STOP;
            I_ss_ramp = 0.0f;
            relay_8_on_off = 0;
            
            // DCL PI 제어기 내부 상태 초기화
            pi_charge.i10 = 0.0f;       // 적분기 값
            pi_charge.i6  = 1.0f;       // Saturation flag
            pi_charge.i11 = 0.0f;       // Tustin integrator
            pi_discharge.i10  = 0.0f;
            pi_discharge.i6   = 1.0f;
            pi_discharge.i11  = 0.0f;

            if (run == 1 && master_fault_flag == 0)
            {
                state = STATE_NO_OP;
            }
        break;
    }

#else  // ===== 신 버전 (현재 사용 중) =====

    if (run == 1)
    {
        switch (sequence_step)
        {
            case 0:     // Precharge 단계
                if (start_stop == START)
                {
                    V_max_cmd = V_batt_display;
                    V_min_cmd = 0;
                    I_out_ref = 2;

                    // 프리차지 완료 조건: V_out ≈ V_batt (±2V)
                    if ((V_out_display - V_batt_display) < 2 && (V_out_display - V_batt_display) > -2)
                    {
                        pre_chg_ok = 1;
                        sequence_step = 10;
                    }
                    else
                    {
                        pre_chg_ok = 0;
                    }
                }
                else    // Pre-charge 해제
                {
                    V_max_cmd = 0;
                    V_min_cmd = 0;
                    I_out_ref = 0;
                    pre_chg_ok = 0;
                    relay_8_on_off = 0;     // Relay 8 OFF
                }
            break;

            case 10:    // 프리차지 완료 후 전류를 0으로 변경
                V_max_cmd = V_batt_display;
                V_min_cmd = 0;
                I_out_ref = 0;
                pre_chg_cnt++;

                if (pre_chg_cnt >= 20000) // 1초 대기 (20kHz × 20000 = 1s)
                {
                    pre_chg_cnt = 20000;
                    relay_8_on_off = 1;     // Relay 8 ON (메인 릴레이)
                    sequence_step = 20;
                }
            break;

            case 20:    // 정상 운전 (UI 전류 지령에 따라 동작)
                // UI에서 STOP 명령 시
                if (start_stop == STOP)
                {
                    sequence_step = 0;
                }
            break;
        }
    }
    else    // run == 0 (비상 정지 또는 고장)
    {
        V_max_cmd = 0;
        V_min_cmd = 0;
        I_out_ref = 0;
        sequence_step = 0;
        relay_8_on_off = 0;     // Relay 8 OFF
        start_stop = STOP;
    }
#endif
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
// SCIA RX 인터럽트 (상위 마스터로부터 전류 지령 수신)
//--------------------------------------------------
// master_id != 0 (하위 마스터)일 때만 상위로부터 4바이트 프레임 수신
// 프레임 형식: [STX][Current_L][Current_H][ETX]
//--------------------------------------------------
__interrupt void SCIA_Rx_ISR(void)
{
    uint8_t rx = SCI_readCharNonBlocking(SCIA_BASE);

    if (master_id != 0)     // 하위 마스터일 때만 수신
    {
        rxBuffer[rxIndex++] = rx;
        
        if (rxIndex >= 4)
        {
            // 4바이트 프레임 수신 완료: STX 체크 후 전류 지령 추출
            if (rxBuffer[0] == 0x02)    // STX
            {
                uint16_t current = ((uint16_t)rxBuffer[2] << 8) | rxBuffer[1];
                I_cmd_from_master = current;    // 상위에서 받은 전류 지령 저장
            }
            rxIndex = 0;
        }
    }

    SCI_clearOverflowStatus(SCIA_BASE);
    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_RXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}

//--------------------------------------------------
// SPIC RX 인터럽트 (FPGA ADC 데이터 수신)
//--------------------------------------------------
// FPGA로부터 I_out, V_out, V_batt 원시값 수신 (3 워드)
//--------------------------------------------------
__interrupt void SPIC_Rx_ISR(void)
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
// 7바이트 패킷: [STX][ID+Status][Current_H][Current_L][Temp][Checksum][ETX]
//--------------------------------------------------
void Send_Slave_Data_To_SCADA(void)
{
    static uint8_t slave_index = 0;     // 현재 송신할 슬레이브 인덱스 (0~14)
    static uint8_t max_channels = 15;   // 최대 슬레이브 수
    
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
        if (current_cA > 32767) current_cA = 32767;
        if (current_cA < -32768) current_cA = -32768;

        // 전류를 uint16 코드로 변환 (Center=32768)
        current_code = (uint16_t)(current_cA + 32768);

        // 온도 (0.5℃ 단위 스케일링)
        temp_data = (uint8_t)(temp_slave_raw[slave_index] * 2);
        if (temp_data > 255) temp_data = 255;
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

        current_code = (uint16_t)(current_cA + 32768);
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
    if (slave_index >= max_channels) slave_index = 0;
}

//--------------------------------------------------
// System Voltage → SCADA 송신 (50ms 주기)
//--------------------------------------------------
// 7바이트 패킷: [STX][master_id+Rack_Ch][Voltage_H][Voltage_L][Reserved][Checksum][ETX]
//--------------------------------------------------
void Send_System_Voltage_To_SCADA(void)
{
    uint16_t voltage_scaled = 0;

    // 테스트 모드 제어 스위치
    #define USE_TEST_DATA 0     // 0: 실제 V_out_display, 1: 테스트 값

#if (USE_TEST_DATA == 0)
    // --- 실제 데이터 사용 ---
    voltage_scaled = (uint16_t)(V_out_display * 10.0f + 300);  // 0.1V 단위
    if (voltage_scaled > 65535) voltage_scaled = 65535;

#else
    // --- 테스트 데이터 ---
    test_voltage += 500;                    // 호출할 때마다 +5.0V 증가
    if (test_voltage > 5000) test_voltage = 0;  // 500.0V 도달 시 0으로 리셋

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
// SCID RX 인터럽트 (SCADA 패킷 수신)
//--------------------------------------------------
// 10바이트 패킷 수신: [STX][CMD][Vmax_H][Vmax_L][Vmin_H][Vmin_L][Icmd_H][Icmd_L][Checksum][ETX]
//--------------------------------------------------
__interrupt void SCID_Rx_Ready_ISR(void)
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

            if (scada_rx_index == 10)       // 10바이트 다 받았을 때만
            {
                if (scada_rx_buffer[0] == 0x02 && scada_rx_buffer[9] == 0x03)
                {
                    uint8_t checksum = 0;
                    for (uint8_t i = 1; i <= 7; i++)    // Byte1~7 합산
                    {
                        checksum += scada_rx_buffer[i];
                    }
                    if ((checksum & 0xFF) == scada_rx_buffer[8])
                    {
                        scada_packet_ready = 1;     // 유효 프레임 확정
                        Modbus_Parse();
                    }
                }
                scada_rx_index = 0;         // 다음 패킷 준비
            }
        }
    }
    
    // 3) 인터럽트 플래그 클리어
    SCI_clearInterruptStatus(SCID_BASE, SCI_INT_RXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP8);
}

//--------------------------------------------------
// SCADA 패킷 파싱 (제어 변수 반영)
//--------------------------------------------------
// 수신된 10바이트 패킷 해석 → start_stop, operation_mode, V_max_cmd, V_min_cmd, I_out_ref 갱신
//--------------------------------------------------
void Modbus_Parse(void)
{
    // 패킷 해석
    scada_rx_data.start_byte    = scada_rx_buffer[0];
    scada_rx_data.command       = scada_rx_buffer[1];   // bit0=run, bit2:1=Mode
    scada_rx_data.max_voltage   = (int16_t)((scada_rx_buffer[2] << 8) | scada_rx_buffer[3]);
    scada_rx_data.min_voltage   = (int16_t)((scada_rx_buffer[4] << 8) | scada_rx_buffer[5]);

    // Current Command (2바이트, Center=32768, 0.01A 단위)
    scada_rx_data.current_cmd   = ((int16_t)((scada_rx_buffer[6] << 8) | scada_rx_buffer[7])) - 32768;

    scada_rx_data.checksum      = scada_rx_buffer[8];
    scada_rx_data.end_byte      = scada_rx_buffer[9];

    // 제어 변수 반영
    start_stop      = (scada_rx_data.command & 0x01);                       // bit0 = run
    operation_mode  = (OperationMode_t)((scada_rx_data.command >> 1) & 0x03);
    V_max_cmd       = (float)scada_rx_data.max_voltage;
    V_min_cmd       = (float)scada_rx_data.min_voltage;
    I_out_ref       = (float)scada_rx_data.current_cmd / 10.0f;            // A 단위 변환

    scada_packet_ready = 0;     // 다음 패킷 대기
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
