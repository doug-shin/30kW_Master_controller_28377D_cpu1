//
// HABA_control.c - Control logic, communication handlers and ISR implementation
//

#include "HABA_control.h"
#include <string.h>

//==================================================
// 디버그 모드 설정
//==================================================
#ifndef _DEBUG_CAN_STATUS_ENABLE_
#define _DEBUG_CAN_STATUS_ENABLE_      (0)
#endif

#ifndef _DEBUG_SCI_STATUS_ENABLE_
#define _DEBUG_SCI_STATUS_ENABLE_      (0)
#endif

#ifndef _DEBUG_SPI_STATUS_ENABLE_
#define _DEBUG_SPI_STATUS_ENABLE_      (0)
#endif

#ifndef _DEBUG_CLK_STATUS_ENABLE_
#define _DEBUG_CLK_STATUS_ENABLE_      (0)
#endif

//==================================================
// 파일 내부 전역 변수
//==================================================
volatile uint16_t latestCurrentValue = 0;
volatile uint8_t  gRxFrame[4];
volatile uint16_t rxIndex = 0;
volatile uint8_t  rxBuffer[4];
int8_t            parseCount = 0;

//==================================================
// 함수 전방 선언
//==================================================
static void parseFrame(uint8_t *buf, uint16_t len);
//==================================================
// [1] 제어 태스크 (5-Phase, 20kHz @ RAMFUNC)
//==================================================

//========================
// Task 1: Sensing (전압/전류 센싱 및 캘리브레이션)
//========================
#pragma CODE_SECTION(sensing_task, ".TI.ramfunc");
void sensing_task(void)
{
    // --- 출력 전압 (Vo) 평균 및 캘리브레이션 ---
    Vo_ad_avg = (float32_t)(Vo_ad_sum * 0.2f);  // 5회 평균 (1/5 = 0.2)
    Vo_ad_sum = 0;

    fADC_voltage_out = Vo_ad_avg * 0.019856f - 50.573f;
    Vo_sen = (fADC_voltage_out + 0.091694057f) * 0.9926000888f;  // 캘리브레이션 계수
    Vo = Vo_sen;

    // --- 배터리 전압 (Vbat) 평균 및 캘리브레이션 ---
    Vbat_ad_avg = (float32_t)(Vbat_ad_sum * 0.2f);
    Vbat_ad_sum = 0;

    fADC_voltage_Bat = Vbat_ad_avg * 0.019856f - 50.573f;
    Vbat_sen = (fADC_voltage_Bat - 0.3058461657f) * 0.9945009708f;  // 캘리브레이션 계수
    Vbat = Vbat_sen;
}


//========================
// Task 2: PI Control (소프트 스타트 + PI 제어)
//========================
#pragma CODE_SECTION(pi_control_task, ".TI.ramfunc");
void pi_control_task(void)
{
    task_flag2 = 1;

    // --- 1. 소프트 스타트 램프 ---
    soft_start_limit += CURRENT_LIMIT * 0.00005f;  // 20kHz 램프 속도
    if (soft_start_limit > CURRENT_LIMIT)
        soft_start_limit = CURRENT_LIMIT;

    // --- 2. 소프트 스타트 제한 적용 ---
    if      (I_cmd >  soft_start_limit) I_cmd_ss =  soft_start_limit;
    else if (I_cmd < -soft_start_limit) I_cmd_ss = -soft_start_limit;
    else                                I_cmd_ss =  I_cmd;

    // --- 3. 저역통과 필터 (LPF: fc=1kHz, Ts=50us) ---
    I_cmd_filt = La_Ee * (I_cmd_ss + I_cmd_old) + Lb_Ee * I_cmd_filt;
    I_cmd_old  = I_cmd_ss;

    // --- 4. CLA PI 제어기 anti-windup 한계값 ---
    I_sat = I_cmd_filt;

    // --- 5. PI 제어기 실행 (CLA Task 호출) ---
    PI_Controller();

    // --- 6. 운전 모드별 최종 전류 지령 계산 ---
    if (operation_mode == MODE_PARALLEL)  // 병렬 운전
    {
        if (Master_ID == 0)  // 상위 마스터: PI 출력 사용
        {
            Master_Mode = 1;
            if      (I_cmd_filt >  Voh_pi_out) I_cmd_final = Voh_pi_out;
            else if (I_cmd_filt <  Vol_pi_out) I_cmd_final = Vol_pi_out;
            else                               I_cmd_final = I_cmd_filt;

            if (I_cmd_final >  I_MAX) I_cmd_final =  I_MAX;
            if (I_cmd_final < -I_MAX) I_cmd_final = -I_MAX;
        }
        else  // 하위 마스터: 상위에서 전달받은 값 사용
        {
            Master_Mode = 2;
            I_cmd_final = I_cmd_from_master;
        }
    }
    else if (operation_mode == MODE_INDEPENDENT)  // 독립 운전
    {
        Master_Mode = 1;

        if      (I_cmd_filt >  Voh_pi_out) I_cmd_final = Voh_pi_out;
        else if (I_cmd_filt <  Vol_pi_out) I_cmd_final = Vol_pi_out;
        else                               I_cmd_final = I_cmd_filt;

        if (I_cmd_final >  I_MAX) I_cmd_final =  I_MAX;
        if (I_cmd_final < -I_MAX) I_cmd_final = -I_MAX;
    }
    else  // 정지 모드
    {
        if      (I_cmd_filt >  Voh_pi_out) I_cmd_final = Voh_pi_out;
        else if (I_cmd_filt <  Vol_pi_out) I_cmd_final = Vol_pi_out;
        else                               I_cmd_final = I_cmd_filt;

        if (I_cmd_final >  I_MAX) I_cmd_final =  I_MAX;
        if (I_cmd_final < -I_MAX) I_cmd_final = -I_MAX;
    }

    // --- 7. DAC 출력 변환 (±100A → 0~65535) ---
    I_cmd_DAC = I_cmd_final * 327.68f + 32768.0f;
    if (I_cmd_DAC > 65535) I_cmd_DAC = 65535;

    // --- 8. 정지 상태 변수 초기화 ---
    if (Run == 0)
    {
        GPIO_writePin(8, 0);  // Relay 8 OFF
        Siwtch_flag      = 0;
        soft_start_limit = 0.0f;
        I_cmd_ss         = 0.0f;
        I_out_ref        = 0.0f;
        I_sat            = 0.0f;
        task_flag2       = 0;
    }
}


//========================
// Task 3: Current Command (전류 지령 전송)
//========================
#pragma CODE_SECTION(current_command_task, ".TI.ramfunc");
void current_command_task(void)
{
    task_flag3 = 1;

    if (operation_mode == MODE_PARALLEL)  // 병렬 운전
    {
        if (Master_ID == 0)  // 상위 마스터
        {
            send_485B_CurrentCommand(I_cmd_DAC);  // 자기 슬레이브에 전송
            send_485A_CurrentCommand(I_cmd_DAC);  // 하위 마스터(Master2)에 전송
        }
        else  // 하위 마스터
        {
            I_cmd_final = I_cmd_from_master;
            send_485B_CurrentCommand(I_cmd_from_master);  // 자기 슬레이브에만 전송
        }
    }
    else  // 독립 운전 또는 정지 모드
    {
        send_485A_CurrentCommand(I_cmd_DAC);
        send_485B_CurrentCommand(I_cmd_DAC);
    }
}

//========================
// Task 4: Temperature (고장 체크 및 릴레이 제어)
//========================
#pragma CODE_SECTION(temperature_task, ".TI.ramfunc");
void temperature_task(void)
{
    Fault_Check();
    task_flag4 = 1;
    Emergency_Stop_Switch();
    Relay_control();

    // NTC 온도 센싱 (현재 비활성화)
    // TODO: ADC 기반 NTC 온도 측정 구현 필요 시 활성화
}

//========================
// Task 5: Average (장기 평균 및 시퀀스)
//========================
#pragma CODE_SECTION(average_task, ".TI.ramfunc");
void average_task(void)
{
    task_flag5 = 1;

    // --- 장기 평균 (200회 = 10ms @ 20kHz) ---
    Vo_sen_sum_monitor  += Vo_sen;
    Bat_sen_sum_monitor += Vbat_sen;

    if (++MonitoringCount >= 200)
    {
        Vo_Mean  = Vo_sen_sum_monitor * 0.005f;   // = sum / 200
        Bat_Mean = Bat_sen_sum_monitor * 0.005f;  // = sum / 200

        Vo_sen_sum_monitor  = 0.0f;
        Bat_sen_sum_monitor = 0.0f;
        MonitoringCount = 0;
    }

    Sequence_Module();
}





//==================================================
// [2] 제어 지원 함수
//==================================================

//========================
// 통신 상태 모니터링
//========================
void Communication_Status(void)
{
#if _DEBUG_CAN_STATUS_ENABLE_
    debug_CAN_status();
#endif
#if _DEBUG_SCI_STATUS_ENABLE_
    debug_SCI_status();
#endif
#if _DEBUG_SPI_STATUS_ENABLE_
    debug_SPI_status();
#endif
}

//========================
// UI 모니터링 (HMI 데이터 처리)
//========================
void UI_monitoring(void)
{
    Voh_com = V_high_limit;
    Vol_com = V_low_limit;

    // 슬레이브 출력 전류 합산 (Ch1~10)
    Io_sen_total =
        I_out_ch[1] + I_out_ch[2] + I_out_ch[3] +
        I_out_ch[4] + I_out_ch[5] + I_out_ch[6] + I_out_ch[7] +
        I_out_ch[8] + I_out_ch[9] + I_out_ch[10];

    // 전류 지령 리미터
    Icom_temp = I_out_ref;
    if      (Icom_temp >  I_MAX) Icom_temp =  I_MAX;
    else if (Icom_temp < -I_MAX) Icom_temp = -I_MAX;

    I_cmd = Icom_temp;
}

//========================
// 고장 체크 및 LED 제어
//========================
void Fault_Check(void)
{
    // --- 고장 플래그 설정 ---
    if (Vo_Mean >= OVER_VOLTAGE)                          over_voltage_flag = 1;
    if (fabs(Io_avg) >= OVER_CURRENT)                     over_current_flag = 1;
    if (NTC0_Temp >= OVER_TEMP || NTC1_Temp >= OVER_TEMP) over_temp_flag = 1;

    // --- Run 조건 판정 ---
    if (run_switch) {
        Run = 1;
    } else {
        Run = 0;
    }

    // --- LED 상태 표시 ---
    if ((over_voltage_flag || over_current_flag || over_temp_flag) == 1)
    {
        Master_fault_flag = 0;
        GPIO_writePin(LED_FAULT, 1);      // F_LED3 FAULT ON
        GPIO_writePin(LED_CHARGE, 0);     // F_LED5 CHARGE OFF
        GPIO_writePin(LED_DISCHARGE, 0);  // F_LED4 DISCHARGE OFF
        GPIO_writePin(LED_SINGLE, 0);     // F_LED2 SINGLE OFF
        GPIO_writePin(LED_DUAL, 0);       // F_LED1 DUAL OFF
    }
    else
    {
        Master_fault_flag = 1;
        GPIO_writePin(LED_FAULT, 0);      // F_LED3 FAULT OFF
    }
}

//========================
// PI 제어기 (CLA Task 호출)
//========================
#pragma CODE_SECTION(PI_Controller, ".TI.ramfunc");
void PI_Controller(void)
{
    // CLA 파라미터 설정
    Kp       = Kp_set;
    Ki       = Ki_set;
    T_sample = T_sample_set;
    I_MAX    = CURRENT_LIMIT;

    Voh_PI = Voh_cmd;
    Vol_PI = Vol_cmd;

    // CLA Task 강제 실행
    Cla1ForceTask1();  // PI_high 제어기 (충전 모드)
    Cla1ForceTask2();  // PI_low 제어기 (방전 모드)
}



//==================================================
// [3] CAN 통신 (슬레이브 모듈 제어)
//==================================================

//========================
// CAN 메시지 송신
//========================
#pragma CODE_SECTION(send_CANA_Message, ".TI.ramfunc");
void send_CANA_Message(int8_t CAN_CMD)
{
    CANA_txData[0] = CAN_CMD;
    CANA_txData[1] = 0x00;
    CANA_txData[2] = 0x00;
    CANA_txData[3] = 0x00;

    CAN_sendMessage(CANA_BASE, TX_MSG_OBJ_ID1, MSG_DATA_LENGTH, CANA_txData);
}

//========================
// CAN 메시지 일괄 수신 (사용 안 함)
//========================
#pragma CODE_SECTION(read_CANA_Messages, ".TI.ramfunc");
void read_CANA_Messages(void)
{
    int i;
    for (i = 0; i < RX_MSG_OBJ_COUNT; i++)
    {
        CAN_readMessage(CANA_BASE,
                        RX_MSG_OBJ_BASE_ID + i,
                        CANA_rxDataArray[i]);
    }
}

//========================
// 슬레이브 초기화 (초기 탐색)
//========================
void detect_active_slaves(void)
{
    for (int i = 0; i < 32; i++)
    {
        slave_enabled[i]     = false;
        ch_current[i]        = 0;
        I_out_ch[i]          = 0;
        Temp_ch[i]           = 0;
        DAB_ok_ch[i]         = 0;
        can_rx_fault_cnt[i]  = 0;
    }
}

//========================
// 슬레이브 데이터 읽기 (개별 메일박스)
//========================
#pragma CODE_SECTION(CAN_SlaveRead, ".TI.ramfunc");
bool CAN_SlaveRead(uint16_t mbox)
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

        uint16_t ch = mbox - 1;  // ch index: 1~31

        // 전류 (16bit), 온도 (12bit), DAB_OK (1bit)
        ch_current[ch] = ((uint16_t)byte0 << 8) | byte1;
        Temp_ch[ch]    = (byte2 << 4) | ((byte3 & 0xF0) >> 4);
        DAB_ok_ch[ch]  = byte3 & 0x01;

        // 전류 정규화: 0~65535 → -100~100A
        I_out_ch[ch] = (((float)ch_current[ch] / 65535.0f) * 2.0f - 1.0f) * 100.0f;

        can_rx_fault_cnt[ch] = 0;
        return true;
    }
    else
    {
        // 수신 실패 시 카운트 증가
        uint16_t ch = mbox - 1;
        if (can_rx_fault_cnt[ch]++ >= 10000)
        {
            I_out_ch[ch]         = 0;
            Temp_ch[ch]          = 0;
            DAB_ok_ch[ch]        = 0;
            ch_current[ch]       = 0;
            can_rx_fault_cnt[ch] = 10000;
        }
        return false;
    }
}


//==================================================
// [4] SPI 통신 (DAC 및 FPGA)
//==================================================

//========================
// DAC80502 SPI 제어 (SPIA)
//========================
#pragma CODE_SECTION(SPIDAC1, ".TI.ramfunc");
void SPIDAC1(uint8_t cmd_byte, uint16_t dac_data)
{
    SPI_writeDataBlockingFIFO(SPIA_BASE, ((uint16_t)cmd_byte) << 8);
    SPI_writeDataBlockingFIFO(SPIA_BASE, ((uint16_t)(dac_data >> 8)) << 8);
    SPI_writeDataBlockingFIFO(SPIA_BASE, ((uint16_t)(dac_data & 0xFF)) << 8);
}

//========================
// FPGA 데이터 읽기 (SPIC)
//========================
#pragma CODE_SECTION(read_FPGA_data, ".TI.ramfunc");
void read_FPGA_data(void)
{
    // Dummy Write (FPGA 데이터 밀어내기)
    HWREGH(SPIC_BASE + SPI_O_TXBUF) = 0x0000;
    HWREGH(SPIC_BASE + SPI_O_TXBUF) = 0x0000;
    HWREGH(SPIC_BASE + SPI_O_TXBUF) = 0x0000;

    // 실제 읽기는 ISR에서 수행 (spicRxISR)
}

//==================================================
// [5] RS485 통신 (전류 지령 전송)
//==================================================

//========================
// RS485-A 전류 지령 송신 (상위 마스터 → 하위 마스터)
//========================
#pragma CODE_SECTION(send_485A_CurrentCommand, ".TI.ramfunc");
void send_485A_CurrentCommand(uint16_t current)
{
    if (Master_ID == 0)  // 상위 마스터만 송신
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

//========================
// RS485-B 전류 지령 송신 (마스터 → 슬레이브)
//========================
#pragma CODE_SECTION(send_485B_CurrentCommand, ".TI.ramfunc");
void send_485B_CurrentCommand(uint16_t current)
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

//========================
// 릴레이 제어
//========================
void Relay_control(void)
{
    GPIO_writePin(8, (Relay8_on_off == 1) ? 1 : 0);  // Relay 8 (독립 운전)
    GPIO_writePin(9, (Relay7_on_off == 1) ? 1 : 0);  // Relay 7 (병렬 운전)
}

//========================
// 디지털 입력 읽기
//========================
uint16_t Digtal_Input(uint16_t gpioPin)
{
    return GPIO_readPin(gpioPin);
}

//========================
// Master ID 선택 (GPIO36~39 DIP 스위치)
//========================
void Master_ID_Select(void)
{
#if 1
    DigitalIn.bit.Bit0 = GPIO_readPin(36);
    DigitalIn.bit.Bit1 = GPIO_readPin(37);
    DigitalIn.bit.Bit2 = GPIO_readPin(38);
    DigitalIn.bit.Bit3 = GPIO_readPin(39);

    Master_ID = ((DigitalIn.bit.Bit3 & 0x1) << 3) |
                ((DigitalIn.bit.Bit2 & 0x1) << 2) |
                ((DigitalIn.bit.Bit1 & 0x1) << 1) |
                ((DigitalIn.bit.Bit0 & 0x1) << 0);
#endif
}

//========================
// NTC 온도 계산 (LUT 보간)
//========================
#pragma CODE_SECTION(calcNTCTemp, ".TI.ramfunc");
float calcNTCTemp(float R_ntc)
{
    float temperature_c = 0.0f;
    int i;

    // LUT 탐색 (-40°C ~ 120°C)
    for (i = 0; i < (sizeof(LUT_Resistance)/sizeof(LUT_Resistance[0])) - 1; i++)
    {
        if (R_ntc <= LUT_Resistance[i] && R_ntc >= LUT_Resistance[i+1])
        {
            // 선형 보간
            float R1 = LUT_Resistance[i];
            float R2 = LUT_Resistance[i+1];
            float T1 = (float)(-40 + i);
            float T2 = (float)(-40 + i + 1);

            temperature_c = T1 + (T2 - T1) * (R_ntc - R1) / (R2 - R1);
            break;
        }
    }

    return temperature_c;
}

//========================
// 비상 정지 스위치 읽기 (GPIO11)
//========================
Uint16 Emergency_Stop_Switch(void)
{
    run_switch = GPIO_readPin(11);
    return run_switch;
}




//==================================================
// [7] 시퀀스 제어 모듈
//==================================================

//========================
// 시퀀스 제어 (Precharge → Relay ON → Run)
//========================
// sequence_step 진행:
//   0   → Idle (대기)
//   10  → Precharge (예비 충전)
//   20  → Main relay ON (정상 운전)
//   Fault → 자동 차단 (과전압/과전류/과온)
//========================
void Sequence_Module(void)
{
#if 0  // ===== 구 버전 (state machine, 현재 비활성화) =====
    switch (state)
    {
        case STATE_NO_OP:
            Pre_chg_ok = 0;
            Pre_chg_Fail = 0;
            Voh_cmd = 0;
            Vol_cmd = 0;
            I_out_ref = 0;
            start_stop = STOP;

            if (SCADA_cmd == 1)
            {
                state = STATE_READY;
            }

            if (Run == 0 || Master_fault_flag == 1)
            {
                state = STATE_FAULT;
            }
        break;

        case STATE_READY:
            start_stop = START;
            Voh_cmd = Bat_Mean;
            Vol_cmd = 0;
            I_out_ref = 2;
            Pre_chg_count++;
            if (Pre_chg_count >= 20000) // 1s
            {
                if ((Vo_Mean - Bat_Mean) < 2 && (Vo_Mean - Bat_Mean) > -2)
                {
                    Pre_chg_ok = 1;
                    state = STATE_PRE_CHG_OK;
                }
                else
                {
                    Pre_chg_Fail = 1;
                }
            }

            if (Run == 0 || Master_fault_flag == 1)
            {
                state = STATE_FAULT;
            }
        break;

        case STATE_PRE_CHG_OK:
            Voh_cmd = Bat_Mean;
            Vol_cmd = 0;
            I_out_ref = 2;
            Pre_chg_count++;
            if (Pre_chg_count >= 40000) // 총 2s
            {
                Pre_chg_count = 0;
                Relay8_on_off = 1;  // Relay 8번 ON
                state = STATE_STAND_BY;
            }

            if (Run == 0 || Master_fault_flag == 1)
            {
                state = STATE_FAULT;
            }
        break;

        case STATE_STAND_BY:
            Voh_cmd = Bat_Mean;
            Vol_cmd = 0;
            I_out_ref = 0;

            if (SCADA_cmd == 2)
            {
                state = STATE_RUN;
            }
            else if (SCADA_cmd == 0)
            {
                state = STATE_RL_OFF;
            }

            if (Run == 0 || Master_fault_flag == 1)
            {
                state = STATE_FAULT;
            }
        break;

        case STATE_RUN:
            Voh_cmd = (float)hmi_rx_data.max_voltage;
            Vol_cmd = (float)hmi_rx_data.min_voltage;
            I_out_ref = (float)hmi_rx_data.current_cmd / 10.0f; // A 단위 변환

            if (SCADA_cmd == 3)
            {
                state = STATE_STAND_BY;
            }
            else if (SCADA_cmd == 0)
            {
                state = STATE_RL_OFF;
            }

            if (Run == 0 || Master_fault_flag == 1)
            {
                state = STATE_FAULT;
            }
        break;

        case STATE_RL_OFF:
            Voh_cmd = Bat_Mean;
            Vol_cmd = 0;
            I_out_ref = 0;

            RL_off_delay_count++;
            if (RL_off_delay_count >= 10000)    // 0.5s 경과
            {
                Relay8_on_off = 0;  // Relay8 OFF

                if (Relay8_on_off == 0)
                {
                    state = STATE_NO_OP;
                }
                RL_off_delay_count = 0;
            }

            if (Run == 0 || Master_fault_flag == 1)
            {
                state = STATE_FAULT;
            }
        break;

        case STATE_FAULT:
            Voh_cmd = 0;
            Vol_cmd = 0;
            I_out_ref = 0;
            start_stop = STOP;
            soft_start_limit = 0.0f;
            I_sat           = 0.0f;
            Relay8_on_off = 0;

            if (Run == 1 && Master_fault_flag == 0)
            {
                state = STATE_NO_OP;
            }
        break;
    }


    #else

    if (Run == 1)
    {
        switch (sequence_step)
        {
            case 0:
            if (start_stop == START)
            {
                Voh_cmd = Bat_Mean;
                Vol_cmd = 0;
                I_out_ref = 2;

                if ((Vo_Mean - Bat_Mean) < 2 && (Vo_Mean - Bat_Mean) > -2)
                {
                    Pre_chg_ok = 1;
                    sequence_step = 10;
                }
                else
                {
                    Pre_chg_ok = 0;
                }

            }
            else    // Pre-charge 해제
            {
                Voh_cmd = 0;
                Vol_cmd = 0;
                I_out_ref = 0;
                Pre_chg_ok = 0;
                // Relay7_on_off = 0;           // Relay 7, 독립 운전 시 상위 Master 연결 릴레이
                Relay8_on_off = 0;     // Relay 8, 독립 운전 시 하위 Master 연결 릴레이
            }
            break;

            case 10:    // 프리차지 완료 후 전류를 0으로 변경
                Voh_cmd = Bat_Mean;
                Vol_cmd = 0;
                I_out_ref = 0;
                Pre_chg_count++;
            if (Pre_chg_count >= 20000) // 1s
            {
                Pre_chg_count = 20000;
                // Relay7_on_off = 1;           // Relay 7, 독립 운전 시 상위 Master 연결 릴레이
                Relay8_on_off = 1;     // Relay 8, 독립 운전 시 하위 Master 연결 릴레이
                sequence_step = 20;
            }
            break;

            case 20: // UI의 전류 지령에 따라 동작
            // UI에서 STOP 명령 시
            if (start_stop == STOP)
            {
                sequence_step = 0;
            }
            break;
        }
    }
    else
    {
        Voh_cmd = 0;
        Vol_cmd = 0;
        I_out_ref = 0;
        sequence_step = 0;
        Relay8_on_off = 0;     // Relay 8
        start_stop = STOP;
    }
    #endif
}

//========================
// parseFrame Helper (SCIA 수신 프레임 파싱)
//========================
// STX로 시작하는 4바이트 프레임에서 전류 지령 추출
//========================
void parseFrame(uint8_t *buf, uint16_t len)
{
    if(len == 4 && buf[0] == STX)
    {
        parseCount ++;
        for(int i=0; i<4; i++) gRxFrame[i] = buf[i];

        uint16_t current = ((uint16_t)buf[2] << 8) | buf[1];
        latestCurrentValue = current;

        // === 추가 부분 ===
        I_cmd_from_master = current;   // 상위에서 받은 전류 지령 저장
    }
}

//==================================================
// [8] 인터럽트 서비스 루틴 (ISR)
//==================================================

//========================
// ADCA1 인터럽트 (현재 비활성화)
//========================
__interrupt void INT_ADCA1_ISR(void)
{
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

//========================
// CLA 인터럽트 (CLA Task 완료 시 호출)
//========================
// CLA Task1/Task2 완료 시 호출되는 ISR (현재 별도 처리 없음)
//========================
__interrupt void cla1Isr1 ()
{
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP11);
//    asm(" ESTOP0");
}

__interrupt void cla1Isr2 ()
{
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP11);
}

//========================
// SCIA RX 인터럽트 (상위 마스터로부터 전류 지령 수신)
//========================
// Master_ID != 0 (하위 마스터)일 때만 상위로부터 4바이트 프레임 수신
//========================
__interrupt void sciaRxISR(void)
{
    uint8_t rx = SCI_readCharNonBlocking(SCIA_BASE);

    if (Master_ID != 0)  // 하위일 때만 파싱
    {
        rxBuffer[rxIndex++] = rx;
        if(rxIndex >= 4)
        {
            parseFrame((uint8_t*)rxBuffer, 4);
            rxIndex = 0;
        }
    }

    SCI_clearOverflowStatus(SCIA_BASE);
    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_RXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}

//========================
// SPIC RX 인터럽트 (FPGA ADC 데이터 수신)
//========================
// FPGA로부터 Io, Vo, Vbat ADC 값 수신 (3 워드)
//========================
__interrupt void spicRxISR(void)
{
    Io_ad   = HWREGH(SPIC_BASE + SPI_O_RXBUF);
    Vo_ad   = HWREGH(SPIC_BASE + SPI_O_RXBUF);
    Vbat_ad = HWREGH(SPIC_BASE + SPI_O_RXBUF);

    SPI_clearInterruptStatus(SPIC_BASE, SPI_INT_RXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP6);
}

//==================================================
// [9] HMI 통신 (SCID 프로토콜)
//==================================================

//========================
// Slave 데이터 → HMI 송신 (10ms 주기)
//========================
// 7바이트 패킷: STX | ID+Status | Current_H | Current_L | Temp | Checksum | ETX
//========================
void send_slave_data_to_hmi(void)       // Slave Module Data를 HMI로 송신
{
    static uint8_t current_channel = 0; // 현재 송신할 채널 (0~9 배열 인덱스)
    static uint8_t max_channels = 15;    // 최대 채널 수
    uint8_t slave_id = current_channel + 1;
    int current_cA = 0;
    uint16_t current_code = 0;
    uint8_t temp_data = 0;
    uint8_t checksum = 0;
    uint16_t i;

// === 테스트 모드 제어 스위치 ===
#define USE_TEST_DATA 0 // 0: 실제 CAN 데이터 사용, 1: 테스트 데이터 사용

    memset(slave_tx_buffer, 0, sizeof(slave_tx_buffer));

     if (USE_TEST_DATA == 0)
    {
        // --- 실제 CAN 데이터 사용 ---
        current_cA = (int)(I_out_ch[current_channel] * 100); // 0.01A 단위
        if (current_cA > 32767) current_cA = 32767;
        if (current_cA < -32768) current_cA = -32768;

        // 전류를 uint16 코드로 변환 (Center=32768)
        current_code = (uint16_t)(current_cA + 32768);

        // 온도 (0.5℃ 단위 스케일링)
        temp_data = (uint8_t)(Temp_ch[current_channel] * 2);
        if (temp_data > 255) temp_data = 255;
        slave_tx_buffer[4] = temp_data;

        // ID + Status
        slave_tx_buffer[1] = (slave_id << 3) | (DAB_ok_ch[current_channel] & 0x01);
    }
    else
    {   // --- 테스트 데이터 ---
        if (current_channel == 0) { current_cA = current_cA_TEST; slave_tx_buffer[4] = 40; }
        else if (current_channel == 1) { current_cA =current_cA_TEST; slave_tx_buffer[4] = 40; }
        else if (current_channel == 2) { current_cA = current_cA_TEST; slave_tx_buffer[4] = 40; }
        else if (current_channel == 3) { current_cA = current_cA_TEST; slave_tx_buffer[4] = 40; }
        else if (current_channel == 4) { current_cA = current_cA_TEST; slave_tx_buffer[4] = 40; }
        else if (current_channel == 5) { current_cA = current_cA_TEST; slave_tx_buffer[4] = 40; }
        else if (current_channel == 6) { current_cA = current_cA_TEST; slave_tx_buffer[4] = 40; }
        else if (current_channel == 7) { current_cA = current_cA_TEST; slave_tx_buffer[4] = 40; }
        else if (current_channel == 8) { current_cA = current_cA_TEST; slave_tx_buffer[4] = 40; }
        else if (current_channel == 9) { current_cA = current_cA_TEST; slave_tx_buffer[4] = 40; }
        else { current_cA = 0; slave_tx_buffer[4] = 0; }

        current_code = (uint16_t)(current_cA + 32768);
        slave_tx_buffer[1] = (slave_id << 3) | 0x01;
    }


    // === 패킷 구성 ===
    slave_tx_buffer[0] = 0x02; // STX
    // slave_tx_buffer[1] = ID+Status (위에서 설정함)   DAB_OK : 0 = Fail, 1 = OK
    slave_tx_buffer[2] = (uint8_t)(current_code >> 8); // 전류 상위 바이트
    slave_tx_buffer[3] = (uint8_t)(current_code & 0xFF); // 전류 하위 바이트
    // slave_tx_buffer[4] = 온도 (이미 설정함)

    // 체크섬 (Byte1~4 합)
    checksum = 0;
    for (i = 1; i <= 4; i++) checksum += slave_tx_buffer[i];
    slave_tx_buffer[5] = checksum & 0xFF;
    slave_tx_buffer[6] = 0x03; // ETX

    // === SCI-D 송신 ===
    // (DriverLib HAL, blocking 방식 → 안전하게 프레임 보장)
    for (i = 0; i < 7; i++)
    {
        SCI_writeCharBlockingFIFO(SCID_BASE, slave_tx_buffer[i]);
    }

    // === 다음 채널로 순환 ===
    current_channel++;
    if (current_channel >= max_channels) current_channel = 0;

}

#if 0
void send_slave_data_to_hmi(void)
{
    static uint8_t current_channel = 0; // 현재 채널 index
    static uint8_t slave_count = 0;     // 활성 슬레이브 수
    uint8_t slave_id;
    int current_cA = 0;
    uint16_t current_code = 0;
    uint8_t temp_data = 0;
    uint8_t checksum = 0;
    uint16_t i;

    // === 활성 슬레이브 수 업데이트 ===
    slave_count = 0;
    for (i = 1; i <= 16; i++)
    {
        if (slave_enabled[i])
        {
            slave_count++;
        }
    }
    if (slave_count == 0) return; // 살아있는 슬레이브 없으면 송신 안 함

    // === 현재 채널이 비활성 슬레이브면 다음 채널로 이동 ===
    while (!slave_enabled[current_channel + 1])
    {
        current_channel++;
        if (current_channel >= 31) current_channel = 0;
    }

    slave_id = current_channel;

    memset(slave_tx_buffer, 0, sizeof(slave_tx_buffer));

#if USE_TEST_DATA == 0
    // --- 실제 CAN 데이터 ---
    current_cA = (int)(I_out_ch[current_channel] * 100); // 0.01A 단위
    if (current_cA > 32767) current_cA = 32767;
    if (current_cA < -32768) current_cA = -32768;

    current_code = (uint16_t)(current_cA + 32768);

    temp_data = (uint8_t)(Temp_ch[current_channel] * 2);
    if (temp_data > 255) temp_data = 255;
    slave_tx_buffer[4] = temp_data;

    slave_tx_buffer[1] = (slave_id << 3) | (DAB_ok_ch[current_channel] & 0x01);
#else
    // --- 테스트 데이터 ---
    current_cA = current_cA_TEST;
    current_code = (uint16_t)(current_cA + 32768);
    slave_tx_buffer[4] = 40;
    slave_tx_buffer[1] = (slave_id << 3) | 0x01;
#endif

    // === 패킷 구성 ===
    slave_tx_buffer[0] = 0x02;
    slave_tx_buffer[2] = (uint8_t)(current_code >> 8);
    slave_tx_buffer[3] = (uint8_t)(current_code & 0xFF);
    // [4]는 위에서 설정
    checksum = 0;
    for (i = 1; i <= 4; i++) checksum += slave_tx_buffer[i];
    slave_tx_buffer[5] = checksum & 0xFF;
    slave_tx_buffer[6] = 0x03;

    // === SCI-D 송신 ===
    for (i = 0; i < 7; i++) {
        SCI_writeCharBlockingFIFO(SCID_BASE, slave_tx_buffer[i]);
    }

    // === 다음 채널로 이동 ===
    do {
        current_channel++;
        if (current_channel >= 31) current_channel = 0;
    } while (!slave_enabled[current_channel + 1]);
}
#endif

//========================
// System Voltage → HMI 송신 (50ms 주기)
//========================
// 7바이트 패킷: STX | Master_ID+Rack_Ch | Voltage_H | Voltage_L | Reserved | Checksum | ETX
//========================
void send_system_voltage_to_hmi(void)       // System Voltage를 HMI로 송신
{
    uint16_t voltage_scaled = 0;

    // === 테스트 모드 제어 스위치 ===
    #define USE_TEST_DATA 0   // 0: 실제 Vo_Mean 사용, 1: 테스트 값 사용

#if (USE_TEST_DATA == 0)
    // --- 실제 데이터 사용 ---
    voltage_scaled = (uint16_t)(Vo_Mean * 10.0f + 300); // 0.1V 단위
    if (voltage_scaled > 65535) voltage_scaled = 65535;

#else
    // --- 테스트 데이터 ---
    // 필요에 따라 전압을 가짜 값으로 만들어 송신

    test_voltage += 500;                 // 호출할 때마다 +5.0V 증가
    if (test_voltage > 5000) test_voltage = 0;  // 500.0V 도달 시 0으로 리셋

    voltage_scaled = test_voltage;
#endif

    // === 패킷 구성 ===
    system_tx_buffer[0] = 0x02; // STX
    system_tx_buffer[1] = ((Master_ID & 0x1F) << 3) | (Rack_Channel & 0x07); // ID + 렉채널
    system_tx_buffer[2] = (uint8_t)(voltage_scaled >> 8);   // 전압 상위 바이트
    system_tx_buffer[3] = (uint8_t)(voltage_scaled & 0xFF); // 전압 하위 바이트
    system_tx_buffer[4] = 0x00;                             // Reserved

    uint8_t checksum = 0;
    for (uint8_t i = 1; i <= 4; i++)
    {
        checksum += system_tx_buffer[i];
    }
    system_tx_buffer[5] = checksum & 0xFF;
    system_tx_buffer[6] = 0x03; // ETX

    SCI_writeCharArray(SCID_BASE, system_tx_buffer, 7);
}

//========================
// SCID RX 인터럽트 (HMI 패킷 수신)
//========================
// 10바이트 패킷 수신: STX | CMD | Vmax_H | Vmax_L | Vmin_H | Vmin_L | Icmd_H | Icmd_L | Checksum | ETX
//========================
uint8_t receivedByte;
__interrupt void scidRxReadyISR(void)
{
    // 1) RX 에러 상태 확인
    uint16_t rxst = HWREGH(SCID_BASE + SCI_O_RXST);
    if (rxst & (SCI_RXST_FE | SCI_RXST_OE | SCI_RXST_PE | SCI_RXST_BRKDT))
    {
        SCI_resetRxFIFO(SCID_BASE);
        SCI_clearInterruptStatus(SCID_BASE, SCI_INT_RXFF | SCI_INT_RXERR);
        hmi_rx_index = 0;
        Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP8);
        return;
    }

    // 2) FIFO 카운트만큼 읽기
    uint16_t n = (uint16_t)SCI_getRxFIFOStatus(SCID_BASE);
    while (n--)
    {
        uint8_t b = (uint8_t)(SCI_readCharBlockingFIFO(SCID_BASE) & 0xFF);

        if (hmi_rx_index == 0)      // 첫 바이트는 반드시 STX
        {
            if (b == 0x02) hmi_rx_buffer[hmi_rx_index++] = b;
            // STX 아닐 경우 무시
        }
        else
        {
            // 나머지 바이트 저장
            hmi_rx_buffer[hmi_rx_index++] = b;

            if (hmi_rx_index == 10)     // 10바이트 다 받았을 때만
            {
                if (hmi_rx_buffer[0] == 0x02 && hmi_rx_buffer[9] == 0x03)
                {
                    uint8_t checksum = 0;
                    for (uint8_t i = 1; i <= 7; i++)    // Byte1~7 합산
                    {
                        checksum += hmi_rx_buffer[i];
                    }
                    if ((checksum & 0xFF) == hmi_rx_buffer[8])
                    {
                        hmi_packet_ready = 1; // 유효 프레임 확정
                        modbus_parse();
                    }
                }
                hmi_rx_index = 0;       // 다음 패킷 준비
            }
        }
    }
    // 3) 인터럽트 플래그 클리어
    SCI_clearInterruptStatus(SCID_BASE, SCI_INT_RXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP8);
}

//========================
// HMI 패킷 파싱 (제어 변수 반영)
//========================
// 수신된 10바이트 패킷 해석 → start_stop, operation_mode, Voh_cmd, Vol_cmd, I_out_ref 갱신
//========================
void modbus_parse(void)     // 패킷 해석 및 제어변수 반영
{   // 패킷 해석
    hmi_rx_data.start_byte   = hmi_rx_buffer[0];
    hmi_rx_data.command      = hmi_rx_buffer[1]; // bit0=Run, bit2:1=Mode (0:정지,1:CH1,2:CH2,3:병렬)
    hmi_rx_data.max_voltage  = (int16_t)((hmi_rx_buffer[2] << 8) | hmi_rx_buffer[3]);
    hmi_rx_data.min_voltage  = (int16_t)((hmi_rx_buffer[4] << 8) | hmi_rx_buffer[5]);

    // Current Command (2바이트, Center=32768, 0.01A 단위)
    hmi_rx_data.current_cmd  = ((int16_t)((hmi_rx_buffer[6] << 8) | hmi_rx_buffer[7])) - 32768;

    hmi_rx_data.checksum     = hmi_rx_buffer[8];
    hmi_rx_data.end_byte     = hmi_rx_buffer[9];

    // === 제어 변수 반영 ===
// command 바이트 해석
    start_stop = (hmi_rx_data.command & 0x01);        // bit0 = Run
    operation_mode = (OperationMode_t)((hmi_rx_data.command >> 1) & 0x03);
    Voh_cmd = (float)hmi_rx_data.max_voltage;
    Vol_cmd = (float)hmi_rx_data.min_voltage;
    I_out_ref = (float)hmi_rx_data.current_cmd / 10.0f; // A 단위 변환

    hmi_packet_ready = 0;   // 다음 패킷 대기
}

//==================================================
// [10] 디버그 함수 (조건부 컴파일)
//==================================================

//========================
// 클럭 상태 디버그 (_DEBUG_CLK_STATUS_ENABLE_)
//========================
#if _DEBUG_CLK_STATUS_ENABLE_
void debug_CLK_status(void)
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

//========================
// CAN 상태 디버그 (_DEBUG_CAN_STATUS_ENABLE_)
//========================
#if _DEBUG_CAN_STATUS_ENABLE_
void debug_CAN_status(void)
{
    // CAN 에러 상태 및 에러 카운트 레지스터 읽기
    uint16_t errorStatus = HWREGH(CANA_BASE + CAN_O_ES);
    uint16_t errorCount  = HWREGH(CANA_BASE + CAN_O_ERRC);

    // 에러 카운트 추출
    uint16_t txErrCnt = errorCount & 0x00FF;
    uint16_t rxErrCnt = (errorCount >> 8) & 0x00FF;

    // 에러 상태 플래그 추출 (스타일 통일)
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

    debug_tx_err_count = txErrCnt;
    debug_rx_err_count = rxErrCnt;
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
#endif  // _DEBUG_CAN_STATUS_ENABLE_

//========================
// SCI 상태 디버그 (_DEBUG_SCI_STATUS_ENABLE_)
//========================
#if _DEBUG_SCI_STATUS_ENABLE_
void debug_SCI_status(void)
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

    // 예시: LED 디버깅 출력
    if (isOverrunError)   GPIO_togglePin(LED31);
    if (isFramingError)   GPIO_togglePin(LED32);
    if (isParityError)    GPIO_togglePin(LED33);

    // 디버깅 변수 기록
    debug_sci_rx_error  = isOverrunError;
    debug_sci_frame_err = isFramingError;
    debug_sci_parity_err= isParityError;
    debug_sci_break     = isBreakDetected;
    debug_sci_tx_ready  = txReady;
    debug_sci_rx_ready  = rxReady;
}
#endif // _DEBUG_SCI_STATUS_ENABLE_

//========================
// SPI 상태 디버그 (_DEBUG_SPI_STATUS_ENABLE_)
//========================
#if _DEBUG_SPI_STATUS_ENABLE_
void debug_SPI_status(void)
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

    // 디버그 출력 또는 변수 설정
    debug_spi_tx_full     = tx_buf_full;
    debug_spi_int         = int_flag;
    debug_spi_overrun     = overrun_error;
    debug_spi_tx_ready    = tx_fifo_ready;
    debug_spi_rx_hasdata  = rx_fifo_data;
    debug_spi_rx_overflow = rx_fifo_ovf;
}
#endif // _DEBUG_SPI_STATUS_ENABLE_

//==================================================
// End of HABA_Ctrl.c
//==================================================
