//
// HABA_setup.c - Hardware Initialization and System Setup
//
#include "HABA_setup.h"
#include "HABA_control.h"

// CLA Task 함수 선언 (HABA_cla_tasks.cla에 정의됨)
extern __interrupt void Cla1Task1(void);
extern __interrupt void Cla1Task2(void);
extern __interrupt void Cla1Task3(void);

void Init_System()
{
    Interrupt_initModule();
    Interrupt_initVectorTable();

    // Device_bootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_FLASH);


    Init_GPIO_CPU1();
    Init_GPIO_CPU2();


    Init_CANA();
    Init_SCIA();
    Init_SCIB();
    Init_SCID_SCADA();
    Init_SPI_DAC1();
    //Init_SPIA();
    Init_SPIC();
    Init_ADCA();
    Init_EPWM();

    Init_CPU1_CLA1();

    Init_INTERRUPT();


    Read_Master_ID_From_DIP();

    lpf_coeff_a = wc * Ts / (2 + wc*Ts);
    lpf_coeff_b = (2 - wc * Ts) / (2 + wc * Ts);
    if (master_id == 0)
    {
        GPIO_writePin(SCIA_RS485_MM_DE_GPIO, 1);   // 상위만 DE=1
        GPIO_writePin(30, 1);               // 디버깅 핀
    }
}


void Init_EPWM(void)
{
    EPWMConfig epwm1_cfg = {
        .base = EPWM1_BASE,
        .period = 900,
        .dutyA = 800,
        .dutyB = 0,
        .useInterrupt = true,
        .enableSOCA = false,
        .enableOutput = true, // EPWM1A 출력 활성화
        .socType = EPWM_SOC_A,
        .socTriggerSource = EPWM_SOC_TBCTR_U_CMPA,
        .socEventPrescale = 1
    };

    EPWMConfig epwm3_cfg = {
        .base = EPWM3_BASE,
        .period = 2000,
        .dutyA = 500,
        .dutyB = 0,
        .useInterrupt = false,
        .enableSOCA = true,
        .enableOutput = false, // EPWM3A 출력 비활성화
        .socType = EPWM_SOC_A,
        .socTriggerSource = EPWM_SOC_TBCTR_U_CMPA,
        .socEventPrescale = 1
    };

    Set_EPWM(&epwm1_cfg);
    Set_EPWM(&epwm3_cfg);
}

void Set_EPWM(const EPWMConfig *cfg)
{
    EPWM_setClockPrescaler(cfg->base, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
    EPWM_setTimeBasePeriod(cfg->base, cfg->period);
    EPWM_setTimeBaseCounter(cfg->base, 0);
    EPWM_setTimeBaseCounterMode(cfg->base, EPWM_COUNTER_MODE_UP);
    EPWM_disablePhaseShiftLoad(cfg->base);
    EPWM_setPhaseShift(cfg->base, 0);
    EPWM_setSyncOutPulseMode(cfg->base, EPWM_SYNC_OUT_PULSE_DISABLED);

    // PWM Compare 설정
    EPWM_setCounterCompareValue(cfg->base, EPWM_COUNTER_COMPARE_A, cfg->dutyA);
    EPWM_setCounterCompareValue(cfg->base, EPWM_COUNTER_COMPARE_B, cfg->dutyB);
    EPWM_setCounterCompareShadowLoadMode(cfg->base, EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setCounterCompareShadowLoadMode(cfg->base, EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);

    // PWM 출력 설정
    if (cfg->enableOutput)
    {
        EPWM_setActionQualifierAction(cfg->base, EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
        EPWM_setActionQualifierAction(cfg->base, EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_HIGH,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    }
    else
    {
        EPWM_setActionQualifierAction(cfg->base, EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_NO_CHANGE,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
        EPWM_setActionQualifierAction(cfg->base, EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_NO_CHANGE,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    }


    // SOC 트리거 설정
    if (cfg->enableSOCA)
    {
        EPWM_setADCTriggerSource(cfg->base, cfg->socType, cfg->socTriggerSource);
        EPWM_setADCTriggerEventPrescale(cfg->base, cfg->socType, cfg->socEventPrescale);
        EPWM_enableADCTrigger(cfg->base, cfg->socType);
    }
    else
    {
        EPWM_disableADCTrigger(cfg->base, cfg->socType);
    }

    // 인터럽트 설정
    if (cfg->useInterrupt)
    {
        EPWM_setInterruptSource(cfg->base, EPWM_INT_TBCTR_PERIOD);
        EPWM_setInterruptEventCount(cfg->base, 1);
        EPWM_enableInterrupt(cfg->base);
    }
}



void Init_CANA(void)
{
    uint16_t i;
    CAN_initModule(CANA_BASE);

    CAN_setBitRate(CANA_BASE, DEVICE_SYSCLK_FREQ, 1000000, 10);

    // 송신 메일박스
    CAN_setupMessageObject(CANA_BASE, TX_MSG_OBJ_ID1, TX_MSG_ID1,
                           CAN_MSG_FRAME_STD, CAN_MSG_OBJ_TYPE_TX, 0,
                           CAN_MSG_OBJ_NO_FLAGS, MSG_DATA_LENGTH);

    // 수신 메일박스: enum을 기반으로 반복 설정

    for(i = 1; i <= SLAVE_NUM; i++)
    {
        CAN_setupMessageObject(CANA_BASE,
                               1 + i,                  // Mailbox 번호 2~32
                               TX_MSG_ID1 + i,         // ID 0x0E1 ~ 0x0FF
                               CAN_MSG_FRAME_STD,
                               CAN_MSG_OBJ_TYPE_RX,
                               0,
                               CAN_MSG_OBJ_NO_FLAGS,
                               4);
    }

    CAN_startModule(CANA_BASE);

}




void Init_SCIA(void)
{
    // 1. SCI 소프트 리셋
    SCI_performSoftwareReset(SCIA_BASE);

    // 2. SCI 설정 (8bit, 1 stop, 짝수 패리티)
    SCI_setConfig(SCIA_BASE, DEVICE_SYSCLK_FREQ, DEVICE_SCI_BAUDRATE,
                  (SCI_CONFIG_WLEN_8 | SCI_CONFIG_STOP_ONE | SCI_CONFIG_PAR_EVEN));

    // 3. SCI 모듈 및 채널 활성화
    SCI_enableModule(SCIA_BASE);
    SCI_resetChannels(SCIA_BASE);

    // 4. 루프백 모드 (테스트 시만 사용)
    // SCI_enableLoopback(SCIA_BASE);  // 실사용 시 주석 처리

    // 5. FIFO 설정
    SCI_enableFIFO(SCIA_BASE);
    SCI_setFIFOInterruptLevel(SCIA_BASE, SCI_FIFO_TX0, SCI_FIFO_RX1); // RX FIFO Level = 1
    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_TXFF | SCI_INT_RXFF);
    SCI_enableInterrupt(SCIA_BASE, SCI_INT_RXFF); // RX FIFO 인터럽트만 활성화


    GPIO_WritePin(SCIB_RS485_MS_DE_GPIO, 1);
}


void Init_SCIB(void)
{
    // 1. SCI 소프트 리셋
    SCI_performSoftwareReset(SCIB_BASE);

    // 2. SCI 설정 (8bit, 1 stop, 짝수 패리티)
    SCI_setConfig(SCIB_BASE, DEVICE_SYSCLK_FREQ, DEVICE_SCI_BAUDRATE,
                  (SCI_CONFIG_WLEN_8 | SCI_CONFIG_STOP_ONE | SCI_CONFIG_PAR_EVEN));

    // 3. SCI 모듈 및 채널 활성화
    SCI_enableModule(SCIB_BASE);
    SCI_resetChannels(SCIB_BASE);

    // 4. 루프백 모드 (테스트 시만 사용)
    // SCI_enableLoopback(SCIB_BASE);  // 실사용 시 주석 처리

    // 5. FIFO 설정
    SCI_enableFIFO(SCIB_BASE);
    SCI_setFIFOInterruptLevel(SCIB_BASE, SCI_FIFO_TX0, SCI_FIFO_RX1); // RX FIFO Level = 1
    SCI_clearInterruptStatus(SCIB_BASE, SCI_INT_TXFF | SCI_INT_RXFF);
    SCI_enableInterrupt(SCIB_BASE, SCI_INT_RXFF);  // RX FIFO 인터럽트만 활성화
}

void Init_SCID_SCADA(void)
{
    // 1. SCI 소프트 리셋
    SCI_performSoftwareReset(SCID_BASE);

    // 2. SCI 설정
    SCI_setConfig(SCID_BASE, DEVICE_LSPCLK_FREQ, SCADA_BAUD_RATE,
                  (SCI_CONFIG_WLEN_8 | SCI_CONFIG_STOP_ONE | SCI_CONFIG_PAR_NONE));

    // 3. 모듈 Enable + 채널 Reset
    SCI_enableModule(SCID_BASE);
    SCI_resetChannels(SCID_BASE);

    // 4. FIFO Reset + 설정
    SCI_resetTxFIFO(SCID_BASE);
    SCI_resetRxFIFO(SCID_BASE);

    SCI_enableFIFO(SCID_BASE);
    SCI_setFIFOInterruptLevel(SCID_BASE, SCI_FIFO_TX0, SCI_FIFO_RX9);

    // 5. 인터럽트 클리어 및 Enable
    SCI_clearInterruptStatus(SCID_BASE, SCI_INT_TXFF | SCI_INT_RXFF | SCI_INT_RXERR);
    SCI_enableInterrupt(SCID_BASE, (SCI_INT_RXFF | SCI_INT_RXERR));
}

void Init_SPI_DAC1(void)
{
    SPI_disableModule(SPIA_BASE);

    // Mode-1: CPOL=0, CPHA=1 (DAC 요구사항)
    SPI_setConfig(SPIA_BASE,
                  DEVICE_LSPCLK_FREQ,
                  SPI_PROT_POL0PHA0,
                  SPI_MODE_CONTROLLER,
                  SPI_DAC1_BITRATE,     // 6.25MHz 이하 권장
                  8);          // 8bit 전송

    // CS 자동 모드: 4-Wire 구성 유지
    SPI_disableTriWire(SPIA_BASE);  // 4-Wire Mode
    SPI_enableFIFO(SPIA_BASE);


    SPI_disableLoopback(SPIA_BASE);
    SPI_setEmulationMode(SPIA_BASE, SPI_EMULATION_STOP_AFTER_TRANSMIT);
    SPI_enableTalk(SPIA_BASE);
    SPI_enableModule(SPIA_BASE);
}

void Init_SPIA(void)
{
    // SPIA 모듈 비활성화
    SPI_disableModule(SPIA_BASE);

    // FIFO 사용 ON
    SPI_enableFIFO(SPIA_BASE);

    // 인터럽트 레벨 설정: RX FIFO에 3워드 이상 쌓였을 때 발생
    SPI_clearInterruptStatus(SPIA_BASE, SPI_INT_RXFF | SPI_INT_TXFF);
    SPI_setFIFOInterruptLevel(SPIA_BASE, SPI_FIFO_TXEMPTY, SPI_FIFO_RX3);

    // SPIA 설정
    // CPOL/CPHA 모드는 TLV5638 데이터시트에 맞게 수정 필요
    SPI_setConfig(SPIA_BASE,
                  DEVICE_LSPCLK_FREQ,
                  SPI_PROT_POL1PHA1,   // TLV5638: CPOL=1, CPHA=1 추천
                  SPI_MODE_MASTER,
                  10000000,            // 10 MHz
                  16);                 // 16-bit 전송

    // 디버깅 중에도 동작 유지
    SPI_setEmulationMode(SPIA_BASE, SPI_EMULATION_FREE_RUN);

    // SPIA 모듈 활성화
    SPI_enableModule(SPIA_BASE);

    // RX FIFO 인터럽트 활성화 (필요 시)
    SPI_enableInterrupt(SPIA_BASE, SPI_INT_RXFF);
}



void Init_SPIC(void)
{
    SPI_disableModule(SPIC_BASE);

    // FIFO 사용 ON
    SPI_enableFIFO(SPIC_BASE);

    // 인터럽트 레벨 설정: RX FIFO에 3워드 이상 쌓였을 때 발생
    SPI_clearInterruptStatus(SPIC_BASE, SPI_INT_RXFF | SPI_INT_TXFF);
    SPI_setFIFOInterruptLevel(SPIC_BASE, SPI_FIFO_TXEMPTY, SPI_FIFO_RX3);

    SPI_setConfig(SPIC_BASE,
                  DEVICE_LSPCLK_FREQ,
                  SPI_PROT_POL0PHA0,
                  SPI_MODE_MASTER,
                  10000000,
                  16);

    SPI_setEmulationMode(SPIC_BASE, SPI_EMULATION_FREE_RUN);

    SPI_enableModule(SPIC_BASE);

    // RX FIFO 인터럽트 활성화
    SPI_enableInterrupt(SPIC_BASE, SPI_INT_RXFF);
}


void Init_ADCA(void)
{
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_4_0);
    ADC_setMode(ADCA_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
    ADC_setInterruptPulseMode(ADCA_BASE, ADC_PULSE_END_OF_CONV);
    ADC_enableConverter(ADCA_BASE);

    // A0 → SOC0 (NTC1)
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM3_SOCA,
                 ADC_CH_ADCIN0, 14);
    // A1 → SOC1 (NTC2)
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM3_SOCA,
                 ADC_CH_ADCIN1, 14);

    // INT1 ← SOC0 기준 인터럽트
    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER0);
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);

    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);

}


void Init_INTERRUPT(void)
{
    Interrupt_register(INT_EPWM1, &INT_EPWM1_ISR);
    Interrupt_register(INT_ADCA1, &INT_ADCA1_ISR);

    Interrupt_register(INT_CLA1_1, &CLA1_ISR1);
    Interrupt_register(INT_CLA1_2, &CLA1_ISR2);


    Interrupt_register(INT_SPIC_RX, SPIC_FPGA_Rx_ISR);
    Interrupt_register(INT_SCIA_RX, SCIA_RS485_MM_Rx_ISR);
    Interrupt_register(INT_SCID_RX, &SCID_SCADA_Rx_ISR);   // SCI-D(SCADA)



    Interrupt_enable(INT_SPIC_RX);
    Interrupt_enable(INT_SCIA_RX);
    Interrupt_enable(INT_SCID_RX); // SCI-D(SCADA)

    Interrupt_enable(INT_EPWM1);
    Interrupt_enable(INT_ADCA1);

    Interrupt_enable(INT_CLA1_1);
    Interrupt_enable(INT_CLA1_2);

    Interrupt_enableInCPU(INT_EPWM1);
    Interrupt_enableInCPU(INT_SCIA_RX); // ★ 반드시 필요
    Interrupt_enableInCPU(INT_SCID_RX); // SCI-D(SCADA)

    Interrupt_enableMaster();
}


//
//=============================
// CLA 초기화 및 인터럽트
//=============================
void Init_CPU1_CLA1(void)
{

    #ifdef _FLASH
    extern uint32_t Cla1funcsLoadStart, Cla1funcsLoadSize, Cla1funcsRunStart;
    memcpy((void*)&Cla1funcsRunStart, (void*)&Cla1funcsLoadStart, (uint32_t)&Cla1funcsLoadSize);
#endif

    // 메시지 RAM 초기화 (CLA1↔CPU)
    MemCfg_initSections(MEMCFG_SECT_MSGCLA1TOCPU | MEMCFG_SECT_MSGCPUTOCLA1);

    // LS4, LS5를 CLA 코드용 영역으로 지정
    MemCfg_setLSRAMControllerSel(MEMCFG_SECT_LS4, MEMCFG_LSRAMCONTROLLER_CPU_CLA1);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS4, MEMCFG_CLA_MEM_PROGRAM);

    MemCfg_setLSRAMControllerSel(MEMCFG_SECT_LS5, MEMCFG_LSRAMCONTROLLER_CPU_CLA1);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS5, MEMCFG_CLA_MEM_PROGRAM);

    // LS0, LS1을 CLA 데이터 영역으로 지정
    MemCfg_setLSRAMControllerSel(MEMCFG_SECT_LS0, MEMCFG_LSRAMCONTROLLER_CPU_CLA1);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS0, MEMCFG_CLA_MEM_DATA);

    MemCfg_setLSRAMControllerSel(MEMCFG_SECT_LS1, MEMCFG_LSRAMCONTROLLER_CPU_CLA1);
    MemCfg_setCLAMemType(MEMCFG_SECT_LS1, MEMCFG_CLA_MEM_DATA);

    CLA_mapTaskVector(CLA1_BASE, CLA_MVECT_1, (uint16_t)&Cla1Task1);
    CLA_mapTaskVector(CLA1_BASE, CLA_MVECT_2, (uint16_t)&Cla1Task2);
    CLA_mapTaskVector(CLA1_BASE, CLA_MVECT_3, (uint16_t)&Cla1Task3);

    CLA_enableIACK(CLA1_BASE);
    CLA_enableTasks(CLA1_BASE, CLA_TASKFLAG_ALL);

    //==================================================
    // DCL PI 제어기 초기화
    //==================================================
    // DCL_PI_L1: 이산시간 PI 제어기
    // Ki는 연속시간 게인에 샘플링 시간을 곱한 값
    // DCL_PI_CLA는 리셋 함수가 없으므로 수동 초기화
    //==================================================
    
    // pi_charge: 충전 모드 (V_max_cmd 기준)
    pi_charge.Kp   = Kp_set;                        // 비례 게인 = 1
    pi_charge.Ki   = Ki_set * T_sample_set;         // 적분 게인 = 3000 * 50e-6 = 0.15
    pi_charge.Umax = CURRENT_LIMIT_PARALLEL;        // 출력 상한 = 960A (최대 모드 기준)
    pi_charge.Umin = -2.0f;                         // 출력 하한 = -2A
    pi_charge.Imax = CURRENT_LIMIT_PARALLEL;        // Anti-windup 상한 = 960A (동적 업데이트)
    pi_charge.Imin = -2.0f;                         // Anti-windup 하한 = -2A
    // 내부 상태 수동 초기화 (DCL_PI_CLA에는 reset 함수 없음)
    pi_charge.i10  = 0.0f;                          // 적분기 값 초기화
    pi_charge.i6   = 1.0f;                          // Saturation flag 초기화
    pi_charge.i11  = 0.0f;                          // Tustin integrator 초기화

    // pi_discharge: 방전 모드 (V_min_cmd 기준)
    pi_discharge.Kp    = Kp_set;                    // 비례 게인 = 1
    pi_discharge.Ki    = Ki_set * T_sample_set;     // 적분 게인 = 3000 * 50e-6 = 0.15
    pi_discharge.Umax  = 2.0f;                      // 출력 상한 = 2A
    pi_discharge.Umin  = -CURRENT_LIMIT_PARALLEL;   // 출력 하한 = -960A (최대 모드 기준)
    pi_discharge.Imax  = 2.0f;                      // Anti-windup 상한 = 2A
    pi_discharge.Imin  = -CURRENT_LIMIT_PARALLEL;   // Anti-windup 하한 = -960A (동적 업데이트)
    // 내부 상태 수동 초기화
    pi_discharge.i10   = 0.0f;                      // 적분기 값 초기화
    pi_discharge.i6    = 1.0f;                      // Saturation flag 초기화
    pi_discharge.i11   = 0.0f;                      // Tustin integrator 초기화

    // pi_cv: Battery 모드 CV 제어 (V_cmd 기준)
    pi_cv.Kp   = Kp_set;                            // 비례 게인 = 1
    pi_cv.Ki   = Ki_set * T_sample_set;             // 적분 게인 = 3000 * 50e-6 = 0.15
    pi_cv.Umax = CURRENT_LIMIT_PARALLEL;            // 출력 상한 = 960A (I_max_cmd로 동적 업데이트)
    pi_cv.Umin = -CURRENT_LIMIT_PARALLEL;           // 출력 하한 = -960A (I_min_cmd로 동적 업데이트)
    pi_cv.Imax = CURRENT_LIMIT_PARALLEL;            // Anti-windup 상한 = 960A (동적 업데이트)
    pi_cv.Imin = -CURRENT_LIMIT_PARALLEL;           // Anti-windup 하한 = -960A (동적 업데이트)
    // 내부 상태 수동 초기화
    pi_cv.i10  = 0.0f;                              // 적분기 값 초기화
    pi_cv.i6   = 1.0f;                              // Saturation flag 초기화
    pi_cv.i11  = 0.0f;                              // Tustin integrator 초기화
}



void Init_GPIO_CPU1(void)
{

    GPIO_setPinConfig(GPIO_0_EPWM1A);
    GPIO_setDirectionMode(0, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(0, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(0, GPIO_CORE_CPU1);  // CPU1 사용 시

    // Digital Input 설정 (GPIO11 ~ GPIO18)
    GPIO_setPinConfig(GPIO_11_GPIO11);
    GPIO_setPadConfig(11, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(11, GPIO_QUAL_ASYNC);
    GPIO_setDirectionMode(11, GPIO_DIR_MODE_IN);
    GPIO_setControllerCore(11, GPIO_CORE_CPU1);

    GPIO_setPinConfig(GPIO_12_GPIO12);
    GPIO_setPadConfig(12, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(12, GPIO_QUAL_ASYNC);
    GPIO_setDirectionMode(12, GPIO_DIR_MODE_IN);
    GPIO_setControllerCore(12, GPIO_CORE_CPU1);

    GPIO_setPinConfig(GPIO_13_GPIO13);
    GPIO_setPadConfig(13, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(13, GPIO_QUAL_ASYNC);
    GPIO_setDirectionMode(13, GPIO_DIR_MODE_IN);
    GPIO_setControllerCore(13, GPIO_CORE_CPU1);

    GPIO_setPinConfig(GPIO_14_GPIO14);
    GPIO_setPadConfig(14, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(14, GPIO_QUAL_ASYNC);
    GPIO_setDirectionMode(14, GPIO_DIR_MODE_IN);
    GPIO_setControllerCore(14, GPIO_CORE_CPU1);

    GPIO_setPinConfig(GPIO_15_GPIO15);
    GPIO_setPadConfig(15, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(15, GPIO_QUAL_ASYNC);
    GPIO_setDirectionMode(15, GPIO_DIR_MODE_IN);
    GPIO_setControllerCore(15, GPIO_CORE_CPU1);

    // Digital Output 설정 (GPIO8, GPIO9, GPIO22~27)
    GPIO_setPinConfig(GPIO_8_GPIO8);
    GPIO_setPadConfig(8, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(8, GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(8, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(8, GPIO_CORE_CPU1);
    GPIO_writePin(8, 0);

    GPIO_setPinConfig(GPIO_9_GPIO9);
    GPIO_setPadConfig(9, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(9, GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(9, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(9, GPIO_CORE_CPU1);
    GPIO_writePin(9, 0);

    GPIO_setPinConfig(GPIO_22_GPIO22);
    GPIO_setPadConfig(22, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(22, GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(22, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(22, GPIO_CORE_CPU1);
    GPIO_writePin(22, 0);

    GPIO_setPinConfig(GPIO_23_GPIO23);
    GPIO_setPadConfig(23, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(23, GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(23, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(23, GPIO_CORE_CPU1);
    GPIO_writePin(23, 0);

    GPIO_setPinConfig(GPIO_24_GPIO24);
    GPIO_setPadConfig(24, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(24, GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(24, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(24, GPIO_CORE_CPU1);
    GPIO_writePin(24, 0);

    GPIO_setPinConfig(GPIO_25_GPIO25);
    GPIO_setPadConfig(25, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(25, GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(25, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(25, GPIO_CORE_CPU1);
    GPIO_writePin(25, 0);

    GPIO_setPinConfig(GPIO_26_GPIO26);
    GPIO_setPadConfig(26, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(26, GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(26, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(26, GPIO_CORE_CPU1);
    GPIO_writePin(26, 0);

    GPIO_setPinConfig(GPIO_27_GPIO27);
    GPIO_setPadConfig(27, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(27, GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(27, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(27, GPIO_CORE_CPU1);
    GPIO_writePin(27, 0);


    // GPIO28 설정
    GPIO_setPinConfig(GPIO_28_GPIO28);
    GPIO_setPadConfig(28, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(28, GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(28, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(28, GPIO_CORE_CPU1);

    // GPIO29 설정
    GPIO_setPinConfig(GPIO_29_GPIO29);
    GPIO_setPadConfig(29, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(29, GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(29, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(29, GPIO_CORE_CPU1);

    // GPIO30 설정
    GPIO_setPinConfig(GPIO_30_GPIO30);
    GPIO_setPadConfig(30, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(30, GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(30, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(30, GPIO_CORE_CPU1);

    // GPIO31 설정
    GPIO_setPinConfig(GPIO_31_GPIO31);
    GPIO_setPadConfig(31, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(31, GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(31, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(31, GPIO_CORE_CPU1);

    // GPIO32 설정
    GPIO_setPinConfig(GPIO_32_GPIO32);
    GPIO_setPadConfig(32, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(32, GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(32, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(32, GPIO_CORE_CPU1);

    // GPIO33 설정
    GPIO_setPinConfig(GPIO_33_GPIO33);
    GPIO_setPadConfig(33, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(33, GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(33, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(33, GPIO_CORE_CPU1);

    // GPIO34 설정
    GPIO_setPinConfig(GPIO_34_GPIO34);
    GPIO_setPadConfig(34, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(34, GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(34, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(34, GPIO_CORE_CPU1);

    // GPIO35 설정
    GPIO_setPinConfig(GPIO_35_GPIO35);
    GPIO_setPadConfig(35, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(35, GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(35, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(35, GPIO_CORE_CPU1);

    // GPIO36 ~ 39 ADRESS
    // GPIO36 설정
    GPIO_setPinConfig(GPIO_36_GPIO36);
    GPIO_setPadConfig(36, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(36, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(36, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(36, GPIO_CORE_CPU1);

    // GPIO37 설정
    GPIO_setPinConfig(GPIO_37_GPIO37);
    GPIO_setPadConfig(37, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(37, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(37, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(37, GPIO_CORE_CPU1);

    // GPIO38 설정
    GPIO_setPinConfig(GPIO_38_GPIO38);
    GPIO_setPadConfig(38, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(38, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(38, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(38, GPIO_CORE_CPU1);

    // GPIO39 설정
    GPIO_setPinConfig(GPIO_39_GPIO39);
    GPIO_setPadConfig(39, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(39, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(39, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(39, GPIO_CORE_CPU1);

    // GPIO42 설정
    GPIO_setPinConfig(GPIO_42_GPIO42);
    GPIO_setPadConfig(42, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(42, GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(42, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(42, GPIO_CORE_CPU1);

    // GPIO43 설정
    GPIO_setPinConfig(GPIO_43_GPIO43);
    GPIO_setPadConfig(43, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(43, GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(43, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(43, GPIO_CORE_CPU1);

    // GPIO46 설정
    GPIO_setPinConfig(GPIO_46_GPIO46);
    GPIO_setPadConfig(46, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(46, GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(46, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(46, GPIO_CORE_CPU1);

    // GPIO47 설정
    GPIO_setPinConfig(GPIO_47_GPIO47);
    GPIO_setPadConfig(47, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(47, GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(47, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(47, GPIO_CORE_CPU1);

    // GPIO49 설정
    GPIO_setPinConfig(GPIO_49_GPIO49);
    GPIO_setPadConfig(49, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(49, GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(49, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(49, GPIO_CORE_CPU1);

    // GPIO67 설정
    GPIO_setPinConfig(GPIO_67_GPIO67);
    GPIO_setPadConfig(67, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(67, GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(67, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(67, GPIO_CORE_CPU1);

    // GPIO68 설정
    GPIO_setPinConfig(GPIO_68_GPIO68);
    GPIO_setPadConfig(68, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(68, GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(68, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(68, GPIO_CORE_CPU1);

    // CANRXA - GPIO5
    GPIO_setPinConfig(CANA_SLAVE_ID_CANRX_PIN_CONFIG);               // GPIO_5_CANRXA
    GPIO_setPadConfig(CANA_SLAVE_ID_CANRX_GPIO, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(CANA_SLAVE_ID_CANRX_GPIO, GPIO_QUAL_ASYNC);
    GPIO_setDirectionMode(CANA_SLAVE_ID_CANRX_GPIO, GPIO_DIR_MODE_IN);

    // CANTXA - GPIO4
    GPIO_setPinConfig(CANA_SLAVE_ID_CANTX_PIN_CONFIG);               // GPIO_4_CANTXA
    GPIO_setPadConfig(CANA_SLAVE_ID_CANTX_GPIO, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(CANA_SLAVE_ID_CANTX_GPIO, GPIO_QUAL_ASYNC);
    GPIO_setDirectionMode(CANA_SLAVE_ID_CANTX_GPIO, GPIO_DIR_MODE_OUT);

    // =======================
    // SCIA RS485 (Master-to-Master)
    // =======================

    // SCIA RS485 MM DE (Driver Enable) - GPIO66
    GPIO_setPinConfig(SCIA_RS485_MM_DE_PIN_CONFIG);
    GPIO_setPadConfig(SCIA_RS485_MM_DE_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(SCIA_RS485_MM_DE_GPIO, GPIO_QUAL_SYNC);
    GPIO_setDirectionMode(SCIA_RS485_MM_DE_GPIO, GPIO_DIR_MODE_OUT);

    // SCIA RS485 MM RX (GPIO64)
    GPIO_setPinConfig(SCIA_RS485_MM_RX_PIN_CONFIG);
    GPIO_setPadConfig(SCIA_RS485_MM_RX_GPIO, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(SCIA_RS485_MM_RX_GPIO, GPIO_QUAL_ASYNC);
    GPIO_setDirectionMode(SCIA_RS485_MM_RX_GPIO, GPIO_DIR_MODE_IN);

    // SCIA RS485 MM TX (GPIO65)
    GPIO_setPinConfig(SCIA_RS485_MM_TX_PIN_CONFIG);
    GPIO_setPadConfig(SCIA_RS485_MM_TX_GPIO, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(SCIA_RS485_MM_TX_GPIO, GPIO_QUAL_ASYNC);
    GPIO_setDirectionMode(SCIA_RS485_MM_TX_GPIO, GPIO_DIR_MODE_OUT);


    // =======================
    // SCIB RS485 (Master-to-Slave)
    // =======================

    // SCIB RS485 MS DE (Driver Enable) - GPIO69
    GPIO_setPinConfig(SCIB_RS485_MS_DE_PIN_CONFIG);
    GPIO_setPadConfig(SCIB_RS485_MS_DE_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(SCIB_RS485_MS_DE_GPIO, GPIO_QUAL_SYNC);
    GPIO_setDirectionMode(SCIB_RS485_MS_DE_GPIO, GPIO_DIR_MODE_OUT);

    // SCIB RS485 MS RX (GPIO71)
    GPIO_setPinConfig(SCIB_RS485_MS_RX_PIN_CONFIG);
    GPIO_setPadConfig(SCIB_RS485_MS_RX_GPIO, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(SCIB_RS485_MS_RX_GPIO, GPIO_QUAL_ASYNC);
    GPIO_setDirectionMode(SCIB_RS485_MS_RX_GPIO, GPIO_DIR_MODE_IN);

    // SCIB RS485 MS TX (GPIO70)
    GPIO_setPinConfig(SCIB_RS485_MS_TX_PIN_CONFIG);
    GPIO_setPadConfig(SCIB_RS485_MS_TX_GPIO, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(SCIB_RS485_MS_TX_GPIO, GPIO_QUAL_ASYNC);
    GPIO_setDirectionMode(SCIB_RS485_MS_TX_GPIO, GPIO_DIR_MODE_OUT);

    // SCI-D _ SCADA RX (GPIO77)
    GPIO_setPinConfig(GPIO_77_SCIRXDD);
    GPIO_setDirectionMode(77, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(77, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(77, GPIO_QUAL_ASYNC);

    // SCI-D _ SCADA TX (GPIO76)
    GPIO_setPinConfig(GPIO_76_SCITXDD);
    GPIO_setDirectionMode(76, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(76, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(76, GPIO_QUAL_ASYNC);

	// SPIA -> SPI_DAC1 Pinmux
	GPIO_setPinConfig(SPI_DAC1_SPIPICO_PIN_CONFIG);
	GPIO_setPadConfig(SPI_DAC1_SPIPICO_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(SPI_DAC1_SPIPICO_GPIO, GPIO_QUAL_ASYNC);
    GPIO_setDirectionMode(SPI_DAC1_SPIPICO_GPIO, GPIO_DIR_MODE_OUT);

	GPIO_setPinConfig(SPI_DAC1_SPIPOCI_PIN_CONFIG);
	GPIO_setPadConfig(SPI_DAC1_SPIPOCI_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(SPI_DAC1_SPIPOCI_GPIO, GPIO_QUAL_ASYNC);
    GPIO_setDirectionMode(SPI_DAC1_SPIPOCI_GPIO, GPIO_DIR_MODE_IN);

    GPIO_setPinConfig(SPI_DAC1_SPICLK_PIN_CONFIG);
	GPIO_setPadConfig(SPI_DAC1_SPICLK_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(SPI_DAC1_SPICLK_GPIO, GPIO_QUAL_ASYNC);
    GPIO_setDirectionMode(SPI_DAC1_SPICLK_GPIO, GPIO_DIR_MODE_OUT);

    GPIO_setPinConfig(SPI_DAC1_SPIPTE_PIN_CONFIG);
	GPIO_setPadConfig(SPI_DAC1_SPIPTE_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(SPI_DAC1_SPIPTE_GPIO, GPIO_QUAL_ASYNC);
    GPIO_setDirectionMode(SPI_DAC1_SPIPTE_GPIO, GPIO_DIR_MODE_OUT);

    // GPIO58 → SPIA_SIMO
    GPIO_setPinConfig(GPIO_58_SPISIMOA);
    GPIO_setPadConfig(58, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(58, GPIO_QUAL_ASYNC);

    // GPIO59 → SPIA_SOMI
    GPIO_setPinConfig(GPIO_59_SPISOMIA);
    GPIO_setPadConfig(59, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(59, GPIO_QUAL_ASYNC);

    // GPIO60 → SPIA_CLK
    GPIO_setPinConfig(GPIO_60_SPICLKA);
    GPIO_setPadConfig(60, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(60, GPIO_QUAL_ASYNC);


    // GPIO50 → SPIC_SIMO
    GPIO_setPinConfig(GPIO_50_SPISIMOC);
    GPIO_setPadConfig(50, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(50, GPIO_QUAL_ASYNC);

    // GPIO51 → SPIC_SOMI
    GPIO_setPinConfig(GPIO_51_SPISOMIC);
    GPIO_setPadConfig(51, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(51, GPIO_QUAL_ASYNC);

    // GPIO52 → SPIC_CLK
    GPIO_setPinConfig(GPIO_52_SPICLKC);
    GPIO_setPadConfig(52, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(52, GPIO_QUAL_ASYNC);

    // GPIO53 → SPIC_STE
    GPIO_setPinConfig(GPIO_53_SPISTEC);
    GPIO_setPadConfig(53, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(53, GPIO_QUAL_ASYNC);


    // GPIO90 ~ GPIO94 설정
    GPIO_setPinConfig(GPIO_90_GPIO90);
    GPIO_setPadConfig(90, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(90, GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(90, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(90, GPIO_CORE_CPU1);

    GPIO_setPinConfig(GPIO_91_GPIO91);
    GPIO_setPadConfig(91, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(91, GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(91, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(91, GPIO_CORE_CPU1);

    GPIO_setPinConfig(GPIO_92_GPIO92);
    GPIO_setPadConfig(92, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(92, GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(92, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(92, GPIO_CORE_CPU1);

    GPIO_setPinConfig(GPIO_93_GPIO93);
    GPIO_setPadConfig(93, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(93, GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(93, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(93, GPIO_CORE_CPU1);

    GPIO_setPinConfig(GPIO_94_GPIO94);
    GPIO_setPadConfig(94, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(94, GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(94, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(94, GPIO_CORE_CPU1);

}


void Init_GPIO_CPU2(void)
{

    // //
    // // CANRXB (GPIO7)
    // //
    // GPIO_setPinConfig(GPIO_7_CANRXB);
    // GPIO_setPadConfig(7, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);
    // GPIO_setQualificationMode(7, GPIO_QUAL_ASYNC);
    // GPIO_setDirectionMode(7, GPIO_DIR_MODE_IN);

    // //
    // // CANTXB (GPIO8)
    // //
    // GPIO_setPinConfig(GPIO_8_CANTXB);
    // GPIO_setPadConfig(8, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);
    // GPIO_setQualificationMode(8, GPIO_QUAL_ASYNC);
    // GPIO_setDirectionMode(8, GPIO_DIR_MODE_OUT);

    // //
    // // CANB Peripheral Clock Enable (CPU1)
    // //
    // SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CANB);

    // //
    // // CANB Peripheral Ownership을 CPU2로 이전
    // //
    // EALLOW;
    // SysCtl_selectCPUForPeripheral(SYSCTL_CPUSEL8_CAN, 2, SYSCTL_CPUSEL_CPU2);
    // EDIS;

}

