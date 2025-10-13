#ifndef HABA_SETUP_H
#define HABA_SETUP_H

#include "HABA_globals.h"

#define ADCA1_BASE ADCA_BASE
#define ADCA1_RESULT_BASE ADCARESULT_BASE
#define ADCA1_ADCINA1 ADC_SOC_NUMBER0
#define ADCA1_FORCE_ADCINA1 ADC_FORCE_SOC0
#define ADCA1_SAMPLE_WINDOW_ADCINA1 100
#define ADCA1_TRIGGER_SOURCE_ADCINA1 ADC_TRIGGER_CPU1_TINT0
#define ADCA1_CHANNEL_ADCINA1 ADC_CH_ADCIN1
#define INT_ADCA1_1 INT_ADCA1
#define INT_ADCA1_1_INTERRUPT_ACK_GROUP INTERRUPT_ACK_GROUP1




#define DEVICE_SCI_BAUDRATE   5625000UL   // Setting Baudrate (5.625Mbps)

#define TX_MSG_OBJ_ID1     1    // e.g., Mailbox 1번 사용
#define TX_MSG_ID1         0xE0 // 예: 송신할 CAN ID
#define MSG_DATA_LENGTH    4    // 전송할 데이터 바이트 수

#define SPI_DAC1_BASE SPIA_BASE
#define SPI_DAC1_BITRATE    5000000
#define SPI_DAC1_DATAWIDTH 8



#define LED31_GPIO_PIN_CONFIG GPIO_31_GPIO31
#define LED32_GPIO_PIN_CONFIG GPIO_32_GPIO32
#define LED33_GPIO_PIN_CONFIG GPIO_33_GPIO33
#define LED34_GPIO_PIN_CONFIG GPIO_34_GPIO34
#define LED35_GPIO_PIN_CONFIG GPIO_35_GPIO35


#define EPWM3_PERIOD 900
#define EPWM3_DUTY   810

#define SLAVE_NUM 31

extern uint16_t txMsgData[4];




typedef struct {
    uint32_t base;
    uint16_t period;
    uint16_t dutyA;
    uint16_t dutyB;
    bool useInterrupt;
    bool enableSOCA;
    bool enableOutput;
    EPWM_ADCStartOfConversionType socType;         // ✅ OK
    EPWM_ADCStartOfConversionSource socTriggerSource; // ✅ OK
    uint16_t socEventPrescale;
} EPWMConfig;




    typedef enum {
    RX_MSG_ID_SLAVE1  = 0xF1,
    RX_MSG_ID_SLAVE2,
    RX_MSG_ID_SLAVE3,
    RX_MSG_ID_SLAVE4,
    RX_MSG_ID_SLAVE5,
    RX_MSG_ID_SLAVE6,
    RX_MSG_ID_SLAVE7,
    RX_MSG_ID_SLAVE8,
    RX_MSG_ID_SLAVE9,
    RX_MSG_ID_SLAVE10,
    RX_MSG_ID_SLAVE11,
    RX_MSG_ID_SLAVE12,
    RX_MSG_ID_SLAVE13,
    RX_MSG_ID_SLAVE14,
    RX_MSG_ID_SLAVE15  // 0xFF
    } CAN_RxMsgID_t;

    typedef enum {
    RX_MBOX_ID_SLAVE1  = 2,
    RX_MBOX_ID_SLAVE2,
    RX_MBOX_ID_SLAVE3,
    RX_MBOX_ID_SLAVE4,
    RX_MBOX_ID_SLAVE5,
    RX_MBOX_ID_SLAVE6,
    RX_MBOX_ID_SLAVE7,
    RX_MBOX_ID_SLAVE8,
    RX_MBOX_ID_SLAVE9,
    RX_MBOX_ID_SLAVE10,
    RX_MBOX_ID_SLAVE11,
    RX_MBOX_ID_SLAVE12,
    RX_MBOX_ID_SLAVE13,
    RX_MBOX_ID_SLAVE14,
    RX_MBOX_ID_SLAVE15  // 16
    } CAN_RxMboxID_t;



    // HAL 초기화 함수
    void Init_System(void);
    void Init_GPIO_CPU1(void);
    void Init_GPIO_CPU2(void);
    void Init_EPWM(void);
    void Set_EPWM(const EPWMConfig *cfg);
    void Init_CANA(void);
    void Init_SCIA(void);
    void Init_SCIB(void);
    void Init_SCID_SCADA(void);
    void Init_SPIA(void);
    void Init_SPIC(void);
    void Init_SPI_DAC1(void);
    void Init_ADCA(void);
    void Init_INTERRUPT(void);
    void Init_CPU1_CLA1(void);
    void Select_master_id(void);

#endif // HABA_SETUP_H
