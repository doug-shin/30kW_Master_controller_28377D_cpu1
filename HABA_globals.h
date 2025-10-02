#ifndef HABA_GLOBALS_H
#define HABA_GLOBALS_H

#include "F28x_Project.h"              // TI 기본 레거시 포함
#include "F2837xD_Cla_defines.h"       // CLA 구조체

#include "driverlib.h"
#include "device.h"
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

#define STOP (0)
#define START (1)

#define CURRENT_LIMIT (80.0f)       // 최대 전류 제한값 (A)
#define MODULE_COUNT (1)            // 모듈 개수

#define OVER_VOLTAGE    (1400)         // 과전압 보호 임계값 (V)
#define OVER_CURRENT    (88.0f)       // 최대 전류 제한값 (A)
#define OVER_TEMP       (120)

// CANRXA - GPIO Settings
#define GPIO_PIN_CANRXA              5
#define CANA_SLAVE_ID_CANRX_GPIO     5
#define CANA_SLAVE_ID_CANRX_PIN_CONFIG GPIO_5_CANRXA

// CANTXA - GPIO Settings
#define GPIO_PIN_CANTXA              4
#define CANA_SLAVE_ID_CANTX_GPIO     4
#define CANA_SLAVE_ID_CANTX_PIN_CONFIG GPIO_4_CANTXA


// SCIA RX - GPIO64
#define SCIA_RS485_RX_GPIO        64
#define SCIA_RS485_RX_PIN_CONFIG  GPIO_64_SCIRXDA

// SCIA TX - GPIO65
#define SCIA_RS485_TX_GPIO        65
#define SCIA_RS485_TX_PIN_CONFIG  GPIO_65_SCITXDA

// RS485-A DE (Driver Enable) 제어용 - GPIO66
#define RS485A_DE_GPIO            66
#define RS485A_DE_PIN_CONFIG      GPIO_66_GPIO66

// SCIB RX - GPIO71
#define SCIB_RS485_RX_GPIO        71
#define SCIB_RS485_RX_PIN_CONFIG  GPIO_71_SCIRXDB

// SCIB TX - GPIO70
#define SCIB_RS485_TX_GPIO        70
#define SCIB_RS485_TX_PIN_CONFIG  GPIO_70_SCITXDB

// RS485 DE (Driver Enable) 제어용 - GPIO69
#define RS485B_DE_GPIO         69
#define RS485B_DE_PIN_CONFIG   GPIO_69_GPIO69

// SPIA_PICO - GPIO Settings
#define GPIO_PIN_SPIA_PICO 16
#define SPI_DAC1_SPIPICO_GPIO 16
#define SPI_DAC1_SPIPICO_PIN_CONFIG GPIO_16_SPISIMOA

// SPIA_POCI - GPIO Settings
#define GPIO_PIN_SPIA_POCI 17
#define SPI_DAC1_SPIPOCI_GPIO 17
#define SPI_DAC1_SPIPOCI_PIN_CONFIG GPIO_17_SPISOMIA

// SPIA_CLK - GPIO Settings
#define GPIO_PIN_SPIA_CLK 18
#define SPI_DAC1_SPICLK_GPIO 18
#define SPI_DAC1_SPICLK_PIN_CONFIG GPIO_18_SPICLKA

// SPIA_PTE - GPIO Settings
#define GPIO_PIN_SPIA_PTE 19
#define SPI_DAC1_SPIPTE_GPIO 19
#define SPI_DAC1_SPIPTE_PIN_CONFIG GPIO_19_SPISTEA

// SPIC SIMO (DSP → FPGA)
#define SPIC_SIMO_GPIO       50
#define SPIC_SIMO_PIN_CONFIG GPIO_50_SPISIMOC

// SPIC SOMI (FPGA → DSP)
#define SPIC_SOMI_GPIO       51
#define SPIC_SOMI_PIN_CONFIG GPIO_51_SPISOMIC

// SPIC CLK
#define SPIC_CLK_GPIO        52
#define SPIC_CLK_PIN_CONFIG  GPIO_52_SPICLKC

// SPIC CS (STE)
#define SPIC_STE_GPIO        53
#define SPIC_STE_PIN_CONFIG  GPIO_53_SPISTEC


// === 메시지 관련 상수 ===
#define MSG_DATA_LENGTH       4
#define TX_MSG_OBJ_ID1        1
#define RX_MSG_OBJ_BASE_ID    2
#define RX_MSG_OBJ_COUNT      30

// === 통신 제어 ===
#define STX                   0x1B
#define ETX                   0x03
#define _1ms_num              100  // 10us * 100 = 1ms

#define Kp_set                1
#define Ki_set                3000
#define T_sample_set          50e-6f
#define I_out_MAX             80

// Low Pass Filer용 값
#define wc (1e+3)   // cut-off frequency =  1ms
#define Ts (50e-6)  // 샘플링 타임 = 50us


// === GPIO / LED 상수 ===
#define LED31                 31
#define LED32                 32
#define LED33                 33
#define LED34                 34
#define LED35                 35
#define LED36                 36
#define LED37                 37
#define LED38                 38
#define LED39                 39
#define LED_PWR               46        // F_LED6 전면 LED PWR
#define LED_CHARGE            47        // F_LED5 전면 LED CHARGE
#define LED_DISCHARGE         42        // F_LED4 전면 LED DISCHARGE
#define LED_FAULT             43        // F_LED3 전면 LED FAULT
#define LED_SINGLE            67        // F_LED2 전면 LED SINGLE
#define LED_DUAL              68        // F_LED1 전면 LED DUAL


#define MAX_SLAVES   31
extern bool slave_enabled[MAX_SLAVES+1];

extern Uint16 ch_current[31];
extern Uint16 Temp_ch[31];
extern Uint16 DAB_ok_ch[31];
extern float  I_out_ch[31];
extern Uint16 can_rx_fault_cnt[31];
extern const float LUT_Resistance[161];

extern uint8_t Master_Mode;
extern uint16_t Module_Ch;

extern uint16_t over_current_flag;
extern uint16_t over_temp_flag;
extern uint16_t Master_fault_flag;

// Digital Input Register 구조체 정의
typedef union {
    struct {
        uint16_t Bit0 : 1;  // GPIO36
        uint16_t Bit1 : 1;  // GPIO37
        uint16_t Bit2 : 1;  // GPIO38
        uint16_t Bit3 : 1;  // GPIO39
        uint16_t rsvd : 12;
    } bit;
    uint16_t all;
} DIGITAL_REG;

typedef enum {
    MODE_STOP        = 0,   // 정지
    MODE_INDEPENDENT = 1,   // 독립 운전 (CH1, CH2 각각 자기 지령)
    MODE_PARALLEL    = 2,   // 병렬 운전 (상위 마스터만 PI, 하위는 전달만)
    MODE_RESERVED    = 3    // 예약 (사용 안 함)
} OperationMode_t;


typedef enum {
    STATE_NO_OP,
    STATE_READY,
    STATE_PRE_CHG_OK,
    STATE_STAND_BY,
    STATE_RUN,
    STATE_RL_OFF,
    STATE_FAULT
} SystemState;

extern SystemState state;


extern uint16_t cnt_10ms;
extern uint16_t cnt_50ms;
extern bool flag_10ms;
extern bool flag_50ms;

extern OperationMode_t operation_mode;
extern int test_mode;
extern int current_cA_TEST;
extern int DAB_OK_TEST;
extern int Temp_ch_TEST;
extern int I_out_ch_TEST ;
extern uint16_t test_voltage;
extern int32_t Vo_ad_sum;
extern float32_t Vo_ad_avg;

extern int32_t Bat_ad_sum;
extern float32_t Bat_ad_avg;
extern uint16_t fpga1_data, fpga2_data, fpga3_data;
// === extern 변수 선언 ===

extern uint32_t control_phase;

// CPU ↔ CLA 공유 변수
extern float32_t I_sat, I_MAX, Kp, Ki, T_sample, Vo;
extern float32_t Voh_cmd,Voh_Err,Voh_p_out,Voh_i_out,Voh_pi_out,Voh_PI;
extern float32_t Vol_cmd,Vol_Err,Vol_p_out,Vol_i_out,Vol_pi_out,Vol_PI;

extern float32_t Vo_ad;
extern int32_t Vo_ad_sum;
extern float32_t Vo_ad_avg;

extern float32_t Vbat;
extern float32_t Vbat_ad;
extern int32_t   Vbat_ad_sum;
extern float32_t Vbat_ad_avg;
extern float32_t Vbat_sen;


extern float32_t I_cmd_ch1;
extern float32_t I_cmd_ch2;

extern float32_t Io_avg;

extern float32_t Io_ad;
extern float32_t Io_ad_avg;
extern uint32_t Io_ad_sum;
extern float32_t Io_sen_sum;
extern uint16_t I_cmd_from_master;

extern int16_t run_switch;
extern float I_cmd_filt;   // 필터 출력
extern float I_cmd_old;   // 이전 입력 저장
extern float La_Ee;
extern float Lb_Ee;

extern float32_t fADC_voltage_out, Vo_sen;

extern float32_t fADC_voltage_Bat, Vbat_sen;
extern float32_t Vbat_sen;


extern uint16_t adc0;
extern uint16_t adc1;
extern uint16_t adc_current;
extern uint16_t adc_batt_voltage;
extern uint16_t adc_out_voltage;


extern uint16_t adc_current;
extern uint16_t adc_batt_voltage;
extern uint16_t adc_out_voltage;

extern float32_t Io_Mean;
extern float32_t Io_sen_sum_monitor;
extern uint16_t Current_Average;


extern float32_t Vo_sen_sum_monitor, Bat_sen_sum_monitor;
extern uint16_t MonitoringCount;

// === ADC 샘플 원시값 ===
extern uint16_t sensorSample0;
extern uint16_t sensorSample1;

// === 변환 중간 값 ===
extern float Vadc0, Vadc1;
extern float R_ntc0, R_ntc1;
extern float tempK0, tempK1;

// === 최종 섭씨 온도 결과 ===
extern float NTC0_Temp;
extern float NTC1_Temp;



extern volatile uint32_t SPIclk;
extern volatile uint32_t epwmclk;
extern volatile uint32_t sysclk;
extern volatile uint32_t lspclk;
extern volatile uint32_t Epwm1CLK;
extern volatile uint32_t Epwm3CLK;

// === CAN 디버그 변수 ===
extern volatile uint32_t debug_sysclk_freq;
extern volatile uint32_t debug_device_sysclk_freq;
extern volatile uint32_t debug_can_clock_source;
extern volatile uint32_t debug_btr_value;
extern volatile uint32_t debug_calculated_bitrate;
extern volatile uint16_t debug_actual_prescaler;
extern volatile uint16_t debug_total_tq;
extern volatile uint16_t debug_tseg1_tq;
extern volatile uint16_t debug_tseg2_tq;
extern volatile uint16_t debug_tx_err_count;
extern volatile uint16_t debug_rx_err_count;
extern volatile uint16_t debug_can_es;
extern volatile uint16_t debug_can_bus_off;
extern volatile uint16_t debug_can_warn;
extern volatile uint16_t debug_can_passive;

// === SCI 디버그 변수 ===
extern volatile uint16_t debug_sci_rx_error;
extern volatile uint16_t debug_sci_frame_err;
extern volatile uint16_t debug_sci_parity_err;
extern volatile uint16_t debug_sci_break;
extern volatile uint16_t debug_sci_tx_ready;
extern volatile uint16_t debug_sci_rx_ready;

// === SPI 디버그 변수 ===
extern volatile uint16_t debug_spi_tx_full;
extern volatile uint16_t debug_spi_tx_ready;
extern volatile uint16_t debug_spi_int;
extern volatile uint16_t debug_spi_overrun;
extern volatile uint16_t debug_spi_rx_hasdata;
extern volatile uint16_t debug_spi_rx_overflow;


//=============================
// Current Command and Output
//=============================
extern float32_t I_cmd;         // 기본 전류 지령 (Modbus 수신)
extern float32_t I_cmd_ss;      // 소프트 스타트 제한 적용 전류
extern float32_t I_cmd_final;   // 최종 전류 지령 (DAC 출력용)

//=============================
// Current Sensing (ADC 기반)
//=============================
extern uint16_t  I_out_ADC;         // 실시간 ADC 전류값 (100kHz)
extern uint32_t I_out_ADC_sum;     // 누적합 (5회 평균용)
extern uint32_t I_out_ADC_avg;     // 5회 평균값 (20kHz)

//=============================
// Current Feedback (장기 평균용)
//=============================
extern float32_t I_fb_sum;      // 누적 피드백 전류 합
extern float32_t I_fb_avg;      // 피드백 전류 평균값
extern int16_t  I_cal_cnt;     // 피드백 평균 계산용 카운터

//=============================
// Soft Start Control
//=============================
extern float32_t soft_start_limit;  // 소프트 스타트 전류 제한

//=============================
// DAC Output (SPI 출력용)
//=============================
extern uint16_t I_cmd_DAC;     // DAC 출력값 (0~4095)



//=============================
// Voltage Commands and Limits
//=============================
extern float32_t V_max_lim;     // 고전압 지령 (충전 모드)
extern float32_t V_min_lim;     // 저전압 지령 (방전 모드)
extern float32_t V_batt_avg;    // 배터리 평균 전압

//=============================
// Voltage Sensing and ADC
//=============================
extern volatile float32_t V_out_ADC;     // SPI ADC 전압 원시값 (SpiaRegs.SPIRXBUF)
extern float32_t V_out_ADC_avg;          // 전압 평균값 (5회 평균)
extern uint32_t  V_out_ADC_sum;          // 전압 누적합 (5회 평균용)
extern float32_t V_out;                  // PI 제어용 전압
extern float32_t V_fb;                   // 전압 피드백 (센싱)

//=============================
// Voltage Feedback and Monitoring
//=============================
extern float32_t V_fb_sum;     // 장기 평균 전압 합
extern uint32_t  V_cal_cnt;    // 전압 계산 카운터
extern float32_t Vfb;          // PI 제어기 피드백 전압
extern uint16_t cla_cnt;
                               //
//=============================
// Voltage Control Errors
//=============================
extern float32_t V_max_error;  // 고전압 오차 (충전)
extern float32_t V_min_error;  // 저전압 오차 (방전)

//=============================
// PI Control Outputs
//=============================
extern float32_t V_max_PI;     // 고전압 PI 출력
extern float32_t V_min_PI;     // 저전압 PI 출력

//=============================
// Protection Flags
//=============================
extern uint16_t over_voltage_flag;  // 과전압 보호 플래그

extern int16_t run_switch;

// extern uint8_t rxData[4];

extern uint8_t rxData[4];
extern uint16_t CANA_txData[4];
extern uint16_t CANA_rxDataArray[RX_MSG_OBJ_COUNT][4];
extern uint16_t gSciATxBuf[4];
extern uint16_t gSciBTxBuf[4];
extern int16_t  I_out_ref;
extern float32_t I_out_ref_test;
extern uint32_t IcomTemp;
extern uint32_t cnt;
extern uint16_t start;
extern uint16_t SPI_CMD, SPI_DATA;
extern uint8_t byte1, byte2, byte3;

extern uint16_t pass, fail;
extern volatile uint32_t epwm1_interrupt_count;
extern uint32_t toggle_count;
extern uint32_t _1ms_count;
extern uint32_t _100us_count;
extern uint32_t can_tx_flag;

extern int32_t   Test_count;
extern uint32_t _1ms_flag;
extern uint32_t _100us_flag;
extern uint32_t INT_EPMW1_Flag;
extern uint32_t Run;
extern uint8_t  test_flag;
extern uint16_t I_CMD;
extern uint8_t Relay_Flag;
extern uint8_t Siwtch_flag;


extern DIGITAL_REG DigitalIn;

extern int16_t Io_sen_total;
extern float32_t Io_sen_real;
extern float32_t Voh_com, Vol_com;
extern uint16_t V_high_limit, V_low_limit;

extern uint16_t Board_ID;
extern uint16_t Master_ID;

extern uint16_t switch_1_old_old;
extern uint16_t switch_1_old;
extern uint16_t switch_1;
extern uint16_t filtered_switch_input;

extern uint8_t Relay1_on_off;
extern uint8_t Relay2_on_off;
extern uint8_t Relay3_on_off;
extern uint8_t Relay4_on_off;
extern uint8_t Relay5_on_off;
extern uint8_t Relay6_on_off;
extern uint8_t Relay7_on_off;
extern uint8_t Relay8_on_off;

extern uint16_t sequence_step;
extern uint16_t start_stop;

extern float32_t Voh_cmd;
extern float32_t Vol_cmd;

extern float32_t Icom_temp;

extern float32_t Vo_Mean;
extern float32_t Bat_Mean;

extern uint16_t Pre_chg_ok;
extern uint16_t Pre_chg_Fail;
extern uint32_t Pre_chg_count;
extern uint32_t RL_off_delay_count;
extern uint8_t task_flag1;
extern uint8_t task_flag2;
extern uint8_t task_flag3;
extern uint8_t task_flag4;
extern uint8_t task_flag5;



extern void (*control_task_functions[5])(void);

// CLA Task extern 선언
extern __interrupt void Cla1Task1(void);
extern __interrupt void Cla1Task2(void);

// ISR extern 선언
extern __interrupt void cla1Isr1(void);
extern __interrupt void cla1Isr2(void);

extern __interrupt void INT_ADCA1_ISR(void);
extern __interrupt void INT_Init_EPWM1_ISR(void);

extern void sensing_task(void);
extern void pi_control_task(void);
extern void current_command_task(void);
extern void temperature_task(void);
extern void average_task(void);

extern void (*control_task_functions[5])(void);

// HMI 변수 선언들================================================================

#define HMI_SCITX_PIN 76U
#define HMI_SCIRX_PIN 77U
#define GPIO_76_SCIDTX 0x00861806U
#define GPIO_77_SCIDRX 0x00861A06U
#define HMI_SCITX_PIN_CONFIG GPIO_76_SCIDTX
#define HMI_SCIRX_PIN_CONFIG GPIO_77_SCIDRX
#define HMI_BAUD_RATE 115200U
#define HMI_RX_BUFFER_SIZE 10
#define Rack_Channel 0  // 렉번호 0~3

typedef struct
{
    uint8_t  start_byte;    // 0x02
    uint8_t  command;       // rev2.0: bit0=Run, bit2:1=Mode (0:정지,1:CH1,2:CH2,3:병렬)
    int16_t  max_voltage;   // Byte2~3 (단위: V)
    int16_t  min_voltage;   // Byte4~5 (단위: V)
    int16_t  current_cmd;    // Byte6~7 (단위: A)
    uint8_t  checksum;      // Byte8 (Sum Byte1~7)
    uint8_t  end_byte;      // Byte9 (0x03)
} HMI_PACKET;

typedef struct
{
    uint8_t stx;
    uint8_t id;
    int16_t systemVoltage;
    uint8_t reserved;
    uint8_t checksum;
    uint8_t etx;
} SYSTEM_TX_PACKET;

typedef struct
{
    uint8_t stx;
    uint8_t idAndStatus;
    int16_t slaveCurrent;
    uint8_t slaveTemp;
    uint8_t checksum;
    uint8_t etx;
} SLAVE_TX_PACKET;

// 외부 공개 변수 (main.c에서 이 변수들을 사용)
extern volatile HMI_PACKET hmi_rx_data;
extern volatile uint16_t hmi_packet_ready;
extern volatile uint8_t hmi_rx_buffer[HMI_RX_BUFFER_SIZE];
extern volatile uint16_t hmi_rx_index;

extern uint8_t slave_tx_buffer[7];
extern uint8_t system_tx_buffer[7];
extern uint8_t checksum;
extern uint16_t i;

extern volatile uint16_t g_systemRunCommand;
extern volatile int16_t g_maxVoltageSet;
extern volatile int16_t g_minVoltageSet;
extern volatile int8_t g_currentCommandSet;

extern volatile uint8_t SCADA_cmd;

// HMI 선언들 끝====================================================================

#ifdef __cplusplus
}
#endif

#endif // HABA_GLOBALS_H
