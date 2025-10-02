#include "HABA_shared.h"


#pragma DATA_SECTION(fVal,       "CpuToCla1MsgRAM");
#pragma DATA_SECTION(Voh_cmd,    "CpuToCla1MsgRAM");
#pragma DATA_SECTION(Vol_cmd,    "CpuToCla1MsgRAM");
#pragma DATA_SECTION(I_sat,      "CpuToCla1MsgRAM");
#pragma DATA_SECTION(Vfb,        "CpuToCla1MsgRAM");
#pragma DATA_SECTION(Kp,         "CpuToCla1MsgRAM");
#pragma DATA_SECTION(Ki,         "CpuToCla1MsgRAM");
#pragma DATA_SECTION(T_sample,   "CpuToCla1MsgRAM");
#pragma DATA_SECTION(I_MAX,      "CpuToCla1MsgRAM");


#pragma DATA_SECTION(fResult,   "Cla1ToCpuMsgRAM");
#pragma DATA_SECTION(Voh_pi_out,"Cla1ToCpuMsgRAM");
#pragma DATA_SECTION(Voh_Err,   "Cla1ToCpuMsgRAM");
#pragma DATA_SECTION(Voh_p_out, "Cla1ToCpuMsgRAM");
#pragma DATA_SECTION(Voh_i_out, "Cla1ToCpuMsgRAM");
#pragma DATA_SECTION(Vol_pi_out,"Cla1ToCpuMsgRAM");
#pragma DATA_SECTION(Vol_Err,   "Cla1ToCpuMsgRAM");
#pragma DATA_SECTION(Vol_p_out, "Cla1ToCpuMsgRAM");
#pragma DATA_SECTION(Vol_i_out, "Cla1ToCpuMsgRAM");
#pragma DATA_SECTION(cla_cnt,      "Cla1ToCpuMsgRAM");


// High-Frequency Control Timing
uint32_t control_phase = 0; // 100kHz → 20kHz 분주 카운터 (0~4)

SystemState state = STATE_NO_OP;

OperationMode_t operation_mode = MODE_STOP;
//==================================================
// 함수 포인터 배열 (5개 task)
//==================================================
void (*control_task_functions[5])(void) = {
    sensing_task,
    pi_control_task,
    current_command_task,
    temperature_task,
    average_task
};

HMI_PACKET my_hmi_packet = {
    .start_byte = 0x02,
    .command = 0,
    .max_voltage = 0,
    .min_voltage = 0,
    .current_cmd = 10, // <-- 원하는 초기값 10을 할당
    .checksum = 0,
    .end_byte = 0x03
};

uint16_t cnt_10ms = 0;
uint16_t cnt_50ms = 0;
bool flag_10ms = false;
bool flag_50ms = false;

int current_cA_TEST =0;
int DAB_OK_TEST =0;
int Temp_ch_TEST =0;
int I_out_ch_TEST  =0;
uint16_t test_voltage = 0;
int test_mode = 0;
uint16_t over_current_flag  = 0;
uint16_t over_temp_flag     = 0;
uint16_t Master_fault_flag  = 0;

uint8_t Master_Mode = 0;

uint16_t adc0 = 0;
uint16_t adc1 = 0;
float32_t fVal         = 0.0f;
float32_t fResult      = 0.0f;

uint16_t Module_Ch;
uint16_t fpga1_data, fpga2_data, fpga3_data;

Uint16 ch_current[31]       = {0, };
Uint16 Temp_ch[31]          = {0, };
Uint16 DAB_ok_ch[31]        = {0, };
float  I_out_ch[31]         = {0.0f, };


Uint16 can_rx_fault_cnt[31] = {0};

uint16_t adc_current;
uint16_t adc_batt_voltage;
uint16_t adc_out_voltage;

float32_t Io_Mean = 0.0f;
float32_t Io_sen_sum_monitor = 0.0f;
uint16_t Current_Average = 0;

float32_t Vo_sen_sum_monitor = 0.0f, Bat_sen_sum_monitor = 0.0f;
uint16_t MonitoringCount = 0;


// uint8_t rxData[4];
uint8_t rxData[4];

uint16_t CANA_txData[4] = {0x00, 0x00, 0x00, 0x00};
uint8_t gSciATxBuf[4] = {0x1B, 0, 0, 0x03};
uint8_t gSciBTxBuf[4] = {0x1B, 0, 0, 0x03};
uint8_t CANA_rxDataArray[RX_MSG_OBJ_COUNT][4] = {0};

bool slave_enabled[MAX_SLAVES+1] = {0}; 

// === ADC 샘플 원시값 ===
uint16_t sensorSample0;
uint16_t sensorSample1;

// === 변환 중간 값 ===
float Vadc0, Vadc1;
float R_ntc0, R_ntc1;
float tempK0, tempK1;

// === 최종 섭씨 온도 결과 ===
float NTC0_Temp;
float NTC1_Temp;


float I_sat = 0;  // Declare before usage
float I_ss = 0;
float I_sat1 = 0;
float I_sat_old =0;
float I_cmd_filt = 0.0f;   // 필터 출력
float I_cmd_old  = 0.0f;   // 이전 입력 저장
float La_Ee = 0;
float Lb_Ee = 0;


volatile uint32_t SPIclk                    = 0;
volatile uint32_t epwmclk                   = 0;
volatile uint32_t sysclk                    = 0;
volatile uint32_t lspclk                    = 0;
volatile uint32_t Epwm1CLK                  = 0;
volatile uint32_t Epwm3CLK                  = 0;

// === CAN 디버그 변수 ===
volatile uint32_t debug_sysclk_freq         = 0;
volatile uint32_t debug_device_sysclk_freq  = 0;
volatile uint32_t debug_can_clock_source    = 0;
volatile uint32_t debug_btr_value           = 0;
volatile uint32_t debug_calculated_bitrate  = 0;
volatile uint16_t debug_actual_prescaler    = 0;
volatile uint16_t debug_total_tq            = 0;
volatile uint16_t debug_tseg1_tq            = 0;
volatile uint16_t debug_tseg2_tq            = 0;
volatile uint16_t debug_tx_err_count        = 0;
volatile uint16_t debug_rx_err_count        = 0;
volatile uint16_t debug_can_es              = 0;
volatile uint16_t debug_can_bus_off         = 0;
volatile uint16_t debug_can_warn            = 0;
volatile uint16_t debug_can_passive         = 0;

// === SCI 디버그 변수 ===
volatile uint16_t debug_sci_rx_error        = 0;
volatile uint16_t debug_sci_frame_err       = 0;
volatile uint16_t debug_sci_parity_err      = 0;
volatile uint16_t debug_sci_break           = 0;
volatile uint16_t debug_sci_tx_ready        = 0;
volatile uint16_t debug_sci_rx_ready        = 0;

// === SPI 디버그 변수 ===
volatile uint16_t debug_spi_tx_full         = 0;
volatile uint16_t debug_spi_tx_ready        = 0;
volatile uint16_t debug_spi_int             = 0;
volatile uint16_t debug_spi_overrun         = 0;
volatile uint16_t debug_spi_rx_hasdata      = 0;
volatile uint16_t debug_spi_rx_overflow     = 0;

float32_t Io_avg = 0;
float32_t Io_ad = 0;
float32_t Io_ad_avg = 0;
uint32_t Io_ad_sum = 0;
float32_t Io_sen_sum = 0;

float32_t I_sat,I_MAX,Kp,Ki,T_sample,Vo;
float32_t Voh_cmd,Voh_Err,Voh_p_out,Voh_i_out,Voh_pi_out,Voh_PI;
float32_t Vol_cmd,Vol_Err,Vol_p_out,Vol_i_out,Vol_pi_out,Vol_PI;

float32_t Vo_ad = 0;
int32_t   Vo_ad_sum = 0;
int32_t   Test_count = 0;
float32_t Vo_ad_avg = 0.0f;
float32_t Vbat = 0;
float32_t Vbat_ad = 0;
int32_t   Vbat_ad_sum = 0;
float32_t Vbat_ad_avg = 0.0f;

float32_t fADC_voltage_out, Vo_sen;

float32_t fADC_voltage_Bat, Vbat_sen;
float32_t Vbat_sen = 0;

// Current Commands and Control Outputs
float32_t I_cmd = 0.0f;      // 기본 전류 지령 (Modbus에서 수신)
float32_t I_cmd_ss = 0.0f;   // 소프트 스타트 제한 적용된 전류 지령
float32_t I_cmd_final = 0.0f; // 최종 전류 지령 (PI 제한 적용, DAC 출력용)

// Current Sensing and ADC
uint16_t I_out_ADC = 0;         // 현재 전류 ADC 값 (100kHz)
uint32_t I_out_ADC_sum = 0;     // 전류 ADC 합계 (5회 평균용)
uint32_t I_out_ADC_avg = 0; // 5회 평균된 ADC 전류값 (20kHz)

// Current Feedback and Monitoring
float32_t I_fb_sum = 0.0f; // 전류 센서 합계 (장기 평균용)
float32_t I_fb_avg = 0.0f; // 전류 피드백 평균 (모니터링용)
int16_t I_cal_cnt = 0;    // 전류 계산 카운터

// Soft Start Control
float32_t soft_start_limit = 0.0f; // 소프트 스타트 전류 제한

// DAC Output
uint16_t I_cmd_DAC = 0; // 전류 지령 DAC 값 (0~4095)



// Voltage Commands and Limits
float32_t V_max_lim = 0.0f;   // 고전압 지령 (충전 모드용)
float32_t V_min_lim = 0.0f;   // 저전압 지령 (방전 모드용)
float32_t V_batt_avg = 0.0f;  // 배터리 평균 전압

// Voltage Sensing and ADC
volatile float32_t V_out_ADC = 0.0f; // SPI ADC 전압 원시값 (SpiaRegs.SPIRXBUF)
float32_t V_out_ADC_avg = 0.0f;      // SPI ADC 전압 평균값 (5회 평균)
uint32_t V_out_ADC_sum = 0;          // SPI ADC 전압 합계 (5회 평균용)
float32_t V_out = 0.0f;              // 출력 전압 (PI 제어용)
float32_t V_fb = 0.0f;               // 전압 피드백 (센싱용)

uint16_t cla_cnt = 0;

// Voltage Feedback and Monitoring
float32_t V_fb_sum = 0.0f; // 전압 피드백 합계 (장기 평균용)
uint32_t V_cal_cnt = 0;    // 전압 계산 카운터
float32_t Vfb = 0;         // PI 제어기 피드백 전압
// Voltage Control Errors
float32_t V_max_error = 0.0f; // 고전압 오차 (충전 모드)
float32_t V_min_error = 0.0f; // 저전압 오차 (방전 모드)

// PI Control Outputs
float32_t V_max_PI = 0.0f; // 고전압 PI 출력 (충전 모드)
float32_t V_min_PI = 0.0f; // 저전압 PI 출력 (방전 모드)

// Protection Flags
uint16_t over_voltage_flag = 0; // 과전압 보호 플래그

// Digital Input Status
int16_t run_switch = 0;              // 운전 스위치 상태 (GPIO54)

int16_t  I_out_ref = 0;
float32_t I_out_ref_test = 0;


uint16_t I_cmd_from_master = 0;

uint32_t IcomTemp = 0;
uint32_t cnt = 0;
uint16_t start = 1;
uint16_t SPI_CMD, SPI_DATA;
uint8_t byte1, byte2, byte3;

uint16_t pass=0;
uint16_t fail=0;

uint16_t Board_ID = 0;
uint16_t Master_ID = 0;
DIGITAL_REG DigitalIn = {0};

volatile uint32_t epwm1_interrupt_count = 0;
uint32_t toggle_count = 0;
uint32_t _1ms_count = 0;
uint32_t _100us_count = 0;
uint32_t can_tx_flag = 0;
uint32_t _1ms_flag =0;
uint32_t _100us_flag =0;
uint32_t INT_EPMW1_Flag = 0;
uint32_t Run = 0;
uint8_t test_flag = 0;
uint8_t Relay_Flag = 0;
uint8_t Siwtch_flag = 0;
uint8_t task_flag1 = 0;
uint8_t task_flag2 = 0;
uint8_t task_flag3 = 0;
uint8_t task_flag4 = 0;
uint8_t task_flag5 = 0;

int16_t Io_sen_total = 0;
float32_t Io_sen_real = 0;
float32_t Voh_com = 0, Vol_com = 0;
uint16_t V_high_limit, V_low_limit = 0;

uint8_t Relay1_on_off =0;
uint8_t Relay2_on_off =0;
uint8_t Relay3_on_off =0;
uint8_t Relay4_on_off =0;
uint8_t Relay5_on_off =0;
uint8_t Relay6_on_off =0;
uint8_t Relay7_on_off =0;
uint8_t Relay8_on_off =0;

uint16_t sequence_step = 0;
uint16_t start_stop = 0;

float32_t Voh_cmd = 0;
float32_t Vol_cmd = 0;

float32_t Icom_temp = 0;

float32_t Vo_Mean = 0.0f;
float32_t Bat_Mean = 0.0f;

uint16_t Pre_chg_ok = 0;
uint16_t Pre_chg_Fail = 0;
uint32_t Pre_chg_count = 0;
uint32_t RL_off_delay_count = 0;

float32_t I_cmd_ch1 = 0.0f;
float32_t I_cmd_ch2 = 0.0f;

#define T_MIN   -40
#define T_MAX    120
#define LUT_SIZE (T_MAX - T_MIN + 1)

// LUT_Resistance[i] → (T_MIN + i) °C 에 해당하는 저항값 (kΩ)
const float LUT_Resistance[161] = {
311.858f, 295.637f, 279.416f, 263.195f, 246.974f, 230.753f, 218.955f, 207.158f,
195.360f, 183.563f, 171.765f, 163.140f, 154.516f, 145.891f, 137.267f, 128.642f,
122.302f, 115.963f, 109.623f, 103.284f, 96.944f, 92.258f, 87.572f, 82.885f,
78.199f, 73.513f, 70.029f, 66.545f, 63.062f, 59.578f, 56.094f, 53.489f,
50.883f, 48.278f, 45.672f, 43.067f, 41.108f, 39.148f, 37.189f, 35.229f,
33.270f, 31.787f, 30.304f, 28.822f, 27.339f, 25.856f, 24.728f, 23.600f,
22.471f, 21.343f, 20.215f, 19.351f, 18.488f, 17.624f, 16.761f, 15.897f,
15.232f, 14.567f, 13.903f, 13.238f, 12.573f, 12.058f, 11.544f, 11.029f,
10.515f, 10.000f, 9.599f, 9.198f, 8.798f, 8.397f, 7.996f, 7.683f,
7.369f, 7.056f, 6.742f, 6.429f, 6.182f, 5.935f, 5.689f, 5.442f,
5.195f, 5.000f, 4.805f, 4.610f, 4.415f, 4.220f, 4.065f, 3.910f,
3.754f, 3.599f, 3.444f, 3.320f, 3.196f, 3.073f, 2.949f, 2.825f,
2.726f, 2.626f, 2.527f, 2.427f, 2.328f, 2.248f, 2.168f, 2.087f,
2.007f, 1.927f, 1.862f, 1.797f, 1.732f, 1.667f, 1.602f, 1.549f,
1.496f, 1.443f, 1.390f, 1.337f, 1.298f, 1.258f, 1.219f, 1.180f,
1.141f, 1.102f, 1.062f, 1.023f, 0.984f, 0.944f, 0.909f, 0.874f,
0.839f, 0.804f, 0.770f, 0.735f, 0.700f, 0.671f, 0.643f, 0.615f,
0.586f, 0.558f, 0.529f, 0.501f, 0.472f, 0.444f, 0.416f, 0.388f,
0.363f, 0.346f, 0.330f, 0.313f
};

// HMI 변수 정의들 ===================================================

// HMI와 연동될 실제 시스템 변수들을 이 파일에서 정의(생성)합니다.
volatile uint16_t g_systemRunCommand = 0;
volatile int16_t g_maxVoltageSet = 0;
volatile int16_t g_minVoltageSet = 0;
volatile int8_t g_currentCommandSet = 0;

// HMI 통신 모듈 내부 변수
volatile HMI_PACKET hmi_rx_data;
volatile uint16_t hmi_packet_ready = 0;
volatile uint8_t hmi_rx_buffer[HMI_RX_BUFFER_SIZE] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00};
volatile uint16_t hmi_rx_index = 0;
uint8_t system_tx_buffer[7];
uint8_t slave_tx_buffer[7];

volatile uint8_t SCADA_cmd = 0;

// HMI 변수 정의들 끝 ==================================================
