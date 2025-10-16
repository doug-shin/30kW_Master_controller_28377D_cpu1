//==================================================
// HABA_globals.c - Global Variables Definition
//==================================================
// TI F28377D 30kW Master Controller
// 전역 변수 정의 및 초기화
//==================================================

#include "HABA_globals.h"

//==================================================
// [1] CLA 공유 메모리 변수 (CPU ↔ CLA)
//==================================================

// CPU → CLA (제어 지령) - CpuToCla1MsgRAM 섹션
#pragma DATA_SECTION(V_max_cmd,        "CpuToCla1MsgRAM");
#pragma DATA_SECTION(V_min_cmd,        "CpuToCla1MsgRAM");
#pragma DATA_SECTION(V_fb,             "CpuToCla1MsgRAM");

float32_t V_max_cmd         = 0.0f;     // 최대 전압 지령 (충전)
float32_t V_min_cmd         = 0.0f;     // 최소 전압 지령 (방전)
float32_t V_fb              = 0.0f;     // 전압 피드백

// CLA PI 제어기 (DCL) - CpuToCla1MsgRAM 섹션
#pragma DATA_SECTION(pi_charge,        "CpuToCla1MsgRAM");
#pragma DATA_SECTION(pi_discharge,     "CpuToCla1MsgRAM");
#pragma DATA_SECTION(pi_cv,            "CpuToCla1MsgRAM");

DCL_PI_CLA pi_charge    = PI_CLA_DEFAULTS;  // PI 제어기 (충전, V_max_cmd 기준) - CLA Task 1
DCL_PI_CLA pi_discharge = PI_CLA_DEFAULTS;  // PI 제어기 (방전, V_min_cmd 기준) - CLA Task 2
DCL_PI_CLA pi_cv        = PI_CLA_DEFAULTS;  // PI 제어기 (배터리 모드 CV, V_cmd 기준) - CLA Task 3

// CLA → CPU (제어 결과) - Cla1ToCpuMsgRAM 섹션
#pragma DATA_SECTION(I_PI_charge_out,    "Cla1ToCpuMsgRAM");
#pragma DATA_SECTION(I_PI_discharge_out, "Cla1ToCpuMsgRAM");
#pragma DATA_SECTION(I_PI_cv_out,        "Cla1ToCpuMsgRAM");
#pragma DATA_SECTION(cla_cnt,            "Cla1ToCpuMsgRAM");

float32_t I_PI_charge_out    = 0.0f;    // PI 출력 (충전 전류, Task 1)
float32_t I_PI_discharge_out = 0.0f;    // PI 출력 (방전 전류, Task 2)
float32_t I_PI_cv_out        = 0.0f;    // PI 출력 (배터리 모드 CV 전류, Task 3)
uint16_t  cla_cnt            = 0;       // CLA 실행 카운터 (디버그용)

//==================================================
// [2] 시스템 상태 및 타이밍
//==================================================

SystemState      state              = STATE_NO_OP;      // 시스템 상태
OperationMode_t  operation_mode     = MODE_STOP;        // 운전 모드
uint32_t         control_phase      = 0;                // 제어 Phase (0~4)
uint16_t         sequence_step      = 0;                // 시퀀스 단계
uint16_t         start_stop         = 0;                // START(1) / STOP(0)
uint32_t         run                = 0;                // 실행 플래그
int16_t          run_switch         = 0;                // 운전 스위치 상태

// 타이밍 플래그
uint32_t cnt_1ms            = 0;        // 1ms 카운터
uint32_t flag_1ms           = 0;        // 1ms 플래그
uint16_t cnt_10ms           = 0;        // 10ms 카운터
bool     flag_10ms          = false;    // 10ms 플래그
uint16_t cnt_50ms           = 0;        // 50ms 카운터
bool     flag_50ms          = false;    // 50ms 플래그
uint32_t flag_epwm1_int     = 0;        // EPWM1 인터럽트 플래그

//==================================================
// [3] 마스터 제어
//==================================================

uint16_t master_id          = 0;        // 마스터 ID (0=상위, 1=하위)
uint8_t  master_mode        = 0;        // 마스터 모드 (1=PI제어, 2=전달만)

//==================================================
// [4] 전압 센싱 및 피드백
//==================================================

// 출력 전압 (V_out)
float32_t V_out             = 0.0f;     // 캘리브레이션된 출력 전압
float32_t V_out_raw         = 0.0f;     // 원시 출력 전압
int32_t   V_out_raw_sum     = 0;        // 출력 전압 누적합 (평균용)
float32_t V_out_raw_avg     = 0.0f;     // 출력 전압 평균
float32_t V_out_uncal       = 0.0f;     // 미캘리브레이션 출력 전압

// 배터리 전압 (V_batt)
float32_t V_batt            = 0.0f;     // 캘리브레이션된 배터리 전압
float32_t V_batt_raw        = 0.0f;     // 원시 배터리 전압
int32_t   V_batt_raw_sum    = 0;        // 배터리 전압 누적합 (평균용)
float32_t V_batt_raw_avg    = 0.0f;     // 배터리 전압 평균
float32_t V_batt_uncal      = 0.0f;     // 미캘리브레이션 배터리 전압

// 디스플레이용 전압
float32_t V_out_display         = 0.0f; // 출력 전압 표시값
float32_t V_batt_display        = 0.0f; // 배터리 전압 표시값
float32_t V_out_display_sum     = 0.0f; // 출력 전압 표시 누적합
float32_t V_batt_display_sum    = 0.0f; // 배터리 전압 표시 누적합
uint16_t  V_display_calc_cnt    = 0;    // 디스플레이 계산 카운터

//==================================================
// [5] 전류 센싱 및 피드백
//==================================================

// 전류 지령
float32_t I_cmd             = 0.0f;     // 기본 전류 지령 (SCADA 수신)
float32_t I_cmd_ramped      = 0.0f;     // 램프 제한 적용 전류
float32_t I_cmd_PI_limited  = 0.0f;     // PI 제한 적용 전류 (DAC 출력용)
uint16_t  I_cmd_from_master = 0;        // 상위 마스터로부터 수신한 지령

// 전류 센싱 (FPGA SPI)
float32_t I_out_raw         = 0.0f;     // 원시 출력 전류

// 전류 피드백 및 평균
float32_t I_out_avg         = 0.0f;     // 출력 전류 평균
int16_t   I_out_ref         = 0;        // 전류 레퍼런스

//==================================================
// [6] 소프트 스타트 및 필터
//==================================================

float32_t I_ss_ramp         = 0.0f;     // 소프트 스타트 램프 제한
float32_t I_cmd_filtered    = 0.0f;     // LPF 필터 출력
float32_t I_cmd_prev        = 0.0f;     // 이전 입력 저장
float32_t lpf_coeff_a       = 0.0f;     // LPF 전향 계수
float32_t lpf_coeff_b       = 0.0f;     // LPF 피드백 계수

//==================================================
// [7] DAC 출력
//==================================================

uint16_t I_cmd_to_slave          = 0;        // DAC 출력값 (0~65535)

//==================================================
// [8] 슬레이브 관리 (CAN 통신)
//==================================================

bool         slave_enabled[MAX_SLAVES+1] = {0};         // 슬레이브 활성화 상태
uint16_t     I_out_slave_raw[31]         = {0};         // 슬레이브 전류 원시값
float32_t    I_out_slave[31]             = {0.0f};      // 슬레이브 전류 (캘리브레이션)
uint16_t     temp_slave_raw[31]          = {0};         // 슬레이브 온도 원시값
uint16_t     DAB_ok_slave[31]            = {0};         // 슬레이브 DAB OK 상태
uint16_t     can_rx_fault_cnt[31]        = {0};         // CAN 수신 고장 카운터
int16_t      I_out_slave_total           = 0;           // 슬레이브 총 전류

//==================================================
// [9] 온도 센싱 (NTC)
//==================================================

float32_t NTC_0_temp        = 0.0f;     // NTC0 온도 (°C)
float32_t NTC_1_temp        = 0.0f;     // NTC1 온도 (°C)

// NTC 저항 Look-Up Table (-40°C ~ +120°C)
// LUT_Resistance[i] → (T_MIN + i)°C에 해당하는 저항값 (kΩ)
#define T_MIN   -40
#define T_MAX    120
#define LUT_SIZE (T_MAX - T_MIN + 1)

const float LUT_Resistance[161] = {
    311.858f, 295.637f, 279.416f, 263.195f, 246.974f, 230.753f, 218.955f, 207.158f,
    195.360f, 183.563f, 171.765f, 163.140f, 154.516f, 145.891f, 137.267f, 128.642f,
    122.302f, 115.963f, 109.623f, 103.284f,  96.944f,  92.258f,  87.572f,  82.885f,
     78.199f,  73.513f,  70.029f,  66.545f,  63.062f,  59.578f,  56.094f,  53.489f,
     50.883f,  48.278f,  45.672f,  43.067f,  41.108f,  39.148f,  37.189f,  35.229f,
     33.270f,  31.787f,  30.304f,  28.822f,  27.339f,  25.856f,  24.728f,  23.600f,
     22.471f,  21.343f,  20.215f,  19.351f,  18.488f,  17.624f,  16.761f,  15.897f,
     15.232f,  14.567f,  13.903f,  13.238f,  12.573f,  12.058f,  11.544f,  11.029f,
     10.515f,  10.000f,   9.599f,   9.198f,   8.798f,   8.397f,   7.996f,   7.683f,
      7.369f,   7.056f,   6.742f,   6.429f,   6.182f,   5.935f,   5.689f,   5.442f,
      5.195f,   5.000f,   4.805f,   4.610f,   4.415f,   4.220f,   4.065f,   3.910f,
      3.754f,   3.599f,   3.444f,   3.320f,   3.196f,   3.073f,   2.949f,   2.825f,
      2.726f,   2.626f,   2.527f,   2.427f,   2.328f,   2.248f,   2.168f,   2.087f,
      2.007f,   1.927f,   1.862f,   1.797f,   1.732f,   1.667f,   1.602f,   1.549f,
      1.496f,   1.443f,   1.390f,   1.337f,   1.298f,   1.258f,   1.219f,   1.180f,
      1.141f,   1.102f,   1.062f,   1.023f,   0.984f,   0.944f,   0.909f,   0.874f,
      0.839f,   0.804f,   0.770f,   0.735f,   0.700f,   0.671f,   0.643f,   0.615f,
      0.586f,   0.558f,   0.529f,   0.501f,   0.472f,   0.444f,   0.416f,   0.388f,
      0.363f,   0.346f,   0.330f,   0.313f
};

//==================================================
// [10] 보호 기능
//==================================================

uint16_t over_voltage_flag  = 0;        // 과전압 플래그
uint16_t over_current_flag  = 0;        // 과전류 플래그
uint16_t over_temp_flag     = 0;        // 과온도 플래그
uint16_t master_fault_flag  = 0;        // 마스터 고장 플래그

//==================================================
// [11] 시퀀스 제어 (프리차지 등)
//==================================================

uint16_t pre_chg_ok             = 0;    // 프리차지 완료 플래그
uint16_t pre_chg_fail           = 0;    // 프리차지 실패 플래그
uint32_t pre_chg_cnt            = 0;    // 프리차지 카운터
uint32_t relay_off_delay_cnt    = 0;    // 릴레이 OFF 지연 카운터

//==================================================
// [12] 릴레이 제어
//==================================================
// Relay 상태 변수 제거됨 - Phase 3에서 GPIO 직접 제어 방식으로 변경

//==================================================
// [13] 통신 버퍼
//==================================================

// CAN 통신
uint8_t  rxData[4]          = {0};      // CAN 수신 버퍼
uint16_t CANA_txData[4]     = {0x00, 0x00, 0x00, 0x00};  // CAN 송신 버퍼
uint16_t CANA_rxDataArray[RX_MSG_OBJ_COUNT][4] = {0};    // CAN 수신 배열

// SCI 통신 (RS485)
uint8_t scia_rs485_mm_tx_buf[4] = {0x1B, 0, 0, 0x03};    // SCIA RS485 Master-to-Master 송신 버퍼
uint8_t scib_rs485_ms_tx_buf[4] = {0x1B, 0, 0, 0x03};    // SCIB RS485 Master-to-Slave 송신 버퍼

//==================================================
// [14] SCADA 인터페이스
//==================================================

// SCADA 수신 버퍼 (공통)
volatile uint16_t scada_packet_ready = 0;                // 패킷 준비 플래그
volatile uint8_t  scada_rx_buffer[SCADA_RX_BUFFER_SIZE] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00
};
volatile uint16_t scada_rx_index    = 0;                 // 수신 버퍼 인덱스
uint8_t slave_tx_buffer[7]          = {0};               // 슬레이브 송신 버퍼
uint8_t system_tx_buffer[7]         = {0};               // 시스템 송신 버퍼

// SCADA 제어 변수
volatile ControlMode_t control_mode = CONTROL_MODE_CHARGE_DISCHARGE;  // 제어 모드 (초기값: 충방전)
volatile uint8_t  ready_state   = 0;                     // Ready 상태 (bit[6], 초기값: IDLE)
volatile uint8_t  run_state     = 0;                     // Run 상태 (bit[5], 초기값: STOP)
volatile uint8_t  parallel_mode = 0;                     // Parallel 모드 (bit[4], 초기값: Individual)

// 배터리 모드 제어 지령
float32_t V_cmd     = 0.0f;                              // 목표 전압 (배터리 모드 CV 제어)
float32_t I_max_cmd = 80.0f;                             // 최대 전류 제한 (초기값: 80A)
float32_t I_min_cmd = -80.0f;                            // 최소 전류 제한 (초기값: -80A)

// CRC 검증
uint32_t scada_crc_error_cnt = 0;                        // CRC 에러 카운터

//==================================================
// [15] 클럭 정보
//==================================================

volatile uint32_t SPIclk        = 0;    // SPI 클럭
volatile uint32_t epwmclk       = 0;    // EPWM 클럭
volatile uint32_t sysclk        = 0;    // 시스템 클럭
volatile uint32_t lspclk        = 0;    // 저속 주변장치 클럭
volatile uint32_t Epwm1CLK      = 0;    // EPWM1 클럭
volatile uint32_t Epwm3CLK      = 0;    // EPWM3 클럭

//==================================================
// [16] ADC 변수
//==================================================
// ADC 기반 센싱은 사용하지 않음 (FPGA SPI 사용)

//==================================================
// [17] 테스트 변수 (디버그용)
//==================================================

int16_t  current_cA_test    = 0;        // 전류 테스트 변수 (cA 단위)
uint16_t test_voltage       = 0;        // 전압 테스트 변수

//==================================================
// 디버그 변수 (조건부 컴파일)
//==================================================

#ifndef _DEBUG_CLK_STATUS_ENABLE_
#define _DEBUG_CLK_STATUS_ENABLE_ (0)
#endif
#ifndef _DEBUG_CAN_STATUS_ENABLE_
#define _DEBUG_CAN_STATUS_ENABLE_ (0)
#endif
#ifndef _DEBUG_SCI_STATUS_ENABLE_
#define _DEBUG_SCI_STATUS_ENABLE_ (0)
#endif
#ifndef _DEBUG_SPI_STATUS_ENABLE_
#define _DEBUG_SPI_STATUS_ENABLE_ (0)
#endif

#if (_DEBUG_CLK_STATUS_ENABLE_ || _DEBUG_CAN_STATUS_ENABLE_)
volatile uint32_t debug_sysclk_freq         = 0;
volatile uint32_t debug_device_sysclk_freq  = 0;
volatile uint32_t debug_can_clock_source    = 0;
#endif

#if _DEBUG_CAN_STATUS_ENABLE_
volatile uint32_t debug_btr_value           = 0;
volatile uint32_t debug_calculated_bitrate  = 0;
volatile uint16_t debug_actual_prescaler    = 0;
volatile uint16_t debug_total_tq            = 0;
volatile uint16_t debug_tseg1_tq            = 0;
volatile uint16_t debug_tseg2_tq            = 0;
volatile uint16_t debug_tx_err_cnt          = 0;
volatile uint16_t debug_rx_err_cnt          = 0;
volatile uint16_t debug_can_es              = 0;
volatile uint16_t debug_can_bus_off         = 0;
volatile uint16_t debug_can_warn            = 0;
volatile uint16_t debug_can_passive         = 0;
#endif

#if _DEBUG_SCI_STATUS_ENABLE_
volatile uint16_t debug_sci_rx_error        = 0;
volatile uint16_t debug_sci_frame_err       = 0;
volatile uint16_t debug_sci_parity_err      = 0;
volatile uint16_t debug_sci_break           = 0;
volatile uint16_t debug_sci_tx_ready        = 0;
volatile uint16_t debug_sci_rx_ready        = 0;
#endif

#if _DEBUG_SPI_STATUS_ENABLE_
volatile uint16_t debug_spi_tx_full         = 0;
volatile uint16_t debug_spi_tx_ready        = 0;
volatile uint16_t debug_spi_int             = 0;
volatile uint16_t debug_spi_overrun         = 0;
volatile uint16_t debug_spi_rx_hasdata      = 0;
volatile uint16_t debug_spi_rx_overflow     = 0;
#endif

//==================================================
// End of HABA_globals.c
//==================================================
