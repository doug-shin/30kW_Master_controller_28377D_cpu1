//==================================================
// HABA_globals.h - Global Variables and Constants Header
//==================================================
// TI F28377D 30kW Master Controller
// 전역 변수, 상수, 구조체 선언
//==================================================

#ifndef HABA_GLOBALS_H
#define HABA_GLOBALS_H

//==================================================
// Include Files
//==================================================
#include "F28x_Project.h"              // TI 기본 레거시 포함
#include "F2837xD_Cla_defines.h"       // CLA 구조체
#include "driverlib.h"
#include "device.h"
#include <math.h>
#include "DCL.h"                       // DCL (Digital Control Library) - CPU + CLA
#include "DCLCLA.h"                    // DCL CLA-specific functions

#ifdef __cplusplus
extern "C" {
#endif

//==================================================
// 디버그 모드 설정
//==================================================
// 타이밍 측정용 GPIO 토글 활성화 여부
// 0: 비활성화 (릴리스 빌드, 성능 최적화)
// 1: 활성화 (디버깅/타이밍 측정용)
#define ENABLE_TIMING_DEBUG     0

//==================================================
// GPIO 고속 액세스 매크로 (Direct Register Access)
//==================================================
// ISR 내부용 - driverlib보다 ~10배 빠름 (5-10 사이클 vs 50-100 사이클)
// SET 레지스터: 1 쓰면 HIGH, CLEAR 레지스터: 1 쓰면 LOW
//==================================================

// --- Bank A (GPIO 0-31) - Relay 제어 (Low-level) ---
#define GPIO8_SET()     (GpioDataRegs.GPASET.bit.GPIO8 = 1)
#define GPIO8_CLEAR()   (GpioDataRegs.GPACLEAR.bit.GPIO8 = 1)
#define GPIO8_READ()    ((GpioDataRegs.GPADAT.bit.GPIO8) != 0U)

#define GPIO9_SET()     (GpioDataRegs.GPASET.bit.GPIO9 = 1)
#define GPIO9_CLEAR()   (GpioDataRegs.GPACLEAR.bit.GPIO9 = 1)
#define GPIO9_READ()    ((GpioDataRegs.GPADAT.bit.GPIO9) != 0U)

// --- Relay 제어 (하드웨어 기능 중심 매크로) ---
#define MAIN_RELAY_ON()             GPIO8_SET()      // 메인 릴레이 ON (프리차지 후 항상 ON)
#define MAIN_RELAY_OFF()            GPIO8_CLEAR()    // 메인 릴레이 OFF
#define IS_MAIN_RELAY_ON()          GPIO8_READ()     // 메인 릴레이 상태 읽기

#define PARALLEL_LINK_ON()          GPIO9_SET()      // 병렬 연결 릴레이 ON (병렬 모드 전용)
#define PARALLEL_LINK_OFF()         GPIO9_CLEAR()    // 병렬 연결 릴레이 OFF
#define IS_PARALLEL_LINK_ON()       GPIO9_READ()     // 병렬 연결 릴레이 상태 읽기

// --- Bank C (GPIO 64-95) - 디버그 타이밍 측정 ---
#define GPIO90_SET()    (GpioDataRegs.GPCSET.bit.GPIO90 = 1)
#define GPIO90_CLEAR()  (GpioDataRegs.GPCCLEAR.bit.GPIO90 = 1)
#define GPIO91_SET()    (GpioDataRegs.GPCSET.bit.GPIO91 = 1)
#define GPIO91_CLEAR()  (GpioDataRegs.GPCCLEAR.bit.GPIO91 = 1)
#define GPIO92_SET()    (GpioDataRegs.GPCSET.bit.GPIO92 = 1)
#define GPIO92_CLEAR()  (GpioDataRegs.GPCCLEAR.bit.GPIO92 = 1)

// --- 디버그 GPIO 래퍼 (조건부 컴파일) ---
#if ENABLE_TIMING_DEBUG
    #define DEBUG_ISR_START()    GPIO90_SET()
    #define DEBUG_ISR_END()      GPIO90_CLEAR()
    #define DEBUG_1MS_START()    GPIO91_SET()
    #define DEBUG_1MS_END()      GPIO91_CLEAR()
    #define DEBUG_10MS_START()   GPIO92_SET()
    #define DEBUG_10MS_END()     GPIO92_CLEAR()
#else
    #define DEBUG_ISR_START()    
    #define DEBUG_ISR_END()      
    #define DEBUG_1MS_START()    
    #define DEBUG_1MS_END()      
    #define DEBUG_10MS_START()   
    #define DEBUG_10MS_END()     
#endif

//==================================================
// 시스템 상수 정의
//==================================================

// --- 제어 상수 ---
#define STOP                      (0)
#define START                     (1)
#define CURRENT_LIMIT_INDIVIDUAL  (480.0f)    // 개별 모드 최대 전류 (A)
#define CURRENT_LIMIT_PARALLEL    (960.0f)    // 병렬 모드 최대 전류 (A)

// --- 시스템 구성 상수 ---
#define SLAVES_PER_CHANNEL        (6)         // 채널당 슬레이브 수
#define TOTAL_CHANNELS            (2)         // 전체 채널 수 (CH1, CH2)
#define TOTAL_SLAVES              (12)        // 전체 슬레이브 수 (6 x 2)
#define SLAVE_MAX_CURRENT         (80.0f)     // 슬레이브 최대 전류 (A)

// --- 채널/마스터 ID 정의 ---
#define CH1_ID                    (0)         // 채널1 마스터 ID 값
#define CH2_ID                    (1)         // 채널2 마스터 ID 값

// --- 채널 판별 매크로 (master_id 비교 없이 사용) ---
#define IS_CH1                    (master_id == CH1_ID)     // CH1 마스터인가?
#define IS_CH2                    (master_id == CH2_ID)     // CH2 마스터인가?

// 호환성 매크로
#define MASTER1                   CH1_ID      // 상위 마스터
#define MASTER2                   CH2_ID      // 하위 마스터

// --- 보호 임계값 ---
#define OVER_VOLTAGE        (1400)      // 과전압 보호 임계값 (V)
#define OVER_CURRENT        (88.0f)     // 과전류 보호 임계값 (A)
#define OVER_TEMP           (120)       // 과온도 보호 임계값 (°C)

// --- 시퀀스 제어 단계 ---
#define SEQ_STEP_IDLE               (0)     // 대기 (Precharge 준비)
#define SEQ_STEP_PRECHARGE_DONE     (10)    // Precharge 완료, 메인 릴레이 대기 (1초)
#define SEQ_STEP_NORMAL_RUN         (20)    // 정상 운전 (메인 릴레이 ON)

// --- 타이밍 상수 (20kHz 기준) ---
#define TIMING_1SEC_AT_20KHZ        (20000)     // 1초 = 20kHz × 20000
#define TIMING_2SEC_AT_20KHZ        (40000)     // 2초 = 20kHz × 40000
#define TIMING_500MS_AT_20KHZ       (10000)     // 0.5초 = 20kHz × 10000
#define TIMING_200SAMPLES           (200)       // 200샘플 = 10ms @ 20kHz

// --- 타이밍 상수 (100kHz 기준, ISR) ---
#define TIMING_1MS_AT_100KHZ        (100)       // 1ms = 100kHz × 100
#define TIMING_10MS_AT_100KHZ       (1000)      // 10ms = 100kHz × 1000
#define TIMING_50MS_AT_100KHZ       (5000)      // 50ms = 100kHz × 5000

// --- Phase 제어 상수 ---
#define NUM_CONTROL_PHASES          (5)         // 제어 Phase 개수 (0~4)

// --- CAN 통신 상수 ---
#define CAN_RX_FAULT_THRESHOLD      (10000)     // CAN 수신 고장 판정 임계값
#define CAN_MAILBOX_MIN             (2)         // CAN 메일박스 최소값
#define CAN_MAILBOX_MAX             (32)        // CAN 메일박스 최대값

// --- 데이터 변환 상수 ---
#define DAC_MIN_VALUE               (0)         // DAC 최소값
#define DAC_MAX_VALUE               (65535)     // DAC 최대값
#define DAC_CENTER_VALUE            (32768)     // DAC 중앙값 (0A)
#define CURRENT_RANGE_MAX           (100.0f)    // 전류 범위 최대값 (±100A)

#define INT16_MAX_VALUE             (32767)     // int16 최대값
#define INT16_MIN_VALUE             (-32768)    // int16 최소값
#define UINT8_MAX_VALUE             (255)       // uint8 최대값

#define VOLTAGE_TEST_INCREMENT      (500)       // 전압 테스트 증가값 (5.0V)
#define VOLTAGE_TEST_MAX            (5000)      // 전압 테스트 최대값 (500.0V)

// --- SCADA 프로토콜 상수 ---
#define SCADA_PACKET_SIZE           (10)        // SCADA 패킷 크기 (바이트)
#define SCADA_SLAVE_PACKET_SIZE     (7)         // 슬레이브 상태 패킷 크기
#define SCADA_MAX_SLAVES            (15)        // SCADA 송신 최대 슬레이브 수

// --- 전압 차이 임계값 (Precharge) ---
#define PRECHARGE_VOLTAGE_DIFF_OK   (2.0f)      // Precharge 완료 판정 전압 차 (±2V)

//==================================================
// 센싱 및 캘리브레이션 상수
//==================================================
// 전압/전류 센싱의 2단계 변환 과정:
//   Step 1: ADC 원시값 → 물리값 (HW 센서 특성, 고정)
//   Step 2: 물리값 → 캘리브레이션된 값 (실측 보정, 가변)
//==================================================

// --- Step 1: 전압 ADC 원시값 → 물리 전압 변환 (공통) ---
#define VOLTAGE_ADC_SCALE           (0.019856f)     // ADC LSB당 전압 (V/count)
#define VOLTAGE_ADC_OFFSET          (50.573f)       // ADC 영점 오프셋 (V)

// --- Step 2: 출력 전압 (V_out) 실측 캘리브레이션 ---
#define VOUT_CALIB_OFFSET           (0.091694057f)  // 실측 오프셋 보정 (V)
#define VOUT_CALIB_GAIN             (0.9926000888f) // 실측 게인 보정

// --- Step 2: 배터리 전압 (V_batt) 실측 캘리브레이션 ---
#define VBATT_CALIB_OFFSET          (-0.3058461657f) // 실측 오프셋 보정 (V)
#define VBATT_CALIB_GAIN            (0.9945009708f)  // 실측 게인 보정

// --- 전류 변환 상수 (CAN 슬레이브 전류) ---
#define CURRENT_RAW_TO_NORM_SCALE   (2.0f)          // 0~1 → -1~+1 정규화 스케일
#define CURRENT_RAW_TO_NORM_OFFSET  (1.0f)          // 정규화 오프셋
#define CURRENT_NORM_TO_AMP_SCALE   (100.0f)        // 정규화 → A 변환 (±100A)

// --- DAC 변환 상수 (슬레이브 전류 지령 → DAC 코드) ---
#define SLAVE_CURRENT_TO_DAC_SCALE  (327.68f)       // = 32768 / 100 (±100A → 0~65535)
#define SLAVE_CURRENT_TO_DAC_OFFSET (32768.0f)      // DAC 중앙값 (0A 기준)

// --- 소프트스타트 램프 상수 ---
#define SOFTSTART_RAMP_COEFF        (0.00005f)      // 20kHz 기준 램프 계수
// 램프율 계산: I_ss_ramp += current_limit * SOFTSTART_RAMP_COEFF
//   개별 모드: 480A * 0.00005 * 20000 = 480A/1s → 9.6초 도달
//   병렬 모드: 960A * 0.00005 * 20000 = 960A/1s → 19.2초 도달

// --- 평균 계산 계수 ---
#define AVG_5_SAMPLES_COEFF         (0.2f)          // = 1/5 (5회 평균, Phase 0)
#define AVG_200_SAMPLES_COEFF       (0.005f)        // = 1/200 (10ms @ 20kHz, Phase 4)

//==================================================
// 릴레이 상태 상수
//==================================================

// --- 릴레이 제어 (GPIO 직접 제어 방식) ---
// 호환성 매크로 (하드웨어 기능 매크로 사용 권장)
#define IS_RELAY_INDIVIDUAL_ON()    IS_MAIN_RELAY_ON()          // → IS_MAIN_RELAY_ON() 사용 권장
#define IS_RELAY_PARALLEL_ON()      IS_PARALLEL_LINK_ON()       // → IS_PARALLEL_LINK_ON() 사용 권장

// --- PI 제어 파라미터 ---
#define Kp_set              1           // 비례 게인
#define Ki_set              3000        // 적분 게인
#define T_sample_set        50e-6f      // 샘플링 주기 (50us)
#define I_out_MAX           80          // 최대 출력 전류 (A)

// --- 저역통과 필터 파라미터 ---
#define wc                  (1e+3)      // Cut-off frequency = 1kHz
#define Ts                  (50e-6)     // 샘플링 타임 = 50us

// --- 통신 프로토콜 상수 ---
#define STX                 0x1B        // Start of Text
#define ETX                 0x03        // End of Text
#define _1ms_num            100         // 10us * 100 = 1ms

// --- CAN 통신 상수 ---
#define MSG_DATA_LENGTH     4
#define TX_MSG_OBJ_ID1      1
#define RX_MSG_OBJ_BASE_ID  2
#define RX_MSG_OBJ_COUNT    30
#define MAX_SLAVES          31

//==================================================
// GPIO 핀 매핑 정의
//==================================================

// --- CAN 통신 (CANA) ---
#define GPIO_PIN_CANRXA                 5
#define CANA_SLAVE_ID_CANRX_GPIO        5
#define CANA_SLAVE_ID_CANRX_PIN_CONFIG  GPIO_5_CANRXA

#define GPIO_PIN_CANTXA                 4
#define CANA_SLAVE_ID_CANTX_GPIO        4
#define CANA_SLAVE_ID_CANTX_PIN_CONFIG  GPIO_4_CANTXA

// --- SCIA RS485 (Master-to-Master 통신) ---
#define SCIA_RS485_MM_RX_GPIO           64
#define SCIA_RS485_MM_RX_PIN_CONFIG     GPIO_64_SCIRXDA
#define SCIA_RS485_MM_TX_GPIO           65
#define SCIA_RS485_MM_TX_PIN_CONFIG     GPIO_65_SCITXDA
#define SCIA_RS485_MM_DE_GPIO           66      // Driver Enable
#define SCIA_RS485_MM_DE_PIN_CONFIG     GPIO_66_GPIO66

// --- SCIB RS485 (Master-to-Slave 통신) ---
#define SCIB_RS485_MS_RX_GPIO           71
#define SCIB_RS485_MS_RX_PIN_CONFIG     GPIO_71_SCIRXDB
#define SCIB_RS485_MS_TX_GPIO           70
#define SCIB_RS485_MS_TX_PIN_CONFIG     GPIO_70_SCITXDB
#define SCIB_RS485_MS_DE_GPIO           69      // Driver Enable
#define SCIB_RS485_MS_DE_PIN_CONFIG     GPIO_69_GPIO69

// --- SPI-A (DAC80502) ---
#define GPIO_PIN_SPIA_PICO              16
#define SPI_DAC1_SPIPICO_GPIO           16
#define SPI_DAC1_SPIPICO_PIN_CONFIG     GPIO_16_SPISIMOA

#define GPIO_PIN_SPIA_POCI              17
#define SPI_DAC1_SPIPOCI_GPIO           17
#define SPI_DAC1_SPIPOCI_PIN_CONFIG     GPIO_17_SPISOMIA

#define GPIO_PIN_SPIA_CLK               18
#define SPI_DAC1_SPICLK_GPIO            18
#define SPI_DAC1_SPICLK_PIN_CONFIG      GPIO_18_SPICLKA

#define GPIO_PIN_SPIA_PTE               19
#define SPI_DAC1_SPIPTE_GPIO            19
#define SPI_DAC1_SPIPTE_PIN_CONFIG      GPIO_19_SPISTEA

// --- SPI-C (FPGA ADC) ---
#define SPIC_SIMO_GPIO                  50      // DSP → FPGA
#define SPIC_SIMO_PIN_CONFIG            GPIO_50_SPISIMOC
#define SPIC_SOMI_GPIO                  51      // FPGA → DSP
#define SPIC_SOMI_PIN_CONFIG            GPIO_51_SPISOMIC
#define SPIC_CLK_GPIO                   52
#define SPIC_CLK_PIN_CONFIG             GPIO_52_SPICLKC
#define SPIC_STE_GPIO                   53      // Chip Select
#define SPIC_STE_PIN_CONFIG             GPIO_53_SPISTEC

// --- 전면 패널 LED ---
#define LED31                           31
#define LED32                           32
#define LED33                           33
#define LED34                           34
#define LED35                           35
#define LED36                           36
#define LED37                           37
#define LED38                           38
#define LED39                           39
#define LED_PWR                         46      // F_LED6 전원
#define LED_CHARGE                      47      // F_LED5 충전
#define LED_DISCHARGE                   42      // F_LED4 방전
#define LED_FAULT                       43      // F_LED3 고장
#define LED_SINGLE                      67      // F_LED2 개별운전
#define LED_DUAL                        68      // F_LED1 병렬운전

//==================================================
// Enumeration Types (열거형)
//==================================================

// --- 운전 모드 ---
typedef enum {
    MODE_STOP        = 0,   // 정지
    MODE_INDIVIDUAL  = 1,   // 개별 운전 (CH1, CH2 각각 독립 제어)
    MODE_PARALLEL    = 2,   // 병렬 운전 (상위 마스터만 PI, 하위는 전달만)
    MODE_RESERVED    = 3    // 예약 (미사용)
} OperationMode_t;

// --- 시스템 상태 ---
typedef enum {
    STATE_NO_OP,            // 동작 없음
    STATE_READY,            // 준비
    STATE_PRE_CHG_OK,       // 프리차지 완료
    STATE_STAND_BY,         // 대기
    STATE_RUN,              // 운전
    STATE_RL_OFF,           // 릴레이 OFF
    STATE_FAULT             // 고장
} SystemState;

//==================================================
// SCADA 통신 구조체
//==================================================

#define SCADA_SCITX_PIN                 76U
#define SCADA_SCIRX_PIN                 77U
#define GPIO_76_SCIDTX                  0x00861806U
#define GPIO_77_SCIDRX                  0x00861A06U
#define SCADA_SCITX_PIN_CONFIG          GPIO_76_SCIDTX
#define SCADA_SCIRX_PIN_CONFIG          GPIO_77_SCIDRX
#define SCADA_BAUD_RATE                 115200U
#define SCADA_RX_BUFFER_SIZE            10
#define Rack_Channel                    0       // 랙 번호 (0~3)

// --- SCADA 수신 패킷 (SCADA → Master) ---
typedef struct
{
    uint8_t  start_byte;    // 0x02 (STX)
    uint8_t  command;       // bit[0]: Run(0=Stop,1=Start), bit[2:1]: Mode
    int16_t  max_voltage;   // Byte2~3 최대 전압 (V)
    int16_t  min_voltage;   // Byte4~5 최소 전압 (V)
    int16_t  current_cmd;   // Byte6~7 전류 지령 (A)
    uint8_t  checksum;      // Byte8 체크섬 (Sum Byte1~7)
    uint8_t  end_byte;      // 0x03 (ETX)
} SCADA_PACKET;

// --- 시스템 전압 송신 패킷 (Master → SCADA) ---
typedef struct
{
    uint8_t stx;            // 0x02
    uint8_t id;             // ID + Channel
    int16_t systemVoltage;  // 시스템 전압 (Big-endian, ÷10)
    uint8_t reserved;       // 예약
    uint8_t checksum;       // Sum(Byte1~4) & 0xFF
    uint8_t etx;            // 0x03
} SYSTEM_TX_PACKET;

// --- 슬레이브 데이터 송신 패킷 (Master → SCADA) ---
typedef struct
{
    uint8_t stx;            // 0x02
    uint8_t idAndStatus;    // bit[7:3]: Slave ID, bit[0]: DAB_OK
    int16_t slaveCurrent;   // 슬레이브 전류 (Big-endian, Center=32768)
    uint8_t slaveTemp;      // 슬레이브 온도 (×0.5)
    uint8_t checksum;       // Sum(Byte1~4) & 0xFF
    uint8_t etx;            // 0x03
} SLAVE_TX_PACKET;

//==================================================
// 디버그 플래그 (조건부 컴파일)
//==================================================

#ifndef _DEBUG_CLK_STATUS_ENABLE_
#define _DEBUG_CLK_STATUS_ENABLE_       (0)
#endif

#ifndef _DEBUG_CAN_STATUS_ENABLE_
#define _DEBUG_CAN_STATUS_ENABLE_       (0)
#endif

#ifndef _DEBUG_SCI_STATUS_ENABLE_
#define _DEBUG_SCI_STATUS_ENABLE_       (0)
#endif

#ifndef _DEBUG_SPI_STATUS_ENABLE_
#define _DEBUG_SPI_STATUS_ENABLE_       (0)
#endif

//==================================================
// 외부 변수 선언 (extern)
//==================================================

//--------------------------------------------------
// [1] CLA 공유 메모리 변수 (CPU ↔ CLA)
//--------------------------------------------------

// CPU → CLA (제어 지령)
extern float32_t V_max_cmd;            // 최대 전압 지령 (충전)
extern float32_t V_min_cmd;            // 최소 전압 지령 (방전)
extern float32_t V_fb;                 // 전압 피드백

// CLA PI 제어기 (DCL - Digital Control Library)
extern DCL_PI_CLA pi_charge;           // PI 제어기 (충전 모드, V_max_cmd 기준)
extern DCL_PI_CLA pi_discharge;        // PI 제어기 (방전 모드, V_min_cmd 기준)

// CLA → CPU (제어 결과)
extern float32_t I_PI_charge_out;      // PI 출력 (충전 전류)
extern float32_t I_PI_discharge_out;   // PI 출력 (방전 전류)
extern uint16_t  cla_cnt;              // CLA 실행 카운터 (디버그용)

//--------------------------------------------------
// [2] 시스템 상태 및 타이밍
//--------------------------------------------------

extern SystemState      state;              // 시스템 상태
extern OperationMode_t  operation_mode;     // 운전 모드
extern uint32_t         control_phase;      // 제어 Phase (0~4)
extern uint16_t         sequence_step;      // 시퀀스 단계
extern uint16_t         start_stop;         // START(1) / STOP(0)
extern uint32_t         run;                // 실행 플래그
extern int16_t          run_switch;         // 운전 스위치 상태

// 타이밍 플래그
extern uint32_t cnt_1ms;                    // 1ms 카운터
extern uint32_t flag_1ms;                   // 1ms 플래그
extern uint16_t cnt_10ms;                   // 10ms 카운터
extern bool     flag_10ms;                  // 10ms 플래그
extern uint16_t cnt_50ms;                   // 50ms 카운터
extern bool     flag_50ms;                  // 50ms 플래그
extern uint32_t flag_epwm1_int;             // EPWM1 인터럽트 플래그

//--------------------------------------------------
// [3] 마스터 제어
//--------------------------------------------------

extern uint16_t master_id;                  // 마스터 ID (0=상위, 1=하위)
extern uint8_t  master_mode;                // 마스터 모드 (1=PI제어, 2=전달만)

//--------------------------------------------------
// [4] 전압 센싱 및 피드백
//--------------------------------------------------

// 출력 전압 (V_out)
extern float32_t V_out;                     // 캘리브레이션된 출력 전압
extern float32_t V_out_raw;                 // 원시 출력 전압
extern int32_t   V_out_raw_sum;             // 출력 전압 누적합 (평균용)
extern float32_t V_out_raw_avg;             // 출력 전압 평균
extern float32_t V_out_uncal;               // 미캘리브레이션 출력 전압

// 배터리 전압 (V_batt)
extern float32_t V_batt;                    // 캘리브레이션된 배터리 전압
extern float32_t V_batt_raw;                // 원시 배터리 전압
extern int32_t   V_batt_raw_sum;            // 배터리 전압 누적합 (평균용)
extern float32_t V_batt_raw_avg;            // 배터리 전압 평균
extern float32_t V_batt_uncal;              // 미캘리브레이션 배터리 전압
extern float32_t V_batt_avg;                // 배터리 평균 전압 (장기)

// 전압 모니터링
extern volatile float32_t V_out_ADC;        // SPI ADC 전압 원시값
extern float32_t V_out_ADC_avg;             // SPI ADC 전압 평균 (5회)
extern uint32_t  V_out_ADC_sum;             // SPI ADC 전압 누적합
extern float32_t V_fb_sum;                  // 전압 피드백 누적합 (장기 평균)
extern uint32_t  V_cal_cnt;                 // 전압 계산 카운터

// 디스플레이용 전압
extern float32_t V_out_display;             // 출력 전압 표시값
extern float32_t V_batt_display;            // 배터리 전압 표시값
extern float32_t V_out_display_sum;         // 출력 전압 표시 누적합
extern float32_t V_batt_display_sum;        // 배터리 전압 표시 누적합
extern uint16_t  V_display_calc_cnt;        // 디스플레이 계산 카운터

//--------------------------------------------------
// [5] 전류 센싱 및 피드백
//--------------------------------------------------

// 전류 지령
extern float32_t I_cmd;                     // 기본 전류 지령 (SCADA 수신)
extern float32_t I_cmd_ramped;              // 램프 제한 적용 전류
extern float32_t I_cmd_PI_limited;          // PI 제한 적용 전류 (DAC 출력용)
// I_cmd_tmp - 삭제됨 (불필요, I_out_ref를 직접 사용)
extern float32_t I_cmd_ch1;                 // CH1 전류 지령 (개별 운전)
extern float32_t I_cmd_ch2;                 // CH2 전류 지령 (개별 운전)
extern uint16_t  I_cmd_from_master;         // 상위 마스터로부터 수신한 지령

// 전류 센싱 (FPGA SPI)
extern float32_t I_out_raw;                 // 원시 출력 전류
extern uint32_t  I_out_raw_sum;             // 출력 전류 누적합
extern float32_t I_out_raw_avg;             // 출력 전류 평균
extern float32_t I_out_sen_sum;             // 전류 센싱 누적합
extern float32_t I_out_sen_sum_monitor;     // 모니터용 전류 누적합

// 전류 센싱 (ADC)
extern uint16_t  I_out_ADC;                 // 실시간 ADC 전류값 (100kHz)
extern uint32_t  I_out_ADC_sum;             // 전류 ADC 누적합 (5회 평균용)
extern uint32_t  I_out_ADC_avg;             // 5회 평균 ADC 전류값 (20kHz)

// 전류 피드백 및 평균
extern float32_t I_out_avg;                 // 출력 전류 평균
extern float32_t I_fb_sum;                  // 전류 피드백 누적합 (장기 평균)
extern float32_t I_fb_avg;                  // 전류 피드백 평균
extern int16_t   I_cal_cnt;                 // 전류 계산 카운터
extern int16_t   I_out_ref;                 // 전류 레퍼런스

//--------------------------------------------------
// [6] 소프트 스타트 및 필터
//--------------------------------------------------

extern float32_t I_ss_ramp;                 // 소프트 스타트 램프 제한
extern float32_t I_cmd_filtered;            // LPF 필터 출력
extern float32_t I_cmd_prev;                // 이전 입력 저장
extern float32_t lpf_coeff_a;               // LPF 전향 계수
extern float32_t lpf_coeff_b;               // LPF 피드백 계수

//--------------------------------------------------
// [7] DAC 출력
//--------------------------------------------------

extern uint16_t I_cmd_to_slave;                  // DAC 출력값 (0~65535)

//--------------------------------------------------
// [8] 슬레이브 관리 (CAN 통신)
//--------------------------------------------------

extern bool         slave_enabled[MAX_SLAVES+1];    // 슬레이브 활성화 상태
extern uint16_t     I_out_slave_raw[31];            // 슬레이브 전류 원시값
extern float32_t    I_out_slave[31];                // 슬레이브 전류 (캘리브레이션)
extern uint16_t     temp_slave_raw[31];             // 슬레이브 온도 원시값
extern uint16_t     DAB_ok_slave[31];               // 슬레이브 DAB OK 상태
extern uint16_t     can_rx_fault_cnt[31];           // CAN 수신 고장 카운터
extern int16_t      I_out_slave_total;              // 슬레이브 총 전류

//--------------------------------------------------
// [9] 온도 센싱 (NTC)
//--------------------------------------------------

extern float32_t NTC_0_temp;                // NTC0 온도 (°C)
extern float32_t NTC_1_temp;                // NTC1 온도 (°C)
extern const float LUT_Resistance[161];     // NTC 저항 Look-Up Table

//--------------------------------------------------
// [10] 보호 기능
//--------------------------------------------------

extern uint16_t over_voltage_flag;          // 과전압 플래그
extern uint16_t over_current_flag;          // 과전류 플래그
extern uint16_t over_temp_flag;             // 과온도 플래그
extern uint16_t master_fault_flag;          // 마스터 고장 플래그

//--------------------------------------------------
// [11] 시퀀스 제어 (프리차지 등)
//--------------------------------------------------

extern uint16_t pre_chg_ok;                 // 프리차지 완료 플래그
extern uint16_t pre_chg_fail;               // 프리차지 실패 플래그
extern uint32_t pre_chg_cnt;                // 프리차지 카운터
extern uint32_t relay_off_delay_cnt;        // 릴레이 OFF 지연 카운터

//--------------------------------------------------
// [12] 릴레이 제어
//--------------------------------------------------
// Relay 상태 변수 제거됨 - Phase 3에서 GPIO 직접 제어 방식으로 변경
// 상태 확인이 필요한 경우 IS_RELAY_INDIVIDUAL_ON(), IS_RELAY_PARALLEL_ON() 매크로 사용

//--------------------------------------------------
// [13] 통신 버퍼
//--------------------------------------------------

// CAN 통신
extern uint8_t  rxData[4];                  // CAN 수신 버퍼
extern uint16_t CANA_txData[4];             // CAN 송신 버퍼
extern uint16_t CANA_rxDataArray[RX_MSG_OBJ_COUNT][4];  // CAN 수신 배열

// SCI 통신 (RS485)
extern uint8_t scia_rs485_mm_tx_buf[4];     // SCIA RS485 Master-to-Master 송신 버퍼
extern uint8_t scib_rs485_ms_tx_buf[4];     // SCIB RS485 Master-to-Slave 송신 버퍼

//--------------------------------------------------
// [14] SCADA 인터페이스
//--------------------------------------------------

extern volatile SCADA_PACKET scada_rx_data;             // SCADA 수신 패킷
extern volatile uint16_t scada_packet_ready;            // 패킷 준비 플래그
extern volatile uint8_t  scada_rx_buffer[SCADA_RX_BUFFER_SIZE];  // SCADA 수신 버퍼
extern volatile uint16_t scada_rx_index;                // 수신 버퍼 인덱스
extern uint8_t slave_tx_buffer[7];                      // 슬레이브 송신 버퍼
extern uint8_t system_tx_buffer[7];                     // 시스템 송신 버퍼

// SCADA 명령
extern volatile uint16_t g_systemRunCommand;            // 시스템 실행 명령
extern volatile int16_t  g_maxVoltageSet;               // 최대 전압 설정
extern volatile int16_t  g_minVoltageSet;               // 최소 전압 설정
extern volatile int8_t   g_currentCommandSet;           // 전류 지령 설정
extern volatile uint8_t  SCADA_cmd;                     // SCADA 명령

//--------------------------------------------------
// [15] 클럭 정보
//--------------------------------------------------

extern volatile uint32_t SPIclk;            // SPI 클럭
extern volatile uint32_t epwmclk;           // EPWM 클럭
extern volatile uint32_t sysclk;            // 시스템 클럭
extern volatile uint32_t lspclk;            // 저속 주변장치 클럭
extern volatile uint32_t Epwm1CLK;          // EPWM1 클럭
extern volatile uint32_t Epwm3CLK;          // EPWM3 클럭

//--------------------------------------------------
// [16] ADC 변수 (사용 여부 확인 필요)
//--------------------------------------------------

extern uint16_t adc_current;                // ADC 전류 (미사용?)
extern uint16_t adc_batt_voltage;           // ADC 배터리 전압 (미사용?)
extern uint16_t adc_out_voltage;            // ADC 출력 전압 (미사용?)

//--------------------------------------------------
// [17] 테스트 변수 (디버그용)
//--------------------------------------------------

extern int16_t  current_cA_test;            // 전류 테스트 변수 (cA 단위)
extern uint16_t test_voltage;               // 전압 테스트 변수

//==================================================
// 디버그 변수 (조건부 컴파일)
//==================================================

#if (_DEBUG_CLK_STATUS_ENABLE_ || _DEBUG_CAN_STATUS_ENABLE_)
extern volatile uint32_t debug_sysclk_freq;
extern volatile uint32_t debug_device_sysclk_freq;
extern volatile uint32_t debug_can_clock_source;
#endif

#if _DEBUG_CAN_STATUS_ENABLE_
extern volatile uint32_t debug_btr_value;
extern volatile uint32_t debug_calculated_bitrate;
extern volatile uint16_t debug_actual_prescaler;
extern volatile uint16_t debug_total_tq;
extern volatile uint16_t debug_tseg1_tq;
extern volatile uint16_t debug_tseg2_tq;
extern volatile uint16_t debug_tx_err_cnt;
extern volatile uint16_t debug_rx_err_cnt;
extern volatile uint16_t debug_can_es;
extern volatile uint16_t debug_can_bus_off;
extern volatile uint16_t debug_can_warn;
extern volatile uint16_t debug_can_passive;
#endif

#if _DEBUG_SCI_STATUS_ENABLE_
extern volatile uint16_t debug_sci_rx_error;
extern volatile uint16_t debug_sci_frame_err;
extern volatile uint16_t debug_sci_parity_err;
extern volatile uint16_t debug_sci_break;
extern volatile uint16_t debug_sci_tx_ready;
extern volatile uint16_t debug_sci_rx_ready;
#endif

#if _DEBUG_SPI_STATUS_ENABLE_
extern volatile uint16_t debug_spi_tx_full;
extern volatile uint16_t debug_spi_tx_ready;
extern volatile uint16_t debug_spi_int;
extern volatile uint16_t debug_spi_overrun;
extern volatile uint16_t debug_spi_rx_hasdata;
extern volatile uint16_t debug_spi_rx_overflow;
#endif

//==================================================
// 함수 선언
//==================================================

// CLA Task 인터럽트
extern __interrupt void Cla1Task1(void);
extern __interrupt void Cla1Task2(void);

// CLA ISR
extern __interrupt void CLA1_ISR1(void);
extern __interrupt void CLA1_ISR2(void);

// 메인 ISR
extern __interrupt void INT_ADCA1_ISR(void);
extern __interrupt void INT_EPWM1_ISR(void);

// 제어 Phase 함수
extern void Update_Voltage_Sensing(void);              // Phase 0: 전압 센싱
extern void Execute_Current_Control(void);             // Phase 1: PI 제어
extern void Transmit_Current_Command(void);            // Phase 2: 전류 지령 전송
extern void Check_System_Safety(void);                 // Phase 3: 안전 체크
extern void Update_Monitoring_And_Sequence(void);      // Phase 4: 모니터링

#ifdef __cplusplus
}
#endif

#endif // HABA_GLOBALS_H

//==================================================
// End of HABA_globals.h
//==================================================
