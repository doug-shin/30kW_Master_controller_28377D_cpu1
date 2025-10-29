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
#include "vcu2/vcu2_crc.h"             // VCU2 CRC 하드웨어 가속 (CRC_Obj, CRC_Handle)

#ifdef __cplusplus
extern "C" {
#endif

//==================================================
// 디버그 모드 설정
//==================================================
// 타이밍 측정용 GPIO 토글 활성화 여부
// 0: 비활성화 (릴리스 빌드, 성능 최적화)
// 1: 활성화 (디버깅/타이밍 측정용)
#define ENABLE_TIMING_DEBUG     1

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
    #define DEBUG_ISR_START()    ((void)0)
    #define DEBUG_ISR_END()      ((void)0)
    #define DEBUG_1MS_START()    ((void)0)
    #define DEBUG_1MS_END()      ((void)0)
    #define DEBUG_10MS_START()   ((void)0)
    #define DEBUG_10MS_END()     ((void)0)
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

// --- 채널/마스터 ID 정의 ---
#define CH1_ID                    (0)         // 채널1 마스터 ID 값
#define CH2_ID                    (1)         // 채널2 마스터 ID 값

// --- 채널 판별 매크로 (master_id 비교 없이 사용) ---
#define IS_CH1                    (master_id == CH1_ID)     // CH1 마스터인가?
#define IS_CH2                    (master_id == CH2_ID)     // CH2 마스터인가?

// --- 보호 임계값 ---
#define OVER_VOLTAGE        (1400)      // 과전압 보호 임계값 (V)
#define OVER_CURRENT        (88.0f)     // 과전류 보호 임계값 (A)
#define OVER_TEMP           (85)        // 과온도 보호 임계값 (°C) - 방열판 온도, 강제 공랭(팬×3)
#define OVER_TEMP_WARNING   (75)        // 과온도 경고 임계값 (°C) - Derating 시작 레벨

// --- 시퀀스 제어 단계 (Enum으로 타입 안전성 강화) ---
typedef enum {
    SEQ_STEP_IDLE = 0,              // 대기 (Precharge 준비, SCADA cmd_ready=0)
    SEQ_STEP_PRECHARGE_DONE = 10,   // Precharge 완료, 메인 릴레이 대기 (1초)
    SEQ_STEP_NORMAL_RUN = 20        // 정상 운전 (메인 릴레이 ON, SCADA cmd_run=1)
} SequenceStep_t;

// --- 타이밍 상수 (20kHz 기준) ---
#define TIMING_1SEC_AT_20KHZ        (20000)     // 1초 = 20kHz × 20000
#define TIMING_200SAMPLES           (200)       // 200샘플 = 10ms @ 20kHz

// --- 타이밍 상수 (100kHz 기준, ISR) ---
#define TIMING_1MS_AT_100KHZ        (100)       // 1ms = 100kHz × 100
#define TIMING_10MS_AT_100KHZ       (1000)      // 10ms = 100kHz × 1000
#define TIMING_50MS_AT_100KHZ       (5000)      // 50ms = 100kHz × 5000

// --- 제어 주기 상수 (ISR 내부) ---
#define ISR_FREQUENCY_HZ            (100000)    // EPWM1 ISR 주파수 (100kHz)
#define PHASE_CYCLE_FREQUENCY_HZ    (20000)     // Phase 순환 주파수 (20kHz, 5 phases @ 100kHz)
#define ISR_PERIOD_US               (10)        // ISR 주기 (10μs)
#define PHASE_PERIOD_US             (50)        // Phase 주기 (50μs)

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

#define INT16_MAX_VALUE             (32767)     // int16 최대값
#define INT16_MIN_VALUE             (-32768)    // int16 최소값
#define UINT8_MAX_VALUE             (255)       // uint8 최대값

#define VOLTAGE_TEST_INCREMENT      (500)       // 전압 테스트 증가값 (5.0V)
#define VOLTAGE_TEST_MAX            (5000)      // 전압 테스트 최대값 (500.0V)

// --- SCADA 프로토콜 상수 ---
#define SCADA_PACKET_SIZE       (16)        // SCADA 수신 패킷 크기 (16 bytes 고정)
#define SCADA_SLAVE_PACKET_SIZE (16)        // 슬레이브 배치 패킷 크기 (16 bytes)
#define SCADA_PAYLOAD_SIZE      (14)        // Payload 크기 (STX/ETX 제외)
#define SCADA_SLAVES_PER_PACKET (3)         // 배치 전송 슬레이브 수 (3개/패킷)
#define SCADA_MAX_SLAVES        (6)         // Active Slave List 최대 6개

// 하위 호환성 (deprecated)
#define REV5_PACKET_SIZE        SCADA_PACKET_SIZE
#define REV5_PAYLOAD_SIZE       SCADA_PAYLOAD_SIZE
#define REV5_SLAVES_PER_PACKET  SCADA_SLAVES_PER_PACKET
#define REV5_MAX_ACTIVE_SLAVES  SCADA_MAX_SLAVES

// --- 전압 차이 임계값 (Precharge) ---
#define PRECHARGE_VOLTAGE_DIFF_OK   (2.0f)      // Precharge 완료 판정 전압 차 (±2V)

// --- 고장 플래그 상수 ---
#define FAULT_FLAG_ACTIVE           (1)         // 고장 발생 (활성)
#define FAULT_FLAG_INACTIVE         (0)         // 정상 상태 (비활성)

// --- LED 상태 상수 ---
#define LED_ON                      (1)         // LED 켜짐
#define LED_OFF                     (0)         // LED 꺼짐

// --- 시퀀스 타이밍 상수 ---
#define SEQ_PRECHARGE_DELAY_COUNT   (TIMING_1SEC_AT_20KHZ)  // Precharge 완료 후 1초 대기 (20000 counts @ 20kHz)

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

// --- PI 제어 파라미터 ---
#define Kp_set              1           // 비례 게인
#define Ki_set              3000        // 적분 게인
#define T_sample_set        50e-6f      // 샘플링 주기 (50us)

// --- 저역통과 필터 파라미터 ---
#define wc                  (1e+3)      // Cut-off frequency = 1kHz
#define Ts                  (50e-6)     // 샘플링 타임 = 50us

// --- 통신 프로토콜 상수 ---
#define STX                 0x1B        // Start of Text
#define ETX                 0x03        // End of Text

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

// --- 운전 모드 (채널 구성) ---
// 주의: Run/Stop 제어와 독립적 (scada_cmd.cmd_run, run 변수로 제어)
typedef enum {
    MODE_INDIVIDUAL  = 0,   // 개별 운전 (CH1, CH2 각각 독립 제어)
    MODE_PARALLEL    = 1    // 병렬 운전 (CH1+CH2 통합, Master1이 PI, Master2는 패스스루)
} OperationMode_t;

// --- 제어 모드 ---
typedef enum {
    CONTROL_MODE_CHARGE_DISCHARGE = 0,  // 충방전 모드 (bit[7]=0): V_max/V_min 제한, I_cmd 제어
    CONTROL_MODE_BATTERY          = 1   // 배터리 모드 (bit[7]=1): V_cmd CV 제어, I_max/I_min 제한
} ControlMode_t;

//==================================================
// SCADA 인터페이스 구조체
//==================================================

/**
 * @brief SCADA 명령 구조체 (SCADA → Master)
 *
 * SCADA가 마스터에게 내리는 명령들을 포함
 * Parse_SCADA_Command()에서 수신 패킷을 파싱하여 이 구조체에 저장
 */
typedef struct {
    // Command byte (SCADA → Master 명령)
    ControlMode_t control_mode;     // bit[7]: 제어 모드 (0=Charge/Discharge, 1=Battery)
    uint8_t       cmd_ready;        // bit[6]: Ready 명령 (0=IDLE, 1=READY - Precharge 시작하라)
    uint8_t       cmd_run;          // bit[5]: Run 명령 (0=STOP, 1=RUN - 정상 운전)
    uint8_t       parallel_mode;    // bit[4]: 병렬 모드 (0=Individual, 1=Parallel)

    // Parameters (제어 변수 - 모드에 따라 의미 다름)
    float32_t     V_max_cmd;        // 충전 최대 전압 (충방전 모드)
    float32_t     V_min_cmd;        // 방전 최소 전압 (충방전 모드)
    float32_t     I_cmd;            // 전류 지령 (충방전 모드)
    float32_t     V_cmd;            // 전압 지령 (배터리 모드 CV)
    float32_t     I_max_cmd;        // 최대 전류 제한 (배터리 모드)
    float32_t     I_min_cmd;        // 최소 전류 제한 (배터리 모드)
} SCADA_Command_t;

/**
 * @brief 마스터 상태 구조체 (Master → SCADA)
 *
 * 마스터의 현재 상태를 SCADA로 피드백하기 위한 구조체
 * Send_System_Status_To_SCADA()에서 이 구조체를 패킷으로 변환하여 송신
 */
typedef struct {
    // 시퀀스 상태 (마스터가 현재 상태 알려줌)
    uint8_t       ready;            // Precharge 완료 여부 (0=IDLE, 1=READY)
    uint8_t       running;          // 운전 중 여부 (0=STOP, 1=RUN)
    uint8_t       precharge_ok;     // Precharge 완료 플래그 (1=완료, V_out ≈ V_batt)
    SequenceStep_t sequence_step;   // 현재 시퀀스 단계 (IDLE/PRECHARGE_DONE/NORMAL_RUN)

    // 고장 상태 (마스터 자체 고장)
    uint8_t       fault_latched;    // 고장 래치 플래그 (1=고장 발생, IDLE로 리셋 가능)
    uint8_t       over_voltage;     // 과전압 플래그 (V_out ≥ 1400V)
    uint8_t       over_current;     // 과전류 플래그 (|I_out| ≥ 88A)
    uint8_t       over_temp;        // 과온도 플래그 (NTC ≥ 85°C)

    // 측정값 (디스플레이용 - 캘리브레이션 적용)
    float32_t     V_out;            // 출력 전압 (V)
    float32_t     V_batt;           // 배터리 전압 (V)
    float32_t     I_out;            // 출력 전류 (A)
} Master_Status_t;

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

//==================================================
// SCADA 프로토콜 구조체
//==================================================

// --- SCADA → Master 수신 패킷 (16 bytes) ---
#pragma pack(1)
typedef struct
{
    uint8_t     stx;                // [0] STX (0x02)
    uint8_t     cmd;                // [1] Command byte
    int16_t     param1;             // [2-3] Parameter 1 (Big-endian)
    int16_t     param2;             // [4-5] Parameter 2
    int16_t     param3;             // [6-7] Parameter 3
    uint8_t     reserved[3];        // [8-10] Reserved (미래 확장)
    uint32_t    crc32;              // [11-14] CRC-32 (Big-endian)
    uint8_t     etx;                // [15] ETX (0x03)
} SCADA_RxPacket_t;
#pragma pack()

// --- Master → SCADA 송신 패킷: 시스템 전압 (16 bytes) ---
#pragma pack(1)
typedef struct
{
    uint8_t     stx;                // [0] STX
    uint8_t     id_channel;         // [1] Master ID (bit[7:3]) + Rack_Ch (bit[2:0])
    int16_t     voltage;            // [2-3] 전압 (0.1V 단위, Big-endian)
    uint8_t     reserved[10];       // [4-13] Reserved
    uint8_t     checksum;           // [14] Sum Checksum
    uint8_t     etx;                // [15] ETX
} System_Voltage_Packet_t;
#pragma pack()

// --- Master → SCADA 송신 패킷: 슬레이브 배치 (16 bytes) ---
#pragma pack(1)
typedef struct
{
    uint8_t     stx;                // [0] STX

    // Slave 1
    uint8_t     id1_status;         // [1] ID (bit[7:3]) + Status (bit[2:0])
    int16_t     current1;           // [2-3] 전류 (0.01A 단위, Big-endian)
    uint8_t     temp1;              // [4] 온도 (0.5℃ 단위)

    // Slave 2
    uint8_t     id2_status;         // [5] ID + Status
    int16_t     current2;           // [6-7] 전류
    uint8_t     temp2;              // [8] 온도

    // Slave 3
    uint8_t     id3_status;         // [9] ID + Status
    int16_t     current3;           // [10-11] 전류
    uint8_t     temp3;              // [12] 온도

    uint8_t     reserved;           // [13] Reserved
    uint8_t     checksum;           // [14] Sum Checksum
    uint8_t     etx;                // [15] ETX
} Slave_Batch_Packet_t;
#pragma pack()

// --- Active Slave List 구조체 ---
typedef struct
{
    uint8_t     slave_ids[REV5_MAX_ACTIVE_SLAVES];  // 활성 슬레이브 ID 목록 (1~15)
    uint8_t     count;                               // 현재 활성 슬레이브 수 (0~6)
    uint32_t    last_updated_ms;                     // 마지막 업데이트 타임스탬프 (ms)
} Active_Slave_List_t;

// --- 슬레이브 Fault/Warning 비트맵 ---
typedef struct
{
    uint16_t    fault_bitmap;           // Fault 플래그 (bit 0~15 = Slave ID 1~16)
    uint16_t    warning_bitmap;         // Warning 플래그
    uint32_t    last_fault_time_ms;     // 마지막 Fault 발생 시각
    uint32_t    last_warning_time_ms;   // 마지막 Warning 발생 시각
} Slave_Status_Bitmap_t;

// --- 연결 감시 구조체 ---
typedef struct
{
    uint32_t    last_rx_time_ms;        // 마지막 수신 시각 (ms)
    uint32_t    timeout_threshold_ms;   // 타임아웃 임계값 (ms)
    uint8_t     connection_lost;        // 연결 끊김 플래그 (0=정상, 1=타임아웃)
    uint32_t    reconnect_time_ms;      // 재연결 시각 (타임아웃 복구 시)
} Connection_Watchdog_t;

//==================================================
// Master-to-Master 통신 프로토콜 (Rev 1.0)
//==================================================

// --- Master-to-Master 명령 타입 ---
typedef enum {
    MM_CMD_CURRENT      = 0x01,  // 전류 지령 (M1 → M2, 병렬 모드)
    MM_CMD_SLAVE_COUNT  = 0x02,  // 슬레이브 개수 (M2 → M1, 병렬 모드)
    MM_CMD_RESERVED_1   = 0x03,  // 예약 (향후 확장)
    MM_CMD_RESERVED_2   = 0x04   // 예약 (향후 확장)
} MM_CommandType_t;

// --- Master-to-Master 수신 상태 ---
typedef enum {
    MM_RX_WAIT_STX      = 0,     // STX 대기
    MM_RX_RECEIVING     = 1,     // 데이터 수신 중
    MM_RX_COMPLETE      = 2      // 수신 완료 (예약)
} MM_RxState_t;

// --- Master-to-Master 프레임 크기 ---
#define MM_FRAME_SIZE_CURRENT       9    // 전류 지령 프레임: [STX][CMD][Data_L][Data_H][CRC32(4)][ETX]
#define MM_FRAME_SIZE_SLAVE_COUNT   8    // 슬레이브 개수 프레임: [STX][CMD][Count][CRC32(4)][ETX]
#define MM_MAX_FRAME_SIZE           9    // 최대 프레임 크기

// --- Master-to-Master 전류 지령 프레임 구조체 (M1 → M2) ---
// 주의: TI C2000 컴파일러는 기본적으로 packed 구조체 사용 (pragma pack 불필요)
typedef struct {
    uint8_t     stx;            // 0x02 (STX)
    uint8_t     cmd;            // 0x01 (MM_CMD_CURRENT)
    uint16_t    current;        // 전류 지령 (DAC 코드, Little-endian)
    uint32_t    crc32;          // CRC-32 (Byte 1~3, Little-endian)
    uint8_t     etx;            // 0x03 (ETX)
} MM_CurrentFrame_t;

// --- Master-to-Master 슬레이브 개수 프레임 구조체 (M2 → M1) ---
typedef struct {
    uint8_t     stx;            // 0x02 (STX)
    uint8_t     cmd;            // 0x02 (MM_CMD_SLAVE_COUNT)
    uint8_t     count;          // 슬레이브 개수 (0~6)
    uint32_t    crc32;          // CRC-32 (Byte 1~2, Little-endian)
    uint8_t     etx;            // 0x03 (ETX)
} MM_SlaveCountFrame_t;

// --- Master-to-Master 통신 통계 ---
typedef struct {
    uint32_t    tx_count;           // 송신 프레임 수
    uint32_t    rx_count;           // 수신 프레임 수 (정상)
    uint32_t    crc_error_count;    // CRC 에러 수
    uint32_t    frame_error_count;  // 프레임 에러 수 (STX/ETX 불일치)
    uint32_t    spike_reject_count; // 급격한 변화 거부 수
    uint32_t    timeout_count;      // 타임아웃 발생 수
} MM_Statistics_t;

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
extern float32_t V_max_cmd;            // 최대 전압 지령 (충전 모드, Rev 2.1)
extern float32_t V_min_cmd;            // 최소 전압 지령 (방전 모드, Rev 2.1)
extern float32_t V_fb;                 // 전압 피드백

// CLA PI 제어기 (DCL - Digital Control Library)
extern DCL_PI_CLA pi_charge;           // PI 제어기 (충전 모드, V_max_cmd 기준) - CLA Task 1
extern DCL_PI_CLA pi_discharge;        // PI 제어기 (방전 모드, V_min_cmd 기준) - CLA Task 2
extern DCL_PI_CLA pi_cv;               // PI 제어기 (배터리 모드 CV 제어, V_cmd 기준) - CLA Task 3

// CLA → CPU (제어 결과)
extern float32_t I_PI_charge_out;      // PI 출력 (충전 전류, Task 1)
extern float32_t I_PI_discharge_out;   // PI 출력 (방전 전류, Task 2)
extern float32_t I_PI_cv_out;          // PI 출력 (배터리 모드 CV 전류, Task 3)
extern uint16_t  cla_cnt;              // CLA 실행 카운터 (디버그용)

//--------------------------------------------------
// [2] 시스템 상태 및 타이밍
//--------------------------------------------------

//==================================================
// SCADA 인터페이스 전역 변수
//==================================================
extern SCADA_Command_t  scada_cmd;          // SCADA 명령 (SCADA → Master)
extern Master_Status_t  master_status;      // 마스터 상태 (Master → SCADA)

//==================================================
// 시스템 제어 변수
//==================================================
extern SystemState      state;              // 시스템 상태
extern OperationMode_t  operation_mode;     // 운전 모드
extern uint32_t         control_phase;      // 제어 Phase (0~4)
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
extern uint16_t  I_cmd_from_master;         // 상위 마스터로부터 수신한 지령

// 전류 센싱 (FPGA SPI)
extern float32_t I_out_raw;                 // 원시 출력 전류

// 전류 피드백 및 평균
extern float32_t I_out_avg;                 // 출력 전류 평균
extern int16_t   I_cmd_scada;               // SCADA 수신 전류 지령 (충방전 모드)

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

// 고장 래칭 메커니즘
extern volatile bool fault_latched;         // 고장 래치 플래그 (SCADA 리셋으로만 해제)

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
// 상태 확인이 필요한 경우 IS_MAIN_RELAY_ON(), IS_PARALLEL_LINK_ON() 매크로 사용

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

// RS485 통신 모니터링 (TX FIFO 블로킹 방지)
extern uint32_t rs485_mm_skip_cnt;          // Master-to-Master TX 스킵 카운터
extern uint32_t rs485_ms_skip_cnt;          // Master-to-Slave TX 스킵 카운터

//--------------------------------------------------
// [14] SCADA 인터페이스
//--------------------------------------------------

// SCADA 수신 버퍼
extern volatile uint16_t scada_packet_ready;            // 패킷 준비 플래그
extern volatile uint16_t scada_rx_index;                // 수신 버퍼 인덱스
extern volatile uint8_t scada_rx_buffer[SCADA_PACKET_SIZE];  // SCADA 수신 버퍼 (16 bytes)

// SCADA 제어 변수 (레거시 호환성 - scada_cmd 구조체와 동기화됨)
extern volatile ControlMode_t control_mode;             // 제어 모드 (충방전/배터리)
extern volatile uint8_t  ready_state;                   // Ready 상태 (bit[6])
extern volatile uint8_t  run_state;                     // Run 상태 (bit[5])
extern volatile uint8_t  parallel_mode;                 // Parallel 모드 (bit[4])

// 시퀀스 상태 변수 (레거시 호환성 - master_status.sequence_step과 동기화됨)
extern volatile uint16_t sequence_step;                 // 시퀀스 단계 (IDLE/PRECHARGE_DONE/NORMAL_RUN)

// 배터리 모드 제어 지령
extern float32_t V_cmd;                                 // 목표 전압 (배터리 모드 CV 제어)
extern float32_t I_max_cmd;                             // 최대 전류 제한 (배터리 모드)
extern float32_t I_min_cmd;                             // 최소 전류 제한 (배터리 모드)

// CRC 검증
extern uint32_t scada_crc_error_cnt;                    // CRC 에러 카운터

//--------------------------------------------------
// [14-1] Rev 5 프로토콜 전역 변수
//--------------------------------------------------

// --- Active Slave List ---
extern Active_Slave_List_t      active_slave_list;

// --- Fault/Warning 비트맵 ---
extern Slave_Status_Bitmap_t    slave_status_bitmap;

// --- 연결 감시 ---
extern Connection_Watchdog_t    scada_watchdog;
extern Connection_Watchdog_t    mm_watchdog;        // Master-to-Master 연결 감시

// --- Master-to-Master 통신 변수 ---
extern MM_RxState_t             mm_rx_state;        // 수신 상태 머신
extern uint8_t                  mm_rx_buffer[MM_MAX_FRAME_SIZE];  // 수신 버퍼
extern uint8_t                  mm_rx_index;        // 수신 버퍼 인덱스
extern uint16_t                 I_cmd_from_master_prev;  // 이전 전류 지령 (급격한 변화 감지용)
extern uint8_t                  ch2_slave_count;    // CH2 슬레이브 개수 (M1이 수신)
extern MM_Statistics_t          mm_statistics;      // Master-to-Master 통신 통계
extern CRC_Obj                  crcObj_MM;          // Master-to-Master CRC 객체
extern CRC_Handle               handleCRC_MM;       // Master-to-Master CRC 핸들

// --- Keep-Alive 토글 플래그 ---
extern uint8_t                  keepalive_toggle;

// --- SCADA 송신 패킷 버퍼 ---
extern System_Voltage_Packet_t  system_voltage_packet;
extern Slave_Batch_Packet_t     slave_batch_packets[2];  // 6개 슬레이브 → 2패킷

// --- 배치 전송 인덱스 ---
extern uint8_t                  batch_index;              // 현재 배치 인덱스 (0~1)

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
// [16] ADC 변수
//--------------------------------------------------
// ADC 기반 센싱은 사용하지 않음 (FPGA SPI 사용)

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
