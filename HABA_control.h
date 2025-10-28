//==================================================
// HABA_control.h - Control Logic and Communication Functions Header
//==================================================
// TI F28377D 30kW Master Controller
// 제어 알고리즘, 통신 핸들러, ISR 함수 선언
//==================================================

#ifndef HABA_CONTROL_H
#define HABA_CONTROL_H

#include "HABA_globals.h"

#ifdef __cplusplus
extern "C" {
#endif

//==================================================
// [1] 5-Phase 제어 함수 (20kHz @ RAMFUNC)
//==================================================
// 100kHz ISR 내에서 순환 호출되는 제어 Phase 함수 (파이프라이닝 구조)

void Sensing_And_Trigger_PI(void);              // Phase 0: 전압 센싱 + PI 준비 + CLA Force
void Apply_PI_And_Convert_DAC(void);            // Phase 1: PI 결과 적용 + DAC 변환 (10us 후)
void Transmit_Current_Command(void);            // Phase 2: RS485 전류 지령 전송
void Check_System_Safety(void);                 // Phase 3: 고장 체크 + 릴레이 제어
void Update_Monitoring_And_Sequence(void);      // Phase 4: 모니터링 + 시퀀스 실행

//==================================================
// [2] 제어 알고리즘
//==================================================

// Execute_PI_Controller() - Execute_Current_Control()에 인라인 통합됨
void Check_Fault(void);                         // 고장 체크 및 LED 제어
void Apply_Current_Reference_Limit(void);       // 시스템 상태 업데이트 (슬레이브 모니터링 + 전류 지령)
void Update_System_Sequence(void);              // 시퀀스 제어 모듈 (Precharge → Run)

//==================================================
// [3] CAN 통신 (슬레이브 제어)
//==================================================

void Send_CANA_Message(int8_t CAN_CMD);         // CAN 메시지 송신 (Master → Slave)
void Read_CANA_Messages(void);                  // CAN 메시지 일괄 수신 (사용 안 함)
bool Read_CAN_Slave(uint16_t mbox);             // 개별 슬레이브 데이터 읽기
void Init_Slave_Variables(void);                // 슬레이브 초기화 (초기 탐색)

//==================================================
// [4] SPI 통신 (DAC 및 FPGA)
//==================================================

void Write_SPI_DAC1(uint8_t cmd_byte, uint16_t dac_data);  // DAC80502 제어 (SPIA)
void Read_FPGA_Data(void);                      // FPGA ADC 데이터 읽기 (SPIC)

//==================================================
// [5] RS485 통신 (전류 지령 전송)
//==================================================

void Send_RS485_MM_Current(uint16_t current);       // SCIA RS485: Master-to-Master 전류 지령 (M1 → M2)
void Send_RS485_MM_SlaveCount(uint8_t count);       // SCIA RS485: Master-to-Master 슬레이브 개수 (M2 → M1)
void Send_RS485_MS_Current(uint16_t current);       // SCIB RS485: Master-to-Slave 전류 지령

//==================================================
// [6] GPIO 제어 및 유틸리티
//==================================================

// Control_Relay(), Read_Emergency_Stop_Switch() - Check_System_Safety()에 인라인 통합됨
void Read_Master_ID_From_DIP(void);             // Master ID 읽기 (GPIO36~39 DIP)

//==================================================
// [7] SCADA 통신 (SCID 프로토콜)
//==================================================

// Active Slave List 관리
void Update_Active_Slave_List(uint8_t slave_id);    // CAN 수신 시 슬레이브 ID 추가

// 패킷 송수신
void Parse_SCADA_Command(void);                     // SCADA 패킷 파싱 (CRC-32)
void Send_Slave_Batch_To_SCADA(void);               // 슬레이브 배치 패킷 송신 (3개씩)
void Send_System_Voltage_To_SCADA(void);            // 시스템 전압 송신 (16 bytes)
void Send_KeepAlive_Packet(void);                   // Keep-Alive 100ms 주기 전송

// 연결 감시
void Update_SCADA_Watchdog(void);                   // SCADA 200ms 타임아웃 감지
void Update_MM_Watchdog(void);                      // Master-to-Master 5ms 타임아웃 감지 (병렬 모드 전용)

//==================================================
// [8] 인터럽트 서비스 루틴 (ISR)
//==================================================

__interrupt void INT_ADCA1_ISR(void);           // ADCA1 인터럽트 (비활성화)
__interrupt void CLA1_ISR1(void);               // CLA Task1 완료 ISR
__interrupt void CLA1_ISR2(void);               // CLA Task2 완료 ISR
__interrupt void SCIA_RS485_MM_Rx_ISR(void);    // SCIA RS485 Master-to-Master 수신 ISR (상위 마스터 지령)
__interrupt void SPIC_FPGA_Rx_ISR(void);        // SPIC FPGA 수신 ISR (FPGA ADC 데이터)
__interrupt void SCID_SCADA_Rx_ISR(void);       // SCID SCADA 수신 ISR (SCADA 패킷)

//==================================================
// [9] 디버그 함수 (조건부 컴파일)
//==================================================

// Debug_Communication_Status() - 사용되지 않아 삭제됨 (직접 개별 디버그 함수 호출)
void Debug_CLK_Status(void);                    // 클럭 상태 디버그
void Debug_CAN_Status(void);                    // CAN 상태 디버그
void Debug_SCI_Status(void);                    // SCI 상태 디버그
void Debug_SPI_Status(void);                    // SPI 상태 디버그

// Fault/Warning 비트맵
void Clear_Slave_Status_Bitmap(void);                   // Fault/Warning 비트맵 클리어

#ifdef __cplusplus
}
#endif

#endif // HABA_CONTROL_H

//==================================================
// End of HABA_control.h
//==================================================
