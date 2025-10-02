#ifndef HABA_CTRL_H
#define HABA_CTRL_H

#include "HABA_shared.h"

#ifdef __cplusplus
extern "C" {
#endif




//==================================================
// CLA 제어 및 초기화 관련
//==================================================

__interrupt void cla1Isr1(void);
__interrupt void cla1Isr2(void);
//==================================================
// 인터럽트 핸들러
//==================================================
__interrupt void INT_ADCA1_ISR(void);
__interrupt void spicRxISR(void);
__interrupt void sciaRxISR(void);
//==================================================

// 제어 알고리즘 (PI 등)
//==================================================
void PI_Controller(void);

//==================================================
// 통신 관련
//==================================================
void Communication_Status(void);
void send_CANA_Message(int8_t CAN_CMD);
void read_CANA_Messages(void);
void debug_CAN_status(void);
void debug_SCI_status(void);
void debug_SPI_status(void);
void debug_CLK_status(void);

bool CAN_SlaveRead(uint16_t mbox);
// void CAN_SlaveRead(uint16_t ch);
void CAN_UpdateAllSlaves(void);
//==================================================
// SPI / DAC 관련
//==================================================
void SPIDAC1(uint8_t cmd_byte, uint16_t dac_data);

//==================================================
// RS485 전류 명령 송신
//==================================================
void send_485A_CurrentCommand(uint16_t current);
void send_485B_CurrentCommand(uint16_t current);
uint16_t Digtal_Input(uint16_t gpioPin);
void Relay_control(void);
void Master_ID_Select(void);
uint16_t Emergency_Stop_Switch(void);

void Fault_Check(void);
float calcNTCTemp(float R_ntc);
void read_FPGA_data(void);
void Sequence_Module(void);
void UI_monitoring(void);
void detect_active_slaves(void);
// HMI 함수 선언들 =================================================
void modbus_parse(void);
void send_slave_data_to_hmi(void);
void send_system_voltage_to_hmi(void);
__interrupt void scidRxReadyISR(void);
// HMI 함수 선언들 끝 ==============================================

#ifdef __cplusplus
}
#endif

#endif // HABA_CTRL_H
