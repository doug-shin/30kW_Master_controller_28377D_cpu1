//==================================================
// HABA_main.c - 30kW Master Controller Main Entry Point
//==================================================
//
// TI F28377D Dual-Core DSP Firmware
// 30kW Battery Power Conversion System (Master Controller)
// Real-time PI control @ 100kHz with CLA acceleration
//
//==================================================

//==================================================
// 개발 이력
//==================================================
/*
 =======================================
            작업사항 요약
 =======================================

 [07.03] EPWM3 인터럽트 적용 테스트 완
 [07.04] 드라이버 레벨 헤더 수정 및 재구성
 [07.07] SLAVE용 CAN 송수신 루틴 구현 및 완료
 [07.08] SCI (RS485) 통신 완료 및 통합 테스트 적용
 [07.08] SysConfig 제거 후 수동 GPIO 설정 구조 전환
 [07.09] CLA Task 예제 추가 및 정상 동작 확인 완료
 [07.09] CLA 기반 PI 제어기 완성
 [07.10] DAC80502 SPI 통신 완료 및 DAC 출력 확인
 [07.11] NTC 온도 측정을 위한 ADCA 채널 추가 적용
 [07.11] 전체 코드 분류 (소스/헤더 나눔) 및 정리
 [07.12] Digital Input/Output (DIO) 구성 및 테스트 완료
 [07.15] EPWM3_SIR 함수 구현 완료 (80%)
        * 함수 포인터 배열을 사용한 20kHz 분주 제어
        * - Phase 0: 전압 센싱, DAC 센싱 구현 완료
        * - Phase 1: PI 제어, CLA 구현 완료
        * - Phase 2: 전류 지령 처리, 소프트 스타트 및 485 전류지령 출력 확인완료
        * - Phase 3: 온도 처리 NTC1, NTC2 구현 완료
        * - Phase 4: 평균 계산, 미구현

 [08.06] EPWM1 ↔ EPWM3 용도 변경
        * EPWM1: 메인 제어 루프, EPMW1A CS-100k
        * EPWM3: ADC 트리거
 [09.01] CAN Slave 12bit → 16bit 변경
 [09.08] SCIA 기초 구현완료
 [09.09] Current_command_task 모드에 따른 동작 구현 완료
 [09.10] run 변수 오동작 분석 완료
 [09.11] HMI protocol_rev2_0 구현 완료, average_task 함수 200번 평균 if 조건문 ++ 위치 버그 수정
 [09.12] PI 제어기 피드백 전압 변경, sensing_task 함수 Vo_sen, Vbat_sen 캘리브레이션
 [09.15] HMI protocol_rev2_0 코드 통합 완료
 [09.18] 독립운전, 병렬운전 릴레이 수정
        * - Relay8 (GPIO8): 독립운전 릴레이
        * - Relay7 (GPIO9): 병렬운전 릴레이
        ※ 프리차징 Relay 소스코드 주석처리 추후 코드 복귀작업 필요(20250918)
 [09.22] 전면 상태 LED 구현 완료
 [09.23] UI 데이터 전송 문제 해결
        * - 10us → SCI 데이터 전송 시간 변경 10ms / 50ms
 [09.25] PI 제어기 동작 방식 수정, Module_Sequence 동작 신호 변경(run → Start_Stop)
 [09.30] Sequence_Module 변경
 [10.01] UI 개선, 1~15까지 동작 확인 완료
        * - 16~31 추가 시 while문 검증 필요
*/

//==================================================
// Include Files
//==================================================
#include "device.h"
#include "HABA_setup.h"
#include "HABA_control.h"
#include "HABA_globals.h"

//==================================================
// 함수 전방 선언
//==================================================
// 제어 Phase 함수 선언 (HABA_control.c에서 정의)
void Sensing_And_Trigger_PI(void);
void Apply_PI_And_Convert_DAC(void);
void Transmit_Current_Command(void);
void Check_System_Safety(void);
void Update_Monitoring_And_Sequence(void);
__interrupt void INT_EPWM1_ISR(void);

//==================================================
// 5-Phase 제어 함수 테이블 (20kHz effective)
//==================================================
// Phase별 함수를 배열로 정의하여 가독성 향상
// switch-case 없이 인덱싱으로 간결하게 호출
//==================================================
typedef void (*PhaseFunction)(void);

static const PhaseFunction control_phase_table[5] = {
    Sensing_And_Trigger_PI,          // Phase 0: 센싱 + PI 준비 + CLA Force
    Apply_PI_And_Convert_DAC,        // Phase 1: PI 결과 적용 + DAC 변환
    Transmit_Current_Command,        // Phase 2: RS485 전류 지령 전송
    Check_System_Safety,             // Phase 3: 안전 체크 + 릴레이 제어
    Update_Monitoring_And_Sequence   // Phase 4: 모니터링 + 시퀀스 실행
};

//==================================================
// main() - 시스템 엔트리 포인트
//==================================================
void main(void)
{
    //==================================
    // 시스템 초기화
    //==================================
    Device_init();      // TI Device 초기화
    Init_System();        // 하드웨어 초기화 (GPIO, PWM, CAN, SCI, SPI, ADC, CLA)

    GPIO_writePin(LED_PWR, 1);  // 전원 LED ON

    //==================================
    // 메인 루프
    //==================================
    while(1)
    {
        //========================
        // 1ms 주기 작업
        //========================
        if (flag_1ms)
        {
            DEBUG_1MS_START();
            
            flag_1ms = 0;

            // CAN 슬레이브 제어 (Start/Stop에 따라 Buck Enable 제어)
            if (start_stop == START)
            {
                // run 조건: EMG_SW 정상 + 과전압/과전류/과온 없음 + UI_run = 1
                Send_CANA_Message(0xA0);    // Slave ON, Buck_EN = 1
            }
            else
            {
                Send_CANA_Message(0x00);    // Slave OFF
            }

            DEBUG_1MS_END();
        }

        //========================
        // 10ms 주기 작업
        //========================
        if (flag_10ms)
        {
            DEBUG_10MS_START();
            
            flag_10ms = false;

            Update_System_Status();             // 시스템 상태 업데이트 (슬레이브 모니터링 + 전류 지령)
            Send_Slave_Data_To_SCADA();   // Slave 모듈 데이터 → SCADA 송신

            // CAN 슬레이브 상태 읽기 (최대 16개 채널)
            for (uint16_t mbox = 1; mbox <= 16; mbox++)
            {
                Read_CAN_Slave(mbox);
            }

            DEBUG_10MS_END();
        }

        //========================
        // 50ms 주기 작업
        //========================
        if (flag_50ms)
        {
            flag_50ms = false;

            Send_System_Voltage_To_SCADA();   // 시스템 전압 → SCADA 송신
            Select_master_id();             // Master ID 선택 (상위/하위 구분)
        }
    }
}

//==================================================
// INT_EPWM1_ISR - 메인 제어 인터럽트 (100kHz)
//==================================================
// 실시간 제어 루프:
//   - FPGA ADC 데이터 수신 (V_out, V_batt, I_out)
//   - 5-Phase 제어 태스크 실행 (20kHz 분주)
//   - 1ms/10ms/50ms 플래그 생성
//   - Watchdog 서비스
//==================================================
#pragma CODE_SECTION(INT_EPWM1_ISR, ".TI.ramfunc");
__interrupt void INT_EPWM1_ISR(void)
{
    DEBUG_ISR_START();

    //========================
    // FPGA SPI 데이터 수신
    //========================
    Read_FPGA_Data();       // V_out_raw, V_batt_raw, I_out_raw 읽기

    flag_epwm1_int = 1;

    //========================
    // 전압 센싱 누적 (5회 평균용)
    //========================
    V_out_raw_sum   += V_out_raw;
    V_batt_raw_sum += V_batt_raw;

    //========================
    // PI 제어 피드백 전압 선택
    //========================
    // sequence_step == 20 (정상 운전): V_batt 피드백
    // 그 외 (프리차징 등): V_out 피드백
    if (sequence_step == 20)
        V_fb = V_batt;
    else
        V_fb = V_out;

    //========================
    // 5-Phase 제어 실행 (20kHz, 파이프라이닝 구조)
    //========================
    // Phase 0: 센싱 + CLA Force → Phase 1: CLA 결과 사용 (10us 후)
    // CPU-CLA 병렬 실행으로 성능 최적화
    // 함수 테이블로 간결하게 구현
    //========================
    
    // 안전장치: Phase 범위 체크 (0~4)
    if (control_phase >= 5)
        control_phase = 0;
    
    // Phase 함수 실행 (함수 포인터 배열 호출)
    control_phase_table[control_phase]();
    
    // Phase 인덱스 증가 및 순환 (0→1→2→3→4→0)
    if (++control_phase >= 5)
        control_phase = 0;

    //========================
    // 타이머 플래그 생성
    //========================

    // 1ms 플래그 (100kHz / 100 = 1kHz)
    if (++cnt_1ms >= 100)
    {
        cnt_1ms = 0;
        SysCtl_serviceWatchdog();   // Watchdog 서비스
        flag_1ms = true;
    }

    // 10ms 플래그 (100kHz / 1000 = 100Hz)
    if (++cnt_10ms >= 1000)
    {
        cnt_10ms = 0;
        flag_10ms = true;
    }

    // 50ms 플래그 (100kHz / 5000 = 20Hz)
    if (++cnt_50ms >= 5000)
    {
        cnt_50ms = 0;
        flag_50ms = true;
    }

    //========================
    // 인터럽트 플래그 클리어
    //========================
    EPWM_clearEventTriggerInterruptFlag(EPWM1_BASE);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);

    DEBUG_ISR_END();
}

//==================================================
// End of HABA_main.c
//==================================================
