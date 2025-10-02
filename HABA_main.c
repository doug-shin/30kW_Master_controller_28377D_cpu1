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
 [09.10] Run 변수 오동작 분석 완료
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
 [09.25] PI 제어기 동작 방식 수정, Module_Sequence 동작 신호 변경(Run → Start_Stop)
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
void voltage_sensing_task(void);
void pi_control_task(void);
void current_command_task(void);
void temperature_task(void);
void average_task(void);
__interrupt void INT_Init_EPWM1_ISR(void);

//==================================================
// main() - 시스템 엔트리 포인트
//==================================================
void main(void)
{
    //==================================
    // 시스템 초기화
    //==================================
    Device_init();      // TI Device 초기화
    HABA_init();        // 하드웨어 초기화 (GPIO, PWM, CAN, SCI, SPI, ADC, CLA)

    GPIO_writePin(LED_PWR, 1);  // 전원 LED ON

    //==================================
    // 메인 루프
    //==================================
    while(1)
    {
        //========================
        // 1ms 주기 작업
        //========================
        if (_1ms_flag)
        {
            GPIO_writePin(91, 1);   // 디버그 GPIO 토글 시작

            _1ms_flag = 0;

            // CAN 슬레이브 제어 (Start/Stop에 따라 Buck Enable 제어)
            if (start_stop == START)
            {
                // Run 조건: EMG_SW 정상 + 과전압/과전류/과온 없음 + UI_Run = 1
                send_CANA_Message(0xA0);    // Slave ON, Buck_EN = 1
            }
            else
            {
                send_CANA_Message(0x00);    // Slave OFF
            }

            GPIO_writePin(91, 0);   // 디버그 GPIO 토글 종료
        }

        //========================
        // 10ms 주기 작업
        //========================
        if (flag_10ms)
        {
            GPIO_writePin(92, 1);   // 디버그 GPIO 토글 시작

            flag_10ms = false;

            UI_monitoring();            // HMI 수신 데이터 처리
            send_slave_data_to_hmi();   // Slave 모듈 데이터 → HMI 송신

            // CAN 슬레이브 상태 읽기 (최대 16개 채널)
            for (uint16_t mbox = 1; mbox <= 16; mbox++)
            {
                CAN_SlaveRead(mbox);
            }

            GPIO_writePin(92, 0);   // 디버그 GPIO 토글 종료
        }

        //========================
        // 50ms 주기 작업
        //========================
        if (flag_50ms)
        {
            flag_50ms = false;

            send_system_voltage_to_hmi();   // 시스템 전압 → HMI 송신
            Master_ID_Select();             // Master ID 선택 (상위/하위 구분)
        }
    }
}

//==================================================
// INT_Init_EPWM1_ISR - 메인 제어 인터럽트 (100kHz)
//==================================================
// 실시간 제어 루프:
//   - FPGA ADC 데이터 수신 (Vo, Vbat, Io)
//   - 5-Phase 제어 태스크 실행 (20kHz 분주)
//   - 1ms/10ms/50ms 플래그 생성
//   - Watchdog 서비스
//==================================================
#pragma CODE_SECTION(INT_Init_EPWM1_ISR, ".TI.ramfunc");
__interrupt void INT_Init_EPWM1_ISR(void)
{
    GPIO_writePin(90, 1);   // ISR 실행 시작 (디버그 GPIO)

    //========================
    // FPGA SPI 데이터 수신
    //========================
    read_FPGA_data();       // Vo_ad, Vbat_ad, Io_ad 읽기

    INT_EPMW1_Flag = 1;

    //========================
    // 전압 센싱 누적 (5회 평균용)
    //========================
    Vo_ad_sum   += Vo_ad;
    Vbat_ad_sum += Vbat_ad;

    //========================
    // PI 제어 피드백 전압 선택
    //========================
    // sequence_step == 20 (정상 운전): Vbat 피드백
    // 그 외 (프리차징 등): Vo 피드백
    if (sequence_step == 20)
        Vfb = Vbat;
    else
        Vfb = Vo;

    //========================
    // 5-Phase 제어 태스크 실행 (20kHz)
    //========================
    // Phase 0: sensing_task         - 전압/전류 센싱 및 캘리브레이션
    // Phase 1: pi_control_task      - 소프트 스타트 + PI 제어
    // Phase 2: current_command_task - 전류 지령 전송
    // Phase 3: temperature_task     - 온도 측정 및 고장 체크
    // Phase 4: average_task         - 장기 평균 및 시퀀스
    control_task_functions[control_phase]();
    if (++control_phase >= 5)
        control_phase = 0;

    //========================
    // 타이머 플래그 생성
    //========================

    // 1ms 플래그 (100kHz / 100 = 1kHz)
    if (++_1ms_count >= 100)
    {
        _1ms_count = 0;
        SysCtl_serviceWatchdog();   // Watchdog 서비스
        _1ms_flag = true;
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

    GPIO_writePin(90, 0);   // ISR 실행 종료 (디버그 GPIO)
}

//==================================================
// End of HABA_main.c
//==================================================
