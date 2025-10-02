/*ㅇㄴㅁ

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
 [07.15] EPWM3_SIR 함수 구현 완료( 80% )

            * 함수 포인터 배열을 사용한 20kHz 분주 제어
            * - Phase 0: 전압 센싱, DAC 센싱 구현 완료
            * - Phase 1: PI 제어, CLA 구현 완료
            * - Phase 2: 전류 지령 처리, 소프트 스타트 및 485 전류지령 출력 확인완료
            * - Phase 3: 온도 처리 NTC1 , NTC2 구현 완료
            * - Phase 4: 평균 계산, 미구현


 [08.06] EPWM1 ↔ EPWM3 용도 변경
            * EPWM1 : 메인 제어 루프, EPMW1A CS-100k
            * EPWM3 : ADC 트리거
 [09.01] CAN Slave 12bit → 16bit 변경
 [09.08] SCIA 기초 구현완료
 [09.09] Current_command_task 모드에 따른 동작 구현 완료
 [09.10] Run 변수 오동작 분석 완료
 [09.11] HMI protocol_rev2_0 구현 완료, average_task함수 _ 200번 평균 _ if 조건문 ++ 위치 버그 수정
 [09.12] PI 제어기 피드백 전압 변경, sensing_task 함수 _ Vo_sen, Vbat_sen 캘리브레이션
 [09.15] HMI protocol_rev2_0 코드 통합 완료
 [09.18] 독립운전, 병렬운전 릴레이 수정
            * - Relay8 (GPIO8) : 독립운전 릴레이
            * - Relay7 (GPIO9) : 병렬운전 릴레이
            ※ 프리차징 Relay 소스코드 주석처리 추후 코드 복귀작업 필요(20250918)
 [09.22] 전면 상태 LED 구현 완료
 [09.23] UI 데이터 전송 문제 해결
            * - 10us → SCI 데이터 전송 시간 변경 10ms / 50ms
 [09.25] PI 제어기 동작 방식 수정, Module_Sequence 동작 신호 변경(Run → Start_Stop)
 [09.30] Sequence_Module 변경
 [10.01] UI 개선, 1~15까지 동작 확인 완료
            * - 16~31 추가 시 while문 검증 필요
            *
*/


//
// main.c - System Entry Point
//

#include "device.h"
#include "HABA_Init.h"
#include "HABA_Ctrl.h"
#include "HABA_shared.h"

                                     //

//==================================================
// 제어 태스크 함수 선언 (정의는 따로 구현)
//==================================================
void voltage_sensing_task(void);
void pi_control_task(void);
void current_command_task(void);
void temperature_task(void);
void average_task(void);
__interrupt void INT_Init_EPWM1_ISR(void);


void main(void)
{
    //==================================
    // 시스템 초기화
    //==================================

    Device_init();
    HABA_init();
    GPIO_writePin(LED_PWR, 1); // F_LED6 전면 LED PWR ON// 1초씩 5번 반복

    //==================================
    // 메인 루프
    //==================================
    while(1)
    {
      if (_1ms_flag) //각 시간마다 시간 체크할것. (50ms 파형 확인). soft_INT(루프 우선순위 1ms, 10ms, 50ms) 검토 필요
      {
        GPIO_writePin(91, 1);

        _1ms_flag = 0;

        if (start_stop == START) // 프리차징, 출력 릴레이 on, 충방전 단계 중에서 Run은 프리차징부터 1로 됨, Run = 1이면 Buck_EN = 1이 됨
        {             // Run = 1은 run_switch(Emg_SW) = 정상, (over_voltage_flag || over_current_flag || over_temp_flag) = 정상이고 UI에서 UI_Run = 1일 때임
          send_CANA_Message(0xA0); // Slave on, Buck_EN = 1로 줌
        }
        else
        {
          send_CANA_Message(0x00); // Slave off
        }

        GPIO_writePin(91, 0);
      }

      if (flag_10ms)
      {
        GPIO_writePin(92, 1);

        flag_10ms = false;
        UI_monitoring();              // UI에서 송신된 데이터를 가져옴
        send_slave_data_to_hmi();     // Slave Module Data를 HMI로 송신 //10ms


        for (uint16_t mbox = 1; mbox <= 16; mbox++) //10ms UI 데이터 전송 (Max Ch 32))
        {
            CAN_SlaveRead(mbox);
        }

        GPIO_writePin(92, 0);
      }

      if (flag_50ms)
      {


        flag_50ms = false;
        send_system_voltage_to_hmi(); // System Voltage를 HMI로 송신 //50ms
        Master_ID_Select();


      }

    }
}




//
//=============================
// EPWM 인터럽트
//=============================
#pragma CODE_SECTION(INT_Init_EPWM1_ISR, ".TI.ramfunc");
__interrupt void INT_Init_EPWM1_ISR(void)  // 100khz
{
    GPIO_writePin(90, 1);
    read_FPGA_data();

    INT_EPMW1_Flag = 1;


    Vo_ad_sum   += Vo_ad;
    Vbat_ad_sum += Vbat_ad;

    if (sequence_step == 20) Vfb = Vbat;
    else Vfb = Vo;

    // // control_task_functions[4]();
    // // 제어 태스크 실행
    control_task_functions[control_phase]();
    if (++control_phase >= 5) control_phase = 0;


    // 타이머 플래그 (1ms, 100us 등)
    if (++_1ms_count >= 100) {   // 1ms
        _1ms_count = 0;
        SysCtl_serviceWatchdog();
        _1ms_flag = true;
    }

    // === 10ms (10000us / 10us = 1000회) ===
    if (++cnt_10ms >= 1000) {
        cnt_10ms = 0;
        flag_10ms = true;
    }

    // === 50ms (50000us / 10us = 5000회) ===
    if (++cnt_50ms >= 5000) {
        cnt_50ms = 0;
        flag_50ms = true;
    }

    EPWM_clearEventTriggerInterruptFlag(EPWM1_BASE);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);

    GPIO_writePin(90, 0);
}


