# RS232/RS485 기반 Master-Slave 제어 구조 정의

## 1. 시스템 구성

- **PC(SCADA)**
  - RS232 포트 2개 사용  
    - RS232-1 → 상위 마스터 SCID 연결  
    - RS232-2 → 하위 마스터 SCID 연결  
  - 운전 모드 및 전류 지령을 각각의 마스터에 전달  

- **Master 보드 (2개)**
  - 동일 펌웨어 사용  
  - DIP 스위치로 `master_id` 결정  
    - master_id = 0 → 상위 마스터  
    - master_id = 1 → 하위 마스터  
  - 통신 포트:
    - **SCID (RS232)** : SCADA ↔ 상/하위 마스터 (모드 및 지령 전달)  
    - **SCIA (RS485)** : 상위 ↔ 하위 마스터 간 전류 지령 공유  
    - **SCIB (RS485)** : 각 마스터 → 자기 슬레이브 보드 전류 지령 전송  

- **Slave 보드**
  - 각 마스터에 종속  
  - RS485(SCIB) 통해 전류 지령 수신  

---

## 2. 운전 모드 정의

### 병렬 운전 (Parallel)

- **상위 마스터**
  - SCADA에서 총 전류 지령 수신
  - PI 제어 수행
  - 자기 슬레이브(SCIB)에 지령 전송
  - 하위 마스터(SCIA, RS485)에 동일 지령 전송

- **하위 마스터**
  - 상위 마스터 지령 수신
  - 자기 슬레이브(SCIB)에 그대로 전송 (PI 제어 없음)

---

### 독립 운전 (Independent)

- **상위 마스터**
  - SCADA에서 CH1 지령 수신
  - PI 제어 수행
  - 자기 슬레이브(SCIB)에 전송

- **하위 마스터**
  - SCADA에서 CH2 지령 수신
  - PI 제어 수행
  - 자기 슬레이브(SCIB)에 전송

※ 독립 모드에서는 A, B 중 하나만 동작하거나 둘 다 각각 동작할 수 있음  
※ 사용하지 않는 채널은 SCADA에서 전류 지령을 `0`으로 전송  

---

## 3. Task 구조

- **Task1 (20kHz) — Sensing**
  - 전압/전류 ADC 평균 계산
  - PI 제어 입력 데이터 갱신

- **Task2 (20kHz) — PI 제어**
  - 병렬: 상위만 PI 제어, 하위는 상위 지령 반영
  - 독립: 상위(CH1)/하위(CH2) 각각 PI 수행 (지령≠0일 때만)

- **Task3 (20kHz) — Current Command 송신**
  - 상위: 병렬 시 자기+하위 모두 전송, 독립 시 자기만 전송
  - 하위: 병렬 시 상위 지령을 그대로 슬레이브에 전송, 독립 시 자기 지령으로 PI 수행 후 전송

- **Task4 (20kHz) — Temperature & Fault**
  - Fault 체크, Relay 제어, Emergency Stop 처리

- **Task5 (20kHz) — Average & Sequence**
  - 장기 평균 (Vo, Vbat, Io)
  - 시퀀스 단계 진행 (Pre-charge 등)

---

## 4. 주요 변수 정의

- **master_id**
  - DIP 스위치에서 읽음
  - 0 = 상위, 1 = 하위  

- **operation_mode**
  - SCADA에서 수신
  - MODE_PARALLEL: 병렬 운전
  - MODE_INDEPENDENT: 독립 운전

- **I_cmd_total**
  - 병렬 운전 시 총 전류 지령

- **I_cmd_ch1, I_cmd_ch2**
  - 독립 운전 시 CH1, CH2 전류 지령

- **I_cmd_DAC**
  - PI 제어 출력 (DAC 스케일 값)

- **I_cmd_from_master**
  - 하위 마스터가 상위에서 SCIA(RS485)로 수신한 지령

---

## 5. 동작 요약

| 모드   |                       상위 마스터                        |                  하위 마스터                 |
| ------ |---------------------------------------------------------|---------------------------------------------|
| 병렬   | SCADA 지령 → PI 제어 → 자기 슬레이브 + 하위 마스터에 전송  | 상위 지령 수신 → 자기 슬레이브에 반영         |
| 독립   | SCADA CH1 지령 → PI 제어 → 자기 슬레이브 제어             | SCADA CH2 지령 → PI 제어 → 자기 슬레이브 제어 |

<br><br><br><br>
📌 구현 단계별 진행 순서


Step 1. 기본 환경 정리

 master_id_Select() 함수 → 부팅 시 DIP 스위치 값 읽어서 master_id 설정

 operation_mode 초기값을 MODE_STOP으로 세팅

 전역변수 (I_cmd_total, I_cmd_ch1, I_cmd_ch2, I_cmd_from_master) 선언 및 초기화


<br>Step 2. SCADA ↔ Master (RS232, SCID) 통신

 프레임 파서 구현 (parse_SCADA_Command())

수신 10바이트 프로토콜 파싱

operation_mode, I_cmd_total, I_cmd_ch1, I_cmd_ch2 값 갱신

 scidRxISR() → ISR에서 프레임 수신 후 파서 호출


<br>Step 3. Master ↔ Master (RS485, SCIA) 통신

 상위 마스터 (master_id==0)

병렬 모드일 때 I_cmd_DAC 전송 (하위에게)

 하위 마스터 (master_id==1)

sciaRxISR()에서 상위 지령 수신 후 I_cmd_from_master에 저장


<br>Step 4. Master ↔ Slave (RS485, SCIB) 통신

 send_485B_CurrentCommand() 구현 완료 확인

 **Task3 (current_command_task)**에서 조건 분기:

상위 Master

병렬 → 자기 + 하위

독립 → CH1만

하위 Master

병렬 → 상위 지령(I_cmd_from_master) 그대로

독립 → CH2 지령으로 PI 돌린 결과 전송

<br>
Step 5. 제어 루프 (20kHz Task2 – PI Control)

 병렬:

상위 Master만 PI 제어 → I_cmd_DAC 산출

하위 Master는 PI 수행 안 함 (I_cmd_from_master만 사용)

 독립:

상위 Master: I_cmd_ch1 ≠ 0 → PI 제어

하위 Master: I_cmd_ch2 ≠ 0 → PI 제어


<br>
Step 6. Sequence & Fault (Task5, Task4)

 Sequence_Module() → Pre-charge, Relay ON 단계 유지

 Fault_Check() → OverVoltage, OverCurrent, OverTemp 동작 검증

 <br>

Step 7. 통합 동작 검증

 DIP 스위치 조합으로 master_id 테스트 (0=상위, 1=하위)

 SCADA에서 Independent / Parallel 모드 전송

 SCIB 슬레이브들이 정상 지령 받는지 확인

 <br>

🚀 최종 실행 흐름

부팅 시: DIP 스위치 → master_id 결정

SCADA 지령 수신: operation_mode, I_cmd_xx 갱신

Task2 (PI Control): 모드/ID 조건 따라 PI 제어 수행

Task3 (485 송신): 결과를 슬레이브/하위 마스터에 배포

Task4/5: Fault + Sequence 보조