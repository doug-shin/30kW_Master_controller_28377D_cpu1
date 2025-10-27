# RS-232 Interface Protocol Rev4

## 📌 Rev 4.0 변경사항 (2025-10-15)

**Master → SCADA (7 bytes):**
- **Control Byte 재구성**: Master/Slave 데이터 명확히 구분
  - bit[0]: Data Type (0=Master, 1=Slave)
  - Master 데이터: bit[7:2]=Reserved (채널 확장용), bit[1]=Channel
  - Slave 데이터: bit[7:3]=Slave ID (DIP 스위치), bit[2]=DAB_OK
- **데이터 타입 통일**: **int16, ÷10 스케일** (DSP 연산 최적화)
  - 전압: -3276.8V ~ +3276.7V, 해상도: 0.1V
  - 전류: -3276.8A ~ +3276.7A, 해상도: 0.1A
  - DSP 하드웨어 곱셈기 활용 (~3-4 cycles)
  - 일관된 스케일링으로 코드 단순화
- **Sum Checksum 유지**

**SCADA → Master (13 bytes):**
- **CRC-32 도입**: Polynomial 0x04C11DB7 (TI VCU/VCRC 가속)
- **Command 비트 구조**: bit[7]=제어모드, bit[6]=준비상태(IDLE/READY), bit[5]=운전상태(STOP/RUN), bit[4]=병렬
- **모드별 프레임**: Charge/Discharge vs Battery (고정 13 bytes)
- **데이터 타입 통일**: **int16, ÷10 스케일** (양방향 일관성)
  - 전압: int16, ÷10 (Master→SCADA와 동일)
  - 전류: int16, ÷10 (Master→SCADA와 동일)
  - 양방향 동일 타입/스케일로 펌웨어 단순화

---

## 📋 개요
SCADA 시스템과 Master 컨트롤러 간 양방향 바이너리 통신 프로토콜

---

## 📤 1. Master → SCADA (7 bytes) - **Rev4 Byte 1 재구성**

### ⚡ 1.1 시스템 전압 (Master 데이터, bit[0]=0)

| Byte | Field | Description | Type | Range |
|------|-------|-------------|------|-------|
| 0 | STX | Start of Text | 0x02 | - |
| 1 | Control Byte | **bit[7:2]**: Reserved (향후 채널 확장용)<br>**bit[1]**: Channel (0=Ch1, 1=Ch2)<br>**bit[0]**: Data Type (0=Master) | uint8 | 0x00 (Ch1)<br>0x02 (Ch2) |
| 2-3 | System Voltage | Big-endian<br>**Scale: ÷10**<br>계산식: **Value ÷ 10** | **int16** | **-3276.8 ~ +3276.7 V** |
| 4 | Reserved | Future use | 0x00 | - |
| 5 | Checksum | Sum(Byte1~4) & 0xFF | uint8 | 0~255 |
| 6 | ETX | End of Text | 0x03 | - |

**전압 계산:**
- `Voltage (V) = Raw_Value ÷ 10`
- `Raw_Value = Voltage × 10`
- `0.0 V → 0 (0x0000)`
- `-30.0 V → -300 (0xFED4)`
- `+300.0 V → 3000 (0x0BB8)`
- `+1100.0 V → 11000 (0x2AF8)`

**DSP 최적화 코드:**
```c
// TI C2000 - 하드웨어 곱셈기 활용 (단일 사이클)
int16_t voltage_raw = adc_voltage_x10;  // 이미 0.1V 단위
frame[2] = (voltage_raw >> 8) & 0xFF;   // MSB (Big-endian)
frame[3] = voltage_raw & 0xFF;           // LSB
// 총 연산: ~3-4 cycles
```

**예제:**
```
전압 0.0V → Raw: 0 (0x0000)
Frame: 02 00 00 00 00 00 03

전압 -30.0V → Raw: -300 (0xFED4)
Frame: 02 00 FE D4 00 D2 03

전압 300.0V → Raw: 3000 (0x0BB8)
Frame: 02 00 0B B8 00 C3 03

전압 1100.0V → Raw: 11000 (0x2AF8)
Frame: 02 00 2A F8 00 22 03
```

---

### 🔌 1.2 슬레이브 데이터 (Slave 데이터, bit[0]=1) - **Rev4 변경**

| Byte | Field | Description | Type | Range |
|------|-------|-------------|------|-------|
| 0 | STX | Start of Text | 0x02 | - |
| 1 | Control Byte | **bit[7:3]**: Slave ID (DIP 스위치 설정)<br>**bit[2]**: DAB_OK<br>**bit[1]**: Reserved<br>**bit[0]**: Data Type (1=Slave) | uint8 | Slave ID: 0~31<br>DAB_OK: 0=Fail, 1=OK |
| 2-3 | Slave Current | Big-endian<br>**Scale: ÷10** | **int16** | **-3276.8 ~ +3276.7 A** |
| 4 | Slave Temperature | Scale: ×0.5 | uint8 | 0 ~ 127.5 °C |
| 5 | Checksum | Sum(Byte1~4) & 0xFF | uint8 | 0~255 |
| 6 | ETX | End of Text | 0x03 | - |

**전류 계산:**
- `Current (A) = Raw_Value ÷ 10`
- `Raw_Value = Current × 10`
- `0 A → 0 (0x0000)`
- `+100 A → 1000 (0x03E8)`
- `-100 A → -1000 (0xFC18)`
- `+327.6 A → 3276 (0x0CCC)`

**온도 계산:**
- `Temperature (°C) = Raw_Value × 0.5`
- `25.0°C → 50 (0x32)`
- `50.5°C → 101 (0x65)`

**Control Byte 구성 (Rev4):**
```
Slave ID=5, DAB_OK=1
bit[7:3] = 5 (0b00101)
bit[2] = 1 (DAB_OK)
bit[1] = 0 (Reserved)
bit[0] = 1 (Slave)
Control Byte = 0b00101101 = 0x2D
```

**DSP 최적화 코드:**
```c
// TI C2000 - 하드웨어 곱셈기 활용 (단일 사이클)
int16_t current_raw = adc_current_x10;  // 이미 0.1A 단위
frame[2] = (current_raw >> 8) & 0xFF;   // MSB (Big-endian)
frame[3] = current_raw & 0xFF;           // LSB
// 총 연산: ~3-4 cycles (전압과 동일 패턴)
```

**예제:**
```
ID=5, Current=50.2A, Temp=30.5°C, DAB_OK=1
Control Byte = 0x2D (bit[7:3]=5, bit[2]=1, bit[0]=1)
Current = 50.2A → 502 (0x01F6)
Temp = 30.5°C → 61 (0x3D)

Frame: 02 2D 01 F6 3D [CS] 03
Checksum = (0x2D + 0x01 + 0xF6 + 0x3D) & 0xFF = 0x5B
완성: 02 2D 01 F6 3D 5B 03
```

---

## 📥 2. SCADA → Master (13 bytes) - **Rev4 모드별 프레임**

프레임 구조는 Command bit[7]에 따라 결정됩니다 (고정 13 bytes).

### 🔋 2.1 Charge/Discharge Mode (bit[7]=0)

| Byte | Field | Description | Type | Range |
|------|-------|-------------|------|-------|
| 0 | STX | Start of Text | 0x02 | - |
| 1 | Command | **bit[7]=0**: Charge/Discharge Mode<br>**bit[6]**: 준비 상태 (0=IDLE, 1=READY)<br>**bit[5]**: 운전 상태 (0=STOP, 1=RUN)<br>**bit[4]**: 병렬 상태<br>bit[3:0]: 예약 | uint8 | - |
| 2-3 | **V_max_cmd** | 최대 전압 지령<br>Big-endian<br>**Scale: ÷10** | **int16** | **-3276.8 ~ +3276.7 V** |
| 4-5 | **V_min_cmd** | 최소 전압 지령<br>Big-endian<br>**Scale: ÷10** | **int16** | **-3276.8 ~ +3276.7 V** |
| 6-7 | **I_cmd** | 전류 지령<br>Big-endian<br>**Scale: ÷10** | **int16** | **-3276.8 ~ +3276.7 A** |
| 8-11 | **CRC-32** | CRC-32 checksum<br>Polynomial: 0x04C11DB7<br>Big-endian<br>CRC(Byte1~7) | uint32 | 0~0xFFFFFFFF |
| 12 | ETX | End of Text | 0x03 | - |

### 🔌 2.2 Battery Mode (bit[7]=1)

| Byte | Field | Description | Type | Range |
|------|-------|-------------|------|-------|
| 0 | STX | Start of Text | 0x02 | - |
| 1 | Command | **bit[7]=1**: Battery Mode<br>**bit[6]**: 준비 상태 (0=IDLE, 1=READY)<br>**bit[5]**: 운전 상태 (0=STOP, 1=RUN)<br>**bit[4]**: 병렬 상태<br>bit[3:0]: 예약 | uint8 | - |
| 2-3 | **V_cmd** | 전압 지령<br>Big-endian<br>**Scale: ÷10** | **int16** | **-3276.8 ~ +3276.7 V** |
| 4-5 | **I_max_cmd** | 최대 전류 지령<br>Big-endian<br>**Scale: ÷10** | **int16** | **-3276.8 ~ +3276.7 A** |
| 6-7 | **I_min_cmd** | 최소 전류 지령<br>Big-endian<br>**Scale: ÷10** | **int16** | **-3276.8 ~ +3276.7 A** |
| 8-11 | **CRC-32** | CRC-32 checksum<br>Polynomial: 0x04C11DB7<br>Big-endian<br>CRC(Byte1~7) | uint32 | 0~0xFFFFFFFF |
| 12 | ETX | End of Text | 0x03 | - |

**Command 필드 상세 (Rev4):**

| 비트 | 기능 | 값 | 의미 |
|------|------|-----|------|
| **bit[7]** | 제어 모드 | 0 | Charge/Discharge Mode |
|  |  | 1 | Battery Mode |
| **bit[6]** | 준비 상태 | 0 | **IDLE** (유휴) |
|  |  | 1 | **READY** (준비 완료) |
| **bit[5]** | 운전 상태 | 0 | **STOP** (정지) |
|  |  | 1 | **RUN** (운전) |
| **bit[4]** | 병렬 상태 | 0 | Individual (개별 운전) |
|  |  | 1 | Parallel (병렬 운전) |
| **bit[3:0]** | 예약 | - | 향후 확장용 |

**Command 조합 예시:**

| Command (hex) | bit[7:4] (binary) | 제어모드 | 준비 | 운전 | 병렬 | 설명 |
|---------------|-------------------|----------|------|------|------|------|
| `0x00` | `0000` | Charge/Discharge | IDLE | STOP | Individual | 유휴 정지 |
| `0x40` | `0100` | Charge/Discharge | READY | STOP | Individual | 준비 완료, 정지 |
| `0x20` | `0010` | Charge/Discharge | IDLE | RUN | Individual | ⚠️ 비정상 (준비 안됨) |
| `0x60` | `0110` | Charge/Discharge | READY | RUN | Individual | 개별 운전 |
| `0x70` | `0111` | Charge/Discharge | READY | RUN | Parallel | 병렬 운전 |
| `0xE0` | `1110` | Battery | READY | RUN | Individual | 배터리 모드 개별 운전 |
| `0xF0` | `1111` | Battery | READY | RUN | Parallel | 배터리 모드 병렬 운전 |

**상태 전이:**
```
IDLE+STOP (0x00) → READY+STOP (0x40) → READY+RUN (0x60)
     ↑                                        ↓
     ←────────────────────────────────────────
```

**주의사항:**
- IDLE+RUN 조합 (bit[6]=0, bit[5]=1)은 비정상 상태로 **Master에서 무시**
- 정상 운전을 위해서는 반드시 READY 상태 필요
- Master 수신 검증 로직:
  ```c
  uint8_t ready = (cmd >> 6) & 0x01;
  uint8_t run = (cmd >> 5) & 0x01;
  if (run && !ready) {
      // IDLE+RUN 무시, 이전 상태 유지
      return;
  }
  ```

**모드별 프레임 구조:**

프레임 길이는 항상 **13 bytes 고정**이며, Command bit[7]에 따라 Byte 2-7의 의미가 결정됩니다.

| Mode | bit[7] | Byte 2-3 | Byte 4-5 | Byte 6-7 |
|------|--------|----------|----------|----------|
| Charge/Discharge | 0 | V_max_cmd | V_min_cmd | I_cmd |
| Battery | 1 | V_cmd | I_max_cmd | I_min_cmd |

---

**전압/전류 지령 계산 (모든 필드 통일):**
- **타입**: int16
- **스케일**: ÷10 (0.1 단위)
- **계산식**: `Raw_Value = Physical_Value × 10`
- **범위**: -3276.8 ~ +3276.7 (V 또는 A)

**전압 예시:**
- `0.0 V → 0 (0x0000)`
- `+35.0 V → 350 (0x015E)`
- `+280.0 V → 2800 (0x0AF0)`
- `-100.0 V → -1000 (0xFC18)`
- `+327.6 V → 3276 (0x0CCC)`

**전류 예시:**
- `0.0 A → 0 (0x0000)`
- `+100.0 A → 1000 (0x03E8)`
- `-100.0 A → -1000 (0xFC18)`
- `+327.6 A → 3276 (0x0CCC)`
- `-327.6 A → -3276 (0xF334)`

**DSP 최적화 (양방향 동일):**
```c
// 송신 (Master → SCADA, SCADA → Master 동일)
int16_t voltage_raw = voltage_V * 10;
int16_t current_raw = current_A * 10;

// 수신 (양방향 동일)
float voltage_V = voltage_raw / 10.0f;
float current_A = current_raw / 10.0f;
```

---

**프레임 예제:**

### Charge/Discharge Mode 예제
```
READY + RUN + 병렬운전, V_max=35.0V, V_min=28.0V, I_cmd=120.0A
Command = 0b01110000 (0x70) - bit[7]=0(C/D), bit[6]=READY, bit[5]=RUN, bit[4]=Parallel
V_max_cmd = 35.0V → 350 (0x015E)
V_min_cmd = 28.0V → 280 (0x0118)
I_cmd = 120.0A → 1200 (0x04B0)

Data: 02 70 01 5E 01 18 04 B0
CRC-32 계산 대상: 70 01 5E 01 18 04 B0 (7 bytes)
CRC-32 = (예: 0x12345678)

완성 프레임 (13 bytes):
02 70 01 5E 01 18 04 B0 12 34 56 78 03
STX|Cmd|V_max |V_min |I_cmd |  CRC-32   |ETX
```

### Battery Mode 예제
```
READY + RUN + 개별운전, V_cmd=30.0V, I_max=200.0A, I_min=-150.0A
Command = 0b11100000 (0xE0) - bit[7]=1(Bat), bit[6]=READY, bit[5]=RUN
V_cmd = 30.0V → 300 (0x012C)
I_max_cmd = 200.0A → 2000 (0x07D0)
I_min_cmd = -150.0A → -1500 (0xFA24)

Data: 02 E0 01 2C 07 D0 FA 24
CRC-32 계산 대상: E0 01 2C 07 D0 FA 24 (7 bytes)
CRC-32 = (예: 0xABCDEF01)

완성 프레임 (13 bytes):
02 E0 01 2C 07 D0 FA 24 AB CD EF 01 03
STX|Cmd|V_cmd |I_max |I_min |  CRC-32   |ETX
```

---

## 📊 3. 데이터 타입 및 스케일링 요약

### 📤 Master → SCADA - **Rev4 완전 통일**
| 데이터 | 타입 | 스케일 | 범위 | 예시 |
|--------|------|--------|------|------|
| Control Byte (Master) | uint8 | - | - | 0x00 (Ch1), 0x02 (Ch2) |
| Control Byte (Slave) | uint8 | - | - | 0x2D (ID=5, DAB=1) |
| 시스템 전압 | **int16** | **÷10** | **-3276.8 ~ +3276.7 V** | 0 = 0.0 V, 3000 = 300.0 V |
| 슬레이브 전류 | **int16** | **÷10** | **-3276.8 ~ +3276.7 A** | 0 = 0.0 A, 1000 = 100.0 A |
| 슬레이브 온도 | uint8 | ×0.5 | 0 ~ 127.5 °C | 50 = 25.0 °C |

### 📥 SCADA → Master - **Rev4 완전 통일**
| 데이터 | 타입 | 스케일 | 범위 | 예시 | Mode |
|--------|------|--------|------|------|------|
| V_max_cmd, V_min_cmd, V_cmd | **int16** | **÷10** | **-3276.8 ~ +3276.7 V** | 350 = 35.0 V | 모드별 |
| I_cmd, I_max_cmd, I_min_cmd | **int16** | **÷10** | **-3276.8 ~ +3276.7 A** | 1000 = 100.0 A | 모드별 |

**✅ 양방향 완전 통일:**
- 모든 전압/전류: **int16, ÷10 스케일**
- Master → SCADA와 SCADA → Master 동일
- DSP 코드 패턴 단순화

---

## 🔐 4. 체크섬/CRC 계산

### ✅ Master → SCADA (Sum Checksum)
**방식:** 단순 합 체크섬 (Sum Checksum)

**계산 방법:**
```python
# Master → SCADA (7 bytes)
checksum = sum(frame[1:5]) & 0xFF
```

**예제:**
```
Frame: 02 29 94 D9 3D [??] 03
Checksum = (0x29 + 0x94 + 0xD9 + 0x3D) & 0xFF
         = 0x251 & 0xFF = 0x51
Result: 02 29 94 D9 3D 51 03
```

---

### 🔒 SCADA → Master (CRC-32) - **Rev4**
**방식:** CRC-32 (Polynomial: 0x04C11DB7)

**특징:**
- TI C2000 VCU/VCRC 하드웨어 가속 지원
- 오류 검출 능력 우수 (32-bit)
- Big-endian 형식

**TI C2000 라이브러리 사용:**
```c
#include "stl_crc.h"

// CRC 객체 초기화
STL_CRC_Obj crcObj;
crcObj.seedValue = 0x00000000;          // Initial value
crcObj.numBytes = 7;                     // Byte 1~7 (Command + Data)
crcObj.parity = STL_CRC_PARITY_EVEN;    // Start from low byte
crcObj.msgBuffer = &frame[1];            // Data starts at Byte 1

// CRC 계산
STL_CRC_Handle crcHandle = &crcObj;
STL_CRC_calculate(crcHandle);
uint32_t crc32 = crcObj.crcResult;

// 프레임에 CRC-32 삽입 (Big-endian)
frame[8] = (crc32 >> 24) & 0xFF;   // MSB
frame[9] = (crc32 >> 16) & 0xFF;
frame[10] = (crc32 >> 8) & 0xFF;
frame[11] = crc32 & 0xFF;           // LSB
frame[12] = 0x03;                   // ETX
```

**Python 구현 (테스트용):**
```python
import zlib

# CRC-32 계산 (Polynomial: 0x04C11DB7)
def calculate_crc32(data):
    crc = zlib.crc32(data) & 0xFFFFFFFF
    return crc

# Charge/Discharge Mode 프레임 생성
frame = bytearray([0x02, 0x30, 0x01, 0x5E, 0x01, 0x18, 0x84, 0xB0])
# STX, Command, V_max, V_min, I_cmd

# CRC-32 계산 (Byte 1~7)
crc32 = calculate_crc32(frame[1:8])

# Big-endian으로 추가
frame.extend([
    (crc32 >> 24) & 0xFF,
    (crc32 >> 16) & 0xFF,
    (crc32 >> 8) & 0xFF,
    crc32 & 0xFF
])
frame.append(0x03)  # ETX

# 결과: 13 bytes
print(f"Frame: {' '.join(f'{b:02X}' for b in frame)}")
```

**CRC-32 검증:**
```c
// 수신 프레임 검증 (13 bytes)
uint32_t receivedCRC = (frame[8] << 24) | (frame[9] << 16) |
                       (frame[10] << 8) | frame[11];

// 계산된 CRC
STL_CRC_Obj crcObj;
crcObj.seedValue = 0x00000000;
crcObj.numBytes = 7;                     // Command + Data (Byte 1~7)
crcObj.parity = STL_CRC_PARITY_EVEN;
crcObj.msgBuffer = &frame[1];

STL_CRC_calculate(&crcObj);

if (crcObj.crcResult == receivedCRC) {
    // CRC 검증 성공
    // Command bit[7]로 모드 판단 후 데이터 파싱
    uint8_t mode = (frame[1] >> 7) & 0x01;
    if (mode == 0) {
        // Charge/Discharge Mode
        int16_t v_max_raw = (frame[2] << 8) | frame[3];
        int16_t v_min_raw = (frame[4] << 8) | frame[5];
        int16_t i_cmd_raw = (frame[6] << 8) | frame[7];
        float v_max = v_max_raw / 10.0f;
        float v_min = v_min_raw / 10.0f;
        float i_cmd = i_cmd_raw / 10.0f;
    } else {
        // Battery Mode
        int16_t v_cmd_raw = (frame[2] << 8) | frame[3];
        int16_t i_max_raw = (frame[4] << 8) | frame[5];
        int16_t i_min_raw = (frame[6] << 8) | frame[7];
        float v_cmd = v_cmd_raw / 10.0f;
        float i_max = i_max_raw / 10.0f;
        float i_min = i_min_raw / 10.0f;
    }
} else {
    // CRC 오류
}
```

---

## ⚙️ 5. 통신 파라미터

| 파라미터 | 값 |
|---------|-----|
| **Baud Rate** | 115200 bps |
| **Data Bits** | 8 |
| **Stop Bits** | 1 |
| **Parity** | None |
| **Flow Control** | None |

---

## 📏 6. 프레임 구조 비교

| 방향 | STX | Command | Data Fields | CRC/Checksum | ETX | 총 길이 |
|------|-----|---------|-------------|--------------|-----|---------|
| Master → SCADA | 0x02 | - | 4 bytes | Sum (1 byte) | 0x03 | **7 bytes** |
| SCADA → Master (Rev4) | 0x02 | 1 byte | **6 bytes** | **CRC-32 (4 bytes)** | 0x03 | **13 bytes** |

**SCADA → Master 프레임 특징:**
- 고정 길이: 항상 13 bytes
- Command bit[7]로 모드 결정 (0=Charge/Discharge, 1=Battery)
- Byte 2-7의 의미가 모드에 따라 변경

---

## 📝 변경 이력

### 🆕 Rev 4.0 (2025-10-15)

**Master → SCADA 변경사항:**
- **Control Byte (Byte 1) 재구성**: Master/Slave 데이터 명확히 구분
  - bit[0]: Data Type (0=Master, 1=Slave)
  - Master: bit[7:2]=Reserved, bit[1]=Channel (2채널 지원, 향후 확장 가능)
  - Slave: bit[7:3]=Slave ID (0~31, DIP 스위치), bit[2]=DAB_OK, bit[1]=Reserved
- **데이터 타입 완전 통일**: **int16, ÷10 스케일**
  - 전압: uint16 offset → int16 ÷10 (offset 제거, 코드 단순화)
  - 전류: uint16 center offset ÷100 → **int16 ÷10** (스케일 통일)
  - DSP 연산 최적화 (하드웨어 곱셈기 활용, ~3-4 cycles)
  - 범위: -3276.8 ~ +3276.7 (V/A)
  - 해상도: 0.1 (V/A)
- **Sum Checksum 유지** (호환성)

**SCADA → Master 변경사항:**
- **프레임 재설계**: 10 bytes → 13 bytes (고정 길이)
- **CRC-32 도입**: Sum Checksum → CRC-32 (Polynomial: 0x04C11DB7)
  - TI C2000 VCU/VCRC 하드웨어 가속 지원
  - 오류 검출 능력 향상 (1 byte → 4 bytes)
- **Command 필드 재구성**: 독립적인 상태 비트로 재설계 (산업 표준 용어)
  - bit[7]=제어모드, bit[6]=준비상태(IDLE/READY), bit[5]=운전상태(STOP/RUN), bit[4]=병렬
- **모드별 선택적 프레임 구조** (고정 13 bytes):
  - Charge/Discharge Mode (bit[7]=0): V_max_cmd, V_min_cmd, I_cmd
  - Battery Mode (bit[7]=1): V_cmd, I_max_cmd, I_min_cmd
  - Command bit[7]에 따라 Byte 2-7의 의미 변경
- **데이터 타입 완전 통일**: **int16, ÷10 스케일**
  - 전압: int16 ÷10 (Master→SCADA와 동일)
  - 전류: int16 ÷10 (Master→SCADA와 동일)
  - 양방향 일관성으로 펌웨어 코드 단순화
- **범용성**: 안전 제한은 Master/Slave에서 처리, 프로토콜은 범용적 설계
- **효율성**: 프레임 크기 최적화 (10 bytes → 13 bytes, 이전 18 bytes 제안 대비 축소)

**Rev4 DSP 연산 최적화 (완전 통일):**
- 모든 전압/전류: **int16, ÷10 스케일**
- Master ↔ SCADA 양방향 동일 패턴
- TI C2000 단일 사이클 곱셈으로 성능 향상
- 코드 재사용성 극대화

### 🔄 Rev 3.1 (2025-10-15)
- **시스템 전압 오프셋 추가**: 음수 전압 범위 지원 (-30.0V ~ +6523.5V)
- 계산식 변경: `voltage_raw / 10` → `(voltage_raw - 300) / 10`
- Master → SCADA 시스템 전압 프레임만 변경, 기타 프레임 동일

### 🔄 Rev 3.0
- Command 필드 확장: 4가지 동작 제어 상태 추가 (NoOp, Standby, Run, Stop)
- Command 비트 재배치: bit[1:0]=동작 제어, bit[3:2]=운전 모드

### 🔄 Rev 2.1
- 시스템 전압 오프셋 추가 (300 offset for negative voltage support)

### 🔄 Rev 2.0
- 병렬운전 모드 추가
- Command 필드: bit[0]=Start/Stop, bit[2:1]=운전모드
- Baud Rate: 115200 bps

### 📌 Rev 1.0
- 초기 프로토콜 정의
- Master ↔ SCADA 양방향 통신
- 7-byte/10-byte 프레임 구조
