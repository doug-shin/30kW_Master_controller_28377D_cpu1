# RS-232 통신 프로토콜 Rev 5.0

**30kW 팩사이클러 마스터 컨트롤러 ↔ SCADA 통신 프로토콜**

---

## 📋 목차

1. [개요](#개요)
2. [통신 설정](#통신-설정)
3. [프레이밍 구조](#프레이밍-구조)
4. [마스터 → SCADA](#마스터--scada)
   - [시스템 상태 패킷](#시스템-상태-패킷-100ms)
   - [슬레이브 상태 패킷](#슬레이브-상태-패킷-200ms-배치)
5. [SCADA → 마스터](#scada--마스터)
6. [연결 감시](#연결-감시-connection-watchdog)
7. [구현 가이드](#구현-가이드)
8. [변경 이력](#변경-이력)

---

## 개요

### 설계 목표

**Rev 5.0 핵심 특징:**
- ✅ **115200 baud**: 산업 표준, 높은 신뢰성
- ✅ **16바이트 고정 패킷**: FIFO 최적화, 메모리 정렬
- ✅ **STX/ETX 프레이밍**: 검증된 산업 표준 (ASCII 제어 문자)
- ✅ **배치 전송**: 슬레이브 3개씩 묶어 전송 (6개 → 2패킷)
- ✅ **100ms 토글 전송**: 시스템/슬레이브 교대, 균일한 부하
- ✅ **200ms 연결 감시**: SCADA 크래시 자동 감지 및 안전 정지
- ✅ **동적 슬레이브 감지**: 연결/해제 자동 처리

### 프로토콜 구조

| 방향 | 패킷 종류 | 크기 | 주기 | 용도 |
|------|-----------|------|------|------|
| 마스터 → SCADA | 시스템 상태 | 16 bytes | 100ms | 제어 파라미터 + 시스템 전압 + 고장 상태 |
| 마스터 → SCADA | 슬레이브 상태 (배치1) | 16 bytes | 200ms | 슬레이브 1,2,3 (전류/온도) |
| 마스터 → SCADA | 슬레이브 상태 (배치2) | 16 bytes | 200ms | 슬레이브 4,5,6 (전류/온도) |
| SCADA → 마스터 | 제어 지령 | 16 bytes | 100ms (Keep-Alive) | 운전 모드 + 지령값 |

### 전송 스케줄

**100ms 토글 구조** (시스템 ↔ 슬레이브 교대):

| 시간 | 패킷 | 내용 | 전송 시간 |
|------|------|------|----------|
| 0ms | 시스템 상태 | 제어 파라미터 + 전압 + 고장 | 1.39ms |
| 100ms | 슬레이브 배치 1+2 | Slave 1,2,3 + Slave 4,5,6 | 2.78ms |
| 200ms | 시스템 상태 | 제어 파라미터 + 전압 + 고장 | 1.39ms |
| 300ms | 슬레이브 배치 1+2 | Slave 1,2,3 + Slave 4,5,6 | 2.78ms |
| 400ms | (반복) | ... | ... |

**특징:**
- ✅ 시스템 상태: **100ms** 주기 업데이트 (빠른 피드백)
- ✅ 슬레이브 전체: **200ms** 스캔 완료 (모니터링 충분)
- ✅ 최대 전송 시간: **2.78ms** (ISR 영향 없음, 실제 블로킹 <1μs)
- ✅ SCADA Keep-Alive: **100ms** 주기 (200ms 타임아웃)

### 성능 비교

| 항목 | Rev4 (115200) | Rev5 (115200) | 개선점 |
|------|---------------|---------------|--------|
| **패킷 크기** | 7+13 bytes (가변) | 16 bytes (고정) | **일관성 ⬆** |
| **프레이밍** | STX/ETX | STX/ETX (유지) | **검증된 표준** |
| **시스템 상태** | 50ms | 100ms | 적절 (실시간 제어는 ISR) |
| **슬레이브 스캔** | 150ms (15개 개별) | 200ms (6개 배치) | **모니터링 충분** |
| **전송 대역폭** | 700 bytes/s | 160 bytes/s | **77% ⬇ (효율)** |
| **CPU 부하** | 높음 (10ms 주기) | 낮음 (100ms 주기) | **10배 ⬇** |
| **연결 감시** | 없음 | 200ms 자동 정지 | **안전성 ⬆** |

---

## 통신 설정

```yaml
보드레이트: 115200 bps
데이터비트: 8
정지비트: 1
패리티: None
흐름제어: None
물리계층: RS-232 (3m 케이블, 차폐 권장)
엔디안: Big-endian (네트워크 바이트 오더)
```

### 보드레이트 설정 (TI F28377D)

**BRR 계산**:
```
BRR = (LSPCLK / (Baud × 8)) - 1
    = (180,000,000 / (115200 × 8)) - 1
    = 195.3125 ≈ 195

실제 보드레이트 = 180,000,000 / ((195+1) × 8)
                = 115,384.6 bps

오차율 = (115384.6 - 115200) / 115200 = +0.16%
```

**구현**:
```c
SCI_setBaud(SCID_BASE, DEVICE_LSPCLK_FREQ, 115200);
// BRR=195, 오차율 +0.16% (RS-232 허용 ±2% 이내)
```

---

## 프레이밍 구조

### STX/ETX 프레이밍

**프레임 구조**: `[STX][Payload 14 bytes][ETX]`

| 바이트 | 필드 | 값 | 설명 |
|--------|------|-----|------|
| 0 | STX | 0x02 | Start of Text (ASCII 제어 문자) |
| 1-14 | Payload | - | 데이터 필드 (14 bytes) |
| 15 | ETX | 0x03 | End of Text (ASCII 제어 문자) |

**장점**:
- ✅ **산업 표준**: ASCII 제어 문자, 수십년 검증된 방식
- ✅ **명확한 프레임 경계**: 시작(STX)과 끝(ETX) 모두 표시
- ✅ **빠른 재동기화**: 패킷 손실 시 ETX 발견으로 즉시 복구
- ✅ **디버깅 용이**: 바이너리 덤프에서 시각적 구분 쉬움

### 고정 길이 프로토콜

**Rev 5는 고정 길이 (16 bytes) 프로토콜**:
- STX 발견 → 16바이트 수신 → ETX 검증
- **바이트스터핑 불필요** (고정 길이)

**바이트스터핑이 필요한 경우** (참고):
- 가변 길이 프로토콜에서 Payload에 STX/ETX 포함 시
- 예: Payload에 0x02, 0x03 → Escape 시퀀스 삽입
- **Rev 5는 해당 없음** (고정 길이로 Payload 범위 명확)

### 패킷 구조 (16 bytes)

```
[0]     STX (0x02)
[1-13]  Payload (13 bytes)
[14]    Checksum 또는 CRC
[15]    ETX (0x03)
```

### 체크섬 방식

**마스터 → SCADA**: Sum Checksum (1 byte)
```c
// Byte 1~13 합산 (8비트 체크섬)
uint8_t checksum = 0;
for (uint8_t i = 1; i <= 13; i++) {
    checksum += buffer[i];
}
buffer[14] = checksum & 0xFF;
buffer[15] = 0x03;  // ETX
```

**SCADA → 마스터**: CRC-32 (4 bytes, Payload에 포함)
```c
// VCU2 하드웨어 가속 CRC-32 (Polynomial: 0x04C11DB7)
crcObj_SCADA.nMsgBytes = 10;  // Byte 1~10 (Command + Params)
crcObj_SCADA.pMsgBuffer = (void *)(&buffer[1]);
CRC_run32BitPoly2(handleCRC_SCADA, ...);
// CRC-32는 Byte 10~13에 저장 (Big-endian)
```

### 프레임 동기화

**수신 로직** (현재 구현):
```c
// 1단계: STX 대기
if (rx_index == 0) {
    if (rx_byte == 0x02) {  // STX
        buffer[rx_index++] = rx_byte;
    }
}
// 2단계: 16바이트 수신
else {
    buffer[rx_index++] = rx_byte;

    if (rx_index == 16) {
        // 3단계: ETX 검증
        if (buffer[0] == 0x02 && buffer[15] == 0x03) {
            // 유효 패킷 → 파싱
            parse_packet();
        }
        rx_index = 0;  // 재동기화
    }
}
```

---

## 마스터 → SCADA

### 시스템 상태 패킷 (100ms)

**목적:** 제어 파라미터 실시간 피드백 + 시스템 전압 + 고장 상태 모니터링

#### 패킷 구조 (16 bytes)

| Byte | 필드 | 타입 | 스케일 | 범위 | 설명 |
|------|------|------|--------|------|------|
| 0 | STX | 0x02 | - | - | Start of Text |
| 1 | Control Status | uint8 | - | 비트맵 | bit0: Data Type = 0<br>bit1: Master Ch<br>bit2: Run Status<br>bit3: Precharge<br>bit4: Parallel Mode<br>bit5: Control Mode<br>bit6-7: Reserved |
| 2-3 | System Voltage | int16 | ÷10 | -3276.8 ~ +3276.7 V | Big-endian, 0.1V 해상도 |
| 4-5 | Param1 | int16 | ÷10 | -3276.8 ~ +3276.7 | **C/D**: I_cmd (A)<br>**Bat**: V_cmd (V) |
| 6-7 | Param2 | int16 | ÷10 | -3276.8 ~ +3276.7 | **C/D**: V_max_cmd (V)<br>**Bat**: I_max_cmd (A) |
| 8-9 | Param3 | int16 | ÷10 | -3276.8 ~ +3276.7 | **C/D**: V_min_cmd (V)<br>**Bat**: I_min_cmd (A) |
| 10-12 | Reserved | 3 bytes | - | 0x00 | 미래 확장용 |
| 13 | Fault/Warning | uint8 | - | 비트맵 | bit7-4: Fault, bit3-0: Warning |
| 14 | Checksum | uint8 | - | 0~255 | Sum(Byte1~13) & 0xFF |
| 15 | ETX | 0x03 | - | - | End of Text |

#### Control Status 비트맵

| 비트 | 필드 | 값 | 의미 |
|-----|------|-----|------|
| 0 | Data Type | 0 | Master Data (시스템 상태) |
|   |           | 1 | Slave Data (슬레이브 상태) |
| 1 | Master Ch | 0 | Ch1 (Master1) |
|   |           | 1 | Ch2 (Master2) |
| 2 | Run Status | 0 | STOP (정지) |
|   |            | 1 | RUN (운전) |
| 3 | Precharge | 0 | Not Ready |
|   |           | 1 | Ready (Precharge 완료) |
| 4 | Parallel Mode | 0 | OFF (Individual 개별) |
|   |               | 1 | ON (Parallel 병렬) |
| 5 | Control Mode | 0 | Charge/Discharge |
|   |              | 1 | Battery |
| 6-7 | Reserved | - | 향후 확장 (0으로 설정) |

#### Fault/Warning 비트맵

| 비트 | 필드 | 값 | 의미 |
|-----|------|-----|------|
| 7 | 과전압 고장 | 0 | 정상 |
|   |           | 1 | V_out ≥ 1400V |
| 6 | 과전류 고장 | 0 | 정상 |
|   |           | 1 | I_out ≥ 88A |
| 5 | 과온도 고장 | 0 | 정상 |
|   |           | 1 | Temp ≥ 85°C |
| 4 | SCADA 타임아웃 고장 | 0 | 정상 |
|   |                    | 1 | 200ms 이상 무응답 (자동 정지됨) |
| 3 | 과전압 경고 | 0 | 정상 |
|   |           | 1 | V_out ≥ 1300V |
| 2 | 과전류 경고 | 0 | 정상 |
|   |           | 1 | I_out ≥ 80A |
| 1 | 과온도 경고 | 0 | 정상 |
|   |           | 1 | Temp ≥ 75°C |
| 0 | SCADA 타임아웃 경고 | 0 | 정상 |
|   |                    | 1 | 100ms 이상 무응답 (곧 정지) |

#### 제어 모드별 파라미터 의미

**Charge/Discharge 모드** (Control Mode bit5 = 0):
```
Param1 = I_cmd      전류 지령 (A)
Param2 = V_max_cmd  전압 상한 (V)
Param3 = V_min_cmd  전압 하한 (V)

동작: I_cmd 전류로 충방전, V_max/V_min 내에서 제어
```

**Battery 모드** (Control Mode bit5 = 1):
```
Param1 = V_cmd      전압 지령 (V)
Param2 = I_max_cmd  전류 상한 (A)
Param3 = I_min_cmd  전류 하한 (A)

동작: V_cmd 전압으로 CV 제어, I_max/I_min 내에서 제어
```

---

### 슬레이브 상태 패킷 (200ms 배치)

**목적:** 슬레이브 3개 데이터를 1패킷에 묶어 전송 (6개 → 2패킷)

**전송 방식:**
- 100ms 시점: 배치1 (Slave 1,2,3) + 배치2 (Slave 4,5,6) **연속 전송**
- 300ms 시점: 배치1 (Slave 1,2,3) + 배치2 (Slave 4,5,6) **연속 전송**
- 슬레이브 전체 스캔: **200ms** 주기

#### 패킷 구조 (16 bytes)

| Byte | 필드 | 타입 | 스케일 | 범위 | 설명 |
|------|------|------|--------|------|------|
| 0 | STX | 0x02 | - | - | Start of Text |
| 1 | Control Status | uint8 | - | - | bit0: Data Type = 1 (Slave Data)<br>bit3-1: Connection (Slave1~3)<br>bit7-4: Reserved |
| 2 | Slave1 ID/Fault | uint8 | - | 비트맵 | bit7: OP (Over Power)<br>bit6: OV (Over Voltage)<br>bit5: OC (Over Current)<br>bit4: OT (Over Temp)<br>bit3-0: Slave ID (1~15) |
| 3-4 | Slave1 Current | int16 | ÷10 | -3276.8 ~ +3276.7 A | Big-endian, 0.1A 해상도 |
| 5 | Slave1 Temp | uint8 | ×0.5 | 0 ~ 127.5 °C | 0.5°C 해상도 |
| 6 | Slave2 ID/Fault | uint8 | - | 비트맵 | (Slave1과 동일) |
| 7-8 | Slave2 Current | int16 | ÷10 | -3276.8 ~ +3276.7 A | Big-endian |
| 9 | Slave2 Temp | uint8 | ×0.5 | 0 ~ 127.5 °C | 0.5°C 해상도 |
| 10 | Slave3 ID/Fault | uint8 | - | 비트맵 | (Slave1과 동일) |
| 11-12 | Slave3 Current | int16 | ÷10 | -3276.8 ~ +3276.7 A | Big-endian |
| 13 | Slave3 Temp | uint8 | ×0.5 | 0 ~ 127.5 °C | 0.5°C 해상도 |
| 14 | Checksum | uint8 | - | 0~255 | Sum(Byte1~13) & 0xFF |
| 15 | ETX | 0x03 | - | - | End of Text |

#### Control Status 비트맵 (연결 상태)

| 비트 | 필드 | 값 | 의미 |
|-----|------|-----|------|
| 0 | Data Type | 1 | Slave Data (슬레이브 상태 패킷) |
| 1 | Slave1 Connection | 0 | 슬레이브1 연결 끊김/고장 |
|   |                   | 1 | 슬레이브1 정상 연결 |
| 2 | Slave2 Connection | 0 | 슬레이브2 연결 끊김/고장 |
|   |                   | 1 | 슬레이브2 정상 연결 |
| 3 | Slave3 Connection | 0 | 슬레이브3 연결 끊김/고장 |
|   |                   | 1 | 슬레이브3 정상 연결 |
| 7-4 | Reserved | - | 향후 확장 (0으로 설정) |

**참고**: `Connection`은 슬레이브 모듈의 통신 및 DAB(Dual Active Bridge) 컨버터 동작 상태를 나타냅니다.

**빠른 판단**:
```c
uint8_t control_status = packet[1];
if (control_status & 0x02) {
    // Slave1 연결 정상 (통신 OK + DAB 동작 중)
}
if (control_status & 0x04) {
    // Slave2 연결 정상
}
if (control_status & 0x08) {
    // Slave3 연결 정상
}
```

#### Slave Fault 비트맵 (고장 상세)

| 비트 | 필드 | 값 | 의미 |
|-----|------|-----|------|
| 7 | OP (Over Power) | 0 | 정상 (P ≤ 35kW) |
|   |                 | 1 | 과전력 (P > 35kW, 30kW 정격 + 15%) |
| 6 | OV (Over Voltage) | 0 | 정상 (V < 1400V) |
|   |                   | 1 | 과전압 (V ≥ 1400V) |
| 5 | OC (Over Current) | 0 | 정상 (I < 88A) |
|   |                   | 1 | 과전류 (I ≥ 88A, 80A 정격 + 10%) |
| 4 | OT (Over Temp) | 0 | 정상 (T < 85°C) |
|   |                | 1 | 과온도 (T ≥ 85°C) |
| 3-0 | Slave ID | 1~15 | 슬레이브 ID (실제 DIP 스위치 값) |

**동시 고장 표시 가능**:
```c
uint8_t fault = packet[2];
uint8_t slave_id = fault & 0x0F;
bool op = (fault >> 7) & 0x01;
bool ov = (fault >> 6) & 0x01;
bool oc = (fault >> 5) & 0x01;
bool ot = (fault >> 4) & 0x01;

// 예: OC=1, OT=1 (과전류+과온도 동시 발생)
```

#### 동적 슬레이브 감지

**Active Slave List 방식:**
- 마스터가 실제 연결된 슬레이브만 전송
- 슬레이브 연결 끊김 시: `Connection=0`, `Current=0`, `Temp=0`
- 슬레이브 중간 추가 시: 자동으로 `Connection=1`로 업데이트
- ID 1~15 중 임의 6개 슬레이브 지원 (ID 순서대로 전송)

**빈 슬롯 처리:**
```
연결된 슬레이브: ID 1, 3, 5 (3개만 연결)

배치1 패킷:
  Slave1: ID=1, Connection=1, Current=78.5A
  Slave2: ID=3, Connection=1, Current=79.2A
  Slave3: ID=5, Connection=1, Current=77.8A

배치2 패킷:
  Slave1: ID=0, Connection=0, Current=0A (빈 슬롯)
  Slave2: ID=0, Connection=0, Current=0A (빈 슬롯)
  Slave3: ID=0, Connection=0, Current=0A (빈 슬롯)
```

---

## SCADA → 마스터

### 제어 지령 패킷 (100ms Keep-Alive)

**목적:** SCADA에서 마스터로 운전 모드 및 제어 파라미터 전송

**전송 주기:**
- 사용자 조작 시: 즉시
- 무조작 시: **100ms 주기로 자동 재전송 (Keep-Alive)**

#### 패킷 구조 (16 bytes)

| Byte | 필드 | 타입 | 스케일 | 범위 | 설명 |
|------|------|------|--------|------|------|
| 0 | STX | 0x02 | - | - | Start of Text |
| 1 | Command | uint8 | - | 비트맵 | bit0-1: Reserved<br>bit2: Precharge<br>bit3: Parallel Mode<br>bit4: Control Mode<br>bit5: Run Status<br>bit6-7: Reserved |
| 2-3 | Param1 | int16 | ÷10 | -3276.8 ~ +3276.7 | **C/D**: I_cmd (A)<br>**Bat**: V_cmd (V) |
| 4-5 | Param2 | int16 | ÷10 | -3276.8 ~ +3276.7 | **C/D**: V_max_cmd (V)<br>**Bat**: I_max_cmd (A) |
| 6-7 | Param3 | int16 | ÷10 | -3276.8 ~ +3276.7 | **C/D**: V_min_cmd (V)<br>**Bat**: I_min_cmd (A) |
| 8-10 | Reserved | 3 bytes | - | 0x00 | 미래 확장용 |
| 11-14 | CRC-32 | uint32 | - | - | Big-endian<br>Polynomial: 0x04C11DB7<br>CRC(Byte[1]~[10]) |
| 15 | ETX | 0x03 | - | - | End of Text |

**주의**: 제어 지령은 **CRC-32** 사용 (무결성 중요)

#### Command 비트맵

| 비트 | 필드 | 값 | 의미 |
|-----|------|-----|------|
| 0-1 | Reserved | - | 향후 확장 (0으로 설정) |
| 2 | Precharge | 0 | Not Ready |
|   |           | 1 | Ready (Precharge 완료) |
| 3 | Parallel Mode | 0 | OFF (Individual 개별) |
|   |               | 1 | ON (Parallel 병렬) |
| 4 | Control Mode | 0 | Charge/Discharge |
|   |              | 1 | Battery |
| 5 | Run Status | 0 | STOP (정지) |
|   |            | 1 | RUN (운전) |
| 6-7 | Reserved | - | 향후 확장 (0으로 설정) |

#### CRC-32 계산

**Polynomial**: 0x04C11DB7 (Ethernet/ZIP 표준)

**계산 범위**: Byte[1]~[10] (Command + Param1~3 + Reserved 3 bytes)

**C 구현 (TI VCU2 하드웨어 가속)**:
```c
// STX 제외하고 Payload만 CRC 계��
crcObj_SCADA.seedValue   = 0x00000000;
crcObj_SCADA.nMsgBytes   = 10;  // Byte[1]~[10]
crcObj_SCADA.parity      = CRC_parity_even;
crcObj_SCADA.pMsgBuffer  = (void *)(&buffer[1]);

CRC_run32BitPoly2(handleCRC_SCADA);

// CRC-32 저장 (Big-endian)
buffer[11] = (uint8_t)(crcObj_SCADA.crcResult >> 24);
buffer[12] = (uint8_t)(crcObj_SCADA.crcResult >> 16);
buffer[13] = (uint8_t)(crcObj_SCADA.crcResult >> 8);
buffer[14] = (uint8_t)(crcObj_SCADA.crcResult & 0xFF);
buffer[15] = 0x03;  // ETX
```

**Python 구현**:
```python
import struct
import zlib

def calc_crc32(data):
    """CRC-32 (Polynomial: 0x04C11DB7)"""
    return zlib.crc32(data) & 0xFFFFFFFF

# 패킷 생성
packet = bytearray(16)
packet[0] = 0x02  # STX
packet[1] = 0x20  # Command
# ... Param1~3, Reserved ...

# CRC-32 계산 및 저장 (Big-endian)
crc = calc_crc32(packet[1:11])
packet[11:15] = struct.pack('>I', crc)
packet[15] = 0x03  # ETX
```

---

## 연결 감시 (Connection Watchdog)

### 개요

마스터는 SCADA 제어 지령 패킷 수신을 감시하여 연결 상태를 확인합니다. SCADA가 크래시하거나 통신이 끊기면 자동으로 안전 정지합니다.

### 마스터 측 타임아웃

| 경과 시간 | 동작 | 설명 |
|----------|------|------|
| **100ms** | 경고 표시 | Fault/Warning bit0 = 1 (경고) |
| **200ms** | 안전 정지 | Fault/Warning bit4 = 1 (고장)<br>운전 중지, IDLE 상태 전환 |

**타임스탬프 갱신 조건**:
- 제어 지령 패킷 수신 성공 (CRC-32 검증 통과)
- 패킷 내용 무관 (동일 지령 재전송도 유효)

**안전 정지 동작** (200ms 타임아웃 시):
```c
start_stop = STOP
sequence_step = SEQ_STEP_IDLE
I_out_ref = 0
V_max_cmd = 0
V_min_cmd = 0
릴레이 OFF (메인 릴레이, 병렬 릴레이)
```

### SCADA 측 Keep-Alive

**목적**: 연결 유지 및 타임아웃 방지

**구현 방법**:
1. 마지막 전송 시각 기록
2. 100ms 경과 시 마지막 지령 재전송
3. 사용자 조작 없어도 자동 전송

**Python 구현 예시**:
```python
class SCADAController:
    def __init__(self):
        self.last_tx_time = time.time()
        self.last_command = None
        self.keep_alive_interval = 0.1  # 100ms

    def send_command(self, cmd):
        """사용자 조작 시 호출"""
        self.serial.write(cmd)
        self.last_tx_time = time.time()
        self.last_command = cmd

    def periodic_task(self):
        """10ms 주기 호출 (메인 루프)"""
        elapsed = time.time() - self.last_tx_time

        if elapsed > self.keep_alive_interval:
            if self.last_command:
                # 마지막 지령 재전송 (Keep-Alive)
                self.serial.write(self.last_command)
                self.last_tx_time = time.time()
```

**C# 구현 예시**:
```csharp
public class SCADAController
{
    private DateTime lastTxTime;
    private byte[] lastCommand;
    private readonly TimeSpan keepAliveInterval = TimeSpan.FromMilliseconds(100);

    public void SendCommand(byte[] cmd)
    {
        serialPort.Write(cmd, 0, cmd.Length);
        lastTxTime = DateTime.Now;
        lastCommand = cmd;
    }

    public void PeriodicTask()  // 10ms 타이머
    {
        var elapsed = DateTime.Now - lastTxTime;

        if (elapsed > keepAliveInterval && lastCommand != null)
        {
            // Keep-Alive: 마지막 지령 재전송
            serialPort.Write(lastCommand, 0, lastCommand.Length);
            lastTxTime = DateTime.Now;
        }
    }
}
```

### 타임아웃 설정 근거

**200ms 타임아웃 선정 이유**:
1. ✅ **빠른 감지**: 0.2초 = 사람 반응 시간 이내
2. ✅ **2회 실패 허용**: Keep-Alive 100ms × 2 = 200ms (1회 노이즈 무시)
3. ✅ **마스터 → SCADA 주기와 일치**: 양방향 100ms 균형
4. ✅ **산업 표준 범위**: 실시간 시스템 50~500ms 권장

**100ms Keep-Alive 선정 이유**:
1. ✅ **SCADA 부하 적절**: 10 packets/sec (Python/C# 충분)
2. ✅ **마스터 전송 주기와 동기**: 시스템 상태 100ms
3. ✅ **일시적 지연 허용**: 노이즈/UI 렉 1회는 무시

---

## 구현 가이드

### DSP 측 구현 (마스터 컨트롤러)

#### 1. 전역 변수 추가

```c
// HABA_globals.h
#define SCADA_TIMEOUT_WARNING_MS  100   // 경고 타임아웃
#define SCADA_TIMEOUT_STOP_MS     200   // 정지 타임아웃

extern uint32_t scada_last_rx_time;     // 마지막 수신 시각 (1ms 단위)
extern uint8_t scada_timeout_flag;      // 타임아웃 플래그
```

#### 2. 타임스탬프 갱신

```c
// HABA_main.c
void main(void) {
    scada_last_rx_time = system_tick_1ms;  // 초기화

    while(1) {
        // SCADA 패킷 수신 즉시 처리
        if (scada_packet_ready) {
            scada_last_rx_time = system_tick_1ms;  // ← 타임스탬프 갱신
            Parse_SCADA_Command();
            scada_packet_ready = 0;
        }

        // 10ms 주기 작업
        if (flag_10ms) {
            Check_SCADA_Timeout();  // ← 타임아웃 체크
            // ...
        }
    }
}
```

#### 3. 타임아웃 체크 함수

```c
// HABA_control.c
void Check_SCADA_Timeout(void) {
    uint32_t elapsed = system_tick_1ms - scada_last_rx_time;

    if (elapsed > SCADA_TIMEOUT_STOP_MS) {
        // 200ms 타임아웃 → 안전 정지
        scada_timeout_flag = 1;

        start_stop = STOP;
        sequence_step = SEQ_STEP_IDLE;
        I_out_ref = 0;
        V_max_cmd = 0;
        V_min_cmd = 0;

        // 릴레이 OFF는 Phase 3에서 자동 처리됨

    } else if (elapsed > SCADA_TIMEOUT_WARNING_MS) {
        // 100ms 경고
        scada_timeout_flag = 1;  // 경고 표시용

    } else {
        scada_timeout_flag = 0;  // 정상
    }
}
```

#### 4. 시스템 상태 패킷 전송

```c
void Send_System_Status_To_SCADA(void) {
    uint8_t tx_buffer[16];

    // STX
    tx_buffer[0] = 0x02;

    // Control Status
    tx_buffer[1] = 0;  // Data Type = 0 (Master)
    if (master_id == CH2_ID) tx_buffer[1] |= 0x02;
    if (start_stop == START) tx_buffer[1] |= 0x04;
    if (ready_state) tx_buffer[1] |= 0x08;
    if (parallel_mode) tx_buffer[1] |= 0x10;
    if (control_mode == CONTROL_MODE_BATTERY) tx_buffer[1] |= 0x20;

    // System Voltage (int16, Big-endian, ÷10)
    int16_t v_sys = (int16_t)(V_out_display * 10.0f);
    tx_buffer[2] = (uint8_t)(v_sys >> 8);
    tx_buffer[3] = (uint8_t)(v_sys & 0xFF);

    // Param1 (int16, Big-endian, ÷10)
    int16_t param1 = (control_mode == CONTROL_MODE_CHARGE_DISCHARGE) ?
                     (int16_t)(I_cmd * 10.0f) : (int16_t)(V_cmd * 10.0f);
    tx_buffer[4] = (uint8_t)(param1 >> 8);
    tx_buffer[5] = (uint8_t)(param1 & 0xFF);

    // Param2, Param3 (동일 방식)
    // ...

    // Reserved
    tx_buffer[10] = 0x00;
    tx_buffer[11] = 0x00;
    tx_buffer[12] = 0x00;

    // Fault/Warning
    tx_buffer[13] = 0;
    if (over_voltage_flag) tx_buffer[13] |= 0x80;
    if (over_current_flag) tx_buffer[13] |= 0x40;
    if (over_temp_flag) tx_buffer[13] |= 0x20;
    if (scada_timeout_flag && (system_tick_1ms - scada_last_rx_time) > 200)
        tx_buffer[13] |= 0x10;  // SCADA 타임아웃 고장
    if (scada_timeout_flag && (system_tick_1ms - scada_last_rx_time) > 100)
        tx_buffer[13] |= 0x01;  // SCADA 타임아웃 경고

    // Checksum
    uint8_t checksum = 0;
    for (uint8_t i = 1; i <= 13; i++) {
        checksum += tx_buffer[i];
    }
    tx_buffer[14] = checksum & 0xFF;

    // ETX
    tx_buffer[15] = 0x03;

    // UART 전송
    for (uint8_t i = 0; i < 16; i++) {
        SCI_writeCharBlockingFIFO(SCID_BASE, tx_buffer[i]);
    }
}
```

#### 5. 슬레이브 배치 전송

```c
void Send_Slave_Batch_To_SCADA(uint8_t batch_index) {
    uint8_t tx_buffer[16];
    uint8_t start_idx = batch_index * 3;  // 0→0, 1→3

    // STX
    tx_buffer[0] = 0x02;

    // Control Status: Connection 비트맵 (bit3-1) + Data Type (bit0)
    tx_buffer[1] = 0x01;  // Data Type = 1
    for (uint8_t i = 0; i < 3; i++) {
        uint8_t list_idx = start_idx + i;
        if (list_idx < active_slave_count) {
            uint8_t slave_id = active_slave_list[list_idx];
            if (DAB_ok_slave[slave_id]) {
                tx_buffer[1] |= (1 << (i + 1));  // bit1~3: Connection
            }
        }
    }

    // 슬레이브 3개 데이터
    for (uint8_t i = 0; i < 3; i++) {
        uint8_t list_idx = start_idx + i;
        uint8_t base = 2 + i * 4;  // Byte offset: 2, 6, 10

        if (list_idx < active_slave_count) {
            uint8_t slave_id = active_slave_list[list_idx];

            // Fault Status (4비트 활용)
            uint8_t fault = slave_id & 0x0F;  // Slave ID

            // 고장 비트 설정
            if (V_out_slave[slave_id] >= 1400.0f) fault |= 0x40;  // OV
            if (fabsf(I_out_slave[slave_id]) >= 88.0f) fault |= 0x20;  // OC
            if (temp_slave[slave_id] >= 85.0f) fault |= 0x10;  // OT

            tx_buffer[base] = fault;

            // Current (Big-endian)
            int16_t current_raw = (int16_t)(I_out_slave[slave_id] * 10.0f);
            tx_buffer[base+1] = (uint8_t)(current_raw >> 8);
            tx_buffer[base+2] = (uint8_t)(current_raw & 0xFF);

            // Temp
            tx_buffer[base+3] = (uint8_t)(temp_slave[slave_id] * 2);
        } else {
            // 빈 슬롯
            tx_buffer[base] = 0x00;
            tx_buffer[base+1] = 0x00;
            tx_buffer[base+2] = 0x00;
            tx_buffer[base+3] = 0x00;
        }
    }

    // Checksum
    uint8_t checksum = 0;
    for (uint8_t i = 1; i <= 13; i++) {
        checksum += tx_buffer[i];
    }
    tx_buffer[14] = checksum & 0xFF;

    // ETX
    tx_buffer[15] = 0x03;

    // UART 전송
    for (uint8_t i = 0; i < 16; i++) {
        SCI_writeCharBlockingFIFO(SCID_BASE, tx_buffer[i]);
    }
}
```

#### 6. Active Slave List 관리

```c
uint8_t active_slave_list[6];   // 실제 연결된 슬레이브 ID
uint8_t active_slave_count = 0;

void Update_Active_Slave_List(void) {
    active_slave_count = 0;

    // ID 1~15 스캔 (최대 6개)
    for (uint8_t id = 1; id <= 15 && active_slave_count < 6; id++) {
        if (DAB_ok_slave[id] == 1) {
            active_slave_list[active_slave_count] = id;
            active_slave_count++;
        }
    }
}

// 메인 루프 100ms 또는 1초 주기 호출
if (flag_100ms) {
    Update_Active_Slave_List();
}
```

### SCADA 측 구현 (Python 예시)

#### 패킷 수신 및 파싱

```python
import serial
import struct
import time
import zlib

class PacketParser:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        self.last_tx_time = time.time()
        self.last_command = None
        self.keep_alive_interval = 0.1  # 100ms

    def receive_packet(self):
        """16바이트 패킷 수신"""
        # STX 탐색
        while True:
            b = self.ser.read(1)
            if len(b) == 0:
                return None
            if b[0] == 0x02:  # STX
                break

        # 나머지 15 bytes 수신
        data = self.ser.read(15)
        if len(data) != 15:
            return None

        packet = b + data

        # ETX 검증
        if packet[15] != 0x03:
            print(f"ETX error: expected 0x03, got {packet[15]:02X}")
            return None

        # Checksum 검증 (마스터→SCADA)
        checksum = sum(packet[1:14]) & 0xFF
        if checksum != packet[14]:
            print(f"Checksum error: calc={checksum}, recv={packet[14]}")
            return None

        return packet

    def parse_system_status(self, packet):
        """시스템 상태 패킷 파싱"""
        control_status = packet[1]
        data_type = control_status & 0x01

        if data_type != 0:
            return None  # Not system packet

        system_voltage = struct.unpack('>h', packet[2:4])[0] / 10.0
        param1 = struct.unpack('>h', packet[4:6])[0] / 10.0
        param2 = struct.unpack('>h', packet[6:8])[0] / 10.0
        param3 = struct.unpack('>h', packet[8:10])[0] / 10.0
        fault_warning = packet[13]

        return {
            'master_ch': (control_status >> 1) & 0x01,
            'run_status': (control_status >> 2) & 0x01,
            'precharge': (control_status >> 3) & 0x01,
            'parallel_mode': (control_status >> 4) & 0x01,
            'control_mode': (control_status >> 5) & 0x01,
            'system_voltage': system_voltage,
            'param1': param1,
            'param2': param2,
            'param3': param3,
            'fault': (fault_warning >> 4) & 0x0F,
            'warning': fault_warning & 0x0F
        }

    def parse_slave_status(self, packet):
        """슬레이브 상태 패킷 파싱 (3개)"""
        control_status = packet[1]
        data_type = control_status & 0x01

        if data_type != 1:
            return None  # Not slave packet

        slaves = []
        for i in range(3):
            # Connection 확인 (Control Status에서)
            connected = (control_status >> (i + 1)) & 0x01

            # Fault Status
            base = 2 + i * 4
            fault_status = packet[base]
            slave_id = fault_status & 0x0F
            op = (fault_status >> 7) & 0x01
            ov = (fault_status >> 6) & 0x01
            oc = (fault_status >> 5) & 0x01
            ot = (fault_status >> 4) & 0x01

            # Current, Temp
            current = struct.unpack('>h', packet[base+1:base+3])[0] / 10.0
            temp = packet[base+3] * 0.5

            slaves.append({
                'id': slave_id,
                'connected': connected,
                'over_power': op,
                'over_voltage': ov,
                'over_current': oc,
                'over_temp': ot,
                'current': current,
                'temp': temp
            })

        return slaves

    def send_command(self, cmd_byte, param1, param2, param3):
        """제어 지령 전송"""
        packet = bytearray(16)

        # STX
        packet[0] = 0x02

        # Command
        packet[1] = cmd_byte

        # Param1 (int16, Big-endian, ×10)
        param1_raw = int(param1 * 10)
        packet[2:4] = struct.pack('>h', param1_raw)

        # Param2, Param3 (동일)
        param2_raw = int(param2 * 10)
        packet[4:6] = struct.pack('>h', param2_raw)

        param3_raw = int(param3 * 10)
        packet[6:8] = struct.pack('>h', param3_raw)

        # Reserved
        packet[8] = 0x00
        packet[9] = 0x00

        # CRC-32 (Byte[1]~[10])
        crc = zlib.crc32(packet[1:11]) & 0xFFFFFFFF
        packet[11:15] = struct.pack('>I', crc)

        # ETX
        packet[15] = 0x03

        # 전송
        self.ser.write(packet)
        self.last_tx_time = time.time()
        self.last_command = packet

    def keep_alive_task(self):
        """Keep-Alive 작업 (메인 루프에서 호출)"""
        elapsed = time.time() - self.last_tx_time

        if elapsed > self.keep_alive_interval:
            if self.last_command:
                # 마지막 지령 재전송
                self.ser.write(self.last_command)
                self.last_tx_time = time.time()
                print(f"Keep-Alive: {elapsed:.3f}s 후 재전송")

# 사용 예시
if __name__ == '__main__':
    parser = PacketParser()

    # 초기 지령 전송
    parser.send_command(
        cmd_byte=0x20,  # Precharge Ready, Run
        param1=100.0,   # I_cmd = 100A
        param2=1200.0,  # V_max = 1200V
        param3=800.0    # V_min = 800V
    )

    # 메인 루프
    while True:
        # 패킷 수신
        packet = parser.receive_packet()
        if packet:
            system = parser.parse_system_status(packet)
            if system:
                print(f"System: {system}")
            else:
                slaves = parser.parse_slave_status(packet)
                if slaves:
                    print(f"Slaves: {slaves}")

        # Keep-Alive 체크
        parser.keep_alive_task()

        time.sleep(0.01)  # 10ms
```

---

## 변경 이력

### Rev 5.0 (2025-10-27)

**주요 변경사항:**

1. **프레이밍 방식**
   - STX/ETX 유지 (0x02/0x03, 검증된 산업 표준)
   - 고정 길이 16 bytes 프로토콜
   - 바이트스터핑 불필요 (고정 길이)

2. **보드레이트 결정**
   - 115200 bps 채택 (산업 표준, 신뢰성 우선)

3. **패킷 구조 표준화**
   - 모든 패킷 16 bytes 고정
   - Big-endian, int16 ÷10 스케일 통일
   - Fault/Warning 비트 추가 (시스템 상태 패킷)

4. **전송 스케줄 최적화**
   - 100ms 토글 구조: 시스템 ↔ 슬레이브 교대
   - 시스템 상태: 100ms 주기
   - 슬레이브 전체: 200ms 스캔 (6개, 배치 전송)

5. **배치 전송 도입**
   - 슬레이브 3개씩 1패킷에 묶음
   - 6개 슬레이브 → 2패킷 (연속 전송)
   - Reserved 바이트 0개 (완벽 활용)

6. **연결 감시 메커니즘**
   - Passive Watchdog: 마스터가 SCADA 수신 감시
   - 타임아웃: 100ms 경고, 200ms 안전 정지
   - SCADA Keep-Alive: 100ms 주기 재전송
   - 자동 안전 정지로 크래시 대응

7. **동적 슬레이브 감지**
   - Active Slave List 방식
   - 슬레이브 추가/제거 자동 감지
   - Connection 비트로 연결 상태 표시
   - ID 1~15 중 임의 6개 지원

8. **성능 개선**
   - CPU 부하 10배 감소 (10ms → 100ms 주기)
   - 전송 대역폭 77% 감소 (효율 향상)
   - 최대 블로킹 시간 2.78ms (ISR 영향 없음)

### Rev 4.0 (이전 버전)

- 보드레이트: 115200 bps
- 프레이밍: STX (0x02) / ETX (0x03)
- 가변 길이 패킷 (7 bytes, 13 bytes)
- 슬레이브 개별 전송 (10ms × 15 = 150ms)
- 시스템 상태: 50ms 주기
- 연결 감시: 없음

---

## 부록

### A. 데이터 타입 참조

| 타입 | 크기 | 범위 | 용도 |
|------|------|------|------|
| uint8 | 1 byte | 0 ~ 255 | Status, Checksum, Temp |
| int16 | 2 bytes | -32768 ~ 32767 | Voltage, Current (×10 스케일) |
| uint32 | 4 bytes | 0 ~ 4294967295 | CRC-32 |

### B. 스케일 참조

| 파라미터 | 타입 | 스케일 | 예시 |
|---------|------|--------|------|
| 전압 | int16 | ÷10 | 1200V → 12000 → 0x2EE0 |
| 전류 | int16 | ÷10 | 80.5A → 805 → 0x0325 |
| 온도 | uint8 | ×0.5 | 42.5°C → 85 → 0x55 |

### C. 오류 코드

| 코드 | 설명 | 처리 |
|------|------|------|
| Checksum 오류 | 합산 불일치 (Master→SCADA) | 패킷 폐기, 재동기화 |
| CRC-32 오류 | CRC 불일치 (SCADA→Master) | 패킷 폐기, 이전 상태 유지 |
| ETX 오류 | 0x03 아님 | 재동기화 |
| SCADA 타임아웃 | 200ms 무응답 | 안전 정지 |

### D. 용어 정리

| 용어 | 설명 |
|------|------|
| Keep-Alive | 연결 유지를 위한 주기적 패킷 재전송 |
| Watchdog | 타임아웃 감시 메커니즘 |
| STX/ETX | Start/End of Text (ASCII 제어 문자) |
| Big-endian | 상위 바이트를 먼저 전송하는 바이트 오더 |
| Active Slave List | 실제 연결된 슬레이브만 관리하는 동적 리스트 |
| Batch Transfer | 여러 데이터를 하나의 패킷으로 묶어 전송 |

---

**문서 버전**: Rev 5.0
**작성일**: 2025-10-27
**작성자**: Doug Shin
