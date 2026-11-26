# XGO Dog Communication API

[English README](../README.md) · [简体中文 API](api.md)

The firmware provides a legacy register protocol and a main/sub-address protocol. Both can be used on the same connection and operate on the same robot state.

Register addresses are hexadecimal. Main addresses, sub-addresses, and device IDs are decimal unless stated otherwise. Types: `u8` is an unsigned 8-bit integer, `f32` is IEEE 754 single precision, and `i32` is a signed 32-bit integer.

## 1. Connection

### 1.1 UART

| Setting | Value |
| --- | --- |
| Default serial settings | 115200 baud, 8 data bits, no parity, 1 stop bit |
| Host UARTs | UART0 and UART1. ESP32 UART1 RX/TX use GPIO4/GPIO5 |
| Baud index | `0`=115200, `1`=500000, `2`=921600, `3`=1000000, `4`=1500000, `5`=2000000 |

A baud command changes only the receiving UART. Switch the host to the new rate after sending the command. Reboot restores the default. If the FDI serial IMU occupies UART0, use UART1 or BLE.

### 1.2 BLE

| Item | UUID or behavior |
| --- | --- |
| Service | `0000fff0-0000-1000-8000-00805f9b34fb` |
| Host writes | `0000fff2-0000-1000-8000-00805f9b34fb`, Write |
| Host receives | `0000fff1-0000-1000-8000-00805f9b34fb`, Notify |
| Advertised name | `XGO-M-`, `XGO-MW-`, or `XGO-L-`, followed by the stored suffix |

Subscribe to notifications before sending requests. BLE splits traffic into chunks of up to `MTU - 3` bytes. Reassemble the byte stream into complete frames. A query reply goes back over the connection that received the request.

## 2. Frame format and data conventions

### 2.1 Requests and query replies

```text
55 00 LEN CMD ADDR DATA... CHECKSUM 00 AA
```

| Field | Bytes | Meaning |
| --- | ---: | --- |
| Header | 2 | `55 00` |
| `LEN` | 1 | Total frame size, `8 + len(DATA)`, from 8 to 255 |
| `CMD` | 1 | Command type |
| `ADDR` | 1 | Register or main address |
| `DATA` | 0–247 | Command data |
| `CHECKSUM` | 1 | `~(LEN + CMD + ADDR + sum(DATA)) & 0xFF` |
| Trailer | 2 | `00 AA` |

The checksum is the one's complement of the low eight bits of the sum. Frames with an invalid checksum or trailer are discarded.

| `CMD` | Operation | Reply |
| --- | --- | --- |
| `0x00` / `0x01` | Legacy register write | No ACK |
| `0x02` | Legacy register read | `0x12`, same `ADDR` |
| `0x03` | Main/sub-address write | No ACK |
| `0x04` | Main/sub-address read | `0x14`, same `ADDR` |

Multibyte fields are little-endian unless specified. Legacy heading offset `0x3F`, integer attitude `0x66–0x68`, and extension servo position `0xA1–0xA9` are big-endian.

Writes have no acknowledgement or separate error code. Query a readable field to confirm its value. Reads return the latest received device state or the current control value. They do not trigger synchronous motor sampling and carry no per-motor timestamps.

Motion commands persist until changed, including after a communication loss. The shutdown sequence or sustained undervoltage stops accepting writes while reads remain available.

### 2.2 Coordinates and units

| Field | Unit or meaning |
| --- | --- |
| Body, foot, and arm position | mm. Body Z is absolute height |
| Body RPY, servo angles, heading | Degrees. IMU yaw is continuous and may exceed ±180° |
| IMU acceleration | m/s², including gravity, in the axes reported by the active driver |
| IMU angular rate | °/s after startup bias removal for I²C IMUs. ICM42670 output also has a deadband below 2°/s |
| Measured wheel linear speed | m/s, converted using a 22 mm wheel radius |
| Wheel odometry | Signed cumulative counts. The motor position value is divided by 16 and adjusted for installation direction |
| `vx` / `vy` | Internal stride control values. The gait uses them on a millimetre scale |
| `yaw_speed` | Internal turn command. It is accumulated once per nominal 5 ms control step for leg heading, and mapped to steering in wheel mode |

Send finite floats. Target poses, joint angles, and motor outputs are constrained by the model, kinematic reach, and output limits.

Battery percentage is estimated from 0 to 100. It holds during commanded motion, actions, and the 600 ms Reset recovery. Updating resumes once commands have been idle for at least 1 s and the voltage spread over the last 300 ms is at most 0.10 V. The idle check uses command and target changes. Voltage sampling and protection continue independently. Sustained mean pack voltage at or below 6.84 V for Mini / wheel-legged Mini or 6.70 V for Lite triggers reset, crouch, torque release, and shutdown after about 4 s.

### 2.3 Model ranges

The legacy scale and main/sub-address pose limits are listed below. Firmware selects the wheel-legged Mini profile when it detects wheels.

| Parameter | Mini | Wheel-legged Mini | Lite |
| --- | ---: | ---: | ---: |
| Body X / mm | −30 to 30 | −30 to 30 | −20 to 20 |
| Body Y / mm | −18 to 18 | −16.2 to 16.2 | −16 to 16 |
| Body Z / mm | 70 to 120 | 70 to 114 | 65 to 110 |
| Default standing height / mm | 100 | 100 | 90 |
| Roll / Pitch / Yaw / ° | −18 to 18 | −18 to 18 | −15 to 15 |
| `vx` command | −20 to 30 | −20 to 25 | −14.3 to 22 |
| `vy` command | −20 to 20 | −20 to 20 | −12.5 to 12.5 |
| `yaw_speed` command | −0.60 to 0.60 | −0.60 to 0.60 | −0.60 to 0.60 |

Legacy scaling:

```text
Write: value = byte / 255 × (maximum − minimum) + minimum
Read:  byte = trunc((value − minimum) / (maximum − minimum) × 255), clamped to 0–255
```

For Mini and wheel-legged Mini, `vx` (`0x30`) uses `0x80` as zero. Bytes `0x00–0x80` map linearly to −20 to 0. Bytes `0x80–0xFF` map linearly to 0 to 30 on Mini and 0 to 25 on wheel-legged Mini. Reads use the same segments and round to the nearest byte. Small commands still pass through the motion deadband.

On Lite in Trot, full-scale `vx` gives a translation half-stride of ±20 mm at high speed and ±25 mm at normal and low speeds. Full-scale `vy` gives ±16 mm. The turning stride is added separately.

`yaw_speed` is the angular increment per nominal 5 ms control step. Full scale corresponds to a target heading rate of ±120°/s.

Only the listed block registers support writing multiple addresses in one request. Standard single-byte registers use the first data byte.

## 3. Legacy register protocol

### 3.1 Writes

Send `CMD=0x01` or `0x00` with the register in `ADDR` and the encoding below in `DATA`.

| Address | `DATA` | Function |
| --- | --- | --- |
| `0x02` | `baud_index:u8` | Set the receiving UART baud index, 0–5 |
| `0x03` | `enabled:u8` | Action loop. 0 disables and resets, nonzero enables the built-in loop |
| `0x04` | `enabled:u8` | Servo zero calibration. Nonzero enters calibration, 0 saves and restarts if calibration is active |
| `0x06` | `01` | Set the heading origin to current IMU yaw |
| `0x08` | `enabled:u8` | 1 enables streaming, other values disable it. Selects the receiving port for feedback |
| `0x09` | `gait:u8` | Gait type from the enumeration |
| `0x13` | `name_bytes[1..30]` | Store the BLE name suffix without a length prefix. Applied with the model prefix after reboot |
| `0x20` | See enable formats | Enable or release servos. Enters direct joint control |
| `0x30` | `vx:u8` | Scaled X motion command |
| `0x31` | `vy:u8` | Scaled Y motion command |
| `0x32` | `yaw_speed:u8` | Scaled turn command |
| `0x33` / `0x34` / `0x35` | `value:u8` | Scaled body X / Y / Z |
| `0x36` / `0x37` / `0x38` | `value:u8` | Scaled body Roll / Pitch / Yaw |
| `0x39` / `0x3A` / `0x3B` | `period:u8` | Body Roll / Pitch / Yaw oscillation period parameter |
| `0x3D` | `preset:u8` | Gait timing preset. 0 normal, 1 slow, 2 fast |
| `0x3E` | `action:u8` | Action ID. 255 resets and stops the action loop |
| `0x3F` | `offset:i16` big-endian | Target heading is `heading_origin + offset`. Enables heading tracking |
| `0x40–0x4B` | `position:u8` or byte block | Scaled foot X/Y/Z for the four legs. Enters direct foot control |
| `0x50–0x5B` | `angle:u8` or byte block | Angles scaled to each of the 12 leg servo ranges. Enters direct joint control |
| `0x5C` | `speed:u8` | Leg servo speed parameter, 0–255. 0 removes the speed limit |
| `0x5D` / `0x5E` | `angle:u8` | Scaled angles for IDs 52 / 53. Switches to arm joint control |
| `0x5F` | `01` | Run the servo centering sequence |
| `0x61` | `mode:u8` | IMU stabilization. 0 disables and clears body RPY, 1 enables, 2 keeps the manual pose branch |
| `0x69–0x6C` | `R:u8 G:u8 B:u8` | Write one logical RGB channel, 1–4 in address order |
| `0x69–0x6C` | `R1 G1 B1 ... R4 G4 B4` | Write all four logical RGB channels when at least 12 bytes are supplied |
| `0x70` | `angle:u8` | Scaled turntable angle, servo ID 54 |
| `0x71` | `angle:u8` | Scaled gripper angle, servo ID 51 |
| `0x72` | `mode:u8` | Arm coordinate mode. 0 follows the body, 1 compensates for body position and pitch |
| `0x73` / `0x74` | `position:u8` | Arm X / Z, scaled to −80 to 160 mm / −100 to 160 mm. Enters coordinate control |
| `0x75` | `speed:u8` | Arm servo speed parameter, 0–255. 0 removes the speed limit |
| `0x76` | `theta:u8` | Polar angle, scaled to 70–270°. Recomputes arm X/Z using the current radius |
| `0x77` | `radius:u8` | Polar radius, scaled to 80–140 mm. Recomputes arm X/Z using the current angle |
| `0x80` / `0x81` / `0x82` | `period:u8` | Body X / Y / Z oscillation period parameter |
| `0x90` / `0x91` | `state:u8` | Extension outputs GPIO26 / GPIO27. 0 low, 1 high |
| `0xA1–0xA9` | `position:u16` big-endian | Raw position of one extension servo, IDs 91–99 |
| `0xB0` | `mode:u8` | 0 hybrid, 1 wheel drive with leg steering, 2 leg / independent wheel, 3 wheel mode. Clears `vx`, `vy`, internal `vyaw`, and four wheel commands while keeping `yaw_speed` |
| `0xB1` | `w10:u8 w20:u8 w30:u8 w40:u8` | Independent wheel drive levels, at least four bytes. Selects mode 2 |
| `0xF0` | `mode:u8` | Factory exercise. 0 off, 1 pose cycle, 2 motion and pose cycle |

Writing `0x30–0x32` selects whole-body control and disables IMU stabilization. Small motion commands fall within a deadband. Changing one axis retains the other axes. To stop, clear all motion axes and independent wheel inputs. The board refreshes the first three physical RGB pixels.

#### Servo enable formats for `0x20`

| Format | Data | Meaning |
| --- | --- | --- |
| All servos | `00` / `01` | `00` enables, `01` releases torque |
| Servo groups | `11–15` / `21–25` | Low nibble is the group, `1x` releases torque and `2x` enables |
| Individual IDs | `[id, disabled]...` | One or more two-byte items. 0 enables, nonzero releases torque |

Groups 1–4 map to IDs 11–13, 21–23, 31–33, and 41–43. Group 5 maps to IDs 51–54. The per-ID byte is `disabled` here and `enabled` in the main/sub-address protocol.

#### Contiguous foot and leg joint blocks

| Leg group | Foot X/Y/Z addresses | Servo addresses | Servo IDs |
| --- | --- | --- | --- |
| 1 | `40 41 42` | `50 51 52` | 11, 12, 13 |
| 2 | `43 44 45` | `53 54 55` | 21, 22, 23 |
| 3 | `46 47 48` | `56 57 58` | 31, 32, 33 |
| 4 | `49 4A 4B` | `59 5A 5B` | 41, 42, 43 |

Addresses are hexadecimal. Writes can start at any address in a block and stop at the end of that block. The foot Y input is a lateral adjustment. Firmware subtracts the corresponding mounting offset.

#### Wheel drive levels for `0xB1`

`0x00` is full reverse, `0x80` is stop, and `0xFF` is full forward. The bytes correspond to IDs 10, 20, 30, and 40. Firmware accounts for installation direction.

```text
d = byte − 128
drive = trunc(230 × d / (d > 0 ? 127 : 128))
```

Drive commands range from −230 to 230. Read `0xB1` for measured speed in m/s.

### 3.2 Reads

Send `CMD=0x02` and `DATA=[N]`, where `N` is the requested number of bytes. The response uses `CMD=0x12` and returns the value in `DATA`.

| Address | Request `N` | Response |
| --- | ---: | --- |
| `0x00` | 1 | `00` |
| `0x01` | 1 | Battery percentage, `u8` |
| `0x02` | 1 | Fixed `01` across models |
| `0x07` | 10 | Firmware version, 10 ASCII bytes with zero padding when needed |
| `0x30–0x38` | 1 | Current body control value encoded as a scaled `u8` |
| `0x3F` | 1 | Heading tracking status, `u8` |
| `0x50` | 1 | Measured ID 11 angle, scaled to `u8` |
| `0x50` | 16 | All 16 measured servo angles as `u8` in the listed ID order |
| `0x51–0x5B` | 1 | Corresponding measured leg angle, scaled to `u8` |
| `0x5D` / `0x5E` / `0x70` / `0x71` | 1 | Measured ID 52 / 53 / 54 / 51 angle, scaled to `u8` |
| `0x61` | 1 | Current IMU stabilization mode, `u8` |
| `0x62` / `0x63` / `0x64` | 4 | IMU Roll / Pitch / Yaw, little-endian `f32` |
| `0x66` / `0x67` / `0x68` | 2 | IMU Roll / Pitch / Yaw truncated toward zero, big-endian `i16` |
| `0x92` | 1 | Quantized GPIO36 analog input, `u8`, 0–255 |
| `0x93` | 1 | GPIO39 digital input, `u8`, 0 or 1 |
| `0xB1` | 16 | Measured linear speed of IDs 10, 20, 30, and 40, four little-endian `f32` values |
| `0xB2` | 16 | Cumulative odometry counts of IDs 10, 20, 30, and 40, four little-endian `i32` values |

For `0x50`, any request length other than 1 returns all 16 angles. Request 16 for this operation. Use the reply's actual length for fixed-length fields. Unknown addresses can return stale response-buffer data and cannot be used for capability detection. Read single-byte fields with `N=1`.


### 3.3 Shutdown feedback

When a long power-button press or sustained undervoltage starts shutdown, the firmware commands the robot to lie down. It sends the first frame about 300 ms later, then sends one frame on each of UART0 and UART1 about every 300 ms until power is cut. The feedback is automatic and requires no query or telemetry subscription.

| `CMD` | `ADDR` | `DATA` | Complete frame |
| --- | --- | --- | --- |
| `0x12` | `0x00` | `FF` | `55 00 09 12 00 FF E5 00 AA` |

On models with an FDI serial IMU, receive this feedback on UART1. UART0 keeps the FDI pin assignment and baud rate.

## 4. Main/sub-address protocol

### 4.1 Subfield encoding

To write, send `CMD=0x03` and `ADDR=main`. Each frame addresses one group.

| Field type | `DATA` format |
| --- | --- |
| Float | `[sub:u8, value:f32 little-endian]...`, five bytes per item |
| Byte | `[sub:u8, value:u8]...`, two bytes per item |
| Custom | See the interface, such as servo enable, BLE name, or RGB |

Omitted float fields keep their current values. The last occurrence of a duplicate sub-address in the same frame wins.

To read, send `CMD=0x04`, `ADDR=main`, and `DATA=[sub1,sub2,...]`. The `0x14` reply lists supported fields in request order as `sub + f32` or `sub + u8`. The firmware version has its own length-prefixed format. Unknown sub-addresses are skipped. Unsupported groups, empty requests, and requests with no known sub-addresses receive no reply. Reply data is limited to 240 bytes.

### 4.2 Status and connection

| Main | Access | Sub-address or format | Function |
| --- | --- | --- | --- |
| 1 | R | Read `[1]`, reply `[1,10,version[10]]` | Firmware version, 10 ASCII bytes with zero padding when needed |
| 2 | R | Read `[1]`, reply `[1,battery:u8]` | Battery percentage |
| 3 | W | Exactly `[1,enabled:u8]` | 1 enables streaming, other values disable it. Selects the receiving port for feedback |
| 4 | W | Exactly `[1,enabled:u8]` | Servo zero calibration, as in legacy `0x04` |
| 5 | W | Exactly `[1,baud_index:u8]` | Set receiving UART baud index, 0–5 |
| 6 | W | `[1,length:u8,name[length]]` | Store the BLE name suffix. Length is 1–30 and must match the data |
| 7 | W | Exactly `[1,mode:u8]` | Factory exercise. 0 off, 1 pose cycle, 2 motion and pose cycle |

Writes to groups 1 and 2 have no effect. Groups 3–7 have no read interface.

### 4.3 Whole-body control

| Main | Access | Sub-address | Type | Function |
| --- | --- | --- | --- | --- |
| 10 | R/W | 1, 2, 3, 4, 5, 6 | `f32` | Body X, Y, Z, Roll, Pitch, Yaw |
| 11 | R/W | 1, 2, 3, 4, 5, 6 | `f32` | Oscillation period parameters for body X, Y, Z, Roll, Pitch, Yaw, each 0–255 |
| 20 | R/W | 1, 2, 3 | `f32` | `vx`, `vy`, `yaw_speed` motion commands. Reads return current control values |
| 21 | W | 1 | `u8` | Action ID. 255 resets and stops the loop |
| 21 | W | 2 | `u8` | Action loop. 0 disables and resets, nonzero enables |
| 22 | W | 1 | `f32` | Target heading is origin plus offset. Enables heading tracking and disables IMU stabilization |
| 23 | W | 1 | `u8` | Gait timing preset. 0 normal, 1 slow, 2 fast |
| 23 | W | 2 | `u8` | Gait type from the enumeration |
| 24 | W | 1 | `u8` | Set heading origin to current IMU yaw with exactly `[1,1]` |

Writing 10, 11, 20, or 22 selects whole-body control. Writing 20 also disables IMU stabilization and applies deadbands to all motion axes. Groups 21 and 23 accept multiple byte items in one frame. Groups 22–24 have no read interface.

When all motion inputs return to their deadbands, the firmware clears the relative heading reference. The current swing finishes, and the next motion uses the valid IMU heading at restart as its reference. Releasing only the turn input during translation keeps the accumulated target heading. Explicit heading commands follow the rules of the selected mode.

```text
period = 0: stop after the current oscillation cycle
period > 0: cycle in milliseconds = 1000 + period / 255 × 3000
```

X/Y and RPY amplitudes are 80% of the model maximum. Z oscillates around the midpoint of the height range with an amplitude of 80% of the half-range. Action 255 clears oscillations immediately and returns the robot to its standing pose.

### 4.4 Feet, servos, and arm

| Main | Access | Sub-address or format | Function |
| --- | --- | --- | --- |
| 30 | R/W | `1..12:f32`, X/Y/Z for each leg in order | Base foot positions. Writing selects direct foot control |
| 40 | R/W | Device ID, angle as `f32` | Write target angles or read measured angles. Writing selects direct joint control |
| 40 | R | Exactly `[255]` | Return all 16 built-in servos as `[id,angle:f32]`, excluding extension servos |
| 41 | W | `[1,id:u8,enabled:u8]...` | Per-ID torque enable. 0 releases torque, 1 enables |
| 41 | W | `[2,group_code:u8]...` | Group enable codes as in legacy `0x20` |
| 43 | W | Exactly `[1,speed:u8]` | Leg servo speed parameter, 0–255. 0 removes the speed limit |
| 44 | W | `[1,1]` | Run the servo centering sequence |
| 51 | R/W | Sub-address 1. Write `u8`, read `f32` | Write gripper percentage from 0 to 100 across its angle range. Higher values saturate at 100. Read measured ID 51 angle in degrees |
| 52 | R/W | `1:f32`, `2:f32`, `3:f32` | Write target angles for IDs 52 / 53 / 54 and enter arm joint control. Read measured angles |
| 53 | R/W | All `f32`, see table below | Arm coordinate control and turntable |
| 55 | R/W | Sub-address 1, `u8` | 0 follows body, 1 compensates body position and pitch. Write exactly two bytes |
| 56 | W | Exactly `[1,speed:u8]` | Arm servo speed parameter, 0–255. 0 removes the speed limit |

Main 30 subtracts the leg mounting offset from written Y. Reads return the internal base Y.

```text
internal Y = written Y − sign × Ld
sign for leg groups 1..4 = −1, +1, +1, −1
Ld = 31.6 mm for Mini and 28.1 mm for Lite
```

In whole-body control, default foot base Y is ±31.6 mm for Mini and ±21.6 mm for wheel-legged Mini. Direct foot Y writes still use the mounting-offset conversion above.

Main 40 supports built-in IDs and extension IDs 91–100. The firmware's normal polling does not query extension servos. A read returns the value held by the firmware. Only a standalone `[255]` expands to all built-in servos. Writing IDs 52 or 53 disables arm coordinate control. Main 41 allows individual and group items in one frame.

#### Arm coordinates at main address 53

| Sub | Write | Read | Unit or range |
| --- | --- | --- | --- |
| 1 | Target X | Current X command | mm, −80 to 160 recommended |
| 2 | Target Z | Current Z command | mm, −100 to 160 recommended |
| 3 | Polar radius r | Current r command | mm, subject to arm reach |
| 4 | Polar angle θ | Current θ command | **Radians** |
| 5 | Target angle of ID 54 | Measured angle of ID 54 | Degrees |

Writing any subfield selects arm coordinate control. Including 3 or 4 recalculates and clamps X/Z, overriding any X/Z values in that frame. The formulas are `X = −r cos(θ)` and `Z = r sin(θ)`. Writing only X or Z retains the other coordinate.

### 4.5 IMU, I/O, and wheels

| Main | Access | Sub-address or format | Function |
| --- | --- | --- | --- |
| 60 | R | `1..9:f32` | 1–3 Roll/Pitch/Yaw, 4–6 AccX/AccY/AccZ, 7–9 GyroX/GyroY/GyroZ |
| 61 | W | Exactly `[1,mode:u8]` | IMU stabilization mode 0/1/2, as in legacy `0x61` |
| 62 | W | `[channel:u8,R:u8,G:u8,B:u8]...` | RGB channels 1–4. Accepts multiple four-byte items |
| 63 | R/W | 1 and 2, `u8` | Write GPIO26 / GPIO27 outputs. Read GPIO36 analog / GPIO39 digital inputs |
| 70 | W | Exactly `[1,mode:u8]` | 0 hybrid, 1 wheel drive with leg steering, 2 leg / independent wheel. Clears `vx`, `vy`, internal `vyaw`, and four wheel commands while keeping `yaw_speed` |
| 71 | R/W | `1..4:f32` | Write independent drive inputs for IDs 10, 20, 30, and 40. Read measured linear speed in m/s |
| 128 | R/W | `1..10:f32` | Ten user debug values with no default control effect |

Use 0 or 1 for GPIO writes. Main 63 reads the analog input as 0–255 and the digital input as 0/1. It does not read back output pins.

The FDI serial IMU updates Roll/Pitch/Yaw only. Its acceleration and gyro fields do not represent valid samples.

Main 71 applies the installation sign and multiplies the input by 100, then clamps the output to −230 through 230. Inputs with magnitude below 0.05 are zeroed. ±2.3 maps to ±230. These are drive commands. Writing selects mode 2 and retains commands for wheels omitted from the frame. If the robot is in direct foot or joint control, first send zero motion on all axes through main 20 to return to whole-body control, then write main 71 or legacy `0xB1`.

Select wheel mode 3 using legacy `0xB0`. Main 70 accepts only 0–2. In mode 3 the wheels handle forward and reverse motion. When `|vy| > 1`, legs provide lateral movement without adding fore/aft stride from wheel drive. The legs use the selected gait and return to neutral through its stop sequence.

Mode 3 considers the robot commanded to move when `|vx| > 5`, `|vy| > 1`, or `|yaw_speed| > 0.1`. With valid IMU data, target yaw starts at current yaw and integrates the turn command by actual control interval, including turns in place.

```text
target_yaw += yaw_speed × dt / 0.005s
error = target_yaw − current_IMU_yaw
```

- Up to 1° of absolute error, heading correction is zero.
- Above 1° and up to 10°, wheel steering is `0.05 × error`.
- Above 10°, wheel correction is zero and leg steering uses `0.33 × error`, subject to the model limit. Within 10° the wheels resume correction and the legs finish their step unless lateral movement is still commanded.
- The wheel outputs are open loop. Forward drive leaves room for the steering component.

  ```text
  turn = clamp(40 × wheel_steering, −230, 230)
  driveLimit = 230 − |turn|
  drive = clamp(20 × vx, −driveLimit, driveLimit)
  order[i] = mounting_sign[i] × drive − turn
  ```

- Returning every motion command to its deadband clears the heading reference and wheel output. Releasing steering alone during translation retains the accumulated target heading.
- When IMU data is stale, invalid, or unavailable, heading integration and correction pause. They re-anchor to current yaw when valid data returns. Wheel drive and leg lateral movement remain available.
- Independent heading tracking through legacy `0x3F` or main 22 is unavailable in mode 3.

## 5. IDs, joint limits, and enumerations

### 5.1 Servo IDs and angles

The order of all built-in servo feedback is fixed. These are decimal device IDs.

```text
11 12 13  21 22 23  31 32 33  41 42 43  51 52 53 54
```

| Device | Mini range / ° | Lite range / ° |
| --- | ---: | ---: |
| First servo in each leg, IDs 11/21/31/41 | −70 to 60 | −70 to 120 |
| Second servo in each leg, IDs 12/22/32/42 | −70 to 120 | −85 to 95 |
| Third servo in each leg, IDs 13/23/33/43 | −35 to 35 | −40 to 55 |
| Gripper, ID 51 | −80 to 80 | −70 to 70 |
| Arm, ID 52 | −55 to 96 | −70 to 75 |
| Arm, ID 53 | −93 to 86 | −93 to 75 |
| Turntable, ID 54 | −180 to 180 | −180 to 180 |

### 5.2 Gait types and timing

| Value | Gait |
| ---: | --- |
| 0 | Trot, diagonal pairs |
| 1 | Walk, one leg at a time |
| 2 | High walk |
| 3 | Low trot |
| 255 | Mark time. Steps in place even with zero motion commands |

| Timing preset | Mini / wheel-legged Mini | Lite |
| ---: | ---: | ---: |
| 0 normal | 0.42 s | 0.41 s |
| 1 slow | 0.52 s | 0.86 s |
| 2 fast | 0.37 s | 0.35 s |

Walk uses a 1.40 s four-leg cycle. Low trot uses a 0.50 s cycle on Mini and a 0.57 s cycle on Lite. Lite High walk uses a fixed 3.20 s cycle. Only 0–2 are valid timing presets.

### 5.3 Action IDs

| ID | Action | ID | Action |
| ---: | --- | ---: | --- |
| 1 | Lie down | 2 | Stand up |
| 3 | Crawl | 4 | Turn around |
| 6 | Squat | 7 | Roll oscillation |
| 8 | Pitch oscillation | 9 | Yaw oscillation |
| 10 | Combined RPY oscillation | 11 | Lift leg |
| 12 | Sit | 13 | Wave |
| 14 | Stretch | 15 | Rock forward and back |
| 16 | Rock side to side | 17 | Ask for food |
| 18 | Find food | 19 | Shake hands |
| 20 | Show arm | 21 | Push-up |
| 22 | Combined pitch/yaw | 23 | Combined Z/yaw |
| 24 | Combined X/roll | 25 | Arm swing |
| 26 | Arm wave | 27 | Arm dance |
| 29 | Standing action | 128 / `0x80` | High grasp |
| 129 / `0x81` | Middle grasp | 130 / `0x82` | Low grasp |
| 131 / `0x83` | Low side grasp | 132 / `0x84` | Side grasp |
| 255 / `0xFF` | Reset and stop action loop | | |

ID 0 clears action state while retaining current motion, oscillations, and pose. IDs 5, 28, and other unlisted values have no action definition. An action starts with about 600 ms of standing recovery. On completion or reset, leg and arm servo speed parameters are set to 40 on the 0–255 scale. A zero motion command retains that limit. Starting locomotion removes the leg speed limit. The action loop runs the built-in sequence, including motion actions.

## 6. Active feedback

Enable the stream with legacy `0x08` or main address 3. The last command that selects a feedback port sets the destination. Only one port streams at a time.

```text
55 00 PAYLOAD[250] 00 AA
```

The frame is exactly **254 bytes** and has no `LEN`, `CMD`, `ADDR`, or checksum. Both offset columns are zero-based. Multibyte values are little-endian, and reserved bytes are zero.

| Packet offset | Payload offset | Bytes | Type | Field |
| ---: | ---: | ---: | --- | --- |
| 2 | 0 | 1 | `u8` | Battery percentage |
| 3 | 1 | 3 | Reserved | 0 |
| 6 | 4 | 4 | `f32` | X motion command scaled by 1/74 |
| 10 | 8 | 4 | `f32` | Y motion command |
| 14 | 12 | 4 | `f32` | Body odometry X, currently zero |
| 18 | 16 | 4 | `f32` | Body odometry Y, currently zero |
| 22 | 20 | 12 | `f32[3]` | IMU Roll, Pitch, Yaw |
| 34 | 32 | 12 | `f32[3]` | IMU AccX, AccY, AccZ |
| 46 | 44 | 12 | `f32[3]` | IMU GyroX, GyroY, GyroZ |
| 58 | 56 | 64 | `f32[16]` | Measured built-in servo angles in degrees, in ID order |
| 122 | 120 | 64 | `f32[16]` | Servo velocity fields, internal `fb_vel × 360 / 4096`. Units are not uniform across servo families |
| 186 | 184 | 16 | `f32[4]` | Measured wheel speed in m/s for IDs 10, 20, 30, and 40 |
| 202 | 200 | 16 | `i32[4]` | Cumulative wheel odometry counts in the same order |
| 218 | 216 | 34 | Reserved | 0 |

The target interval is 20 ms. A busy UART can delay a transmission. Sending 254 bytes at 115200 baud takes about 22 ms, so use a higher baud rate for faster streaming. The receive rate does not give the motor sampling rate. BLE clients reassemble notifications into complete 254-byte frames.

## 7. Examples

| Operation | Complete request |
| --- | --- |
| Read version via legacy register | `55 00 09 02 07 0A E3 00 AA` |
| Set the receiving UART to 500000 baud | `55 00 09 01 02 01 F2 00 AA` |
| Enable active feedback via legacy register | `55 00 09 01 08 01 EC 00 AA` |
| Disable active feedback via legacy register | `55 00 09 01 08 00 ED 00 AA` |
| Stop four wheels via legacy register | `55 00 0C 01 B1 80 80 80 80 41 00 AA` |
| Read IMU RPY via main address 60 | `55 00 0B 04 3C 01 02 03 AE 00 AA` |
| Read all built-in servos via main address 40 | `55 00 09 04 28 FF CB 00 AA` |

Python framing and decoding:

```python
import struct

def frame(command: int, address: int, data: bytes = b"") -> bytes:
    if len(data) > 247:
        raise ValueError("payload exceeds 247 bytes")
    body = bytes([8 + len(data), command, address]) + data
    checksum = (~sum(body)) & 0xFF
    return b"\x55\x00" + body + bytes([checksum, 0x00, 0xAA])

def float_fields(values: dict[int, float]) -> bytes:
    return b"".join(bytes([sub]) + struct.pack("<f", value)
                    for sub, value in values.items())

# Reads send only the list of sub-addresses.
read_imu = frame(0x04, 60, bytes([1, 2, 3]))
read_motors = frame(0x04, 40, bytes([0xFF]))

# Float writes use sub-address plus little-endian float32.
set_pose = frame(0x03, 10, float_fields({1: 0.0, 2: 0.0, 3: 100.0}))
stop_body = frame(0x03, 20, float_fields({1: 0.0, 2: 0.0, 3: 0.0}))
stop_wheels = frame(0x01, 0xB1, bytes([0x80] * 4))

# Decode the DATA of a validated 0x14 float reply as five-byte items.
def decode_float_fields(data: bytes) -> dict[int, float]:
    if len(data) % 5:
        raise ValueError("invalid float field payload")
    return {data[i]: struct.unpack_from("<f", data, i + 1)[0]
            for i in range(0, len(data), 5)}
```

Reassemble and validate the full frame before decoding its data by command and address. Version replies, byte-field replies, and active feedback use separate formats.
