# motor_ws — 갠트리 로봇 모터 제어

ROS2 Humble 기반 갠트리 로봇 EtherCAT 모터 제어 패키지.
iX7NH 서보 드라이버 3대 (X축 1대, Z축 2대) 를 제어합니다.

---

## 목차

1. [시스템 구성](#1-시스템-구성)
2. [소프트웨어 요구사항](#2-소프트웨어-요구사항)
3. [빌드](#3-빌드)
4. [설정 파일](#4-설정-파일-motor_paramsyaml)
5. [실행](#5-실행)
6. [호밍](#6-호밍)
7. [이동 명령](#7-이동-명령)
8. [테스트 모드](#8-테스트-모드)
9. [ROS2 인터페이스](#9-ros2-인터페이스)
10. [문서](#10-문서)

---

## 1. 시스템 구성

```
[Jetson (Ubuntu 22.04)]
        │  EtherCAT (enP8p1s0)
        ├─ Slave 0 : iX7NH  → X축 (NOT 리밋 스위치)
        ├─ Slave 1 : iX7NH  → Z1축 (POT 리밋 스위치)
        └─ Slave 2 : iX7NH  → Z2축 (POT 리밋 스위치)
```

| 항목 | 내용 |
|------|------|
| OS | Ubuntu 22.04 (NVIDIA Jetson) |
| ROS2 | Humble |
| EtherCAT 라이브러리 | pysoem |
| 드라이브 프로파일 | CiA 402 |
| 운전 모드 | CSV (Cyclic Synchronous Velocity, Mode 9) |
| 호밍 모드 | CiA 402 Homing Mode (Mode 6) |
| 엔코더 분해능 | 8,388,608 pulse/rev (2²³) |

---

## 2. 소프트웨어 요구사항

```bash
# ROS2 Humble
sudo apt install ros-humble-desktop

# pysoem (반드시 root 환경에 설치)
sudo pip3 install pysoem

# ROS2 의존성
sudo apt install python3-argcomplete python3-pip
```

> **중요:** EtherCAT은 `CAP_NET_RAW` 권한이 필요합니다.
> pysoem 서브프로세스에서는 파일 capability가 동작하지 않으므로 **반드시 `sudo -s` (root)** 로 실행해야 합니다.

---

## 3. 빌드

```bash
cd /home/gardentech/motor_ws

source /opt/ros/humble/setup.bash
colcon build --symlink-install

# ldconfig 업데이트 (최초 1회)
echo "/opt/ros/humble/lib" | sudo tee /etc/ld.so.conf.d/ros-humble.conf
echo "/home/gardentech/motor_ws/install/motor_control_interfaces/lib" | \
  sudo tee /etc/ld.so.conf.d/motor-ws.conf
sudo ldconfig
```

---

## 4. 설정 파일 (motor_params.yaml)

경로: `src/motor_control/config/motor_params.yaml`

### 테스트 모드 전환

사용하지 않는 축의 슬레이브 인덱스를 `-1` 로 설정하면 해당 축이 비활성화됩니다.

```yaml
# ── 전체 갠트리 ──
num_slaves:   3
x_slave_idx:  0
z1_slave_idx: 1
z2_slave_idx: 2

# ── X축 단독 테스트 ──
# num_slaves:   1
# x_slave_idx:  0
# z1_slave_idx: -1
# z2_slave_idx: -1

# ── Z축 단독 테스트 (1대) ──
# num_slaves:   1
# x_slave_idx:  -1
# z1_slave_idx: 0
# z2_slave_idx: -1
```

### 주요 파라미터

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `ifname` | `enP8p1s0` | EtherCAT 네트워크 인터페이스 |
| `pdo_cycle_ms` | `10` | EtherCAT PDO 사이클 (ms) |
| `default_velocity_rpm` | `30.0` | 기본 이동 속도 (RPM) |
| `default_accel_rpm_s` | `1000.0` | 기본 가속도 (RPM/s) |
| `homing_fast_rpm` | `30.0` | 호밍 탐색 속도 (RPM) |
| `homing_slow_rpm` | `1.0` | 호밍 Index 탐색 속도 (RPM) |
| `homing_timeout_s` | `300.0` | 호밍 타임아웃 (초) |
| `max_sync_error_mm` | `10.0` | Z1-Z2 동기화 오차 긴급정지 임계값 (mm) |
| `x_soft_limit_min_mm` | `0.0` | X축 소프트 리밋 최솟값 |
| `x_soft_limit_max_mm` | `1500.0` | X축 소프트 리밋 최댓값 |
| `z_soft_limit_min_mm` | `-1500.0` | Z축 소프트 리밋 최솟값 |
| `z_soft_limit_max_mm` | `0.1` | Z축 소프트 리밋 최댓값 |

---

## 5. 실행

```bash
# 반드시 root로 실행
sudo -s

source /opt/ros/humble/setup.bash
source /home/gardentech/motor_ws/install/setup.bash

ros2 launch motor_control motor_control.launch.py
```

### 정상 시작 로그 확인

```
[EtherCAT] 3개 슬레이브 발견
  Slave 0 (X): 호밍 모드(6) 사전 설정 완료 (method=1, fast=30.0RPM, slow=1.0RPM)
  Slave 1 (Z): 호밍 모드(6) 사전 설정 완료 (method=2, fast=30.0RPM, slow=1.0RPM)
  Slave 2 (Z): 호밍 모드(6) 사전 설정 완료 (method=2, fast=30.0RPM, slow=1.0RPM)
[EtherCAT] OP 상태 진입 완료
활성 슬레이브: X=0, Z1=1, Z2=2
```

---

## 6. 호밍

호밍은 각 축이 리밋 스위치를 찾아 원점(0mm)을 설정하는 과정입니다.

| 축 | 방법 | 동작 |
|----|------|------|
| X | Method 1 (NOT + Index) | 역방향(-) 이동 → NOT 리밋 스위치 → Index 펄스에서 정지 |
| Z | Method 2 (POT + Index) | 정방향(+) 이동 → POT 리밋 스위치 → Index 펄스에서 정지 |

```bash
# 호밍 실행
ros2 action send_goal /motor/homing motor_control_interfaces/action/Homing \
  "{force_rehome: true}"

# 이미 호밍된 경우 스킵 (재시작 후 재호밍 불필요할 때)
ros2 action send_goal /motor/homing motor_control_interfaces/action/Homing \
  "{force_rehome: false}"
```

---

## 7. 이동 명령

호밍 완료 후 목표 위치로 이동합니다.

```bash
# 일반 이동 (호밍 필요, 소프트 리밋 적용)
ros2 action send_goal /motor/move motor_control_interfaces/action/MoveAxis \
  "{x_target_mm: 100.0, z_target_mm: -50.0, velocity_mm_s: 30.0, force_move: false}"

# 강제 이동 (호밍 미완료 상태, 소프트 리밋 무시)
ros2 action send_goal /motor/move motor_control_interfaces/action/MoveAxis \
  "{x_target_mm: 0.0, z_target_mm: -30.0, velocity_mm_s: 10.0, force_move: true}"
```

### 축 선택 이동

이동하지 않을 축에 `-1.0e9` 를 지정합니다.

```bash
# X축만 이동
ros2 action send_goal /motor/move motor_control_interfaces/action/MoveAxis \
  "{x_target_mm: 200.0, z_target_mm: -1.0e9, velocity_mm_s: 50.0, force_move: false}"

# Z축만 이동
ros2 action send_goal /motor/move motor_control_interfaces/action/MoveAxis \
  "{x_target_mm: -1.0e9, z_target_mm: -100.0, velocity_mm_s: 20.0, force_move: false}"
```

### 속도 단위 변환

```
X축:  RPM = mm/s × 60 ÷ 11.999
Z축:  RPM = mm/s × 60 ÷ 5.999

예시) X축 50 mm/s → 50 × 60 ÷ 11.999 ≈ 250 RPM
```

자세한 내용: [VELOCITY_GUIDE.md](VELOCITY_GUIDE.md)

---

## 8. 테스트 모드

`motor_params.yaml` 수정 후 launch 재시작으로 전환합니다.

| 모드 | num_slaves | x | z1 | z2 |
|------|-----------|---|----|----|
| X 단독 | 1 | 0 | -1 | -1 |
| Z 단독 (1대) | 1 | -1 | 0 | -1 |
| Z 2대 | 2 | -1 | 0 | 1 |
| 전체 갠트리 | 3 | 0 | 1 | 2 |

---

## 9. ROS2 인터페이스

### 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/motor/axis_state` | `AxisState` | 현재 위치·속도·호밍 상태 (10 Hz) |

### 액션

| 액션 | 타입 | 설명 |
|------|------|------|
| `/motor/homing` | `Homing` | 호밍 실행 |
| `/motor/move` | `MoveAxis` | 목표 위치 이동 |

### 서비스

| 서비스 | 타입 | 설명 |
|--------|------|------|
| `/motor/get_status` | `GetDriveStatus` | 드라이브 상태 조회 |

### 상태 확인 명령

```bash
# 현재 위치 확인
ros2 topic echo /motor/axis_state --once

# 드라이브 상태 확인
ros2 service call /motor/get_status motor_control_interfaces/srv/GetDriveStatus
```

---

## 10. 문서

| 파일                                             | 내용                  |
| ---------------------------------------------- | ------------------- |
| [MOTOR_WS_GUIDE.md](.source/MOTOR_WS_GUIDE.md) | 전체 시스템 아키텍처 및 구동 원리 |
| [TEST_X_AXIS.md](TEST_X_AXIS.md)               | X축 단독 테스트 절차        |
| [TEST_Z_AXIS.md](TEST_Z_AXIS.md)               | Z축 단독 테스트 절차        |
| [VELOCITY_GUIDE.md](VELOCITY_GUIDE.md)         | 속도·가속도 설정 및 단위 변환   |
