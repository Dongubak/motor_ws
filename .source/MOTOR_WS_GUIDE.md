# motor_ws — EtherCAT 모터 제어 시스템 가이드

> ROS2 기반 EtherCAT 갠트리 로봇 3축 모터 제어 시스템
> 드라이버: LS Mecapion iX7NH Series (CiA 402)

---

## 목차

1. [프로젝트 개요](#1-프로젝트-개요)
2. [디렉토리 구조](#2-디렉토리-구조)
3. [패키지 설명](#3-패키지-설명)
4. [시스템 아키텍처](#4-시스템-아키텍처)
5. [빌드 방법](#5-빌드-방법)
6. [테스트 모드 설정](#6-테스트-모드-설정)
7. [실행 방법](#7-실행-방법)
8. [원점 복귀 (Homing)](#8-원점-복귀-homing)
9. [ROS2 인터페이스](#9-ros2-인터페이스)
10. [설정 파라미터](#10-설정-파라미터)
11. [사용 예시](#11-사용-예시)
12. [의존성 및 요구사항](#12-의존성-및-요구사항)
13. [안전 기능](#13-안전-기능)

---

## 1. 프로젝트 개요

`motor_ws`는 **EtherCAT 버스**를 통해 3개의 서보 드라이버를 실시간으로 제어하는 ROS2 워크스페이스입니다.

### 하드웨어 구성

| 축 | 모터 수 | 이동 범위 | 리드 |
|---|---|---|---|
| X축 | 1개 | 0 ~ 1000 mm | 11.9993 mm/rev |
| Z축 | 2개 (동기) | 0 ~ 300 mm | 5.9996 mm/rev |

- **엔코더 분해능:** 8,388,608 pulse/rev (2²³, 전기적 기어비 1:1)
- **제어 주기:** 10ms (기본값) / 1ms 권장 (실시간 커널 필요)
- **통신 방식:** EtherCAT — CiA 402 프로파일

### 제어 모드

| 모드 | CiA 402 코드 | 설명 |
|---|---|---|
| CSV (Cyclic Synchronous Velocity) | Mode 9 | 일반 이동 — 사다리꼴 속도 프로파일 |
| Homing | Mode 6 | 원점 복귀 — 리밋 스위치 + 인덱스 펄스 |

### 원점 복귀 방법 (축별 상이)

| 축 | Method | 방향 | 기준 스위치 | 최종 원점 |
|---|---|---|---|---|
| **X축** | **1** | 역방향 (−) | NOT (Negative Over Travel) | Index (Z) 펄스 |
| **Z축** | **2** | 정방향 (+) | POT (Positive Over Travel) | Index (Z) 펄스 |

---

## 2. 디렉토리 구조

```
motor_ws/
├── src/
│   ├── motor_control/                      # 메인 제어 패키지 (Python)
│   │   ├── motor_control/
│   │   │   ├── __init__.py
│   │   │   ├── motor_driver_node.py        # ROS2 메인 노드
│   │   │   ├── ethercat_interface.py       # EtherCAT 버스 통신 핵심
│   │   │   ├── homing_action_server.py     # 원점 복귀 액션 서버
│   │   │   └── move_action_server.py       # 이동 액션 서버
│   │   ├── launch/
│   │   │   ├── motor_control.launch.py     # 메인 런치 파일
│   │   │   └── homing_only.launch.py       # 원점 복귀 전용 런치 파일
│   │   ├── config/
│   │   │   └── motor_params.yaml           # 설정 파라미터 (테스트 모드 포함)
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── setup.cfg
│   │
│   └── motor_control_interfaces/           # 인터페이스 정의 패키지 (CMake)
│       ├── msg/
│       │   └── AxisState.msg
│       ├── srv/
│       │   ├── SetVelocity.srv
│       │   └── GetDriveStatus.srv
│       ├── action/
│       │   ├── Homing.action
│       │   └── MoveAxis.action
│       ├── package.xml
│       └── CMakeLists.txt
│
├── .source/motor-control/                  # 검증 완료 레퍼런스 코드 (Python standalone)
│   ├── motor_csv_main_x_homing.py          # X축 호밍 검증 코드
│   ├── motor_csv_main_x.py                 # X축 이동 검증 코드
│   └── [LSMecapion] iX7NH_KOR_Ver1.6_251001.pdf
│
├── build/
├── install/
└── log/
```

---

## 3. 패키지 설명

### 3-1. `motor_control_interfaces`

커스텀 ROS2 메시지·서비스·액션 타입 정의 패키지 (C++ CMake 빌드).

| 타입 | 이름 | 설명 |
|---|---|---|
| msg | `AxisState` | 모터 축 실시간 상태 (위치, 속도, 원점 여부 등) |
| srv | `SetVelocity` | 특정 축의 속도를 mm/s 단위로 설정 |
| srv | `GetDriveStatus` | 전체 드라이브 상태 조회 |
| action | `Homing` | X·Z축 원점 복귀 시퀀스 실행 |
| action | `MoveAxis` | X·Z축 목표 위치로 이동 |

### 3-2. `motor_control`

Python으로 작성된 메인 제어 패키지.

#### `ethercat_interface.py` — EtherCAT 통신 핵심

- `pysoem` 라이브러리를 통해 EtherCAT 슬레이브와 통신
- 별도 **서브프로세스**에서 제어 루프 실행 (1kHz 목표)
- CiA 402 상태 머신 구현 (INIT → PRE-OP → SAFE-OP → OP)
- 호밍 완료 감지: `bit12_cleared` 추적으로 잔류 플래그 오탐 방지
- Z축 2모터 동기화 오차 보정 (Cross-Coupling)

**핵심 상수:**

```python
EFFECTIVE_PPR   = 8_388_608      # pulse/rev (전기적 기어비 1:1)
X_HOMING_METHOD = 1              # X축: 역방향 + NOT + Index
Z_HOMING_METHOD = 2              # Z축: 정방향 + POT + Index
HOMING_FAST_RPM = 30.0           # 고속 탐색 [RPM]
HOMING_SLOW_RPM = 1.0            # 저속 Index 탐색 [RPM]
```

#### `motor_driver_node.py` — ROS2 메인 노드

- `/motor/axis_state` 토픽을 10Hz로 퍼블리시
- 슬레이브 인덱스 `-1` = 비활성 → 해당 축 모든 처리 건너뜀
- 파라미터: `x_homing_method`, `z_homing_method` 지원

#### `homing_action_server.py` — 원점 복귀 서버

- X축과 Z축 각각 다른 호밍 메서드 적용
- 비활성 축(-1)은 자동으로 완료 상태로 처리
- `bit12_cleared` + `homing_b4_cycles` 로 안정적인 rising edge 확보

#### `move_action_server.py` — 이동 액션 서버

- 사다리꼴 속도 프로파일 (가속 → 등속 → 감속)
- 소프트 리밋 사전 검증
- 동기화 오차 실시간 모니터링

---

## 4. 시스템 아키텍처

```
┌──────────────────────────────────────────────┐
│              메인 프로세스 (ROS2)              │
│                                              │
│  MotorDriverNode                             │
│  ├── HomingActionServer  (/motor/homing)     │
│  │     X: Method 1 (NOT)                    │
│  │     Z: Method 2 (POT)                    │
│  ├── MoveAxisActionServer (/motor/move)      │
│  ├── /motor/set_velocity  (service)          │
│  └── /motor/get_status    (service)          │
│                                              │
│         ↕ 커맨드 큐 (mp.Queue)               │
│         ↕ 공유 메모리 (mp.Array)             │
└──────────────────────────────────────────────┘
                      │
┌──────────────────────────────────────────────┐
│          EtherCAT 프로세스 (제어 루프)          │
│                                              │
│  _ethercat_process_loop()                    │
│  ├── Slave 0: X축 드라이버  (idx=0)          │
│  ├── Slave 1: Z축 드라이버1 (idx=1)          │
│  └── Slave 2: Z축 드라이버2 (idx=2)          │
│                                              │
│  ※ idx=-1 슬레이브는 연결 안 됨 (테스트 모드) │
└──────────────────────────────────────────────┘
                      │ EtherCAT 케이블
┌──────────────────────────────────────────────┐
│           서보 드라이버 (iX7NH ×1~3)          │
└──────────────────────────────────────────────┘
```

### 슬레이브별 공유 상태 슬롯 (8개)

| 슬롯 | 내용 |
|---|---|
| 0 | Status Word (CiA 402) |
| 1 | 이동 중 플래그 (0/1) |
| 2 | 현재 위치 (pulse) |
| 3 | 원점 오프셋 (pulse) |
| 4 | 동기화 오차 플래그 (0/1) |
| 5 | 현재 속도 (pulse/sec) |
| 6 | 원점 복귀 완료 플래그 (0/1) |
| 7 | 원점 복귀 단계 (0=idle / 1=진행중 / 2=완료 / -1=오류) |

---

## 5. 빌드 방법

```bash
cd /home/gardentech/motor_ws

# 인터페이스 먼저 빌드 (의존성 순서)
colcon build --packages-select motor_control_interfaces
colcon build --packages-select motor_control

# 또는 전체 빌드
colcon build

# 매 터미널마다 환경 소싱 필요
source install/setup.bash
```

---

## 6. 테스트 모드 설정

`motor_params.yaml`의 슬레이브 인덱스를 `-1`로 설정하면 해당 축이 비활성화됩니다.
**YAML 파일만 수정**하면 코드 변경 없이 단독/전체 테스트 전환이 가능합니다.

### 테스트 단계 권장 순서

```
1단계: X축 단독 테스트  (갠트리 분리 X 드라이버)
2단계: Z축 단독 테스트  (갠트리 분리 Z 드라이버 1대)
3단계: 전체 갠트리 테스트 (X + Z1 + Z2)
```

### 단계별 YAML 설정

#### 1단계 — X축 단독 (현재 기본값)

```yaml
num_slaves: 1
x_slave_idx:   0    # X축 드라이버
z1_slave_idx: -1    # 비활성
z2_slave_idx: -1    # 비활성
```

> 호밍: X축 Method 1 (역방향 → NOT → Index)

---

#### 2단계 — Z축 단독 (드라이버 1대)

```yaml
num_slaves: 1
x_slave_idx:  -1    # 비활성
z1_slave_idx:  0    # Z 드라이버
z2_slave_idx: -1    # 비활성
```

> 호밍: Z1 Method 2 (정방향 → POT → Index)

---

#### 2단계 변형 — Z축 2대 (드라이버 2대)

```yaml
num_slaves: 2
x_slave_idx:  -1    # 비활성
z1_slave_idx:  0
z2_slave_idx:  1
```

---

#### 3단계 — 전체 갠트리

```yaml
num_slaves: 3
x_slave_idx:  0
z1_slave_idx: 1
z2_slave_idx: 2
```

> 호밍: X=Method 1, Z1/Z2=Method 2 동시 실행

---

## 7. 실행 방법

### 환경 소싱 (필수)

```bash
source /home/gardentech/motor_ws/install/setup.bash
```

### 기본 실행

```bash
# motor_params.yaml 설정 그대로 사용
ros2 launch motor_control motor_control.launch.py

# 네트워크 인터페이스 직접 지정
ros2 launch motor_control motor_control.launch.py ifname:=enp3s0

# 사용 가능한 인터페이스 확인
ip link show
```

### 원점 복귀 전용 런치

```bash
# 드라이버 시작 후 3초 대기 → 자동 원점 복귀
ros2 launch motor_control homing_only.launch.py ifname:=eth0
```

### 실행 흐름

```
1. 하드웨어 전원 ON + EtherCAT 케이블 연결 확인
2. motor_params.yaml에서 테스트 모드 설정
3. colcon build && source install/setup.bash
4. ros2 launch motor_control motor_control.launch.py ifname:=<인터페이스>
5. /motor/homing 액션으로 원점 복귀
6. /motor/move 액션으로 이동 명령
```

---

## 8. 원점 복귀 (Homing)

### X축 — Method 1 (NOT + Index)

```
역방향(−) 이동
    │
    ▼
NOT 리밋 스위치 감지 (DI2)
    │
    ▼
방향 전환 → 정방향(+) 이동
    │
    ▼
Index(Z) 펄스 감지 → 원점 확정
```

### Z축 — Method 2 (POT + Index)

```
정방향(+) 이동
    │
    ▼
POT 리밋 스위치 감지 (DI1)
    │
    ▼
방향 전환 → 역방향(−) 이동
    │
    ▼
Index(Z) 펄스 감지 → 원점 확정
```

### 호밍 안정화 로직 (bit12_cleared)

드라이브 이전 세션에서 bit12(Homing Attained)가 잔류할 수 있습니다.
코드는 다음 두 가지 조건을 모두 만족할 때만 호밍 완료로 인정합니다:

1. `bit12_cleared = True` — 호밍 시작 후 bit12가 한 번 이상 0이었음
2. `homing_b4_cycles >= 3` — Operation Enabled 진입 후 3사이클 동안 bit4=0 유지 (rising edge 확보)

---

## 9. ROS2 인터페이스

### 9-1. 토픽

#### `/motor/axis_state` (퍼블리시, 10Hz)

| 필드 | 타입 | 설명 |
|---|---|---|
| `x_position_mm` | float64 | X축 현재 위치 (mm) |
| `z_position_mm` | float64 | Z축 평균 위치 (mm) |
| `z1_position_mm` | float64 | Z1 현재 위치 (mm) |
| `z2_position_mm` | float64 | Z2 현재 위치 (mm) |
| `x_velocity_mm_s` | float64 | X축 현재 속도 (mm/s) |
| `z_velocity_mm_s` | float64 | Z축 평균 속도 (mm/s) |
| `z_sync_error_mm` | float64 | Z1-Z2 동기화 오차 (mm) |
| `x_homed` | bool | X축 원점 복귀 완료 |
| `z_homed` | bool | Z축 원점 복귀 완료 |
| `drive_state` | string | CiA 402 상태 문자열 |
| `sync_fault` | bool | 동기화 오차 비상 정지 여부 |

> 비활성 축(idx=-1)의 필드는 0.0 또는 True로 채워집니다.

---

### 9-2. 서비스

#### `/motor/set_velocity`

```
# 요청
float64 velocity_mm_s   # 목표 속도 (mm/s, 양수)
string  axis            # "x" | "z" | "all"
---
# 응답
bool   success
string message
```

```bash
ros2 service call /motor/set_velocity motor_control_interfaces/srv/SetVelocity \
  "{velocity_mm_s: 30.0, axis: 'x'}"
```

#### `/motor/get_status`

```
# 요청 (없음)
---
# 응답
bool    ethercat_connected
string  cia402_state
bool    x_homed
bool    z_homed
float64 x_position_mm
float64 z_position_mm
string  error_code
bool    sync_fault
```

```bash
ros2 service call /motor/get_status motor_control_interfaces/srv/GetDriveStatus
```

---

### 9-3. 액션

#### `/motor/homing` — 원점 복귀

```
# 목표
bool force_rehome   # true: 이미 완료 상태여도 강제 재실행
---
# 피드백
string  phase           # "in_progress" | "idle"
float64 current_z_mm    # 현재 Z축 위치 (mm)
---
# 결과
bool    success
string  message
float64 z_home_position_mm
```

활성화된 축만 호밍 실행:
- X 활성 → Method 1 (NOT + Index)
- Z1/Z2 활성 → Method 2 (POT + Index)

```bash
ros2 action send_goal /motor/homing motor_control_interfaces/action/Homing \
  "{force_rehome: false}"
```

#### `/motor/move` — 축 이동

```
# 목표
float64 x_target_mm    # X축 목표 위치 (mm), -1이면 현재 위치 유지
float64 z_target_mm    # Z축 목표 위치 (mm), -1이면 현재 위치 유지
float64 velocity_mm_s  # 이동 속도 (mm/s)
---
# 피드백
float64 x_current_mm
float64 z_current_mm
float64 x_remaining_mm
float64 z_remaining_mm
---
# 결과
bool    success
float64 x_final_mm
float64 z_final_mm
string  message
```

```bash
# X축만 100mm로 이동
ros2 action send_goal /motor/move motor_control_interfaces/action/MoveAxis \
  "{x_target_mm: 100.0, z_target_mm: -1.0, velocity_mm_s: 30.0}"

# Z축만 150mm로 이동
ros2 action send_goal /motor/move motor_control_interfaces/action/MoveAxis \
  "{x_target_mm: -1.0, z_target_mm: 150.0, velocity_mm_s: 20.0}"

# X+Z 동시 이동
ros2 action send_goal /motor/move motor_control_interfaces/action/MoveAxis \
  "{x_target_mm: 100.0, z_target_mm: 150.0, velocity_mm_s: 50.0}"
```

---

## 10. 설정 파라미터

파일: `src/motor_control/config/motor_params.yaml`

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `ifname` | `"eth0"` | EtherCAT 네트워크 인터페이스 |
| `num_slaves` | `1` | 연결된 EtherCAT 슬레이브 수 |
| `pdo_cycle_ms` | `10` | 제어 주기 (ms) |
| `x_slave_idx` | `0` | X축 슬레이브 인덱스 (-1=비활성) |
| `z1_slave_idx` | `-1` | Z1 슬레이브 인덱스 (-1=비활성) |
| `z2_slave_idx` | `-1` | Z2 슬레이브 인덱스 (-1=비활성) |
| `x_homing_method` | `1` | X축 호밍 메서드 (NOT + Index) |
| `z_homing_method` | `2` | Z축 호밍 메서드 (POT + Index) |
| `default_velocity_rpm` | `10.0` | 기본 속도 [RPM] |
| `default_accel_rpm_s` | `10.0` | 기본 가감속 [RPM/s] |
| `enable_coupling` | `true` | Z축 Cross-Coupling 활성화 |
| `coupling_gain` | `0.05` | Cross-Coupling 게인 [1/s] |
| `max_sync_error_mm` | `10.0` | 동기화 오차 비상정지 임계값 [mm] |
| `ma_window` | `5` | 동기화 오차 이동평균 윈도우 |
| `x_soft_limit_min_mm` | `0.0` | X축 소프트 리밋 하한 [mm] |
| `x_soft_limit_max_mm` | `1000.0` | X축 소프트 리밋 상한 [mm] |
| `z_soft_limit_min_mm` | `0.0` | Z축 소프트 리밋 하한 [mm] |
| `z_soft_limit_max_mm` | `300.0` | Z축 소프트 리밋 상한 [mm] |
| `state_publish_hz` | `10.0` | AxisState 발행 주기 [Hz] |

---

## 11. 사용 예시

### X축 단독 테스트 전체 흐름

```bash
# 1. motor_params.yaml → X축 단독 모드 확인
#    num_slaves: 1 / x_slave_idx: 0 / z1/z2: -1

# 2. 빌드 & 소싱
cd /home/gardentech/motor_ws
colcon build && source install/setup.bash

# 3. 시스템 시작
ros2 launch motor_control motor_control.launch.py ifname:=eth0

# 4. 별도 터미널: 상태 확인
source install/setup.bash
ros2 topic echo /motor/axis_state

# 5. 원점 복귀 (X축 Method 1 실행)
ros2 action send_goal /motor/homing motor_control_interfaces/action/Homing \
  "{force_rehome: false}"

# 6. 이동
ros2 action send_goal /motor/move motor_control_interfaces/action/MoveAxis \
  "{x_target_mm: 200.0, z_target_mm: -1.0, velocity_mm_s: 30.0}"
```

### Python 클라이언트 예시

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from motor_control_interfaces.action import Homing, MoveAxis
from motor_control_interfaces.srv import GetDriveStatus

rclpy.init()
node = Node('motor_client')

# ── 상태 조회 ──────────────────────────────────────────
status_cli = node.create_client(GetDriveStatus, '/motor/get_status')
status_cli.wait_for_service()
res = status_cli.call(GetDriveStatus.Request())
print(f"연결: {res.ethercat_connected}, X={res.x_position_mm:.2f}mm")

# ── 원점 복귀 ──────────────────────────────────────────
homing_cli = ActionClient(node, Homing, '/motor/homing')
homing_cli.wait_for_server()

future = homing_cli.send_goal_async(
    Homing.Goal(force_rehome=False),
    feedback_callback=lambda fb: print(f"호밍: {fb.feedback.phase}")
)
rclpy.spin_until_future_complete(node, future)
result = future.result().get_result_async()
rclpy.spin_until_future_complete(node, result)
print(f"호밍 완료: {result.result().result.success}")

# ── X축 이동 ──────────────────────────────────────────
move_cli = ActionClient(node, MoveAxis, '/motor/move')
move_cli.wait_for_server()

future = move_cli.send_goal_async(
    MoveAxis.Goal(x_target_mm=200.0, z_target_mm=-1.0, velocity_mm_s=30.0),
    feedback_callback=lambda fb: print(f"X={fb.feedback.x_current_mm:.1f}mm")
)
rclpy.spin_until_future_complete(node, future)
result = future.result().get_result_async()
rclpy.spin_until_future_complete(node, result)
print(f"이동 완료: X={result.result().result.x_final_mm:.2f}mm")

node.destroy_node()
rclpy.shutdown()
```

---

## 12. 의존성 및 요구사항

### ROS2 패키지

```bash
sudo apt install ros-humble-rclpy ros-humble-std-msgs ros-humble-action-msgs
```

### Python 패키지

```bash
pip3 install pysoem
```

> `pysoem` 은 Linux 전용 EtherCAT 마스터 라이브러리입니다.

### 시스템 요구사항

| 항목 | 요구사항 |
|---|---|
| OS | Linux (Ubuntu 22.04 권장) |
| ROS2 버전 | Humble 이상 |
| Python | 3.8 이상 |
| 커널 | RT Kernel 권장 (1ms 주기 시) |
| 권한 | CAP_NET_RAW (EtherCAT 원시 소켓) |
| 하드웨어 | EtherCAT NIC + iX7NH 드라이버 ×1~3 |

### 네트워크 권한 (root 없이 실행)

```bash
sudo setcap cap_net_raw+ep $(readlink -f $(which python3))
```

---

## 13. 안전 기능

### 소프트 리밋

이동 명령 수신 시 목표 위치를 사전 검증합니다. 범위 초과 시 액션이 즉시 Abort됩니다.

```
X축: 0.0 mm ~ 1000.0 mm
Z축: 0.0 mm ~  300.0 mm
```

### Z축 동기화 오차 감지

Z1-Z2 위치 이동 평균 차이가 `max_sync_error_mm`(기본 10mm) 초과 시 **전체 모터 비상 정지**.

### 원점 복귀 선행 조건

`/motor/move` 명령은 활성화된 모든 축의 원점 복귀 완료 후에만 허용됩니다.

### CiA 402 드라이브 폴트 감지

제어 루프 내 매 사이클마다 Status Word를 감시합니다. Fault 비트 검출 시 해당 슬레이브 및 연동 슬레이브를 즉시 정지합니다.

### 호밍 bit12 잔류 오탐 방지

이전 세션 종료 후 드라이브에 Homing Attained(bit12)가 남아 있을 수 있습니다. `bit12_cleared` 플래그로 실제 호밍 동작 후의 완료 신호만 인정합니다.

---

## 빠른 참조

```bash
# 환경 소싱
source /home/gardentech/motor_ws/install/setup.bash

# 시스템 시작
ros2 launch motor_control motor_control.launch.py ifname:=eth0

# 상태 모니터링
ros2 topic echo /motor/axis_state
ros2 service call /motor/get_status motor_control_interfaces/srv/GetDriveStatus

# 원점 복귀
ros2 action send_goal /motor/homing motor_control_interfaces/action/Homing \
  "{force_rehome: false}"

# X축 이동 (원점 복귀 후)
ros2 action send_goal /motor/move motor_control_interfaces/action/MoveAxis \
  "{x_target_mm: 200.0, z_target_mm: -1.0, velocity_mm_s: 30.0}"

# Z축 이동 (원점 복귀 후)
ros2 action send_goal /motor/move motor_control_interfaces/action/MoveAxis \
  "{x_target_mm: -1.0, z_target_mm: 100.0, velocity_mm_s: 20.0}"
```
