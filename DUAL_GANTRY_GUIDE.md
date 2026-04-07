# 듀얼 갠트리 모터 제어 가이드

ROS2 Humble / EtherCAT (pysoem) 기반 **2대 갠트리** 동시 제어 패키지.

---

## 1. 시스템 구성

```
[Jetson — Ubuntu 22.04]
        │ EtherCAT (enP8p1s0)
        ├── Slave 0 : iX7NH → 갠트리0 X축  (NOT 리밋)
        ├── Slave 1 : iX7NH → 갠트리0 Z1축 (POT 리밋)
        ├── Slave 2 : iX7NH → 갠트리0 Z2축 (POT 리밋)
        ├── Slave 3 : iX7NH → 갠트리1 X축  (NOT 리밋)
        ├── Slave 4 : iX7NH → 갠트리1 Z1축 (POT 리밋)
        └── Slave 5 : iX7NH → 갠트리1 Z2축 (POT 리밋)
```

| 갠트리 | X 슬레이브 | Z1 슬레이브 | Z2 슬레이브 |
|--------|-----------|------------|------------|
| 갠트리 0 | 0 | 1 | 2 |
| 갠트리 1 | 3 | 4 | 5 |

---

## 2. 빌드 및 실행

```bash
# 빌드
cd /home/ihow/motor_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# 실행 (반드시 root — EtherCAT CAP_NET_RAW 필요)
sudo -s
source /opt/ros/humble/setup.bash
source /home/ihow/motor_ws/install/setup.bash
ros2 launch motor_control motor_control.launch.py
```

정상 시작 로그:
```
[EtherCAT] 6개 슬레이브 발견
  Slave 0 (X): 호밍 모드(6) 사전 설정 완료
  ...
[EtherCAT] OP 상태 진입 완료
motor_driver_node 준비 완료.
```

---

## 3. 토픽 / 서비스 / 액션 인터페이스

### 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/motor/gantry0/axis_state` | `AxisState` | 갠트리0 위치·속도·호밍 상태 (10Hz) |
| `/motor/gantry1/axis_state` | `AxisState` | 갠트리1 위치·속도·호밍 상태 (10Hz) |

```bash
ros2 topic echo /motor/gantry0/axis_state --once
ros2 topic echo /motor/gantry1/axis_state --once
```

### 서비스

| 서비스 | 타입 | 설명 |
|--------|------|------|
| `/motor/get_status` | `GetDriveStatus` | 드라이브 상태 조회 (gantry_index 지정) |
| `/motor/set_velocity` | `SetVelocity` | 이동 속도 변경 (두 갠트리 공통) |

```bash
# 갠트리0 상태 확인
ros2 service call /motor/get_status motor_control_interfaces/srv/GetDriveStatus \
  "{gantry_index: 0}"

# 갠트리1 상태 확인
ros2 service call /motor/get_status motor_control_interfaces/srv/GetDriveStatus \
  "{gantry_index: 1}"
```

### 액션

| 액션 | 타입 | 설명 |
|------|------|------|
| `/motor/homing` | `Homing` | 호밍 실행 |
| `/motor/move`   | `MoveAxis` | 목표 위치 이동 |

---

## 4. 호밍

### gantry_index 값

| 값 | 동작 |
|----|------|
| `0` | 갠트리0만 호밍 |
| `1` | 갠트리1만 호밍 |
| `2` | 두 갠트리 동시 호밍 |

```bash
# 갠트리0만 호밍
ros2 action send_goal /motor/homing motor_control_interfaces/action/Homing \
  "{gantry_index: 0, force_rehome: true}"

# 갠트리1만 호밍
ros2 action send_goal /motor/homing motor_control_interfaces/action/Homing \
  "{gantry_index: 1, force_rehome: true}"

# 두 갠트리 동시 호밍
ros2 action send_goal /motor/homing motor_control_interfaces/action/Homing \
  "{gantry_index: 2, force_rehome: true}"

# 이미 호밍된 경우 스킵
ros2 action send_goal /motor/homing motor_control_interfaces/action/Homing \
  "{gantry_index: 2, force_rehome: false}"
```

---

## 5. 이동 명령

### gantry_index 값과 사용 파라미터

| gantry_index | 대상 | 사용 파라미터 |
|---|---|---|
| `0` | 갠트리0 | `gantry0_x_target_mm`, `gantry0_z_target_mm` |
| `1` | 갠트리1 | `gantry1_x_target_mm`, `gantry1_z_target_mm` |
| `2` | 두 갠트리 공통 | `gantry2_x_target_mm`, `gantry2_z_target_mm` |

> 이동하지 않을 축에는 `-1.0e9` 를 지정합니다.

### 갠트리0만 이동

```bash
# 갠트리0: X=100mm, Z=-50mm
ros2 action send_goal /motor/move motor_control_interfaces/action/MoveAxis \
  "{gantry_index: 0,
    gantry0_x_target_mm: 100.0,
    gantry0_z_target_mm: -50.0,
    gantry1_x_target_mm: -1.0e9,
    gantry1_z_target_mm: -1.0e9,
    gantry2_x_target_mm: -1.0e9,
    gantry2_z_target_mm: -1.0e9,
    velocity_mm_s: 30.0,
    force_move: false}"

# 갠트리0: X축만 이동
ros2 action send_goal /motor/move motor_control_interfaces/action/MoveAxis \
  "{gantry_index: 0,
    gantry0_x_target_mm: 200.0,
    gantry0_z_target_mm: -1.0e9,
    gantry1_x_target_mm: -1.0e9,
    gantry1_z_target_mm: -1.0e9,
    gantry2_x_target_mm: -1.0e9,
    gantry2_z_target_mm: -1.0e9,
    velocity_mm_s: 50.0,
    force_move: false}"
```

### 갠트리1만 이동

```bash
ros2 action send_goal /motor/move motor_control_interfaces/action/MoveAxis \
  "{gantry_index: 1,
    gantry0_x_target_mm: -1.0e9,
    gantry0_z_target_mm: -1.0e9,
    gantry1_x_target_mm: 150.0,
    gantry1_z_target_mm: -80.0,
    gantry2_x_target_mm: -1.0e9,
    gantry2_z_target_mm: -1.0e9,
    velocity_mm_s: 30.0,
    force_move: false}"
```

### 두 갠트리 동시·동일 이동 (gantry_index=2)

```bash
# 두 갠트리 모두 X=100mm, Z=-50mm 로 동시 이동
ros2 action send_goal /motor/move motor_control_interfaces/action/MoveAxis \
  "{gantry_index: 2,
    gantry0_x_target_mm: -1.0e9,
    gantry0_z_target_mm: -1.0e9,
    gantry1_x_target_mm: -1.0e9,
    gantry1_z_target_mm: -1.0e9,
    gantry2_x_target_mm: 100.0,
    gantry2_z_target_mm: -50.0,
    velocity_mm_s: 30.0,
    force_move: false}"
```

### 강제 이동 (호밍 미완료, 소프트 리밋 무시)

```bash
ros2 action send_goal /motor/move motor_control_interfaces/action/MoveAxis \
  "{gantry_index: 0,
    gantry0_x_target_mm: 0.0,
    gantry0_z_target_mm: -30.0,
    gantry1_x_target_mm: -1.0e9,
    gantry1_z_target_mm: -1.0e9,
    gantry2_x_target_mm: -1.0e9,
    gantry2_z_target_mm: -1.0e9,
    velocity_mm_s: 10.0,
    force_move: true}"
```

---

## 6. 설정 파일 (motor_params.yaml)

경로: `src/motor_control/config/motor_params.yaml`

### 슬레이브 인덱스 변경

사용하지 않는 축은 `-1` 로 설정합니다.

```yaml
num_slaves: 6

# 갠트리 0
gantry0_x_slave_idx:  0
gantry0_z1_slave_idx: 1
gantry0_z2_slave_idx: 2

# 갠트리 1
gantry1_x_slave_idx:  3
gantry1_z1_slave_idx: 4
gantry1_z2_slave_idx: 5
```

### 주요 파라미터 요약

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `ifname` | `enP8p1s0` | EtherCAT 인터페이스 |
| `num_slaves` | `6` | 총 슬레이브 수 |
| `pdo_cycle_ms` | `10` | PDO 사이클 [ms] |
| `default_velocity_rpm` | `30.0` | 기본 속도 [RPM] |
| `homing_fast_rpm` | `30.0` | 호밍 탐색 속도 [RPM] |
| `homing_slow_rpm` | `1.0` | 호밍 Index 탐색 속도 [RPM] |
| `max_sync_error_mm` | `10.0` | Z1-Z2 동기화 오차 긴급정지 임계값 [mm] |
| `gantry0_x_soft_limit_max_mm` | `1500.0` | 갠트리0 X 소프트 리밋 최댓값 |
| `gantry1_z_soft_limit_min_mm` | `-1500.0` | 갠트리1 Z 소프트 리밋 최솟값 |

---

## 7. 속도 단위 변환

```
X축:  RPM = mm/s × 60 ÷ 11.999
Z축:  RPM = mm/s × 60 ÷ 5.999

예시) X축 50 mm/s → 50 × 60 ÷ 11.999 ≈ 250 RPM
      Z축 20 mm/s → 20 × 60 ÷ 5.999  ≈ 200 RPM
```

---

## 8. 동기화 (Cross Coupling)

각 갠트리의 Z1↔Z2 두 모터는 **독립적으로** 동기화 모니터링 및 Cross Coupling이 적용됩니다.

- `max_sync_error_mm` 초과 시 해당 갠트리의 모든 모터 긴급정지
- `coupling_gain` 으로 보정 강도 조절
- `enable_coupling: false` 로 비활성화 가능

---

## 9. 테스트 모드

일부 축만 연결한 경우 미사용 슬레이브를 `-1` 로 설정하고 `num_slaves` 를 조정합니다.

```yaml
# 갠트리0 X축 단독
num_slaves: 1
gantry0_x_slave_idx:  0
gantry0_z1_slave_idx: -1
gantry0_z2_slave_idx: -1
gantry1_x_slave_idx:  -1
gantry1_z1_slave_idx: -1
gantry1_z2_slave_idx: -1
```


ros2 action send_goal /motor/move motor_control_interfaces/action/MoveAxis \
  "{gantry_index: 2,
    gantry0_x_target_mm: -1.0e9,
    gantry0_z_target_mm: -1.0e9,
    gantry1_x_target_mm: -1.0e9,
    gantry1_z_target_mm: -1.0e9,
    gantry2_x_target_mm: 1000.0,
    gantry2_z_target_mm: -1000.0,
    velocity_mm_s: 30.0,
    force_move: false}"



    ros2 action send_goal /motor/move motor_control_interfaces/action/MoveAxis \
  "{gantry_index: 2,
    gantry0_x_target_mm: -1.0e9,
    gantry0_z_target_mm: -1.0e9,
    gantry1_x_target_mm: -1.0e9,
    gantry1_z_target_mm: -1.0e9,
    gantry2_x_target_mm: 1500.0,
    gantry2_z_target_mm: -1500.0,
    velocity_mm_s: 30.0,
    force_move: false}"

    ros2 action send_goal /motor/move motor_control_interfaces/action/MoveAxis \
  "{gantry_index: 2,
    gantry0_x_target_mm: -1.0e9,
    gantry0_z_target_mm: -1.0e9,
    gantry1_x_target_mm: -1.0e9,
    gantry1_z_target_mm: -1.0e9,
    gantry2_x_target_mm: 1700.0,
    gantry2_z_target_mm: -1700.0,
    velocity_mm_s: 30.0,
    force_move: false}"

    ros2 action send_goal /motor/move motor_control_interfaces/action/MoveAxis \
  "{gantry_index: 0,
    gantry0_x_target_mm: 0.0,
    gantry0_z_target_mm: 0.0,
    gantry1_x_target_mm: -1.0e9,
    gantry1_z_target_mm: -1.0e9,
    gantry2_x_target_mm: -1.0e9,
    gantry2_z_target_mm: -1.0e9,
    velocity_mm_s: 30.0,
    force_move: false}"

    ros2 action send_goal /motor/move motor_control_interfaces/action/MoveAxis \
  "{gantry_index: 1,
    gantry0_x_target_mm: -1.0e9,
    gantry0_z_target_mm: -1.0e9,
    gantry1_x_target_mm: 0.0,
    gantry1_z_target_mm: 0.0,
    gantry2_x_target_mm: -1.0e9,
    gantry2_z_target_mm: -1.0e9,
    velocity_mm_s: 30.0,
    force_move: false}"