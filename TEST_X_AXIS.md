# X축 단독 테스트 가이드

> 갠트리 로봇에서 분리된 X축 드라이버 단독 테스트

---

## 사전 확인

### 하드웨어 체크리스트

- [x] X축 드라이버 전원 ON
- [x] EtherCAT 케이블: **Jetson** `enP8p1s0` 포트 → X축 드라이버 IN 포트
- [x] 모터 인코더 케이블 연결 확인
- [x] NOT 리밋 스위치 배선 확인 (DI2 핀)
- [x] X축 이동 경로 장애물 없음 확인

### NOT 리밋 스위치 배선 (DI2)

```
드라이버 DI2 핀 ──── NOT 스위치 COM
드라이버 COM 핀 ──── GND (0V)
NOT 스위치 NC  ──── 드라이버 DI2  (평상시 ON, 눌리면 OFF)
```

> iX7NH 기본 DI 할당: DI1=POT, DI2=NOT, DI3=HOME

---

## 현재 설정 확인

`motor_params.yaml` 이 다음과 같이 설정되어 있는지 확인:

```yaml
ifname: "enP8p1s0"
num_slaves: 1
x_slave_idx:   0     # X축 드라이버
z1_slave_idx: -1     # 비활성
z2_slave_idx: -1     # 비활성
x_homing_method: 1   # 역방향(-) → NOT → Index
pdo_cycle_ms: 10
default_velocity_rpm: 10.0
```

---

## 테스트 절차

### 터미널 1 — 시스템 시작

```bash
cd ~/motor_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch motor_control motor_control.launch.py
```

**정상 출력 예시:**

```
[motor_driver_node]: EtherCAT 인터페이스: enP8p1s0, 슬레이브 수: 1, 사이클: 10ms
[motor_driver_node]: 활성 슬레이브: X=0, Z1=-1, Z2=-1 (호밍: X=Method1, Z=Method2)
[EtherCAT] 어댑터 'enP8p1s0' 열기 (시도 1)
[EtherCAT] 1개 슬레이브 발견
[EtherCAT] OP 상태 진입 완료. 제어 루프 시작.
[motor_driver_node]: 모든 드라이브 준비 완료.
[motor_driver_node]: motor_driver_node 준비 완료.
[homing_action_server]: 활성 축: X(idx=0, method=1)
```

---

### 터미널 2 — 상태 모니터링

```bash
source /opt/ros/humble/setup.bash
source ~/motor_ws/install/setup.bash

# 실시간 상태 확인
ros2 topic echo /motor/axis_state
```

**정상 출력 예시 (호밍 전):**

```
x_position_mm: 0.0
x_velocity_mm_s: 0.0
x_homed: false
z_homed: true        ← Z 비활성이므로 true (무시)
drive_state: 'Operation enabled'
sync_fault: false
```

---

### 터미널 3 — 테스트 명령 실행

#### STEP 1. 드라이브 상태 조회

```bash
source /opt/ros/humble/setup.bash
source ~/motor_ws/install/setup.bash

ros2 service call /motor/get_status motor_control_interfaces/srv/GetDriveStatus
```

`ethercat_connected: true`, `cia402_state: 'Operation enabled'` 확인.

---

#### STEP 2. 원점 복귀 (Homing)

> X축이 **역방향(−)** 으로 이동하면서 NOT 스위치를 찾습니다.
> 모터가 움직이지 않으면 안전하게 Ctrl+C 후 배선을 확인하세요.

```bash
ros2 action send_goal /motor/homing motor_control_interfaces/action/Homing \
  "{force_rehome: false}"
```

**진행 중 피드백:**

```
feedback:
  phase: in_progress
  current_z_mm: 0.0
```

**완료 시 결과:**

```
result:
  success: true
  message: 호밍 완료.
```

호밍 실패 시 → [트러블슈팅](#트러블슈팅) 참조.

---

#### STEP 3. 저속 이동 테스트 (소폭)

호밍 완료 후, 먼저 짧은 거리로 테스트합니다.

```bash
# 50mm 이동 (속도 10mm/s)
ros2 action send_goal /motor/move motor_control_interfaces/action/MoveAxis \
  "{x_target_mm: 50.0, z_target_mm: -1.0, velocity_mm_s: 10.0}"
```

피드백으로 현재 위치가 출력됩니다. 이동 완료 후 `x_final_mm` 확인.

---

#### STEP 4. 왕복 이동 테스트

```bash
# 200mm로 이동
ros2 action send_goal /motor/move motor_control_interfaces/action/MoveAxis \
  "{x_target_mm: 200.0, z_target_mm: -1.0, velocity_mm_s: 20.0}"

# 원점(0mm)으로 복귀
ros2 action send_goal /motor/move motor_control_interfaces/action/MoveAxis \
  "{x_target_mm: 0.0, z_target_mm: -1.0, velocity_mm_s: 20.0}"
```

---

#### STEP 5. 속도 변경 테스트

```bash
# 속도를 30mm/s 로 변경
ros2 service call /motor/set_velocity motor_control_interfaces/srv/SetVelocity \
  "{velocity_mm_s: 30.0, axis: 'x'}"

# 500mm로 이동
ros2 action send_goal /motor/move motor_control_interfaces/action/MoveAxis \
  "{x_target_mm: 500.0, z_target_mm: -1.0, velocity_mm_s: 30.0}"
```

---

#### STEP 6. 소프트 리밋 확인

범위 초과 명령을 보내면 액션이 Abort 되어야 합니다.

```bash
# 소프트 리밋 초과 (1001mm) → 거부되어야 함
ros2 action send_goal /motor/move motor_control_interfaces/action/MoveAxis \
  "{x_target_mm: 1001.0, z_target_mm: -1.0, velocity_mm_s: 10.0}"
```

`success: false` 와 리밋 초과 메시지가 반환되면 정상.

---

## 테스트 체크리스트

| 항목 | 확인 |
|---|---|
| EtherCAT 연결 (`ethercat_connected: true`) | [ ] |
| Operation Enabled 상태 | [ ] |
| 원점 복귀 완료 (`x_homed: true`) | [ ] |
| 목표 위치 도달 (위치 오차 < 0.1mm) | [ ] |
| 왕복 이동 정상 | [ ] |
| 소프트 리밋 거부 동작 | [ ] |

---

## 트러블슈팅

### 슬레이브를 찾지 못함
```
[EtherCAT] 슬레이브 부족: 필요=1, 발견=0
```
- EtherCAT 케이블 `enP8p1s0` 포트 연결 확인
- 드라이버 전원 확인
- `sudo setcap cap_net_raw+ep $(readlink -f $(which python3))` 실행 후 재시도

### 호밍 오류 (Homing Error)
```
result:
  success: false
  message: 호밍 오류 발생
```
- NOT 스위치(DI2) 배선 확인
- 드라이버 파라미터에서 DI2 기능이 NOT(역방향 리밋)으로 설정되어 있는지 확인
- 모터가 이미 NOT 위치에 있는 경우 반대 방향으로 수동 이동 후 재시도

### 드라이브 Fault 상태
```
drive_state: 'Fault'
```
```bash
# 강제 재호밍으로 Fault 리셋 시도
ros2 action send_goal /motor/homing motor_control_interfaces/action/Homing \
  "{force_rehome: true}"
```

### 이동 거부 (호밍 미완료)
```
success: false
message: ... 호밍 미완료 ...
```
- STEP 2 원점 복귀를 먼저 완료한 후 이동 명령 전송

### 빌드 오류 시

```bash
cd ~/motor_ws
rm -rf build/ install/ log/
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```
