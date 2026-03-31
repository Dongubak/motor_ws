# Z축 단독 테스트 가이드

> 갠트리 로봇에서 분리된 Z축 드라이버 단독 테스트

---

## 사전 확인

### 하드웨어 체크리스트

- [ ] Z축 드라이버 전원 ON
- [ ] EtherCAT 케이블: **Jetson** `enP8p1s0` 포트 → Z축 드라이버 IN 포트
- [ ] 모터 인코더 케이블 연결 확인
- [ ] POT 리밋 스위치 배선 확인 (DI1 핀)
- [ ] Z축 이동 경로 장애물 없음 확인

### POT 리밋 스위치 배선 (DI1)

```
드라이버 DI1 핀 ──── POT 스위치 COM
드라이버 COM 핀 ──── GND (0V)
POT 스위치 NC  ──── 드라이버 DI1  (평상시 ON, 눌리면 OFF)
```

> iX7NH 기본 DI 할당: DI1=POT, DI2=NOT, DI3=HOME

---

## motor_params.yaml 설정 변경

`src/motor_control/config/motor_params.yaml` 을 아래와 같이 수정:

```yaml
ifname: "enP8p1s0"

num_slaves: 1
x_slave_idx:  -1   # 비활성
z1_slave_idx:  0   # Z 드라이버 (단독)
z2_slave_idx: -1   # 비활성

z_homing_method: 2   # 정방향(+) → POT → Index
pdo_cycle_ms: 10
default_velocity_rpm: 10.0
```

변경 후 **재빌드 없이** 바로 실행 가능합니다 (`--symlink-install` 적용됨).

---

## 테스트 절차

### 터미널 1 — 시스템 시작 (root)

```bash
sudo -s
source /opt/ros/humble/setup.bash
source /home/gardentech/motor_ws/install/setup.bash

ros2 launch motor_control motor_control.launch.py
```

**정상 출력 예시:**

```
[motor_driver_node]: EtherCAT 인터페이스: enP8p1s0, 슬레이브 수: 1, 사이클: 10ms
[motor_driver_node]: 활성 슬레이브: X=-1, Z1=0, Z2=-1 (호밍: X=Method1, Z=Method2)
[EtherCAT] 1개 슬레이브 발견
[EtherCAT] OP 상태 진입 완료. 제어 루프 시작.
[motor_driver_node]: 모든 드라이브 준비 완료.
[homing_action_server]: 활성 축: Z1(idx=0, method=2)
```

---

### 터미널 2 — 상태 모니터링 (root)

```bash
sudo -s
source /opt/ros/humble/setup.bash
source /home/gardentech/motor_ws/install/setup.bash

ros2 topic echo /motor/axis_state
```

**정상 출력 예시 (호밍 전):**

```
z_position_mm: 0.0
z1_position_mm: 0.0
z_velocity_mm_s: 0.0
x_homed: true        ← X 비활성이므로 true (무시)
z_homed: false
drive_state: 'Operation enabled'
sync_fault: false
```

---

### 터미널 3 — 테스트 명령 실행 (root)

```bash
sudo -s
source /opt/ros/humble/setup.bash
source /home/gardentech/motor_ws/install/setup.bash
```

#### STEP 1. 드라이브 상태 조회

```bash
ros2 service call /motor/get_status motor_control_interfaces/srv/GetDriveStatus
```

`ethercat_connected: true`, `cia402_state: 'Operation enabled'` 확인.

---

#### STEP 2. 원점 복귀 (Homing)

> Z축이 **정방향(+)** 으로 이동하면서 POT 스위치를 찾습니다.

```bash
ros2 action send_goal /motor/homing motor_control_interfaces/action/Homing \
  "{force_rehome: false}"
```

**진행 중 피드백:**

```
feedback:
  phase: in_progress
  current_z_mm: 12.34
```

**완료 시 결과:**

```
result:
  success: true
  message: 호밍 완료.
  z_home_position_mm: 0.0
```

호밍 실패 시 → [트러블슈팅](#트러블슈팅) 참조.

---

#### STEP 3. 저속 소폭 이동 테스트

```bash
# 30mm 이동 (속도 10mm/s)
ros2 action send_goal /motor/move motor_control_interfaces/action/MoveAxis \
  "{x_target_mm: -1.0, z_target_mm: 30.0, velocity_mm_s: 10.0}"
```

---

#### STEP 4. 왕복 이동 테스트

```bash
# 100mm로 이동
ros2 action send_goal /motor/move motor_control_interfaces/action/MoveAxis \
  "{x_target_mm: -1.0, z_target_mm: 100.0, velocity_mm_s: 15.0}"

# 원점(0mm)으로 복귀
ros2 action send_goal /motor/move motor_control_interfaces/action/MoveAxis \
  "{x_target_mm: -1.0, z_target_mm: 0.0, velocity_mm_s: 15.0}"
```

---

#### STEP 5. 속도 변경 테스트

```bash
# 속도를 30mm/s 로 변경
ros2 service call /motor/set_velocity motor_control_interfaces/srv/SetVelocity \
  "{velocity_mm_s: 30.0, axis: 'z'}"

# 200mm로 이동
ros2 action send_goal /motor/move motor_control_interfaces/action/MoveAxis \
  "{x_target_mm: -1.0, z_target_mm: 200.0, velocity_mm_s: 30.0}"
```

---

#### STEP 6. 소프트 리밋 확인

```bash
# 소프트 리밋 초과 (301mm) → 거부되어야 함
ros2 action send_goal /motor/move motor_control_interfaces/action/MoveAxis \
  "{x_target_mm: -1.0, z_target_mm: 301.0, velocity_mm_s: 10.0}"
```

`success: false` 와 리밋 초과 메시지가 반환되면 정상.

---

## 테스트 체크리스트

| 항목 | 확인 |
|---|---|
| EtherCAT 연결 (`ethercat_connected: true`) | [ ] |
| Operation Enabled 상태 | [ ] |
| 원점 복귀 완료 (`z_homed: true`) | [ ] |
| 목표 위치 도달 (위치 오차 < 0.1mm) | [ ] |
| 왕복 이동 정상 | [ ] |
| 소프트 리밋 거부 동작 | [ ] |

---

## 테스트 완료 후 — 다음 단계 준비

Z축 단독 테스트 통과 후, 전체 갠트리 테스트를 위해 `motor_params.yaml` 을 복원합니다:

```yaml
num_slaves: 3
x_slave_idx:  0
z1_slave_idx: 1
z2_slave_idx: 2
```

---

## 트러블슈팅

### EtherCAT 열기 실패
```
could not open interface enP8p1s0
```
- **반드시 `sudo -s` (root) 로 실행**해야 합니다
- `sudo pip3 install pysoem` 로 root 환경에 pysoem 설치 확인

### 슬레이브를 찾지 못함
```
슬레이브 부족: 필요=1, 발견=0
```
- EtherCAT 케이블 `enP8p1s0` 포트 연결 확인
- Z축 드라이버 전원 확인

### 호밍 오류
```
result: success: false
```
- POT 스위치(DI1) 배선 확인
- 모터가 이미 POT 위치에 있는 경우 수동으로 역방향 이동 후 재시도
- `force_rehome: true` 로 강제 재호밍 시도

### 이동 거부 (호밍 미완료)
```
success: false  ← 호밍 미완료
```
- STEP 2 원점 복귀를 먼저 완료한 후 이동 명령 전송

### 빌드/소싱 오류 시

```bash
cd /home/gardentech/motor_ws
rm -rf build/ install/ log/
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```
