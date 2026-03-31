# RPLidar ROS2 변환 계획서

> **원본 코드:** `.source/motor-control/rplidar.py`
> **대상 환경:** ROS2 Humble, RPLIDAR S2 (`/dev/rplidar`, baudrate=1,000,000)

---

## 1. 원본 코드 구조 분석

### 1-1. 클래스 및 함수 목록

```
rplidar.py
├── 상수 정의 (SYNC, CMD_*)
├── 데이터 클래스
│   ├── DeviceInfo    — 디바이스 정보 (모델, 펌웨어, 시리얼번호)
│   ├── DeviceHealth  — 상태 (Good / Warning / Error)
│   └── Point         — 측정 포인트 (x, y, z, intensity)
├── S2M1Lidar         — 시리얼 통신 저수준 드라이버
│   ├── _send()       — 명령 패킷 전송
│   ├── _read_desc()  — 응답 디스크립터 파싱
│   ├── reset()       — 장치 리셋
│   ├── stop()        — 스캔 정지
│   ├── set_motor_pwm() — 모터 속도 설정 (0~1023)
│   ├── get_info()    — 디바이스 정보 조회
│   ├── get_health()  — 상태 조회
│   └── start_scan()  — 스캔 시작
├── start_lidar()     — 초기화 헬퍼 (health 확인 → PWM 설정 → scan 시작)
├── _stream_raw_scans() — 5바이트 패킷 파싱 → (start_bit, angle, distance, quality)
├── stream_frames()   — 1회전 단위 프레임으로 묶기 → List[Point]
├── stop_lidar()      — 종료 헬퍼
├── _lidar_process_loop() — 별도 프로세스에서 실행되는 데이터 수집 루프
└── LidarController   — 멀티프로세싱 제어 클래스
    ├── start()
    ├── stop()
    └── get_latest_frame()
```

---

### 1-2. 통신 프로토콜 분석

#### 명령 패킷 구조

```
[페이로드 없음]  : 0xA5  CMD
[페이로드 있음]  : 0xA5  CMD  LEN  PAYLOAD...  CHECKSUM
                  ↑SYNC  ↑명령 ↑길이           ↑XOR(LEN ^ PAYLOAD bytes)
```

#### 응답 디스크립터 구조 (7바이트)

```
Byte 0-1 : 0xA5 0x5A  (동기 마커)
Byte 2-5 : raw_len (4바이트 little-endian)
           - bits 0-29 : payload_size (실제 데이터 크기)
           - bits 30-31: mode_bits (응답 모드)
Byte 6   : data_type (mode_byte)
```

#### 스캔 측정 패킷 구조 (5바이트, 반복)

```
Byte 0 : quality(6bit) | inv_start_bit(1bit) | start_bit(1bit)
         start_bit=1 → 새로운 1회전 시작
         inv_start_bit = NOT start_bit (검증용)
Byte 1 : check_bit(1bit=항상1) | angle_q6[6:0](7bit)
Byte 2 : angle_q6[13:7] (8bit)
         → 실제 각도 = angle_q6 / 64.0 (도)
Byte 3 : distance_q2[7:0] (8bit)
Byte 4 : distance_q2[15:8] (8bit)
         → 실제 거리 = dist_q2 / 4.0 (mm)
```

#### 데이터 흐름

```
시리얼 포트 5바이트 패킷
    │
    ▼
_stream_raw_scans()
  - 패킷 파싱 및 검증
  - start_bit, angle(도), distance(mm), quality 반환
    │
    ▼
stream_frames()
  - start_bit=1 감지 → 이전 프레임 yield
  - distance=0 포인트 제거 (drop_invalid)
  - 극좌표 → 직교좌표 변환
    x = distance × cos(angle_rad)
    y = distance × sin(angle_rad)
  - List[Point(x, y, z=0, intensity=quality)] 반환
    │
    ▼
LidarController (별도 프로세스)
  - data_queue에 최신 프레임 저장 (큐 크기 2 유지)
  - get_latest_frame()으로 메인 프로세스에 제공
```

---

## 2. 현재 방식의 한계

| 항목 | 원본 Python 코드 | ROS2 변환 후 |
|------|----------------|-------------|
| 데이터 전달 | multiprocessing Queue | ROS2 토픽 (`/scan`) |
| 데이터 포맷 | `List[Point]` (커스텀) | `sensor_msgs/LaserScan` (표준) |
| 시각화 | 없음 | RViz2 자동 지원 |
| 다른 노드와 연동 | 직접 함수 호출 | 토픽 구독으로 분리 |
| 각도 보정 | 없음 | sllidar_ros2 드라이버가 처리 |
| 재사용성 | 이 프로젝트 전용 | 범용 ROS2 생태계 |

---

## 3. ROS2 변환 전략

### 전략: 직접 구현 대신 sllidar_ros2 드라이버 활용

원본 코드를 그대로 ROS2 노드로 변환하는 것보다
**이미 테스트 완료된 `sllidar_ros2` 드라이버를 사용**하고
그 위에 **처리 노드**를 추가하는 것이 더 효율적입니다.

```
[sllidar_ros2 드라이버]     [커스텀 처리 노드]
  시리얼 통신               /scan 구독
  프로토콜 파싱      →      포인트 필터링
  /scan 발행               장애물 감지
                           모터 제어 연동
```

---

## 4. 변환 구조 설계

### 4-1. 원본 코드 → ROS2 매핑

| 원본 역할 | ROS2 대응 |
|---------|----------|
| `S2M1Lidar` (시리얼 통신) | `sllidar_ros2` 드라이버 내부 처리 |
| `_stream_raw_scans()` | sllidar_ros2 내부 처리 |
| `stream_frames()` (1회전 프레임) | `sensor_msgs/LaserScan` (1회전 = 1메시지) |
| `Point(x, y, intensity)` | `LaserScan.ranges[]` + `LaserScan.intensities[]` |
| `LidarController.get_latest_frame()` | `/scan` 토픽 구독 콜백 |
| `set_motor_pwm(pwm)` | sllidar_ros2 파라미터로 설정 |

### 4-2. sensor_msgs/LaserScan 구조

```
header:
  stamp:      # 타임스탬프
  frame_id:   # "laser" (좌표계 이름)

angle_min:    # 스캔 시작 각도 (라디안)
angle_max:    # 스캔 종료 각도 (라디안)
angle_increment: # 각도 해상도 (라디안)
time_increment:  # 측정 간 시간 간격
scan_time:    # 1회전 시간

range_min:    # 최소 유효 거리 (m) ← 원본: mm 단위, ROS2: m 단위
range_max:    # 최대 유효 거리 (m)

ranges[]      # 거리 배열 (m) ← 원본의 distance/1000.0
intensities[] # 반사 강도 배열 ← 원본의 quality
```

> **주의:** 원본 코드는 거리 단위가 **mm**, ROS2 LaserScan은 **m** 입니다.

### 4-3. 커스텀 처리 노드 구조

```
lidar_processor_node.py
├── 구독: /scan (sensor_msgs/LaserScan)
├── 발행: /lidar/obstacles (커스텀 또는 표준 메시지)
│
├── scan_callback()
│   ├── ranges[], intensities[] 읽기
│   ├── 유효 포인트 필터링 (range_min ~ range_max)
│   ├── 극좌표 → 직교좌표 변환
│   │   x = range × cos(angle_min + i × angle_increment)
│   │   y = range × sin(angle_min + i × angle_increment)
│   ├── 관심 구역(ROI) 필터링
│   └── 장애물 감지 결과 발행
│
└── 파라미터
    ├── roi_x_min/max (m)
    ├── roi_y_min/max (m)
    └── min_quality (강도 임계값)
```

---

## 5. 원본 코드의 좌표 변환 vs ROS2 표준

### 원본 코드

```python
angle_rad = math.radians(angle)   # angle: 0~360도
x = distance * math.cos(angle_rad)
y = distance * math.sin(angle_rad)
# 단위: mm
```

### ROS2 처리 노드

```python
for i, r in enumerate(scan.ranges):
    if scan.range_min <= r <= scan.range_max:
        angle = scan.angle_min + i * scan.angle_increment  # 이미 라디안
        x = r * math.cos(angle)  # 단위: m
        y = r * math.sin(angle)  # 단위: m
```

---

## 6. 구현할 파일 목록

```
src/motor_control/
└── motor_control/
    └── lidar_processor_node.py   ← 신규 작성
```

기존 `motor_driver_node.py` 와 별도 노드로 분리하여
`/scan` 토픽을 구독하고 처리 결과를 발행합니다.

launch 파일에서 sllidar_ros2 노드와 함께 실행:

```
motor_control.launch.py 또는 신규 lidar.launch.py
├── sllidar_node (sllidar_ros2 패키지)
│     → /scan 발행
└── lidar_processor_node (motor_control 패키지)
      → /scan 구독
      → /lidar/obstacles 발행
```

---

## 7. 구현 전 확인 사항

- [ ] `/scan` 토픽 정상 발행 확인: `ros2 topic echo /scan --once`
- [ ] 라이다 장착 방향 확인 (angle_min 기준점이 어느 방향인지)
- [ ] 처리 노드에서 필요한 ROI (관심 구역) 범위 결정
- [ ] 장애물 감지 결과를 어떤 메시지 타입으로 발행할지 결정
  - `geometry_msgs/PointStamped` — 단일 포인트
  - `sensor_msgs/PointCloud2` — 전체 포인트 클라우드
  - 커스텀 msg — 장애물 여부 bool + 최근접 거리
