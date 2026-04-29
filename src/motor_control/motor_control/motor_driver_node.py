"""
motor_driver_node.py - 메인 ROS2 노드 (듀얼 갠트리)

역할:
  - EtherCATInterface 생성 및 수명주기 관리 (슬레이브 6대: 갠트리0=0-2, 갠트리1=3-5)
  - /motor/gantry0/axis_state, /motor/gantry1/axis_state 토픽 발행 (10Hz)
  - /motor/set_velocity 서비스 서버
  - /motor/get_status   서비스 서버 (gantry_index 지정)
  - HomingActionServer, MoveAxisActionServer 인스턴스화
  - MultiThreadedExecutor 로 모든 노드 실행

실행:
  ros2 launch motor_control motor_control.launch.py
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Header
from motor_control_interfaces.msg import AxisState
from motor_control_interfaces.srv import SetVelocity, GetDriveStatus

from motor_control.ethercat_interface import (
    EtherCATInterface, mm_s_to_pps, rpm_to_pps,
    X_HOMING_METHOD, Z_HOMING_METHOD,
    Z_MM_PER_REV, X_MM_PER_REV,
)
from motor_control.homing_action_server import HomingActionServer
from motor_control.move_action_server import MoveAxisActionServer


def _cia402_state_string(sw: int) -> str:
    if   (sw & 0x004F) == 0x0000: return "Not ready to switch on"
    elif (sw & 0x004F) == 0x0040: return "Switch on disabled"
    elif (sw & 0x006F) == 0x0021: return "Ready to switch on"
    elif (sw & 0x006F) == 0x0023: return "Switched on"
    elif (sw & 0x006F) == 0x0027: return "Operation enabled"
    elif (sw & 0x006F) == 0x0007: return "Quick stop active"
    elif (sw & 0x004F) == 0x000F: return "Fault reaction active"
    elif (sw & 0x004F) == 0x0008: return "Fault"
    return f"Unknown (0x{sw:04X})"


class MotorDriverNode(Node):

    def __init__(self):
        super().__init__('motor_driver_node')

        # ── ROS2 파라미터 선언 ──
        self.declare_parameter('ifname',              'enP8p1s0')
        self.declare_parameter('num_slaves',          6)
        self.declare_parameter('pdo_cycle_ms',        10)
        self.declare_parameter('max_sync_error_mm',   10.0)
        self.declare_parameter('coupling_gain',       0.05)
        self.declare_parameter('enable_coupling',     True)
        self.declare_parameter('ma_window',           5)

        # 갠트리 0 슬레이브 인덱스
        self.declare_parameter('gantry0_x_slave_idx',  0)
        self.declare_parameter('gantry0_z1_slave_idx', 1)
        self.declare_parameter('gantry0_z2_slave_idx', 2)

        # 갠트리 1 슬레이브 인덱스
        self.declare_parameter('gantry1_x_slave_idx',  3)
        self.declare_parameter('gantry1_z1_slave_idx', 4)
        self.declare_parameter('gantry1_z2_slave_idx', 5)

        # 공통 속도/가속도
        self.declare_parameter('default_velocity_rpm',  30.0)
        self.declare_parameter('default_accel_rpm_s',  1000.0)
        self.declare_parameter('default_velocity_mm_s', 75.0)

        # 호밍
        self.declare_parameter('x_homing_method',  X_HOMING_METHOD)
        self.declare_parameter('z_homing_method',  Z_HOMING_METHOD)
        self.declare_parameter('homing_fast_rpm',  30.0)
        self.declare_parameter('homing_slow_rpm',   1.0)
        self.declare_parameter('homing_timeout_s', 600.0)

        # 갠트리 0 소프트 리밋
        self.declare_parameter('gantry0_x_soft_limit_min_mm',  0.0)
        self.declare_parameter('gantry0_x_soft_limit_max_mm',  1500.0)
        self.declare_parameter('gantry0_z_soft_limit_min_mm', -1500.0)
        self.declare_parameter('gantry0_z_soft_limit_max_mm',  0.1)

        # 갠트리 1 소프트 리밋
        self.declare_parameter('gantry1_x_soft_limit_min_mm',  0.0)
        self.declare_parameter('gantry1_x_soft_limit_max_mm',  1500.0)
        self.declare_parameter('gantry1_z_soft_limit_min_mm', -1500.0)
        self.declare_parameter('gantry1_z_soft_limit_max_mm',  0.1)

        self.declare_parameter('state_publish_hz', 10.0)

        # ── 파라미터 읽기 ──
        self._ifname     = self.get_parameter('ifname').value
        self._num_slaves = self.get_parameter('num_slaves').value
        self._cycle_ms   = self.get_parameter('pdo_cycle_ms').value

        # 갠트리별 슬레이브 인덱스
        self._g0 = (
            self.get_parameter('gantry0_x_slave_idx').value,
            self.get_parameter('gantry0_z1_slave_idx').value,
            self.get_parameter('gantry0_z2_slave_idx').value,
        )
        self._g1 = (
            self.get_parameter('gantry1_x_slave_idx').value,
            self.get_parameter('gantry1_z1_slave_idx').value,
            self.get_parameter('gantry1_z2_slave_idx').value,
        )

        self._vel_rpm     = self.get_parameter('default_velocity_rpm').value
        self._accel_rpm_s = self.get_parameter('default_accel_rpm_s').value
        self._x_homing_method  = self.get_parameter('x_homing_method').value
        self._z_homing_method  = self.get_parameter('z_homing_method').value
        self._homing_fast_rpm  = self.get_parameter('homing_fast_rpm').value
        self._homing_slow_rpm  = self.get_parameter('homing_slow_rpm').value
        self._homing_timeout_s = self.get_parameter('homing_timeout_s').value

        # 활성 슬레이브 목록 (전체)
        all_indices = list(self._g0) + list(self._g1)
        self._active_indices = [i for i in all_indices if i >= 0]

        self.get_logger().info(
            f"EtherCAT: {self._ifname}, 슬레이브 {self._num_slaves}대, "
            f"사이클 {self._cycle_ms}ms"
        )
        self.get_logger().info(
            f"갠트리0: X={self._g0[0]}, Z1={self._g0[1]}, Z2={self._g0[2]}"
        )
        self.get_logger().info(
            f"갠트리1: X={self._g1[0]}, Z1={self._g1[1]}, Z2={self._g1[2]}"
        )

        # ── EtherCAT 인터페이스 생성 ──
        z_pairs = []
        if self._g0[1] >= 0 and self._g0[2] >= 0:
            z_pairs.append((self._g0[1], self._g0[2]))
        if self._g1[1] >= 0 and self._g1[2] >= 0:
            z_pairs.append((self._g1[1], self._g1[2]))

        self._ec = EtherCATInterface(
            adapter_name=self._ifname,
            num_slaves=self._num_slaves,
            cycle_time_ms=self._cycle_ms,
            max_sync_error_mm=self.get_parameter('max_sync_error_mm').value,
            coupling_gain=self.get_parameter('coupling_gain').value,
            enable_coupling=self.get_parameter('enable_coupling').value,
            ma_window=self.get_parameter('ma_window').value,
            z_pairs=z_pairs,
        )

        # ── 초기 설정 명령 큐잉 ──
        for gantry_idx, (xi, z1i, z2i) in enumerate([self._g0, self._g1]):
            x_method = self._x_homing_method
            z_method = self._z_homing_method
            if xi >= 0:
                self._ec.set_axis(xi, 'x')
                self._ec.set_velocity(xi, self._vel_rpm)
                self._ec.set_accel(xi, self._accel_rpm_s)
                self._ec.set_homing_speed(xi, self._homing_fast_rpm, self._homing_slow_rpm)
                self._ec.set_homing_method(xi, x_method)
            if z1i >= 0:
                self._ec.set_axis(z1i, 'z')
                self._ec.set_velocity(z1i, self._vel_rpm)
                self._ec.set_accel(z1i, self._accel_rpm_s)
                self._ec.set_homing_speed(z1i, self._homing_fast_rpm, self._homing_slow_rpm)
                self._ec.set_homing_method(z1i, z_method)
            if z2i >= 0:
                self._ec.set_axis(z2i, 'z')
                self._ec.set_velocity(z2i, self._vel_rpm)
                self._ec.set_accel(z2i, self._accel_rpm_s)
                self._ec.set_homing_speed(z2i, self._homing_fast_rpm, self._homing_slow_rpm)
                self._ec.set_homing_method(z2i, z_method)

        # ── 버스 시작 ──
        self._ec.start()
        self._wait_drives_ready()

        # ── Publisher (갠트리별) ──
        publish_hz = self.get_parameter('state_publish_hz').value
        self._pub_g0 = self.create_publisher(AxisState, '/motor/gantry0/axis_state', 10)
        self._pub_g1 = self.create_publisher(AxisState, '/motor/gantry1/axis_state', 10)
        self._timer_state = self.create_timer(1.0 / publish_hz, self._publish_state)

        # ── Service servers ──
        self._srv_set_vel = self.create_service(
            SetVelocity, '/motor/set_velocity', self._cb_set_velocity)
        self._srv_get_status = self.create_service(
            GetDriveStatus, '/motor/get_status', self._cb_get_status)

        self.get_logger().info("motor_driver_node 준비 완료.")

    # ── 외부 접근 메서드 ──

    def get_ethercat_interface(self) -> EtherCATInterface:
        return self._ec

    def get_gantry_indices(self) -> dict:
        """갠트리별 (x, z1, z2) 슬레이브 인덱스 반환."""
        return {0: self._g0, 1: self._g1}

    def get_soft_limits(self) -> dict:
        """갠트리별 소프트 리밋 반환."""
        return {
            0: {
                'x_min': self.get_parameter('gantry0_x_soft_limit_min_mm').value,
                'x_max': self.get_parameter('gantry0_x_soft_limit_max_mm').value,
                'z_min': self.get_parameter('gantry0_z_soft_limit_min_mm').value,
                'z_max': self.get_parameter('gantry0_z_soft_limit_max_mm').value,
            },
            1: {
                'x_min': self.get_parameter('gantry1_x_soft_limit_min_mm').value,
                'x_max': self.get_parameter('gantry1_x_soft_limit_max_mm').value,
                'z_min': self.get_parameter('gantry1_z_soft_limit_min_mm').value,
                'z_max': self.get_parameter('gantry1_z_soft_limit_max_mm').value,
            },
        }

    def get_homing_methods(self):
        return self._x_homing_method, self._z_homing_method

    def get_homing_timeout(self):
        return self._homing_timeout_s

    # ── 드라이브 준비 대기 ──
    def _wait_drives_ready(self, timeout_s: float = 10.0):
        self.get_logger().info("드라이브 준비 대기 중...")
        t0 = time.monotonic()
        while True:
            ready = all(self._ec.is_ready(i) for i in self._active_indices)
            if ready:
                self.get_logger().info("모든 드라이브 준비 완료.")
                return
            if time.monotonic() - t0 > timeout_s:
                self.get_logger().error(
                    "드라이브 준비 타임아웃. 전원·EtherCAT 연결 확인 또는 "
                    "이전 프로세스 종료(pkill -9 -f motor_driver_node) 후 재시도하세요."
                )
                return
            time.sleep(0.1)

    # ── AxisState 발행 (갠트리별) ──
    def _publish_state(self):
        if not self._ec.is_alive():
            return
        stamp = self.get_clock().now().to_msg()
        self._pub_g0.publish(self._make_axis_state(0, stamp))
        self._pub_g1.publish(self._make_axis_state(1, stamp))

    def _make_axis_state(self, gantry_idx: int, stamp) -> AxisState:
        ec = self._ec
        xi, z1i, z2i = (self._g0 if gantry_idx == 0 else self._g1)

        x_pos  = ec.get_position_mm(xi,  'x') if xi  >= 0 else 0.0
        z1_pos = ec.get_position_mm(z1i, 'z') if z1i >= 0 else 0.0
        z2_pos = ec.get_position_mm(z2i, 'z') if z2i >= 0 else 0.0

        active_z = [p for i, p in [(z1i, z1_pos), (z2i, z2_pos)] if i >= 0]
        z_pos  = sum(active_z) / len(active_z) if active_z else 0.0
        z_sync = abs(z1_pos - z2_pos) if (z1i >= 0 and z2i >= 0) else 0.0

        x_vel  = ec.get_velocity_mm_s(xi,  'x') if xi  >= 0 else 0.0
        z1_vel = ec.get_velocity_mm_s(z1i, 'z') if z1i >= 0 else 0.0
        z2_vel = ec.get_velocity_mm_s(z2i, 'z') if z2i >= 0 else 0.0
        active_zv = [v for i, v in [(z1i, z1_vel), (z2i, z2_vel)] if i >= 0]
        z_vel  = sum(active_zv) / len(active_zv) if active_zv else 0.0

        active = [i for i in [xi, z1i, z2i] if i >= 0]
        sw_ref = ec.get_status_word(active[0]) if active else 0

        msg = AxisState()
        msg.header = Header()
        msg.header.stamp    = stamp
        msg.header.frame_id = f'gantry{gantry_idx}'
        msg.x_position_mm   = x_pos
        msg.z_position_mm   = z_pos
        msg.z1_position_mm  = z1_pos
        msg.z2_position_mm  = z2_pos
        msg.x_velocity_mm_s = x_vel
        msg.z_velocity_mm_s = z_vel
        msg.z_sync_error_mm = z_sync
        msg.x_homed         = ec.is_homed(xi)  if xi  >= 0 else True
        msg.z_homed         = (
            (ec.is_homed(z1i) if z1i >= 0 else True) and
            (ec.is_homed(z2i) if z2i >= 0 else True)
        )
        msg.drive_state = _cia402_state_string(sw_ref)
        msg.sync_fault  = ec.has_sync_error()
        return msg

    # ── /motor/set_velocity 서비스 ──
    def _cb_set_velocity(self, request, response):
        axis    = request.axis.lower()
        vel_mms = request.velocity_mm_s

        if vel_mms <= 0.0:
            response.success = False
            response.message = "velocity_mm_s는 0보다 커야 합니다."
            return response

        rpm_z = (vel_mms / Z_MM_PER_REV) * 60.0
        rpm_x = (vel_mms / X_MM_PER_REV) * 60.0

        for xi, z1i, z2i in [self._g0, self._g1]:
            if axis in ('z', 'all'):
                if z1i >= 0: self._ec.set_velocity(z1i, rpm_z)
                if z2i >= 0: self._ec.set_velocity(z2i, rpm_z)
            if axis in ('x', 'all'):
                if xi  >= 0: self._ec.set_velocity(xi, rpm_x)

        response.success = True
        response.message = f"속도 설정: {vel_mms:.2f}mm/s (axis={axis}, 두 갠트리 공통)"
        self.get_logger().info(response.message)
        return response

    # ── /motor/get_status 서비스 ──
    def _cb_get_status(self, request, response):
        ec = self._ec
        gantry_idx = request.gantry_index
        xi, z1i, z2i = (self._g0 if gantry_idx == 0 else self._g1)

        active = [i for i in [xi, z1i, z2i] if i >= 0]
        sw_ref = ec.get_status_word(active[0]) if active else 0

        response.ethercat_connected = ec.is_alive()
        response.cia402_state   = _cia402_state_string(sw_ref)
        response.x_homed        = ec.is_homed(xi)  if xi  >= 0 else True
        response.z_homed        = (
            (ec.is_homed(z1i) if z1i >= 0 else True) and
            (ec.is_homed(z2i) if z2i >= 0 else True)
        )
        response.x_position_mm  = ec.get_position_mm(xi,  'x') if xi  >= 0 else 0.0
        response.z_position_mm  = ec.get_position_mm(z1i, 'z') if z1i >= 0 else 0.0
        response.sync_fault     = ec.has_sync_error()
        response.error_code     = f"Fault (sw=0x{sw_ref:04X})" if (sw_ref & 0x0008) else "none"
        return response

    def destroy_node(self):
        self.get_logger().info("모터 드라이버 노드 종료 중...")
        self._ec.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    motor_node = MotorDriverNode()
    ec         = motor_node.get_ethercat_interface()

    if not ec.is_alive():
        motor_node.get_logger().fatal(
            "EtherCAT 서브프로세스가 시작 직후 종료됐습니다. "
            "이전 프로세스가 어댑터를 점유 중이거나 어댑터 이름이 잘못됐을 수 있습니다. "
            "'pkill -9 -f motor_driver_node' 로 이전 프로세스를 종료 후 재시도하세요."
        )
        motor_node.destroy_node()
        rclpy.shutdown()
        return

    gantry_indices  = motor_node.get_gantry_indices()
    x_method, z_method = motor_node.get_homing_methods()

    homing_node = HomingActionServer(
        ec,
        gantry_indices=gantry_indices,
        x_method=x_method,
        z_method=z_method,
        timeout_s=motor_node.get_homing_timeout(),
    )
    move_node = MoveAxisActionServer(
        ec,
        gantry_indices=gantry_indices,
        soft_limits=motor_node.get_soft_limits(),
        max_sync_error_mm=motor_node.get_parameter('max_sync_error_mm').value,
    )

    executor = MultiThreadedExecutor()
    executor.add_node(motor_node)
    executor.add_node(homing_node)
    executor.add_node(move_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        motor_node.destroy_node()
        homing_node.destroy_node()
        move_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
