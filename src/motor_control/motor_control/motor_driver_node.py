"""
motor_driver_node.py - 메인 ROS2 노드

역할:
  - EtherCATInterface 생성 및 수명주기 관리
  - /motor/axis_state 토픽 10Hz 발행
  - /motor/set_velocity 서비스 서버
  - /motor/get_status   서비스 서버
  - HomingActionServer, MoveAxisActionServer 인스턴스화
  - MultiThreadedExecutor 로 모든 노드 실행

실행:
  ros2 run motor_control motor_driver_node
  또는
  ros2 launch motor_control motor_control.launch.py
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter

from std_msgs.msg import Header
from motor_control_interfaces.msg import AxisState
from motor_control_interfaces.srv import SetVelocity, GetDriveStatus

from motor_control.ethercat_interface import (
    EtherCATInterface, mm_s_to_pps, rpm_to_pps,
    X_HOMING_METHOD, Z_HOMING_METHOD,
)
from motor_control.homing_action_server import HomingActionServer
from motor_control.move_action_server import MoveAxisActionServer


# ─────────────────────────────────────────────────────────────
# CiA 402 상태 문자열 변환
# ─────────────────────────────────────────────────────────────
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


# ─────────────────────────────────────────────────────────────
# 메인 드라이버 노드
# ─────────────────────────────────────────────────────────────
class MotorDriverNode(Node):

    def __init__(self):
        super().__init__('motor_driver_node')

        # ── ROS2 파라미터 선언 ──
        self.declare_parameter('ifname',              'eth0')
        self.declare_parameter('num_slaves',          3)
        self.declare_parameter('pdo_cycle_ms',        1)
        self.declare_parameter('max_sync_error_mm',   0.5)
        self.declare_parameter('coupling_gain',       0.01)
        self.declare_parameter('enable_coupling',     True)
        self.declare_parameter('ma_window',           5)
        self.declare_parameter('x_slave_idx',         0)
        self.declare_parameter('z1_slave_idx',        1)
        self.declare_parameter('z2_slave_idx',        2)
        self.declare_parameter('default_velocity_rpm',300.0)
        self.declare_parameter('default_accel_rpm_s', 300.0)
        self.declare_parameter('x_soft_limit_min_mm', 0.0)
        self.declare_parameter('x_soft_limit_max_mm', 1000.0)
        self.declare_parameter('z_soft_limit_min_mm', 0.0)
        self.declare_parameter('z_soft_limit_max_mm', 300.0)
        self.declare_parameter('state_publish_hz',    10.0)
        self.declare_parameter('x_homing_method',     X_HOMING_METHOD)
        self.declare_parameter('z_homing_method',     Z_HOMING_METHOD)
        self.declare_parameter('homing_fast_rpm',     30.0)
        self.declare_parameter('homing_slow_rpm',      1.0)
        self.declare_parameter('homing_timeout_s',   120.0)

        # ── 파라미터 읽기 ──
        self._ifname      = self.get_parameter('ifname').value
        self._num_slaves  = self.get_parameter('num_slaves').value
        self._cycle_ms    = self.get_parameter('pdo_cycle_ms').value
        self._x_idx       = self.get_parameter('x_slave_idx').value
        self._z1_idx      = self.get_parameter('z1_slave_idx').value
        self._z2_idx      = self.get_parameter('z2_slave_idx').value
        self._vel_rpm     = self.get_parameter('default_velocity_rpm').value
        self._accel_rpm_s = self.get_parameter('default_accel_rpm_s').value
        self._x_homing_method  = self.get_parameter('x_homing_method').value
        self._z_homing_method  = self.get_parameter('z_homing_method').value
        self._homing_fast_rpm  = self.get_parameter('homing_fast_rpm').value
        self._homing_slow_rpm  = self.get_parameter('homing_slow_rpm').value
        self._homing_timeout_s = self.get_parameter('homing_timeout_s').value

        # 활성 슬레이브 인덱스 목록 (idx >= 0 인 것만)
        self._active_indices = [
            i for i in [self._x_idx, self._z1_idx, self._z2_idx] if i >= 0
        ]

        self.get_logger().info(
            f"EtherCAT 인터페이스: {self._ifname}, "
            f"슬레이브 수: {self._num_slaves}, "
            f"사이클: {self._cycle_ms}ms"
        )
        self.get_logger().info(
            f"활성 슬레이브: X={self._x_idx}, Z1={self._z1_idx}, Z2={self._z2_idx} "
            f"(호밍: X=Method{self._x_homing_method}, Z=Method{self._z_homing_method})"
        )

        # ── EtherCAT 인터페이스 생성 ──
        self._ec = EtherCATInterface(
            adapter_name=self._ifname,
            num_slaves=self._num_slaves,
            cycle_time_ms=self._cycle_ms,
            max_sync_error_mm=self.get_parameter('max_sync_error_mm').value,
            coupling_gain=self.get_parameter('coupling_gain').value,
            enable_coupling=self.get_parameter('enable_coupling').value,
            ma_window=self.get_parameter('ma_window').value,
            z1_idx=self._z1_idx,
            z2_idx=self._z2_idx,
        )

        # ── 초기 설정 명령 큐잉 (버스 시작 전, 활성 슬레이브만) ──
        if self._x_idx >= 0:
            self._ec.set_axis(self._x_idx, 'x')
            self._ec.set_velocity(self._x_idx, self._vel_rpm)
            self._ec.set_accel(self._x_idx, self._accel_rpm_s)
            self._ec.set_homing_speed(self._x_idx, self._homing_fast_rpm, self._homing_slow_rpm)
            self._ec.set_homing_method(self._x_idx, self._x_homing_method)
        if self._z1_idx >= 0:
            self._ec.set_axis(self._z1_idx, 'z')
            self._ec.set_velocity(self._z1_idx, self._vel_rpm)
            self._ec.set_accel(self._z1_idx, self._accel_rpm_s)
            self._ec.set_homing_speed(self._z1_idx, self._homing_fast_rpm, self._homing_slow_rpm)
            self._ec.set_homing_method(self._z1_idx, self._z_homing_method)
        if self._z2_idx >= 0:
            self._ec.set_axis(self._z2_idx, 'z')
            self._ec.set_velocity(self._z2_idx, self._vel_rpm)
            self._ec.set_accel(self._z2_idx, self._accel_rpm_s)
            self._ec.set_homing_speed(self._z2_idx, self._homing_fast_rpm, self._homing_slow_rpm)
            self._ec.set_homing_method(self._z2_idx, self._z_homing_method)

        # ── 버스 시작 ──
        self._ec.start()

        # ── 드라이브 준비 대기 ──
        self._wait_drives_ready()

        # ── Publisher ──
        publish_hz = self.get_parameter('state_publish_hz').value
        self._pub_state = self.create_publisher(AxisState, '/motor/axis_state', 10)
        self._timer_state = self.create_timer(1.0 / publish_hz, self._publish_state)

        # ── Service servers ──
        self._srv_set_vel = self.create_service(
            SetVelocity, '/motor/set_velocity', self._cb_set_velocity)
        self._srv_get_status = self.create_service(
            GetDriveStatus, '/motor/get_status', self._cb_get_status)

        self.get_logger().info("motor_driver_node 준비 완료.")

    def get_ethercat_interface(self) -> EtherCATInterface:
        return self._ec

    def get_slave_indices(self):
        return self._x_idx, self._z1_idx, self._z2_idx

    def get_soft_limits(self):
        return {
            'x_min': self.get_parameter('x_soft_limit_min_mm').value,
            'x_max': self.get_parameter('x_soft_limit_max_mm').value,
            'z_min': self.get_parameter('z_soft_limit_min_mm').value,
            'z_max': self.get_parameter('z_soft_limit_max_mm').value,
        }

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
                self.get_logger().warn(
                    "드라이브 준비 타임아웃 (계속 진행). "
                    "드라이브 전원 및 EtherCAT 연결을 확인하세요."
                )
                return
            time.sleep(0.1)

    # ── /motor/axis_state 발행 ──
    def _publish_state(self):
        if not self._ec.is_alive():
            return

        ec = self._ec
        xi, z1i, z2i = self._x_idx, self._z1_idx, self._z2_idx

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

        ref_idx = self._active_indices[0] if self._active_indices else 0
        sw_ref  = ec.get_status_word(ref_idx)

        msg = AxisState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'motor'

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
        msg.drive_state     = _cia402_state_string(sw_ref)
        msg.sync_fault      = ec.has_sync_error()

        self._pub_state.publish(msg)

    # ── /motor/set_velocity 서비스 ──
    def _cb_set_velocity(self, request, response):
        axis    = request.axis.lower()
        vel_mms = request.velocity_mm_s
        xi, z1i, z2i = self._x_idx, self._z1_idx, self._z2_idx

        if vel_mms <= 0.0:
            response.success = False
            response.message = "velocity_mm_s는 0보다 커야 합니다."
            return response

        # mm/s → RPM 변환 (Z축 기준)
        from motor_control.ethercat_interface import Z_MM_PER_REV, EFFECTIVE_PPR
        rpm = (vel_mms / Z_MM_PER_REV) * 60.0

        if axis in ('z', 'all'):
            self._ec.set_velocity(z1i, rpm)
            self._ec.set_velocity(z2i, rpm)
        if axis in ('x', 'all'):
            from motor_control.ethercat_interface import X_MM_PER_REV
            rpm_x = (vel_mms / X_MM_PER_REV) * 60.0
            self._ec.set_velocity(xi, rpm_x)

        response.success = True
        response.message = f"속도 설정: {vel_mms:.2f}mm/s (axis={axis})"
        self.get_logger().info(response.message)
        return response

    # ── /motor/get_status 서비스 ──
    def _cb_get_status(self, request, response):
        ec = self._ec
        xi, z1i, z2i = self._x_idx, self._z1_idx, self._z2_idx

        ref_idx = self._active_indices[0] if self._active_indices else 0
        sw_ref  = ec.get_status_word(ref_idx)

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

        if sw_ref & 0x0008:
            response.error_code = f"Fault (sw=0x{sw_ref:04X})"
        else:
            response.error_code = "none"

        return response

    def get_homing_methods(self):
        return self._x_homing_method, self._z_homing_method

    def get_homing_timeout(self):
        return self._homing_timeout_s

    def destroy_node(self):
        self.get_logger().info("모터 드라이버 노드 종료 중...")
        self._ec.stop()
        super().destroy_node()


# ─────────────────────────────────────────────────────────────
# 진입점
# ─────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)

    motor_node   = MotorDriverNode()
    ec           = motor_node.get_ethercat_interface()
    xi, z1i, z2i = motor_node.get_slave_indices()
    x_method, z_method = motor_node.get_homing_methods()

    homing_node  = HomingActionServer(
        ec, xi, z1i, z2i,
        x_method=x_method,
        z_method=z_method,
        timeout_s=motor_node.get_homing_timeout(),
    )
    move_node    = MoveAxisActionServer(
        ec, xi, z1i, z2i,
        soft_limits=motor_node.get_soft_limits(),
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
