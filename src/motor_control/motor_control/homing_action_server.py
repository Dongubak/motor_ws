"""
homing_action_server.py - 호밍 액션 서버 (CiA 402 Homing Mode)

액션 타입: motor_control/action/Homing
토픽:      /motor/homing

슬레이브 인덱스가 -1이면 해당 축을 건너뜀 (비활성).

호밍 메서드 (0x6098):
  X축: Method 1 — 역방향(-) + NOT 리밋 스위치 + Index 펄스
  Z축: Method 2 — 정방향(+) + POT 리밋 스위치 + Index 펄스
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from motor_control_interfaces.action import Homing
from motor_control.ethercat_interface import (
    EtherCATInterface, X_HOMING_METHOD, Z_HOMING_METHOD,
)


class HomingActionServer(Node):

    def __init__(self, ec: EtherCATInterface,
                 x_idx:  int = 0,
                 z1_idx: int = 1,
                 z2_idx: int = 2,
                 x_method: int = X_HOMING_METHOD,
                 z_method: int = Z_HOMING_METHOD,
                 timeout_s: float = 120.0):
        super().__init__('homing_action_server')

        self._ec        = ec
        self._x         = x_idx
        self._z1        = z1_idx
        self._z2        = z2_idx
        self._x_method  = x_method
        self._z_method  = z_method
        self._timeout_s = timeout_s

        # 활성 슬레이브 목록
        self._active = {
            'x':  x_idx  >= 0,
            'z1': z1_idx >= 0,
            'z2': z2_idx >= 0,
        }

        self._action_server = ActionServer(
            self,
            Homing,
            '/motor/homing',
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=ReentrantCallbackGroup(),
        )

        axes_info = []
        if self._active['x']:  axes_info.append(f"X(idx={x_idx}, method={x_method})")
        if self._active['z1']: axes_info.append(f"Z1(idx={z1_idx}, method={z_method})")
        if self._active['z2']: axes_info.append(f"Z2(idx={z2_idx}, method={z_method})")
        self.get_logger().info(
            f"HomingActionServer 준비 완료. 활성 축: {', '.join(axes_info)}"
        )

    def _goal_cb(self, goal_request):
        self.get_logger().info(
            f"호밍 Goal 수신 (force_rehome={goal_request.force_rehome})")
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().info("호밍 Cancel 요청 수신")
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle):
        ec    = self._ec
        force = goal_handle.request.force_rehome

        result = Homing.Result()

        # ── 이미 호밍 완료된 경우 ──
        if not force and self._all_homed():
            self.get_logger().info("이미 호밍 완료 상태 (force_rehome=False → 스킵)")
            result.success            = True
            result.message            = "이미 호밍 완료 상태."
            result.z_home_position_mm = self._z_position()
            goal_handle.succeed()
            return result

        # ── EtherCAT 동작 확인 ──
        if not ec.is_alive():
            result.success = False
            result.message = "EtherCAT 프로세스가 실행 중이지 않습니다."
            goal_handle.abort()
            return result

        # ── 드라이브 준비 대기 ──
        self.get_logger().info("드라이브 준비 대기 중...")
        t0 = time.monotonic()
        active_indices = [
            i for i in [self._x, self._z1, self._z2] if i >= 0
        ]
        while not all(ec.is_ready(i) for i in active_indices):
            if time.monotonic() - t0 > 8.0:
                result.success = False
                result.message = "드라이브 준비 타임아웃."
                goal_handle.abort()
                return result
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "호밍 취소됨."
                return result
            time.sleep(0.1)

        # ── 호밍 시작 (활성 축만) ──
        self.get_logger().info("호밍 시작...")
        if self._active['x']:
            ec.start_homing(self._x, self._x_method)
            self.get_logger().info(
                f"  X축 호밍 시작 (Method {self._x_method}: NOT + Index)")
        if self._active['z1']:
            ec.start_homing(self._z1, self._z_method)
            self.get_logger().info(
                f"  Z1 호밍 시작 (Method {self._z_method}: POT + Index)")
        if self._active['z2']:
            ec.start_homing(self._z2, self._z_method)
            self.get_logger().info(
                f"  Z2 호밍 시작 (Method {self._z_method}: POT + Index)")

        # 서브프로세스가 START_HOMING 명령을 처리해 phase=1 로 전환될 때까지 대기.
        # 큐 처리 전에 폴링을 시작하면 이전 phase=2 를 보고 즉시 완료로 오판한다.
        t_wait = time.monotonic()
        while True:
            ph_x  = ec.get_homing_phase(self._x)  if self._active['x']  else 2
            ph_z1 = ec.get_homing_phase(self._z1) if self._active['z1'] else 2
            ph_z2 = ec.get_homing_phase(self._z2) if self._active['z2'] else 2
            # 하나라도 in_progress(1) 이면 시작된 것으로 판단
            if any(ph == 1 for ph in [ph_x, ph_z1, ph_z2]):
                break
            if time.monotonic() - t_wait > 3.0:
                result.success = False
                result.message = "호밍 시작 타임아웃 (서브프로세스 응답 없음)."
                goal_handle.abort()
                return result
            time.sleep(0.05)

        feedback_msg = Homing.Feedback()
        t0        = time.monotonic()
        timeout_s = self._timeout_s

        while True:
            # Cancel 확인
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "호밍 취소됨."
                return result

            # 타임아웃 확인
            if time.monotonic() - t0 > timeout_s:
                result.success = False
                result.message = "호밍 타임아웃."
                goal_handle.abort()
                return result

            # 각 활성 축의 호밍 단계 확인
            ph_x  = ec.get_homing_phase(self._x)  if self._active['x']  else 2
            ph_z1 = ec.get_homing_phase(self._z1) if self._active['z1'] else 2
            ph_z2 = ec.get_homing_phase(self._z2) if self._active['z2'] else 2

            # 오류 확인
            if ph_x == -1 or ph_z1 == -1 or ph_z2 == -1:
                result.success = False
                result.message = (
                    f"호밍 오류 발생 (phase: X={ph_x}, Z1={ph_z1}, Z2={ph_z2})"
                )
                self.get_logger().error(result.message)
                goal_handle.abort()
                return result

            # 완료 확인 (모든 활성 축 완료)
            if ph_x == 2 and ph_z1 == 2 and ph_z2 == 2:
                self.get_logger().info("모든 축 호밍 완료!")
                break

            # Feedback 발행
            in_progress = any(ph in (0, 1) for ph in [ph_x, ph_z1, ph_z2])
            feedback_msg.phase        = "in_progress" if in_progress else "idle"
            feedback_msg.current_z_mm = self._z_position()
            goal_handle.publish_feedback(feedback_msg)

            time.sleep(0.1)

        # ── 결과 반환 ──
        result.success            = True
        result.message            = "호밍 완료."
        result.z_home_position_mm = self._z_position()

        self.get_logger().info(
            f"호밍 결과: X_homed={self._active['x'] and ec.is_homed(self._x)}, "
            f"Z1_homed={self._active['z1'] and ec.is_homed(self._z1)}, "
            f"Z2_homed={self._active['z2'] and ec.is_homed(self._z2)}"
        )
        goal_handle.succeed()
        return result

    def _all_homed(self) -> bool:
        ec = self._ec
        return (
            (not self._active['x']  or ec.is_homed(self._x))  and
            (not self._active['z1'] or ec.is_homed(self._z1)) and
            (not self._active['z2'] or ec.is_homed(self._z2))
        )

    def _z_position(self) -> float:
        ec = self._ec
        positions = []
        if self._active['z1']:
            positions.append(ec.get_position_mm(self._z1, 'z'))
        if self._active['z2']:
            positions.append(ec.get_position_mm(self._z2, 'z'))
        return sum(positions) / len(positions) if positions else 0.0
