"""
move_action_server.py - 목표 좌표 이동 액션 서버

액션 타입: motor_control/action/MoveAxis
토픽:      /motor/move

이동 흐름:
  1. Goal 수신
  2. 호밍 미완료 시 Abort
  3. 소프트웨어 리밋 검사
  4. X / Z 목표 위치로 이동 명령 전송 (CSV 모드)
  5. 주기적으로 현재 위치 Feedback 발행
  6. 이동 완료 후 Result 반환
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from motor_control_interfaces.action import MoveAxis
from motor_control.ethercat_interface import (
    EtherCATInterface,
    Z_MM_PER_REV, X_MM_PER_REV, EFFECTIVE_PPR,
    mm_s_to_pps, rpm_to_pps,
)


class MoveAxisActionServer(Node):

    def __init__(self, ec: EtherCATInterface,
                 x_idx: int = 0,
                 z1_idx: int = 1,
                 z2_idx: int = 2,
                 soft_limits: dict = None):
        super().__init__('move_action_server')

        self._ec    = ec
        self._x     = x_idx
        self._z1    = z1_idx
        self._z2    = z2_idx
        self._limits = soft_limits or {
            'x_min': 0.0, 'x_max': 1000.0,
            'z_min': 0.0, 'z_max': 300.0,
        }

        self._action_server = ActionServer(
            self,
            MoveAxis,
            '/motor/move',
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info("MoveAxisActionServer 준비 완료.")

    def _goal_cb(self, goal_request):
        self.get_logger().info(
            f"Move Goal 수신: X={goal_request.x_target_mm:.2f}mm, "
            f"Z={goal_request.z_target_mm:.2f}mm, "
            f"vel={goal_request.velocity_mm_s:.2f}mm/s"
        )
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().info("Move Cancel 요청 수신")
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle):
        ec     = self._ec
        result = MoveAxis.Result()

        x_target   = goal_handle.request.x_target_mm
        z_target   = goal_handle.request.z_target_mm
        vel_mms    = goal_handle.request.velocity_mm_s
        force_move = goal_handle.request.force_move

        # -1.0e9 를 "이동 없음" 신호로 사용.
        # 0.0 이나 음수도 유효한 목표이므로 >= 0 으로 판단하면 안 된다.
        _NO_MOVE = -1.0e9
        move_x = (x_target > _NO_MOVE)
        move_z = (z_target > _NO_MOVE)

        if force_move:
            self.get_logger().warn("force_move=True: 호밍 확인 및 소프트 리밋 검사 생략")
        else:
            # ── 호밍 완료 여부 확인 ──
            if move_z and self._z1 >= 0 and not ec.is_homed(self._z1):
                result.success = False
                result.message = "Z1 호밍 미완료. /motor/homing 실행 또는 force_move: true 사용."
                self.get_logger().error(result.message)
                goal_handle.abort()
                return result

            if move_z and self._z2 >= 0 and not ec.is_homed(self._z2):
                result.success = False
                result.message = "Z2 호밍 미완료. /motor/homing 실행 또는 force_move: true 사용."
                self.get_logger().error(result.message)
                goal_handle.abort()
                return result

            if move_x and self._x >= 0 and not ec.is_homed(self._x):
                result.success = False
                result.message = "X축 호밍 미완료. /motor/homing 실행 또는 force_move: true 사용."
                self.get_logger().error(result.message)
                goal_handle.abort()
                return result

            # ── 소프트웨어 리밋 검사 ──
            lim = self._limits
            if move_x and not (lim['x_min'] <= x_target <= lim['x_max']):
                result.success = False
                result.message = (
                    f"X 목표({x_target:.2f}mm)가 소프트 리밋 범위를 벗어남 "
                    f"[{lim['x_min']}, {lim['x_max']}]"
                )
                self.get_logger().error(result.message)
                goal_handle.abort()
                return result

            if move_z and not (lim['z_min'] <= z_target <= lim['z_max']):
                result.success = False
                result.message = (
                    f"Z 목표({z_target:.2f}mm)가 소프트 리밋 범위를 벗어남 "
                    f"[{lim['z_min']}, {lim['z_max']}]"
                )
                self.get_logger().error(result.message)
                goal_handle.abort()
                return result

        # ── EtherCAT 동작 확인 ──
        if not ec.is_alive():
            result.success = False
            result.message = "EtherCAT 프로세스가 실행 중이지 않습니다."
            goal_handle.abort()
            return result

        # ── 속도 설정 (요청 속도가 있을 경우) ──
        if vel_mms > 0.0:
            rpm_z = (vel_mms / Z_MM_PER_REV) * 60.0
            rpm_x = (vel_mms / X_MM_PER_REV) * 60.0
            if move_z:
                ec.set_velocity(self._z1, rpm_z)
                ec.set_velocity(self._z2, rpm_z)
            if move_x:
                ec.set_velocity(self._x, rpm_x)

        # ── 동기화 오류 리셋 ──
        if ec.has_sync_error():
            ec.reset_sync_error()
            time.sleep(0.1)

        # ── 이동 명령 전송 ──
        if move_z:
            ec.move_to_mm(self._z1, z_target)
            ec.move_to_mm(self._z2, z_target)
            self.get_logger().info(f"Z축 이동: {z_target:.2f}mm")

        if move_x:
            ec.move_to_mm(self._x, x_target)
            self.get_logger().info(f"X축 이동: {x_target:.2f}mm")

        time.sleep(0.2)  # 명령이 서브프로세스에 반영될 시간

        # ── 이동 완료 대기 & Feedback 발행 ──
        feedback_msg = MoveAxis.Feedback()
        t0 = time.monotonic()
        timeout_s = 120.0

        while True:
            # Cancel 확인
            if goal_handle.is_cancel_requested:
                self.get_logger().info("이동 취소됨.")
                goal_handle.canceled()
                result.success = False
                result.message = "이동 취소됨."
                return result

            # 타임아웃
            if time.monotonic() - t0 > timeout_s:
                result.success = False
                result.message = "이동 타임아웃."
                self.get_logger().error(result.message)
                goal_handle.abort()
                return result

            # 동기화 오류 확인
            if ec.has_sync_error():
                result.success = False
                result.message = "동기화 오류 발생으로 이동 중단."
                self.get_logger().error(result.message)
                goal_handle.abort()
                return result

            # 현재 위치 읽기
            x_cur  = ec.get_position_mm(self._x,  'x')
            z1_cur = ec.get_position_mm(self._z1, 'z')
            z2_cur = ec.get_position_mm(self._z2, 'z')
            z_cur  = (z1_cur + z2_cur) / 2.0

            # Feedback 발행
            feedback_msg.x_current_mm   = x_cur
            feedback_msg.z_current_mm   = z_cur
            feedback_msg.x_remaining_mm = abs(x_target - x_cur) if move_x else 0.0
            feedback_msg.z_remaining_mm = abs(z_target - z_cur) if move_z else 0.0
            goal_handle.publish_feedback(feedback_msg)

            # 이동 완료 확인
            x_done = (not move_x) or (not ec.is_moving(self._x))
            z_done = (not move_z) or (
                not ec.is_moving(self._z1) and not ec.is_moving(self._z2)
            )

            if x_done and z_done:
                break

            time.sleep(0.05)

        # ── 결과 반환 ──
        x_final = ec.get_position_mm(self._x,  'x')
        z_final = ec.get_position_mm(self._z1, 'z')

        result.success    = True
        result.x_final_mm = x_final
        result.z_final_mm = z_final
        result.message    = (
            f"이동 완료: X={x_final:.3f}mm, Z={z_final:.3f}mm"
        )
        self.get_logger().info(result.message)
        goal_handle.succeed()
        return result
