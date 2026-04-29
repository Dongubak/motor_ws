"""
move_action_server.py - 목표 좌표 이동 액션 서버 (듀얼 갠트리)

액션 타입: motor_control/action/MoveAxis
토픽:      /motor/move

goal.gantry_index:
  0 → 갠트리0만 이동 (gantry0_x/z_target_mm 사용)
  1 → 갠트리1만 이동 (gantry1_x/z_target_mm 사용)
  2 → 두 갠트리 동시 이동 (gantry0_x/z_target_mm → 갠트리0, gantry1_x/z_target_mm → 갠트리1)

이동하지 않을 축에는 -1.0e9 지정.
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from motor_control_interfaces.action import MoveAxis
from motor_control.ethercat_interface import (
    EtherCATInterface,
    Z_MM_PER_REV, X_MM_PER_REV,
)

_NO_MOVE = -1.0e9   # "이동 없음" 신호값


class MoveAxisActionServer(Node):

    def __init__(self, ec: EtherCATInterface,
                 gantry_indices: dict,
                 soft_limits: dict = None,
                 max_sync_error_mm: float = 10.0):
        """
        Parameters
        ----------
        gantry_indices     : {0: (x_idx, z1_idx, z2_idx), 1: (x_idx, z1_idx, z2_idx)}
        soft_limits        : {0: {x_min, x_max, z_min, z_max}, 1: {...}}
        max_sync_error_mm  : Z1↔Z2 이동 전 허용 최대 위치 차이 [mm]
        """
        super().__init__('move_action_server')

        self._ec                 = ec
        self._gantry             = gantry_indices
        self._max_sync_error_mm  = max_sync_error_mm
        self._limits  = soft_limits or {
            0: {'x_min': 0.0, 'x_max': 1500.0, 'z_min': -1500.0, 'z_max': 0.1},
            1: {'x_min': 0.0, 'x_max': 1500.0, 'z_min': -1500.0, 'z_max': 0.1},
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
            f"Move Goal 수신: gantry_index={goal_request.gantry_index}, "
            f"vel={goal_request.velocity_mm_s:.2f}mm/s"
        )
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().info("Move Cancel 요청 수신")
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle):
        ec     = self._ec
        result = MoveAxis.Result()
        req    = goal_handle.request

        gantry_idx = req.gantry_index
        vel_mms    = req.velocity_mm_s
        force_move = req.force_move

        # ── 갠트리별 목표 위치 결정 ──
        if gantry_idx == 2:
            # 두 갠트리 동시: 각 갠트리에 독립적인 목표 적용
            targets = {
                0: (req.gantry0_x_target_mm, req.gantry0_z_target_mm),
                1: (req.gantry1_x_target_mm, req.gantry1_z_target_mm),
            }
            active_gantries = [0, 1]
        elif gantry_idx == 0:
            targets = {0: (req.gantry0_x_target_mm, req.gantry0_z_target_mm)}
            active_gantries = [0]
        elif gantry_idx == 1:
            targets = {1: (req.gantry1_x_target_mm, req.gantry1_z_target_mm)}
            active_gantries = [1]
        else:
            result.success = False
            result.message = f"잘못된 gantry_index={gantry_idx}. 0, 1, 2만 유효합니다."
            goal_handle.abort()
            return result

        if not force_move:
            self.get_logger().warn("force_move=False: 호밍 확인 및 소프트 리밋 검사 수행")
        else:
            self.get_logger().warn("force_move=True: 호밍 확인 및 소프트 리밋 검사 생략")

        # ── 유효성 검사 (force_move=False 시) ──
        if not force_move:
            for g in active_gantries:
                xi, z1i, z2i = self._gantry[g]
                x_target, z_target = targets[g]
                move_x = (x_target > _NO_MOVE)
                move_z = (z_target > _NO_MOVE)
                lim    = self._limits[g]

                if move_x and xi >= 0 and not ec.is_homed(xi):
                    result.success = False
                    result.message = f"갠트리{g} X 호밍 미완료."
                    goal_handle.abort()
                    return result
                if move_z and z1i >= 0 and not ec.is_homed(z1i):
                    result.success = False
                    result.message = f"갠트리{g} Z1 호밍 미완료."
                    goal_handle.abort()
                    return result
                if move_z and z2i >= 0 and not ec.is_homed(z2i):
                    result.success = False
                    result.message = f"갠트리{g} Z2 호밍 미완료."
                    goal_handle.abort()
                    return result
                if move_x and not (lim['x_min'] <= x_target <= lim['x_max']):
                    result.success = False
                    result.message = (
                        f"갠트리{g} X 목표({x_target:.2f}mm)가 소프트 리밋 범위 밖 "
                        f"[{lim['x_min']}, {lim['x_max']}]"
                    )
                    goal_handle.abort()
                    return result
                if move_z and not (lim['z_min'] <= z_target <= lim['z_max']):
                    result.success = False
                    result.message = (
                        f"갠트리{g} Z 목표({z_target:.2f}mm)가 소프트 리밋 범위 밖 "
                        f"[{lim['z_min']}, {lim['z_max']}]"
                    )
                    goal_handle.abort()
                    return result

        # ── EtherCAT 동작 확인 ──
        if not ec.is_alive():
            result.success = False
            result.message = "EtherCAT 프로세스가 실행 중이지 않습니다."
            goal_handle.abort()
            return result

        # ── 속도 설정 ──
        if vel_mms > 0.0:
            rpm_z = (vel_mms / Z_MM_PER_REV) * 60.0
            rpm_x = (vel_mms / X_MM_PER_REV) * 60.0
            for g in active_gantries:
                xi, z1i, z2i = self._gantry[g]
                x_target, z_target = targets[g]
                if (z_target > _NO_MOVE):
                    if z1i >= 0: ec.set_velocity(z1i, rpm_z)
                    if z2i >= 0: ec.set_velocity(z2i, rpm_z)
                if (x_target > _NO_MOVE):
                    if xi  >= 0: ec.set_velocity(xi,  rpm_x)

        # ── 동기화 오류 리셋 ──
        if ec.has_sync_error():
            ec.reset_sync_error()
            time.sleep(0.1)

        # ── 이동 전 Z축 동기화 사전 검사 ──
        # position-hold 중 벌어진 Z1/Z2 위치 차이를 이동 시작 전에 감지
        if not force_move:
            for g in active_gantries:
                _, z1i, z2i = self._gantry[g]
                if z1i >= 0 and z2i >= 0:
                    z1_pos = ec.get_position_mm(z1i, 'z')
                    z2_pos = ec.get_position_mm(z2i, 'z')
                    z_diff = abs(z1_pos - z2_pos)
                    if z_diff > self._max_sync_error_mm:
                        result.success = False
                        result.message = (
                            f"갠트리{g} Z축 동기화 오차({z_diff:.2f}mm)가 "
                            f"임계값({self._max_sync_error_mm}mm)을 초과합니다. "
                            f"force_rehome: true 로 호밍을 재수행하세요."
                        )
                        self.get_logger().error(result.message)
                        goal_handle.abort()
                        return result

        # ── 이동 명령 전송 ──
        for g in active_gantries:
            xi, z1i, z2i = self._gantry[g]
            x_target, z_target = targets[g]
            move_x = (x_target > _NO_MOVE)
            move_z = (z_target > _NO_MOVE)

            if move_z:
                if z1i >= 0: ec.move_to_mm(z1i, z_target)
                if z2i >= 0: ec.move_to_mm(z2i, z_target)
                self.get_logger().info(f"갠트리{g} Z축 이동: {z_target:.2f}mm")
            if move_x:
                if xi  >= 0: ec.move_to_mm(xi,  x_target)
                self.get_logger().info(f"갠트리{g} X축 이동: {x_target:.2f}mm")

        time.sleep(0.2)  # 서브프로세스 반영 대기

        # ── 이동 완료 대기 & Feedback ──
        feedback_msg = MoveAxis.Feedback()
        t0 = time.monotonic()
        timeout_s = 120.0

        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "이동 취소됨."
                return result

            if time.monotonic() - t0 > timeout_s:
                result.success = False
                result.message = "이동 타임아웃."
                self.get_logger().error(result.message)
                goal_handle.abort()
                return result

            if ec.has_sync_error():
                result.success = False
                if ec.has_fault():
                    result.message = (
                        f"모터 Fault 발생으로 이동 중단. "
                        f"({ec.get_fault_info()}) "
                        f"하드 리밋 접촉 또는 과전류 가능성. "
                        f"소프트 리밋 또는 목표값을 줄이세요."
                    )
                else:
                    result.message = (
                        "Z축 동기화 오차 초과로 이동 중단. "
                        "force_rehome: true 로 호밍을 재수행하세요."
                    )
                self.get_logger().error(result.message)
                goal_handle.abort()
                return result

            # 위치 읽기 (전체 갠트리)
            g0_xi, g0_z1i, _ = self._gantry[0]
            g1_xi, g1_z1i, _ = self._gantry[1]
            g0_x_cur = ec.get_position_mm(g0_xi,  'x') if g0_xi  >= 0 else 0.0
            g0_z_cur = ec.get_position_mm(g0_z1i, 'z') if g0_z1i >= 0 else 0.0
            g1_x_cur = ec.get_position_mm(g1_xi,  'x') if g1_xi  >= 0 else 0.0
            g1_z_cur = ec.get_position_mm(g1_z1i, 'z') if g1_z1i >= 0 else 0.0

            feedback_msg.gantry0_x_current_mm = g0_x_cur
            feedback_msg.gantry0_z_current_mm = g0_z_cur
            feedback_msg.gantry1_x_current_mm = g1_x_cur
            feedback_msg.gantry1_z_current_mm = g1_z_cur

            g0_xt, g0_zt = targets.get(0, (_NO_MOVE, _NO_MOVE))
            g1_xt, g1_zt = targets.get(1, (_NO_MOVE, _NO_MOVE))
            feedback_msg.gantry0_x_remaining_mm = abs(g0_xt - g0_x_cur) if g0_xt > _NO_MOVE else 0.0
            feedback_msg.gantry0_z_remaining_mm = abs(g0_zt - g0_z_cur) if g0_zt > _NO_MOVE else 0.0
            feedback_msg.gantry1_x_remaining_mm = abs(g1_xt - g1_x_cur) if g1_xt > _NO_MOVE else 0.0
            feedback_msg.gantry1_z_remaining_mm = abs(g1_zt - g1_z_cur) if g1_zt > _NO_MOVE else 0.0
            goal_handle.publish_feedback(feedback_msg)

            # 완료 확인
            all_done = True
            for g in active_gantries:
                xi, z1i, z2i = self._gantry[g]
                x_target, z_target = targets[g]
                move_x = (x_target > _NO_MOVE)
                move_z = (z_target > _NO_MOVE)

                x_done = (not move_x) or (xi  < 0) or (not ec.is_moving(xi))
                z_done = (not move_z) or (
                    (z1i < 0 or not ec.is_moving(z1i)) and
                    (z2i < 0 or not ec.is_moving(z2i))
                )
                if not (x_done and z_done):
                    all_done = False
                    break

            if all_done:
                break

            time.sleep(0.05)

        # ── 결과 반환 ──
        g0_xi, g0_z1i, _ = self._gantry[0]
        g1_xi, g1_z1i, _ = self._gantry[1]
        result.success = True
        result.gantry0_x_final_mm = ec.get_position_mm(g0_xi,  'x') if g0_xi  >= 0 else 0.0
        result.gantry0_z_final_mm = ec.get_position_mm(g0_z1i, 'z') if g0_z1i >= 0 else 0.0
        result.gantry1_x_final_mm = ec.get_position_mm(g1_xi,  'x') if g1_xi  >= 0 else 0.0
        result.gantry1_z_final_mm = ec.get_position_mm(g1_z1i, 'z') if g1_z1i >= 0 else 0.0
        result.message = (
            f"이동 완료: "
            f"갠트리0(X={result.gantry0_x_final_mm:.3f}mm, Z={result.gantry0_z_final_mm:.3f}mm) "
            f"갠트리1(X={result.gantry1_x_final_mm:.3f}mm, Z={result.gantry1_z_final_mm:.3f}mm)"
        )
        self.get_logger().info(result.message)
        goal_handle.succeed()
        return result
