"""
homing_action_server.py - 호밍 액션 서버 (듀얼 갠트리)

액션 타입: motor_control/action/Homing
토픽:      /motor/homing

goal.gantry_index:
  0 → 갠트리0만 호밍
  1 → 갠트리1만 호밍
  2 → 두 갠트리 동시 호밍

슬레이브 인덱스가 -1이면 해당 축을 건너뜀 (비활성).
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
                 gantry_indices: dict,
                 x_method: int = X_HOMING_METHOD,
                 z_method: int = Z_HOMING_METHOD,
                 timeout_s: float = 600.0):
        """
        Parameters
        ----------
        gantry_indices : {0: (x_idx, z1_idx, z2_idx), 1: (x_idx, z1_idx, z2_idx)}
        """
        super().__init__('homing_action_server')

        self._ec        = ec
        self._gantry    = gantry_indices   # {0: (xi, z1i, z2i), 1: (xi, z1i, z2i)}
        self._x_method  = x_method
        self._z_method  = z_method
        self._timeout_s = timeout_s

        self._action_server = ActionServer(
            self,
            Homing,
            '/motor/homing',
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info(
            f"HomingActionServer 준비 완료. "
            f"갠트리0={gantry_indices[0]}, 갠트리1={gantry_indices[1]}"
        )

    def _goal_cb(self, goal_request):
        self.get_logger().info(
            f"호밍 Goal 수신 (gantry_index={goal_request.gantry_index}, "
            f"force_rehome={goal_request.force_rehome})"
        )
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().info("호밍 Cancel 요청 수신")
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle):
        ec    = self._ec
        force = goal_handle.request.force_rehome
        gantry_idx = goal_handle.request.gantry_index

        result = Homing.Result()

        # 대상 갠트리 결정
        if gantry_idx == 2:
            target_gantries = [0, 1]
        elif gantry_idx in (0, 1):
            target_gantries = [gantry_idx]
        else:
            result.success = False
            result.message = f"잘못된 gantry_index={gantry_idx}. 0, 1, 2만 유효합니다."
            goal_handle.abort()
            return result

        # ── 이미 호밍 완료된 경우 ──
        if not force and all(self._is_gantry_homed(g) for g in target_gantries):
            self.get_logger().info("이미 호밍 완료 상태 (force_rehome=False → 스킵)")
            result.success = True
            result.message = "이미 호밍 완료 상태."
            result.gantry0_z_home_position_mm = self._z_position(0)
            result.gantry1_z_home_position_mm = self._z_position(1)
            goal_handle.succeed()
            return result

        # ── EtherCAT 동작 확인 ──
        if not ec.is_alive():
            result.success = False
            result.message = "EtherCAT 프로세스가 실행 중이지 않습니다."
            goal_handle.abort()
            return result

        # ── 드라이브 준비 대기 ──
        active_indices = []
        for g in target_gantries:
            xi, z1i, z2i = self._gantry[g]
            active_indices += [i for i in [xi, z1i, z2i] if i >= 0]

        self.get_logger().info("드라이브 준비 대기 중...")
        t0 = time.monotonic()
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

        # ── 호밍 시작 ──
        self.get_logger().info(f"갠트리 {target_gantries} 호밍 시작...")
        for g in target_gantries:
            xi, z1i, z2i = self._gantry[g]
            if xi  >= 0:
                ec.start_homing(xi,  self._x_method)
                self.get_logger().info(f"  갠트리{g} X 호밍 시작 (Method {self._x_method})")
            if z1i >= 0:
                ec.start_homing(z1i, self._z_method)
                self.get_logger().info(f"  갠트리{g} Z1 호밍 시작 (Method {self._z_method})")
            if z2i >= 0:
                ec.start_homing(z2i, self._z_method)
                self.get_logger().info(f"  갠트리{g} Z2 호밍 시작 (Method {self._z_method})")

        # 서브프로세스가 phase=1로 전환될 때까지 대기 (오탐 방지)
        t_wait = time.monotonic()
        while True:
            phases = self._collect_phases(target_gantries)
            if any(ph == 1 for ph in phases):
                break
            if time.monotonic() - t_wait > 3.0:
                result.success = False
                result.message = "호밍 시작 타임아웃 (서브프로세스 응답 없음)."
                goal_handle.abort()
                return result
            time.sleep(0.05)

        feedback_msg = Homing.Feedback()
        t0 = time.monotonic()

        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "호밍 취소됨."
                return result

            if time.monotonic() - t0 > self._timeout_s:
                result.success = False
                result.message = "호밍 타임아웃."
                goal_handle.abort()
                return result

            phases = self._collect_phases(target_gantries)

            if any(ph == -1 for ph in phases):
                result.success = False
                result.message = f"호밍 오류 발생 (phases={phases})"
                self.get_logger().error(result.message)
                goal_handle.abort()
                return result

            if all(ph == 2 for ph in phases):
                self.get_logger().info("모든 대상 갠트리 호밍 완료!")
                break

            feedback_msg.phase = "in_progress" if any(ph in (0, 1) for ph in phases) else "idle"
            feedback_msg.gantry0_current_z_mm = self._z_position(0)
            feedback_msg.gantry1_current_z_mm = self._z_position(1)
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)

        result.success = True
        result.message = f"갠트리 {target_gantries} 호밍 완료."
        result.gantry0_z_home_position_mm = self._z_position(0)
        result.gantry1_z_home_position_mm = self._z_position(1)
        goal_handle.succeed()
        return result

    def _collect_phases(self, target_gantries: list) -> list:
        """대상 갠트리의 모든 활성 슬레이브 호밍 phase 수집."""
        phases = []
        for g in target_gantries:
            xi, z1i, z2i = self._gantry[g]
            if xi  >= 0: phases.append(self._ec.get_homing_phase(xi))
            if z1i >= 0: phases.append(self._ec.get_homing_phase(z1i))
            if z2i >= 0: phases.append(self._ec.get_homing_phase(z2i))
        return phases if phases else [2]

    def _is_gantry_homed(self, gantry_idx: int) -> bool:
        ec = self._ec
        xi, z1i, z2i = self._gantry[gantry_idx]
        return (
            (xi  < 0 or ec.is_homed(xi))  and
            (z1i < 0 or ec.is_homed(z1i)) and
            (z2i < 0 or ec.is_homed(z2i))
        )

    def _z_position(self, gantry_idx: int) -> float:
        ec = self._ec
        _, z1i, z2i = self._gantry[gantry_idx]
        positions = []
        if z1i >= 0: positions.append(ec.get_position_mm(z1i, 'z'))
        if z2i >= 0: positions.append(ec.get_position_mm(z2i, 'z'))
        return sum(positions) / len(positions) if positions else 0.0
