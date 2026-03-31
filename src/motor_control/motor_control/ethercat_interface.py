"""
ethercat_interface.py - EtherCAT 버스 인터페이스 (ROS2용)

motor_csv.py 기반으로 Linux / ROS2 환경에 맞게 재작성.

CSV (Cyclic Synchronous Velocity, Mode 9) 모드로 운용하며
호밍 시에는 CiA 402 Homing Mode (Mode 6) 를 사용한다.

PDO 구성 (CSV 모드):
  RxPDO 0x1600: Controlword(0x6040, 16bit) + Target Velocity(0x60FF, 32bit)
  TxPDO 0x1A00: Statusword(0x6041, 16bit) + Actual Pos(0x6064, 32bit) + Actual Vel(0x606C, 32bit)

shared_states 레이아웃 (슬레이브당 SLOT_SIZE 슬롯):
  base+0 : statusword
  base+1 : is_moving (0|1)
  base+2 : actual_position (pulse)
  base+3 : offset (pulse)
  base+4 : sync_error (0|1)
  base+5 : actual_velocity (pulse/sec)
  base+6 : homed (0|1)
  base+7 : homing_phase  (0=idle, 1=in_progress, 2=done, -1=error)
"""

import pysoem
import struct
import time
import math
import multiprocessing as mp
from collections import deque
from typing import List

# ─────────────────────────────────────────────────────────────
# CiA 402 Controlword 상수
# ─────────────────────────────────────────────────────────────
CW_SHUTDOWN          = 0x0006
CW_SWITCH_ON         = 0x0007
CW_ENABLE_OPERATION  = 0x000F
CW_DISABLE_VOLTAGE   = 0x0000
CW_FAULT_RESET       = 0x0080
CW_HOMING_START      = 0x001F   # Enable Operation + bit4 (homing trigger)

# ─────────────────────────────────────────────────────────────
# 기계 상수
# ─────────────────────────────────────────────────────────────
PULSES_PER_REVOLUTION = 8_388_608           # 엔코더 분해능 (2^23)
EFFECTIVE_PPR         = PULSES_PER_REVOLUTION      # 8,388,608 pulse/rev (전기적 기어비 1:1)

X_MM_PER_REV = 11.9993131404   # X축 1회전당 이동 거리 (mm)
Z_MM_PER_REV = 5.99965657019   # Z축 1회전당 이동 거리 (mm)

# ─────────────────────────────────────────────────────────────
# 공유 상태 슬롯 수 / 인덱스
# ─────────────────────────────────────────────────────────────
SLOT_SIZE       = 8
IDX_STATUS      = 0
IDX_IS_MOVING   = 1
IDX_ACT_POS     = 2
IDX_OFFSET      = 3
IDX_SYNC_ERR    = 4
IDX_ACT_VEL     = 5
IDX_HOMED       = 6
IDX_HOMING_PH   = 7   # 0=idle 1=in_progress 2=done -1=error

# ─────────────────────────────────────────────────────────────
# 제어 상수
# ─────────────────────────────────────────────────────────────
DEFAULT_MAX_SYNC_ERROR_MM  = 0.5
DEFAULT_COUPLING_GAIN      = 0.5
DEFAULT_MA_WINDOW          = 5

POSITION_TOLERANCE_PULSE   = 50_000   # 궤적 완료 판정 오차 (≈ 0.018mm, z축)
POSITION_HOLD_GAIN         = 2.0      # 위치 유지 P 게인 [1/sec]
POSITION_HOLD_MAX_VEL_MM_S = 5.0      # 위치 유지 최대 속도 (mm/sec)

HOMING_FAST_RPM    = 30.0  # 호밍 고속 탐색 속도 (RPM)
HOMING_SLOW_RPM    = 1.0   # 호밍 저속 색인 속도 (RPM)
X_HOMING_METHOD    = 1     # X축: 역방향(-) + NOT 리밋 스위치 + Index 펄스
Z_HOMING_METHOD    = 2     # Z축: 정방향(+) + POT 리밋 스위치 + Index 펄스
HOMING_METHOD      = Z_HOMING_METHOD  # 기본값 (하위 호환)


# ─────────────────────────────────────────────────────────────
# 단위 변환
# ─────────────────────────────────────────────────────────────
def mm_to_pulse(mm: float, axis: str = 'z') -> int:
    mm_per_rev = X_MM_PER_REV if axis == 'x' else Z_MM_PER_REV
    return int((mm / mm_per_rev) * EFFECTIVE_PPR)

def pulse_to_mm(pulse: float, axis: str = 'z') -> float:
    mm_per_rev = X_MM_PER_REV if axis == 'x' else Z_MM_PER_REV
    return (pulse / EFFECTIVE_PPR) * mm_per_rev

def rpm_to_pps(rpm: float) -> float:
    return (rpm / 60.0) * EFFECTIVE_PPR

def rpm_per_sec_to_pps2(rpm_per_sec: float) -> float:
    return (rpm_per_sec / 60.0) * EFFECTIVE_PPR

def mm_s_to_pps(mm_s: float, axis: str = 'z') -> float:
    mm_per_rev = X_MM_PER_REV if axis == 'x' else Z_MM_PER_REV
    return (mm_s / mm_per_rev) * EFFECTIVE_PPR


# ─────────────────────────────────────────────────────────────
# 사다리꼴 속도 프로파일 (CSV 모드용)
# ─────────────────────────────────────────────────────────────
class TrapezoidalProfile:
    def __init__(self, start_pos: int, end_pos: int,
                 max_vel_pps: float, accel_pps2: float):
        self.start_pos = start_pos
        self.end_pos   = end_pos
        distance       = end_pos - start_pos
        self._dir      = 1 if distance >= 0 else -1
        D              = abs(float(distance))
        self._A        = max(float(accel_pps2), 1.0)
        V              = max(float(max_vel_pps), 1.0)

        d_accel = V * V / (2.0 * self._A)
        if D <= 2.0 * d_accel:
            self._V_peak  = math.sqrt(self._A * D) if D > 0 else 0.0
            self._t_accel = self._V_peak / self._A
            self._t_coast = 0.0
        else:
            self._V_peak  = V
            self._t_accel = V / self._A
            d_coast       = D - 2.0 * d_accel
            self._t_coast = d_coast / V

        self.t_total = 2.0 * self._t_accel + self._t_coast

    def get_velocity(self, elapsed: float) -> int:
        if elapsed <= 0.0 or self._V_peak == 0.0:
            return 0
        if elapsed <= self._t_accel:
            v = self._A * elapsed
        elif elapsed <= self._t_accel + self._t_coast:
            v = self._V_peak
        elif elapsed <= self.t_total:
            t_dec = elapsed - self._t_accel - self._t_coast
            v = max(0.0, self._V_peak - self._A * t_dec)
        else:
            v = 0.0
        return int(self._dir * v)

    def is_time_complete(self, elapsed: float) -> bool:
        return elapsed >= self.t_total


# ─────────────────────────────────────────────────────────────
# PDO 입출력 헬퍼
# ─────────────────────────────────────────────────────────────
def _read_status_word(slave) -> int:
    return struct.unpack("<H", slave.input[0:2])[0]

def _read_actual_position(slave) -> int:
    return struct.unpack("<i", slave.input[2:6])[0]

def _read_actual_velocity(slave) -> int:
    return struct.unpack("<i", slave.input[6:10])[0]

def _write_csv_outputs(slave, cw: int, target_velocity: int):
    slave.output = struct.pack("<H", cw) + struct.pack("<i", target_velocity)


# ─────────────────────────────────────────────────────────────
# SDO 초기화 헬퍼
# ─────────────────────────────────────────────────────────────
def _sdo_reset_fault(slave):
    try:
        sw = struct.unpack("<H", slave.sdo_read(0x6041, 0))[0]
        if sw & 0x0008:
            slave.sdo_write(0x6040, 0, struct.pack("<H", CW_FAULT_RESET))
            time.sleep(0.2)
            slave.sdo_write(0x6040, 0, struct.pack("<H", CW_SHUTDOWN))
            time.sleep(0.1)
    except Exception:
        pass

def _setup_csv_mode(slave):
    slave.sdo_write(0x211F, 0, struct.pack("<H", (1 << 12)))
    time.sleep(0.01)
    slave.sdo_write(0x6060, 0, struct.pack("<b", 9))
    time.sleep(0.01)

def _configure_csv_pdos(slave):
    # RxPDO 0x1600: Controlword(16bit) + Target Velocity(32bit)
    slave.sdo_write(0x1C12, 0, b'\x00'); time.sleep(0.01)
    slave.sdo_write(0x1600, 0, b'\x00'); time.sleep(0.01)
    slave.sdo_write(0x1600, 1, struct.pack('<I', 0x60400010)); time.sleep(0.01)
    slave.sdo_write(0x1600, 2, struct.pack('<I', 0x60FF0020)); time.sleep(0.01)
    slave.sdo_write(0x1600, 0, b'\x02'); time.sleep(0.01)
    slave.sdo_write(0x1C12, 1, struct.pack('<H', 0x1600)); time.sleep(0.01)
    slave.sdo_write(0x1C12, 0, b'\x01'); time.sleep(0.01)

    # TxPDO 0x1A00: Statusword(16bit) + Actual Pos(32bit) + Actual Vel(32bit)
    slave.sdo_write(0x1C13, 0, b'\x00'); time.sleep(0.01)
    slave.sdo_write(0x1A00, 0, b'\x00'); time.sleep(0.01)
    slave.sdo_write(0x1A00, 1, struct.pack('<I', 0x60410010)); time.sleep(0.01)
    slave.sdo_write(0x1A00, 2, struct.pack('<I', 0x60640020)); time.sleep(0.01)
    slave.sdo_write(0x1A00, 3, struct.pack('<I', 0x606C0020)); time.sleep(0.01)
    slave.sdo_write(0x1A00, 0, b'\x03'); time.sleep(0.01)
    slave.sdo_write(0x1C13, 1, struct.pack('<H', 0x1A00)); time.sleep(0.01)
    slave.sdo_write(0x1C13, 0, b'\x01'); time.sleep(0.01)

def _setup_homing_mode_sdo(slave, method: int,
                            fast_pps: float, slow_pps: float, accel_pps2: float):
    """호밍 모드(6) SDO 설정."""
    slave.sdo_write(0x211F, 0, struct.pack("<H", (1 << 12)))   # 모드 변경 언락 (iX7NH 벤더)
    time.sleep(0.01)
    slave.sdo_write(0x6060, 0, struct.pack("<b", 6))           # 운전 모드 = homing
    time.sleep(0.02)
    slave.sdo_write(0x6098, 0, struct.pack("<b", method))      # 호밍 방법
    time.sleep(0.01)
    slave.sdo_write(0x6099, 1, struct.pack("<I", int(fast_pps)))   # 고속
    time.sleep(0.01)
    slave.sdo_write(0x6099, 2, struct.pack("<I", int(slow_pps)))   # 저속
    time.sleep(0.01)
    slave.sdo_write(0x609A, 0, struct.pack("<I", int(accel_pps2))) # 가속도
    time.sleep(0.01)
    slave.sdo_write(0x607C, 0, struct.pack("<i", 0))           # 홈 오프셋 = 0
    time.sleep(0.01)
    slave.sdo_write(0x201E, 0, struct.pack("<H", 0))           # 호밍 완료 후 동작 없음
    time.sleep(0.01)

def _restore_csv_mode_sdo(slave):
    """호밍 후 CSV 모드(9)로 복귀."""
    slave.sdo_write(0x211F, 0, struct.pack("<H", (1 << 12)))   # 모드 변경 언락 (iX7NH 벤더)
    time.sleep(0.01)
    slave.sdo_write(0x6060, 0, struct.pack("<b", 9))
    time.sleep(0.02)


# ─────────────────────────────────────────────────────────────
# EtherCAT 제어 루프 (별도 프로세스)
# ─────────────────────────────────────────────────────────────
def _ethercat_process_loop(
    adapter_name: str,
    num_slaves: int,
    cycle_time_s: float,
    command_queue: mp.Queue,
    shared_states,          # mp.Array('d', num_slaves * SLOT_SIZE)
    lock: mp.Lock,
    max_sync_error_pulse: int,
    coupling_params,        # mp.Array('d', 2): [0]=gain [1]=enabled
    ma_window: int,
    z1_idx: int = -1,       # Z1 슬레이브 인덱스 (동기화 모니터링용)
    z2_idx: int = -1,       # Z2 슬레이브 인덱스 (동기화 모니터링용)
):
    master = pysoem.Master()

    # 슬레이브별 로컬 상태
    local_states = [{
        'target_pulse'    : 0,
        'offset'          : 0,
        'axis'            : 'z',
        'trajectory'      : None,
        'profile_start'   : 0.0,
        'traj_end'        : 0,
        'config_vel_pps'  : 0.0,
        'config_acc_pps2' : 0.0,
        'last_status'     : 0,
        'homing_active'   : False,   # 이 슬레이브가 호밍 중인지
        'homing_method'   : HOMING_METHOD,
        'homing_fast_pps' : rpm_to_pps(HOMING_FAST_RPM),  # 호밍 탐색 속도 (PDO에 직접 기록)
        'bit12_cleared'   : False,   # bit12 잔류 플래그 오탐 방지
        'homing_b4_cycles': 0,       # rising edge 확보용 카운터
    } for _ in range(num_slaves)]

    is_running          = True
    sync_error_detected = False
    diff_histories = [deque(maxlen=ma_window)]  # Z1↔Z2 동기화 오차 이력 (인덱스 0만 사용)
    hold_max_vel = mm_to_pulse(POSITION_HOLD_MAX_VEL_MM_S, 'z')

    # ── 초기 설정 명령 수집 (버스 열기 전) ──
    slave_configs = {i: {} for i in range(num_slaves)}
    while not command_queue.empty():
        try:
            idx, cmd, val = command_queue.get_nowait()
        except Exception:
            break
        if cmd == 'SET_VELOCITY':
            slave_configs[idx]['rpm'] = val
        elif cmd == 'SET_ACCEL':
            slave_configs[idx]['accel'] = val
        elif cmd == 'SET_AXIS':
            local_states[idx]['axis'] = val
        elif cmd == 'SET_HOMING_SPEED':
            fast_rpm, slow_rpm = val
            slave_configs[idx]['homing_fast_rpm'] = fast_rpm
            slave_configs[idx]['homing_slow_rpm'] = slow_rpm
        elif cmd == 'SET_HOMING_METHOD':
            local_states[idx]['homing_method'] = val

    # ── EtherCAT 초기화 (재시도 포함) ──
    init_success = False
    for retry in range(3):
        try:
            if retry > 0:
                try: master.close()
                except: pass
                time.sleep(1.0)
                master = pysoem.Master()

            print(f"[EtherCAT] 어댑터 '{adapter_name}' 열기 (시도 {retry+1})")
            master.open(adapter_name)

            found = master.config_init()
            if found < num_slaves:
                raise RuntimeError(f"슬레이브 부족: 필요={num_slaves}, 발견={found}")
            print(f"[EtherCAT] {found}개 슬레이브 발견")

            for i in range(num_slaves):
                slave = master.slaves[i]
                print(f"  Slave {i}: {slave.name}")
                slave.dc_sync(True, int(cycle_time_s * 1_000_000_000))
                _sdo_reset_fault(slave)

                cfg = slave_configs.get(i, {})
                if 'rpm' in cfg:
                    pps = rpm_to_pps(cfg['rpm'])
                    local_states[i]['config_vel_pps'] = pps
                    slave.sdo_write(0x6081, 0, struct.pack("<I", int(pps)))
                if 'accel' in cfg:
                    acc_pps2, dec_pps2 = cfg['accel']
                    local_states[i]['config_acc_pps2'] = acc_pps2
                    slave.sdo_write(0x6083, 0, struct.pack("<I", int(acc_pps2)))
                    slave.sdo_write(0x6084, 0, struct.pack("<I", int(dec_pps2)))

                _configure_csv_pdos(slave)

                # X/Z 모두 호밍 모드(6) 사전 설정 — OP 진입 전에 수행 (SM 워치독 회피)
                # iX7NH는 OP 중 PDO 0x60FF=0 을 호밍 속도로 덮어쓰므로
                # 모드 변경은 반드시 OP 전 Safe-OP 단계에서 수행해야 한다.
                axis       = local_states[i]['axis']
                method     = local_states[i]['homing_method']
                fast_rpm   = cfg.get('homing_fast_rpm', HOMING_FAST_RPM)
                slow_rpm   = cfg.get('homing_slow_rpm', HOMING_SLOW_RPM)
                fast_pps   = rpm_to_pps(fast_rpm)
                slow_pps   = rpm_to_pps(slow_rpm)
                accel_pps2 = rpm_per_sec_to_pps2(100.0)
                slave.sdo_write(0x211F, 0, struct.pack("<H", (1 << 12)));    time.sleep(0.01)
                slave.sdo_write(0x6098, 0, struct.pack("<b", method));       time.sleep(0.01)
                slave.sdo_write(0x6099, 1, struct.pack("<I", int(fast_pps)));time.sleep(0.01)
                slave.sdo_write(0x6099, 2, struct.pack("<I", int(slow_pps)));time.sleep(0.01)
                slave.sdo_write(0x609A, 0, struct.pack("<I", int(accel_pps2)));time.sleep(0.01)
                slave.sdo_write(0x607C, 0, struct.pack("<i", 0));            time.sleep(0.01)
                slave.sdo_write(0x201E, 0, struct.pack("<H", 0));            time.sleep(0.01)
                slave.sdo_write(0x6060, 0, struct.pack("<b", 6))             # 호밍 모드
                time.sleep(0.02)
                local_states[i]['homing_fast_pps'] = fast_pps
                print(f"  Slave {i} ({axis.upper()}): 호밍 모드(6) 사전 설정 완료 "
                      f"(method={method}, fast={fast_rpm}RPM, slow={slow_rpm}RPM)")

            master.config_map()
            init_success = True
            break
        except Exception as e:
            print(f"[EtherCAT] 초기화 실패 (retry {retry}): {e}")
            if retry == 2:
                try: master.close()
                except: pass
                return

    if not init_success:
        return

    # ── OP 상태 전환 ──
    try:
        for op_retry in range(3):
            try:
                if op_retry > 0:
                    time.sleep(0.5)
                master.state = pysoem.OP_STATE
                master.write_state()
                is_op = False
                for _ in range(int(4.0 / cycle_time_s)):
                    master.send_processdata()
                    master.receive_processdata()
                    master.read_state()
                    if all(s.state == pysoem.OP_STATE for s in master.slaves):
                        is_op = True
                        break
                    time.sleep(cycle_time_s)
                if not is_op:
                    raise RuntimeError("OP 상태 도달 실패")
                print("[EtherCAT] OP 상태 진입 완료. 제어 루프 시작.")
                for i in range(num_slaves):
                    pos = _read_actual_position(master.slaves[i])
                    local_states[i]['target_pulse'] = pos
                break
            except Exception as e:
                print(f"[EtherCAT] OP 전환 실패 (retry {op_retry}): {e}")
                if op_retry == 2:
                    raise

        # ═══════════════════════════════════════════════════════
        # 메인 제어 루프
        # ═══════════════════════════════════════════════════════
        cycle_counter = 0

        while is_running:
            loop_start    = time.monotonic()
            cycle_counter += 1

            coupling_enabled = coupling_params[1] != 0
            coupling_gain    = coupling_params[0]

            # ── 1. 명령 수신 ──
            move_commands = []
            while not command_queue.empty():
                try:
                    idx, cmd, val = command_queue.get_nowait()
                except Exception:
                    break

                if cmd == 'STOP_ALL':
                    is_running = False
                    break
                elif cmd == 'SET_AXIS':
                    local_states[idx]['axis'] = val
                elif cmd == 'SET_ORIGIN':
                    pos = _read_actual_position(master.slaves[idx])
                    local_states[idx]['offset']       = pos
                    local_states[idx]['target_pulse'] = pos
                    local_states[idx]['trajectory']   = None
                    sync_error_detected = False
                    for h in diff_histories:
                        h.clear()
                    print(f"[원점] 모터 {idx}: offset={pos}")
                elif cmd == 'MOVE_TO_MM':
                    if sync_error_detected:
                        print(f"[경고] 동기화 오류 상태. 모터 {idx} 이동 무시")
                    else:
                        move_commands.append((idx, val))
                elif cmd == 'SET_VELOCITY':
                    pps = rpm_to_pps(val)
                    local_states[idx]['config_vel_pps'] = pps
                elif cmd == 'SET_ACCEL':
                    acc_pps2, _ = val
                    local_states[idx]['config_acc_pps2'] = acc_pps2
                elif cmd == 'START_HOMING':
                    method = val if val is not None else local_states[idx]['homing_method']
                    local_states[idx]['homing_active']    = True
                    local_states[idx]['homing_method']    = method
                    local_states[idx]['trajectory']       = None
                    local_states[idx]['bit12_cleared']    = False
                    local_states[idx]['homing_b4_cycles'] = 0
                    with lock:
                        base = idx * SLOT_SIZE
                        shared_states[base + IDX_HOMED]      = 0
                        shared_states[base + IDX_HOMING_PH]  = 1  # in_progress
                    # 호밍 모드(6)는 초기화 시(OP 진입 전)에 이미 설정됨.
                    # 재호밍인 경우 0x6060=6 재설정이 필요할 수 있음 (최소 SDO, ~20ms)
                    try:
                        slave = master.slaves[idx]
                        cur_mode = struct.unpack("<b", slave.sdo_read(0x6061, 0))[0]
                        if cur_mode != 6:
                            slave.sdo_write(0x211F, 0, struct.pack("<H", (1 << 12)))
                            time.sleep(0.01)
                            slave.sdo_write(0x6060, 0, struct.pack("<b", 6))
                            time.sleep(0.02)
                            print(f"[호밍] 모터 {idx}: 모드 재설정 ({cur_mode} → 6), 호밍 시작")
                        else:
                            print(f"[호밍] 모터 {idx}: 호밍 모드(6) 확인, 시작")
                    except Exception as e:
                        print(f"[호밍] 모터 {idx}: 모드 확인 실패 ({e}), 계속 진행")
                elif cmd == 'RESET_SYNC_ERROR':
                    sync_error_detected = False
                    for h in diff_histories:
                        h.clear()

            if not is_running:
                break

            # ── 2. PDO 통신 ──
            master.send_processdata()
            master.receive_processdata()
            now = time.monotonic()

            positions  = [_read_actual_position(master.slaves[i]) for i in range(num_slaves)]
            velocities = [_read_actual_velocity(master.slaves[i])  for i in range(num_slaves)]
            relative_positions = [
                positions[i] - local_states[i]['offset']
                for i in range(num_slaves)
            ]

            # ── 3. 이동 명령 → 궤적 생성 ──
            if move_commands:
                for idx, target_mm in move_commands:
                    state  = local_states[idx]
                    axis   = state['axis']
                    cur    = positions[idx]
                    target = mm_to_pulse(target_mm, axis) + state['offset']

                    v_max = state['config_vel_pps'] or rpm_to_pps(30)
                    a_max = state['config_acc_pps2'] or rpm_per_sec_to_pps2(30)

                    profile = TrapezoidalProfile(cur, target, v_max, a_max)
                    state['trajectory']    = profile
                    state['profile_start'] = now
                    state['traj_end']      = target
                    state['target_pulse']  = cur

                    dist_mm = abs(pulse_to_mm(target - cur, axis))
                    print(f"[궤적] 모터 {idx}: {target_mm:.2f}mm "
                          f"(거리={dist_mm:.2f}mm, 예상={profile.t_total:.2f}s)")

            # ── 4. 이동 중 모터 목록 (호밍 중인 슬레이브 제외) ──
            moving_motors = [
                i for i in range(num_slaves)
                if local_states[i]['trajectory'] is not None
                and not local_states[i]['homing_active']
            ]

            # ── 5. 동기화 모니터링 (Z1 ↔ Z2 쌍만) ──
            # X축과 Z축은 서로 다른 단위계이므로 비교하지 않음
            if (z1_idx >= 0 and z2_idx >= 0 and not sync_error_detected and
                    local_states[z1_idx]['trajectory'] is not None and
                    local_states[z2_idx]['trajectory'] is not None):
                diff = abs(relative_positions[z1_idx] - relative_positions[z2_idx])
                diff_histories[0].append(diff)
                avg_diff = sum(diff_histories[0]) / len(diff_histories[0])

                if avg_diff > max_sync_error_pulse:
                    sync_error_detected = True
                    for j in range(num_slaves):
                        if local_states[j]['trajectory'] is not None:
                            local_states[j]['trajectory']   = None
                            local_states[j]['target_pulse'] = positions[j]
                    avg_mm = pulse_to_mm(avg_diff, 'z')
                    lim_mm = pulse_to_mm(max_sync_error_pulse, 'z')
                    print(f"\n[긴급정지] Z축 동기화 오차 초과! "
                          f"Z1-Z2 이동평균={avg_mm:.3f}mm, 임계={lim_mm:.3f}mm")

            # ── 6. Cross Coupling 보정값 (Z1 ↔ Z2 쌍만) ──
            coupling_correction = [0] * num_slaves
            if (coupling_enabled and not sync_error_detected and
                    z1_idx >= 0 and z2_idx >= 0 and
                    local_states[z1_idx]['trajectory'] is not None and
                    local_states[z2_idx]['trajectory'] is not None):
                avg_rel = (relative_positions[z1_idx] + relative_positions[z2_idx]) / 2.0
                coupling_correction[z1_idx] = int(coupling_gain * (relative_positions[z1_idx] - avg_rel))
                coupling_correction[z2_idx] = int(coupling_gain * (relative_positions[z2_idx] - avg_rel))

            # ── 7. 각 슬레이브 제어 ──
            for i in range(num_slaves):
                slave = master.slaves[i]
                state = local_states[i]
                sw    = _read_status_word(slave)

                if state['last_status'] != sw:
                    state['last_status'] = sw

                # Fault 감지 (이동 중)
                if (sw & 0x0008) and state['trajectory'] is not None:
                    print(f"\n[긴급정지] 모터 {i} Fault! 0x{sw:04X}")
                    sync_error_detected = True
                    for j in range(num_slaves):
                        if local_states[j]['trajectory'] is not None:
                            local_states[j]['trajectory']   = None
                            local_states[j]['target_pulse'] = positions[j]

                # ── 호밍 모드 처리 ──
                if state['homing_active']:
                    _handle_homing_cycle(
                        slave, i, state, sw, shared_states, lock, master
                    )
                    continue  # CSV 제어 건너뜀

                # ── CSV 제어 ──
                # Controlword 결정
                if   (sw & 0x004F) == 0x0040: cw = CW_SHUTDOWN
                elif (sw & 0x006F) == 0x0021: cw = CW_SWITCH_ON
                elif (sw & 0x006F) == 0x0023: cw = CW_ENABLE_OPERATION
                elif (sw & 0x006F) == 0x0027: cw = CW_ENABLE_OPERATION
                elif (sw & 0x0008):           cw = CW_FAULT_RESET
                else:                         cw = CW_ENABLE_OPERATION

                # 목표 속도 결정
                target_velocity = 0
                if state['trajectory'] is not None:
                    traj    = state['trajectory']
                    elapsed = now - state['profile_start']
                    actual  = positions[i]

                    if traj.is_time_complete(elapsed):
                        pos_err = state['traj_end'] - actual
                        if abs(pos_err) < POSITION_TOLERANCE_PULSE:
                            state['trajectory']   = None
                            state['target_pulse'] = state['traj_end']
                            target_velocity = 0
                            err_mm = pulse_to_mm(pos_err, state['axis'])
                            print(f"[완료] 모터 {i}: 궤적 완료 (잔류오차={err_mm:.4f}mm)")
                        else:
                            v = int(POSITION_HOLD_GAIN * pos_err)
                            target_velocity = max(-hold_max_vel, min(hold_max_vel, v))
                    else:
                        profile_vel     = traj.get_velocity(elapsed)
                        target_velocity = profile_vel - coupling_correction[i]
                        state['target_pulse'] = actual
                else:
                    # 정지 상태 위치 유지
                    pos_err = state['target_pulse'] - positions[i]
                    if abs(pos_err) > POSITION_TOLERANCE_PULSE:
                        v = int(POSITION_HOLD_GAIN * pos_err)
                        target_velocity = max(-hold_max_vel, min(hold_max_vel, v))

                _write_csv_outputs(slave, cw, target_velocity)

                # 상태 공유
                with lock:
                    base = i * SLOT_SIZE
                    shared_states[base + IDX_STATUS]    = sw
                    shared_states[base + IDX_IS_MOVING] = 1 if state['trajectory'] is not None else 0
                    shared_states[base + IDX_ACT_POS]   = positions[i]
                    shared_states[base + IDX_OFFSET]    = state['offset']
                    shared_states[base + IDX_SYNC_ERR]  = 1 if sync_error_detected else 0
                    shared_states[base + IDX_ACT_VEL]   = velocities[i]

            # ── 사이클 타임 유지 ──
            elapsed_loop = time.monotonic() - loop_start
            sleep_time   = cycle_time_s - elapsed_loop
            if sleep_time > 0:
                time.sleep(sleep_time)

    except Exception as e:
        print(f"[EtherCAT] 프로세스 에러: {e}")
        import traceback; traceback.print_exc()
    finally:
        _shutdown_sequence(master, num_slaves, cycle_time_s)


def _handle_homing_cycle(slave, idx, state, sw, shared_states, lock, master):
    """호밍 모드(6) 상태 기계 처리 - 매 사이클 호출.

    bit12_cleared: 이전 세션의 잔류 bit12 오탐 방지 (.source homing_test.py 패턴)
    homing_b4_cycles: rising edge 확보를 위해 bit4=0 최소 3사이클 유지 후 bit4=1
    """
    base = idx * SLOT_SIZE

    homing_attained = bool(sw & (1 << 12))
    homing_error    = bool(sw & (1 << 13))

    # bit12 클리어 추적 — 잔류 플래그 오탐 방지
    if not homing_attained and not homing_error:
        state['bit12_cleared'] = True

    # CiA 402 상태 머신: Enable Operation 유지 / rising edge 확보
    if   (sw & 0x004F) == 0x0040: cw = CW_SHUTDOWN
    elif (sw & 0x006F) == 0x0021: cw = CW_SWITCH_ON
    elif (sw & 0x006F) == 0x0023: cw = CW_ENABLE_OPERATION
    elif (sw & 0x0008):           cw = CW_FAULT_RESET
    else:
        # Operation Enabled 상태: bit4=0을 최소 3사이클 유지 후 rising edge
        if state['homing_b4_cycles'] < 3:
            cw = CW_ENABLE_OPERATION
            state['homing_b4_cycles'] += 1
        else:
            cw = CW_HOMING_START

    if homing_error:
        print(f"[호밍] 모터 {idx}: 호밍 오류! sw=0x{sw:04X}")
        state['homing_active'] = False
        _restore_csv_mode_sdo(slave)
        with lock:
            shared_states[base + IDX_HOMING_PH] = -1  # error

    elif homing_attained and state['bit12_cleared']:
        # bit12_cleared가 True인 경우만 완료로 인정 (잔류 플래그 오탐 방지)
        print(f"[호밍] 모터 {idx}: 호밍 완료! sw=0x{sw:04X}")
        state['homing_active'] = False
        pos = struct.unpack("<i", slave.input[2:6])[0]
        state['offset']       = pos
        state['target_pulse'] = pos
        _restore_csv_mode_sdo(slave)
        with lock:
            shared_states[base + IDX_HOMED]      = 1
            shared_states[base + IDX_HOMING_PH]  = 2   # done
            shared_states[base + IDX_OFFSET]     = pos
    else:
        # 호밍 진행 중 - controlword + 탐색 속도 전송
        # iX7NH는 호밍 모드에서도 PDO 0x60FF 값을 호밍 속도로 사용하므로
        # 0을 쓰면 호밍 속도가 0 RPM 으로 덮어써져 모터가 움직이지 않는다.
        homing_pps = int(state.get('homing_fast_pps', rpm_to_pps(HOMING_FAST_RPM)))
        _write_csv_outputs(slave, cw, homing_pps)
        with lock:
            shared_states[base + IDX_STATUS]    = sw
            shared_states[base + IDX_HOMING_PH] = 1   # in_progress


def _shutdown_sequence(master, num_slaves: int, cycle_time_s: float):
    """안전 종료 시퀀스."""
    print("[EtherCAT] 종료 시퀀스 시작...")
    try:
        master.read_state()
        if len(master.slaves) > 0 and master.slaves[0].state == pysoem.OP_STATE:
            for i in range(num_slaves):
                try: _write_csv_outputs(master.slaves[i], CW_ENABLE_OPERATION, 0)
                except: pass
            for _ in range(5):
                master.send_processdata()
                master.receive_processdata()
                time.sleep(0.02)
            for i in range(num_slaves):
                try: _write_csv_outputs(master.slaves[i], CW_SWITCH_ON, 0)
                except: pass
            master.send_processdata(); time.sleep(0.1)
            for i in range(num_slaves):
                try: _write_csv_outputs(master.slaves[i], CW_SHUTDOWN, 0)
                except: pass
            master.send_processdata(); time.sleep(0.1)
            for i in range(num_slaves):
                try: _write_csv_outputs(master.slaves[i], CW_DISABLE_VOLTAGE, 0)
                except: pass
            master.send_processdata(); time.sleep(0.2)
            master.state = pysoem.INIT_STATE
            master.write_state()
            time.sleep(0.1)
    except Exception as e:
        print(f"[EtherCAT] 종료 중 경고 (무시): {e}")
    finally:
        master.close()
        print("[EtherCAT] 프로세스 종료 완료.")


# ─────────────────────────────────────────────────────────────
# 버스 관리 클래스 (메인 프로세스에서 사용)
# ─────────────────────────────────────────────────────────────
class EtherCATInterface:
    """
    EtherCAT 버스 인터페이스 (ROS2 노드에서 사용).

    Parameters
    ----------
    adapter_name      : Linux 네트워크 인터페이스 (예: "eth0", "enp3s0")
    num_slaves        : 슬레이브 수
    cycle_time_ms     : 제어 사이클 (ms)
    max_sync_error_mm : 긴급정지 임계값 (mm)
    coupling_gain     : Cross Coupling 게인 [1/sec]
    enable_coupling   : Cross Coupling 활성화
    ma_window         : 동기화 오차 이동평균 윈도우
    """

    def __init__(self, adapter_name: str, num_slaves: int,
                 cycle_time_ms: int = 1,
                 max_sync_error_mm: float = DEFAULT_MAX_SYNC_ERROR_MM,
                 coupling_gain: float = DEFAULT_COUPLING_GAIN,
                 enable_coupling: bool = True,
                 ma_window: int = DEFAULT_MA_WINDOW,
                 z1_idx: int = -1,
                 z2_idx: int = -1):

        self._adapter_name = adapter_name
        self._num_slaves   = num_slaves
        self._cycle_time_s = cycle_time_ms / 1000.0

        self._command_queue   = mp.Queue()
        self._lock            = mp.Lock()
        self._shared_states   = mp.Array('d', num_slaves * SLOT_SIZE, lock=False)
        self._coupling_params = mp.Array('d', 2, lock=True)
        self._coupling_params[0] = coupling_gain
        self._coupling_params[1] = 1.0 if enable_coupling else 0.0

        self._max_sync_error_pulse = mm_to_pulse(max_sync_error_mm, 'z')

        self._process = mp.Process(
            target=_ethercat_process_loop,
            args=(
                adapter_name, num_slaves, self._cycle_time_s,
                self._command_queue, self._shared_states, self._lock,
                self._max_sync_error_pulse,
                self._coupling_params,
                ma_window,
                z1_idx,
                z2_idx,
            ),
            daemon=True,
        )

    # ── 버스 제어 ──

    def start(self):
        self._process.start()
        print(f"[EtherCATInterface] 프로세스 시작 (adapter={self._adapter_name})")

    def stop(self):
        if self._process.is_alive():
            self._command_queue.put((-1, 'STOP_ALL', None))
            self._process.join(timeout=5)
            if self._process.is_alive():
                print("[EtherCATInterface] 강제 종료")
                self._process.terminate()
                self._process.join(timeout=2)
        print("[EtherCATInterface] 종료 완료.")

    def is_alive(self) -> bool:
        return self._process.is_alive()

    # ── 명령 전송 ──

    def set_axis(self, idx: int, axis: str):
        self._command_queue.put((idx, 'SET_AXIS', axis))

    def set_velocity(self, idx: int, rpm: float):
        self._command_queue.put((idx, 'SET_VELOCITY', rpm))

    def set_accel(self, idx: int, accel_rpm_per_s: float, decel_rpm_per_s: float = None):
        acc = rpm_per_sec_to_pps2(accel_rpm_per_s)
        dec = acc if decel_rpm_per_s is None else rpm_per_sec_to_pps2(decel_rpm_per_s)
        self._command_queue.put((idx, 'SET_ACCEL', (acc, dec)))

    def set_homing_speed(self, idx: int, fast_rpm: float, slow_rpm: float = 1.0):
        """호밍 탐색 속도 설정 (버스 시작 전 큐잉 — Safe-OP 단계에서 적용됨)."""
        self._command_queue.put((idx, 'SET_HOMING_SPEED', (fast_rpm, slow_rpm)))

    def set_homing_method(self, idx: int, method: int):
        """호밍 방법 설정 (버스 시작 전 큐잉 — Safe-OP 단계에서 적용됨)."""
        self._command_queue.put((idx, 'SET_HOMING_METHOD', method))

    def set_origin(self, idx: int):
        self._command_queue.put((idx, 'SET_ORIGIN', None))

    def move_to_mm(self, idx: int, target_mm: float):
        self._command_queue.put((idx, 'MOVE_TO_MM', target_mm))

    def start_homing(self, idx: int, method: int = HOMING_METHOD):
        self._command_queue.put((idx, 'START_HOMING', method))

    def reset_sync_error(self):
        self._command_queue.put((-1, 'RESET_SYNC_ERROR', None))

    # ── 상태 읽기 ──

    def get_status_word(self, idx: int) -> int:
        return int(self._shared_states[idx * SLOT_SIZE + IDX_STATUS])

    def is_moving(self, idx: int) -> bool:
        return self._shared_states[idx * SLOT_SIZE + IDX_IS_MOVING] != 0

    def get_position_pulse(self, idx: int) -> int:
        return int(self._shared_states[idx * SLOT_SIZE + IDX_ACT_POS])

    def get_position_mm(self, idx: int, axis: str = 'z') -> float:
        base   = idx * SLOT_SIZE
        pos    = self._shared_states[base + IDX_ACT_POS]
        offset = self._shared_states[base + IDX_OFFSET]
        return pulse_to_mm(pos - offset, axis)

    def get_velocity_mm_s(self, idx: int, axis: str = 'z') -> float:
        vel_pulse = self._shared_states[idx * SLOT_SIZE + IDX_ACT_VEL]
        return pulse_to_mm(vel_pulse, axis)

    def has_sync_error(self) -> bool:
        # 어느 슬레이브든 sync_error = 1이면 True
        for i in range(self._num_slaves):
            if self._shared_states[i * SLOT_SIZE + IDX_SYNC_ERR] != 0:
                return True
        return False

    def is_homed(self, idx: int) -> bool:
        return self._shared_states[idx * SLOT_SIZE + IDX_HOMED] != 0

    def get_homing_phase(self, idx: int) -> int:
        return int(self._shared_states[idx * SLOT_SIZE + IDX_HOMING_PH])

    def is_ready(self, idx: int) -> bool:
        """드라이브가 Operation Enabled 상태인지 확인."""
        sw = self.get_status_word(idx)
        return (sw & 0x006F) == 0x0027

    # ── Cross Coupling 런타임 제어 ──

    @property
    def coupling_gain(self) -> float:
        return self._coupling_params[0]

    @coupling_gain.setter
    def coupling_gain(self, value: float):
        self._coupling_params[0] = max(0.0, value)

    @property
    def coupling_enabled(self) -> bool:
        return self._coupling_params[1] != 0

    @coupling_enabled.setter
    def coupling_enabled(self, value: bool):
        self._coupling_params[1] = 1.0 if value else 0.0
