# 동시 이동 버그 분석 및 수정 보고서

## 1. 버그 요약 (Issue #2)

`gantry_index=2`(동시 이동) 시 서로 다른 목표를 지정해도 실제 이동이 발생하지 않고 SUCCEEDED가 반환되는 문제.

**원인**: 기존 코드가 `gantry2_x/z_target_mm`만 읽었는데, 사용자는 `gantry0_x/z`, `gantry1_x/z`에 목표를 넣었기 때문. `gantry2_x/z_target_mm`의 기본값이 `0.0`이므로 `move_x/z = False`가 아닌 "0mm로 이동" 또는 "이동 없음(-1e9)" 처리로 분기되어 실제 이동이 없었음.

---

## 2. 수정 내용

### `MoveAxis.action`
- `gantry2_x_target_mm`, `gantry2_z_target_mm` 필드 **삭제**
- `gantry_index=2`는 `gantry0_x/z`를 갠트리0에, `gantry1_x/z`를 갠트리1에 각각 적용

### `move_action_server.py`
```python
# 수정 전
targets = {
    0: (req.gantry2_x_target_mm, req.gantry2_z_target_mm),
    1: (req.gantry2_x_target_mm, req.gantry2_z_target_mm),
}

# 수정 후
targets = {
    0: (req.gantry0_x_target_mm, req.gantry0_z_target_mm),
    1: (req.gantry1_x_target_mm, req.gantry1_z_target_mm),
}
```

**사용법 변경**:
```bash
# 서로 다른 목표 동시 이동
gantry_index: 2, gantry0_x: 100.0, gantry0_z: -50.0, gantry1_x: 200.0, gantry1_z: -150.0

# 동일 목표 동시 이동 (gantry0 == gantry1)
gantry_index: 2, gantry0_x: 500.0, gantry0_z: -200.0, gantry1_x: 500.0, gantry1_z: -200.0
```

---

## 3. 0→1→2 순서 정상 동작 확인

**시나리오**: `gantry_index=0` 단독 이동 완료 → `gantry_index=1` 단독 이동 완료 → `gantry_index=2` 동시 이동

| 항목 | 결과 |
|------|------|
| 상태 누출 | 없음. `targets`, `active_gantries`는 매 호출마다 새로 생성되는 지역변수 |
| EtherCAT 모터 상태 | 이전 이동 완료 후 `trajectory=None` 유지, position hold만 동작 |
| 동시 이동 명령 전달 | `active_gantries=[0,1]`로 두 갠트리 모두 `MOVE_TO_MM` 큐 전송 |
| 완료 감지 | 두 갠트리의 모든 모터가 `is_moving=False`가 될 때까지 대기 |

**결론**: 정상 동작.

---

## 4. 갠트리 간 Z축 싱크 구조 확인

`motor_driver_node.py`에서 `z_pairs`를 다음과 같이 구성:

```python
z_pairs = [
    (g0_z1_idx, g0_z2_idx),   # 갠트리0 내부 Z쌍
    (g1_z1_idx, g1_z2_idx),   # 갠트리1 내부 Z쌍
]
```

EtherCAT 프로세스는 이 `z_pairs`를 기반으로 동기화를 수행하므로:

| 관계 | 동기화 모니터링 | Cross Coupling 보정 |
|------|:-----------:|:-----------------:|
| 갠트리0 Z1 ↔ 갠트리0 Z2 | ✓ | ✓ |
| 갠트리1 Z1 ↔ 갠트리1 Z2 | ✓ | ✓ |
| 갠트리0 Z ↔ 갠트리1 Z   | 없음 | 없음 |

**결론**: 갠트리 내부 Z축 간 싱크만 존재하고, 갠트리 간 싱크는 의도한 대로 없음.
