std_msgs/Header header
float64 x_position_mm      # X축 현재 위치 [mm]
float64 z_position_mm      # Z축 현재 위치 [mm] (모터 1, 2 평균)
float64 x_velocity_mm_s    # X축 현재 속도 [mm/s]
float64 z_velocity_mm_s    # Z축 현재 속도 [mm/s]
bool    x_homed             # X축 호밍 완료 여부
bool    z_homed             # Z축 호밍 완료 여부
string  drive_state         # CiA 402 상태 문자열
float64 z1_position_mm     # Z축 모터1 개별 위치 [mm]
float64 z2_position_mm     # Z축 모터2 개별 위치 [mm]
float64 z_sync_error_mm    # Z축 동기화 오차 [mm]
bool    sync_fault          # 동기화 오류 플래그
