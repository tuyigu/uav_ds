# 任务唯一 ID (由 Web/Python 生成，如 "TASK_20260204_001")
string task_id

# 任务创建时间戳 (Unix Timestamp)
float64 created_at

# 任务超时时间 (秒)，例如 600.0 (10分钟飞不到就自动取消)
float32 timeout_seconds

# --- 取货点 (Pickup) ---
# 坐标系: Map Frame (相对原点，ENU坐标系)
float32 pickup_x
float32 pickup_y
float32 pickup_z
float32 pickup_yaw

# --- 送货点 (Dropoff) ---
float32 dropoff_x
float32 dropoff_y
float32 dropoff_z
float32 dropoff_yaw