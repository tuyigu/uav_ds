# 对应 DeliveryTask 的 ID
string task_id

# 当前状态枚举
uint8 status
uint8 IDLE=0            # 空闲
uint8 PROCEEDING_TO_PICKUP=1  # 正在去取货
uint8 PICKING_UP=2      # 到达取货点/悬停中
uint8 PROCEEDING_TO_DROPOFF=3 # 正在去送货
uint8 DROPPING_OFF=4    # 到达送货点/悬停中
uint8 COMPLETED=5       # 任务成功结束
uint8 FAILED=6          # 任务失败 (如超时、电量不足)

# 额外信息 (用于 Debug 或展示给用户，如 "Landed safely")
string message