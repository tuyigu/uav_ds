string phase          # 高层飞行阶段：IDLE / TAKING_OFF / CRUISE / HOVER / PRECISION_LAND / LANDED / FAIL 等
string sub_phase      # 可选子状态/原因，例如 LOW_BATTERY / MISSION_ABORTED

float32 x
float32 y
float32 z
float32 yaw

bool connected
bool armed

float32 battery

