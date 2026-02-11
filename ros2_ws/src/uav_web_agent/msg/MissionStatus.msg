string mission_id
string uav_id
string status          # ACCEPTED / EN_ROUTE_PICKUP / WAITING_LOAD / EN_ROUTE_DROPOFF / WAITING_UNLOAD / RETURNING / COMPLETED / ABORTED / FAILED_SAFE
string reason
float32 progress       # 0.0 - 1.0
string user_facing_msg # Human-readable status for UI display
int64 eta_seconds      # Estimated time of arrival, 0 if unknown
