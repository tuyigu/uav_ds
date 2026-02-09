import os
import roslibpy
from ..models import DeliveryMission

class RobotGateway:
    """
    通过 WebSocket (rosbridge) 连接到 ROS 2 系统。
    """
    def __init__(self):
        # 默认连接本地 9090 端口，可通过环境变量配置
        host = os.getenv("ROS_BRIDGE_HOST", "127.0.0.1")
        port = int(os.getenv("ROS_BRIDGE_PORT", 9090))
        
        self.client = roslibpy.Ros(host=host, port=port)
        self.topic = None
        
        try:
            self.client.run(timeout=2) # 尝试非阻塞连接
            print(f"[RobotGateway] Connected to ROS Bridge at {host}:{port}")
            
            # 初始化 Topic 对象
            self.topic = roslibpy.Topic(self.client, '/web/mission_in', 'uav_web_agent/msg/DeliveryMission')
            
            # 订阅任务状态
            self.status_topic = roslibpy.Topic(self.client, '/web/mission_status', 'uav_web_agent/msg/MissionStatus')
            self.status_topic.subscribe(self._ros_status_callback)
            
            self.external_status_callback = None
            
        except Exception as e:
            print(f"[RobotGateway] Warning: improved connection handling needed. Error: {e}")

    def set_status_callback(self, callback):
        """设置外部回调函数，用于处理接收到的状态更新"""
        self.external_status_callback = callback

    def _ros_status_callback(self, msg):
        """内部 ROS 回调，转发给外部注册的函数"""
        if self.external_status_callback:
            try:
                self.external_status_callback(msg)
            except Exception as e:
                print(f"[RobotGateway] Error in status callback: {e}")

    def send_mission(self, mission: DeliveryMission) -> bool:
        if not self.client.is_connected:
            print(f"[RobotGateway] Error: Not connected to ROS Bridge at {self.client.host}:{self.client.port}")
            # 尝试重连 (简单策略)
            try:
                print("[RobotGateway] Attempting to reconnect...")
                self.client.run(timeout=1)
            except Exception as e:
                print(f"[RobotGateway] Reconnect failed: {e}")
                return False

        # 构造 ROS 消息字典 (扁平化，匹配 DeliveryMission.msg)
        msg = {
            "mission_id": mission.mission_id,
            "uav_id": mission.uav_id,
            "pickup_lat": mission.pickup.lat,
            "pickup_lon": mission.pickup.lon,
            "pickup_alt": mission.pickup.alt,
            "dropoff_lat": mission.dropoff.lat,
            "dropoff_lon": mission.dropoff.lon,
            "dropoff_alt": mission.dropoff.alt,
            "priority": mission.priority.value,
            "deadline_ts": mission.deadline_ts or 0,
            "note": mission.note or ""
        }
        
        try:
            print(f"[RobotGateway] Publishing mission {mission.mission_id} to {self.topic.name}...")
            self.topic.publish(roslibpy.Message(msg))
            print(f"[RobotGateway] Successfully published mission {mission.mission_id}")
            return True
        except Exception as e:
            print(f"[RobotGateway] Publish failed: {e}")
            return False

gateway = RobotGateway()

