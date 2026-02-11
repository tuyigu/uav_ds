import os
import roslibpy
from ..models import DeliveryMission, OperatorCommandCreate

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
        self.command_topic = None
        
        try:
            self.client.run(timeout=2) # 尝试非阻塞连接
            print(f"[RobotGateway] Connected to ROS Bridge at {host}:{port}")
            self._ensure_topics()
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

    def _ensure_topics(self):
        """Ensure topics are initialized if client is connected."""
        if self.client.is_connected:
            if not self.topic:
                self.topic = roslibpy.Topic(self.client, '/web/mission_in', 'uav_web_agent/msg/DeliveryMission')
            
            if not self.command_topic:
                self.command_topic = roslibpy.Topic(self.client, '/web/command_in', 'uav_web_agent/msg/OperatorCommand')
            
            if not hasattr(self, 'status_topic') or not self.status_topic:
                self.status_topic = roslibpy.Topic(self.client, '/web/mission_status', 'uav_web_agent/msg/MissionStatus')
                self.status_topic.subscribe(self._ros_status_callback)

    def send_mission(self, mission: DeliveryMission) -> bool:
        if not self.client.is_connected:
            print(f"[RobotGateway] Error: Not connected to ROS Bridge")
            try:
                self.client.run(timeout=2)
            except Exception as e:
                print(f"[RobotGateway] Reconnect failed: {e}")
                return False

        self._ensure_topics()

        if not self.topic:
            print("[RobotGateway] Error: Topic not initialized")
            return False

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
            self.topic.publish(roslibpy.Message(msg))
            print(f"[RobotGateway] Published mission {mission.mission_id}")
            return True
        except Exception as e:
            print(f"[RobotGateway] Publish failed: {e}")
            return False

    def send_command(self, cmd: OperatorCommandCreate) -> bool:
        """Send operator command to Orchestrator via rosbridge."""
        if not self.client.is_connected:
            print("[RobotGateway] Error: Not connected to ROS Bridge")
            try:
                self.client.run(timeout=1)
            except Exception:
                pass # Try anyway

        self._ensure_topics()

        if not self.command_topic:
            print("[RobotGateway] Error: Command topic not initialized")
            return False

        msg = {
            "command": cmd.command,
            "mission_id": cmd.mission_id,
            "reason": cmd.reason or "",
        }

        try:
            self.command_topic.publish(roslibpy.Message(msg))
            print(f"[RobotGateway] Published command: {cmd.command} (mission={cmd.mission_id})")
            return True
        except Exception as e:
            print(f"[RobotGateway] Command publish failed: {e}")
            return False

gateway = RobotGateway()


