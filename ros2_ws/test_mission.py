#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from uav_web_agent.msg import DeliveryMission
import time
import uuid

class MissionInjector(Node):
    def __init__(self):
        super().__init__('mission_injector')
        self.pub = self.create_publisher(DeliveryMission, '/uav/mission/new', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.done = False

    def timer_callback(self):
        if self.done:
            return

        msg = DeliveryMission()
        msg.mission_id = str(uuid.uuid4())
        msg.uav_id = "uav1"
        
        # Current Home is approx (47.3977419, 8.5455939)
        # We want to fly to (10, 0) relative to home.
        # 1 deg lat ~ 111km, 1 deg lon ~ 78km at this lat.
        # +10m East (x) -> +0.00013 lon approx
        # +0m North (y) -> +0 lat
        
        home_lat = 47.3977419
        home_lon = 8.5455939
        
        # Pickup: 10m East, 0m North
        # (This forces it to FLY to pickup, not just climb)
        msg.pickup_lat = home_lat
        msg.pickup_lon = home_lon + 0.00013 
        msg.pickup_alt = 0.0
        
        # Dropoff: 10m East, 20m North
        msg.dropoff_lat = home_lat + 0.00018  # +20m North
        msg.dropoff_lon = home_lon + 0.00013  # +10m East
        msg.dropoff_alt = 0.0
        
        msg.deadline_ts = int(time.time()) + 600
        msg.priority = "normal"
        
        self.get_logger().info(f"Publishing mission {msg.mission_id}...")
        self.pub.publish(msg)
        self.done = True
        
        # Self-terminate after publishing
        time.sleep(1.0)
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = MissionInjector()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
