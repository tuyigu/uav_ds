#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import queue
import sys
import os
import threading

# Add web_server generated protos to path if not present
# Assuming relative path from workspace root
# In a real deployed env, these should be a python package
generated_proto_path = "/home/wss/Dev/Robotics/workspaces/uav_ds/web_server/app/grpc_generated"
if generated_proto_path not in sys.path:
    sys.path.append(generated_proto_path)

try:
    import uav_pb2
except ImportError:
    uav_pb2 = None

from flight_core.msg import UavState
from flight_core.action import Takeoff, Land, MoveTo
from uav_web_agent.msg import MissionIntent, MissionStatus # Legacy msg reuse for BT interface

from .grpc_server import GrpcServerV2
from .intent_engine_v2 import IntentEngineV2, Task

class OrchestratorNodeV2(Node):
    def __init__(self):
        super().__init__('mission_orchestrator_v2')
        self.get_logger().info("Initializing Mission Orchestrator V2...")

        # 1. Action Clients (Flight Core)
        self._ac_takeoff = ActionClient(self, Takeoff, 'takeoff')
        self._ac_land = ActionClient(self, Land, 'land')
        self._ac_moveto = ActionClient(self, MoveTo, 'move_to')

        # 2. Publishers (BT Agent)
        self._pub_intent = self.create_publisher(MissionIntent, '/orchestrator/intent', 10)

        # 3. Subscribers (Flight Core State)
        self.create_subscription(UavState, '/flight/uav_state', self._on_uav_state, 10)
        
        # 4. Subscribers (BT Status)
        # Using MissionStatus msg to get feedback from BT
        self.create_subscription(MissionStatus, '/bt/execution_status', self._on_bt_status, 10)

        # 5. Internal Logic
        self.intent_engine = IntentEngineV2()
        self.command_queue = queue.Queue()
        
        # 6. gRPC Server
        self.grpc_server = GrpcServerV2(self.command_queue)
        self.grpc_server.start()

        # 7. Main Loop Timer (5Hz)
        self.create_timer(0.2, self._control_loop)
        
        self.last_bt_status = "IDLE"
        self.current_uav_state = None

    def _on_uav_state(self, msg: UavState):
        self.current_uav_state = msg
        
        # Update gRPC telemetry
        if uav_pb2:
            # Use UavState message definition
            telemetry = uav_pb2.UavState(
                mission_id=msg.mission_id if hasattr(msg, 'mission_id') else "IDLE",
                mode=msg.mode,
                flight_phase=msg.phase, # msg.phase maps to flight_phase
                battery=msg.battery,
                position=uav_pb2.Position(x=msg.x, y=msg.y, z=msg.z, yaw=msg.yaw),
                armed=msg.armed,
                connected=True, # Assuming connected if we are receiving state
                timestamp=int(self.get_clock().now().nanoseconds / 1e6) # ms timestamp
            )
            self.grpc_server.update_telemetry(telemetry)

    def _on_bt_status(self, msg: MissionStatus):
        # Feedback from BT
        self.last_bt_status = msg.status # e.g., SUCCESS, RUNNING, FAILED

    def _control_loop(self):
        # A. Process gRPC Commands
        while not self.command_queue.empty():
            cmd_type, data = self.command_queue.get()
            self._handle_command(cmd_type, data)

        # B. Run Intent Engine
        if self.current_uav_state:
            intent_type, intent_params = self.intent_engine.update(self.last_bt_status, self.current_uav_state)
            
            # Reset BT status if we consumed a SUCCESS/FAILURE
            if self.last_bt_status in ["SUCCESS", "FAILED"]:
                self.last_bt_status = "IDLE" 

            # C. Publish Intent to BT
            if intent_type != "IDLE":
                msg = MissionIntent()
                msg.intent = intent_type
                msg.phase = self.intent_engine.ctx.state # Map engine state to phase
                
                # Fill params based on intent type
                if "x" in intent_params: msg.target_x = float(intent_params["x"])
                if "y" in intent_params: msg.target_y = float(intent_params["y"])
                if "z" in intent_params: msg.target_z = float(intent_params["z"])
                
                self._pub_intent.publish(msg)

    def _handle_command(self, cmd_type, data):
        self.get_logger().info(f"Processing command: {cmd_type}")
        
        # Direct Action Control (Manual Override)
        if cmd_type == "TAKEOFF":
            if self.intent_engine.ctx.state == "IDLE":
                goal_msg = Takeoff.Goal()
                goal_msg.height = float(data.get('height', 5.0))
                self._ac_takeoff.wait_for_server(timeout_sec=1.0)
                self._ac_takeoff.send_goal_async(goal_msg)
        
        elif cmd_type == "LAND":
            goal_msg = Land.Goal()
            self._ac_land.wait_for_server(timeout_sec=1.0)
            self._ac_land.send_goal_async(goal_msg)

        elif cmd_type == "START_MISSION":
            # Just a test mission for now
            tasks = [
                Task("TAKEOFF", {"height": 5.0}),
                Task("GOTO", {"x": 0.0, "y": 10.0, "z": 5.0}),
                Task("LAND"),
                Task("TAKEOFF", {"height": 5.0}),
                Task("RETURN_HOME"),
                Task("LAND")
            ]
            self.intent_engine.load_mission("TEST_MISSION", tasks)

    def destroy_node(self):
        self.grpc_server.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OrchestratorNodeV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
