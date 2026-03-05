import grpc
import logging
from concurrent import futures
import queue
import threading
import time

# Import generated protobuf code
# In a real setup, these should be installed or configured in python path.
# For now, we assume they are available or we might need to adjust imports.
# To keep it robust, we'll use try-except or assume the user sets PYTHONPATH properly.
try:
    import uav_pb2
    import uav_pb2_grpc
except ImportError as e:
    # If running in ROS 2 environment without generated code in path, 
    # we might need to add the web_server path or regenerate.
    # For now, let's assume standard import works or user fixes path.
    import sys
    logging.warning(f"gRPC protos not found. Error: {e}")
    logging.warning(f"Current sys.path: {sys.path}")
    uav_pb2 = None
    uav_pb2_grpc = None


class GrpcServerV2(uav_pb2_grpc.FlightControlServicer if uav_pb2_grpc else object):
    def __init__(self, command_queue: queue.Queue, port=50051):
        self.command_queue = command_queue
        self.port = port
        self.server = None
        self.logger = logging.getLogger("GrpcServerV2")
        
        # Shared state for telemetry streaming
        self.telemetry_lock = threading.Lock()
        self.latest_telemetry = None

    def start(self):
        if not uav_pb2_grpc:
            self.logger.error("Cannot start gRPC server: protos missing")
            return
            
        self.server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
        uav_pb2_grpc.add_FlightControlServicer_to_server(self, self.server)
        self.server.add_insecure_port(f'[::]:{self.port}')
        self.server.start()
        self.logger.info(f"gRPC Server started on port {self.port}")

    def stop(self):
        if self.server:
            self.server.stop(0)
            self.logger.info("gRPC Server stopped")

    def update_telemetry(self, telemetry_data):
        with self.telemetry_lock:
            self.latest_telemetry = telemetry_data

    # --- gRPC Methods Implementation ---

    def Arm(self, request, context):
        self.logger.info("Received ARM request")
        self.command_queue.put(("ARM", None))
        return uav_pb2.CommandReply(success=True, message="Arming queued")

    def Disarm(self, request, context):
        self.logger.info("Received DISARM request")
        self.command_queue.put(("DISARM", None))
        return uav_pb2.CommandReply(success=True, message="Disarming queued")

    def Takeoff(self, request, context):
        self.logger.info(f"Received TAKEOFF request: height={request.height}")
        self.command_queue.put(("TAKEOFF", {"height": request.height}))
        return uav_pb2.CommandReply(success=True, message="Takeoff queued")

    def Land(self, request, context):
        self.logger.info("Received LAND request")
        self.command_queue.put(("LAND", None))
        return uav_pb2.CommandReply(success=True, message="Landing queued")

    def ReturnToHome(self, request, context):
        self.logger.info("Received RETURN_HOME request")
        self.command_queue.put(("RETURN_HOME", None))
        return uav_pb2.CommandReply(success=True, message="Return to Home queued")

    def GoTo(self, request, context):
        self.logger.info(f"Received GOTO request: x={request.x}, y={request.y}, z={request.z}, yaw={request.yaw}")
        self.command_queue.put(("GOTO", {"x": request.x, "y": request.y, "z": request.z, "yaw": request.yaw}))
        return uav_pb2.CommandReply(success=True, message="GoTo queued")

    def StartMission(self, request, context):
        # Note: Not in current proto, but kept for logic if needed internally or if proto updates
        self.logger.info(f"Received START_MISSION request: {request.mission_id}")
        mission_data = {
            "mission_id": request.mission_id,
            "waypoints": request.waypoints, 
        }
        self.command_queue.put(("START_MISSION", mission_data))
        return uav_pb2.CommandReply(success=True, message="Mission queued")

    def StreamState(self, request, context):
        self.logger.info(f"Client connected to telemetry stream: {context.peer()}")
        while context.is_active():
            with self.telemetry_lock:
                data = self.latest_telemetry
            
            if data:
                yield data
            else:
                # Yield empty/default if no data yet, or just wait
                pass
                
            time.sleep(0.1)  # 10Hz update rate
