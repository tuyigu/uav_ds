import grpc
import logging
from ..config import settings
from ..grpc_generated import uav_pb2, uav_pb2_grpc

logger = logging.getLogger(__name__)

class DroneClient:
    def __init__(self):
        self.channel = None
        self.stub = None

    async def connect(self):
        """Establish gRPC channel"""
        self.channel = grpc.aio.insecure_channel(settings.GRPC_TARGET)
        self.stub = uav_pb2_grpc.FlightControlStub(self.channel)
        logger.info(f"gRPC Channel created to {settings.GRPC_TARGET}")

    async def ensure_connected(self):
        if not self.stub:
            logger.info("Stub is None, attempting to connect...")
            await self.connect()

    async def arm(self):
        await self.ensure_connected()
        logger.info("Sending Arm command...")
        try:
            return await self.stub.Arm(uav_pb2.Empty())
        except Exception as e:
            logger.error(f"Arm failed: {e}")
            raise

    async def disarm(self):
        await self.ensure_connected()
        return await self.stub.Disarm(uav_pb2.Empty())

    async def takeoff(self, height: float):
        await self.ensure_connected()
        request = uav_pb2.TakeoffRequest(height=height)
        logger.info(f"Sending Takeoff command to {height}m...")
        try:
            return await self.stub.Takeoff(request)
        except Exception as e:
            logger.error(f"Takeoff failed: {e}")
            raise

    async def land(self):
        await self.ensure_connected()
        return await self.stub.Land(uav_pb2.Empty())
        
    async def return_to_home(self):
        await self.ensure_connected()
        return await self.stub.ReturnToHome(uav_pb2.Empty())

    async def go_to(self, x: float, y: float, z: float, yaw: float = 0.0):
        await self.ensure_connected()
        pos = uav_pb2.Position(x=x, y=y, z=z, yaw=yaw)
        return await self.stub.GoTo(pos)

    async def stream_state(self):
        """Yields state updates from the drone"""
        if not self.stub:
            await self.connect()
        
        try:
            async for state in self.stub.StreamState(uav_pb2.Empty()):
                yield state
        except grpc.RpcError as e:
            logger.error(f"gRPC Stream Error: {e}")

drone_client = DroneClient()
