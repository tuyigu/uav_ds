import asyncio
import logging
import math
import time
import grpc
from concurrent import futures
import cv2
import numpy as np
import json
from aiohttp import web
from aiortc import MediaStreamTrack, RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaPlayer

# Fix imports to work with the package structure
try:
    from app.grpc_generated import uav_pb2, uav_pb2_grpc
except ImportError:
    # Fallback for running script directly (only if generated code didn't use relative imports)
    import sys
    import os
    sys.path.append(os.path.join(os.path.dirname(__file__), '../app/grpc_generated'))
    try:
        import uav_pb2
        import uav_pb2_grpc
    except ImportError:
        print("CRITICAL: Could not import generated gRPC modules. Please ensure you have run the protoc command.")
        sys.exit(1)

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("MockDrone")

class VideoStreamTrack(MediaStreamTrack):
    """
    A video stream track that generates a bouncing ball animation.
    """
    kind = "video"

    def __init__(self):
        super().__init__()
        self.height, self.width = 480, 640
        self.ball_x, self.ball_y = 320, 240
        self.ball_dx, self.ball_dy = 5, 5
        self.start_time = time.time()

    async def recv(self):
        # Manual timestamp calculation (30 FPS, 90000 timebase)
        pts = int((time.time() - self.start_time) * 90000)
        time_base = 90000
        
        # Update ball position
        self.ball_x += self.ball_dx
        self.ball_y += self.ball_dy

        if self.ball_x <= 20 or self.ball_x >= self.width - 20: self.ball_dx *= -1
        if self.ball_y <= 20 or self.ball_y >= self.height - 20: self.ball_dy *= -1

        # Create frame
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        
        # Draw background grid
        for i in range(0, self.width, 40):
            cv2.line(frame, (i, 0), (i, self.height), (30, 30, 30), 1)
        for i in range(0, self.height, 40):
            cv2.line(frame, (0, i), (self.width, i), (30, 30, 30), 1)

        # Draw HUD
        cv2.putText(frame, f"MOCK DRONE CAM", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"T: {time.time() - self.start_time:.1f}s", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
        
        # Draw ball
        cv2.circle(frame, (int(self.ball_x), int(self.ball_y)), 20, (0, 0, 255), -1)

        # Convert to aiortc VideoFrame
        from av import VideoFrame
        new_frame = VideoFrame.from_ndarray(frame, format="bgr24")
        new_frame.pts = pts
        new_frame.time_base = time_base
        return new_frame

class DroneService(uav_pb2_grpc.FlightControlServicer if 'uav_pb2_grpc' in locals() else object):
    def __init__(self):
        self.state = uav_pb2.UavState()
        self.state.mode = "MANUAL"
        self.state.flight_phase = "IDLE"
        self.state.battery = 1.0
        self.state.armed = False
        self.state.connected = True
        self.state.position.x = 0.0
        self.state.position.y = 0.0
        self.state.position.z = 0.0
        self.state.position.yaw = 0.0
        self.target_height = 0.0
        self._target_pos = None

    async def Arm(self, request, context):
        logger.info("Command: ARM")
        self.state.armed = True
        self.state.mode = "OFFBOARD"
        return uav_pb2.CommandReply(success=True, message="Armed")

    async def Disarm(self, request, context):
        logger.info("Command: DISARM")
        self.state.armed = False
        self.state.mode = "MANUAL"
        return uav_pb2.CommandReply(success=True, message="Disarmed")

    async def Takeoff(self, request, context):
        logger.info(f"Command: TAKEOFF {request.height}m")
        if not self.state.armed:
            return uav_pb2.CommandReply(success=False, message="Not armed")
        
        self.state.flight_phase = "TAKING_OFF"
        self.target_height = request.height
        asyncio.create_task(self._simulate_takeoff())
        return uav_pb2.CommandReply(success=True, message="Taking off")

    async def Land(self, request, context):
        logger.info("Command: LAND")
        self.state.flight_phase = "LANDING"
        asyncio.create_task(self._simulate_land())
        return uav_pb2.CommandReply(success=True, message="Landing")
    
    async def ReturnToHome(self, request, context):
        logger.info("Command: RETURN_HOME")
        self.state.flight_phase = "RETURNING"
        asyncio.create_task(self._simulate_move(to_home=True))
        return uav_pb2.CommandReply(success=True, message="Returning Home")

    async def GoTo(self, request, context):
        logger.info(f"Command: GOTO ({request.x}, {request.y}, {request.z})")
        self.state.flight_phase = "MOVING"
        self._target_pos = request
        asyncio.create_task(self._simulate_move())
        return uav_pb2.CommandReply(success=True, message="Moving")

    async def _simulate_takeoff(self):
        while self.state.position.z < self.target_height:
            self.state.position.z += 0.1
            await asyncio.sleep(0.1)
        self.state.flight_phase = "HOVER"

    async def _simulate_land(self):
        while self.state.position.z > 0:
            self.state.position.z -= 0.1
            await asyncio.sleep(0.1)
        self.state.flight_phase = "IDLE"
        self.state.armed = False

    async def _simulate_move(self, to_home=False):
        steps = 50
        start_x = self.state.position.x
        start_y = self.state.position.y
        
        target_x = 0.0 if to_home else (self._target_pos.x if self._target_pos else 0.0)
        target_y = 0.0 if to_home else (self._target_pos.y if self._target_pos else 0.0)

        dx = (target_x - start_x) / steps
        dy = (target_y - start_y) / steps

        for _ in range(steps):
            self.state.position.x += dx
            self.state.position.y += dy
            await asyncio.sleep(0.1)
        
        self.state.flight_phase = "HOVER"

    async def StreamState(self, request, context):
        while True:
            self.state.timestamp = int(time.time() * 1000)
            # Drain battery slowly
            self.state.battery = max(0.0, self.state.battery - 0.0001)
            yield self.state
            await asyncio.sleep(0.5)

# ── WebRTC Signaling Server (Mock) ─────────────────────────────────────────────

pcs = set()

async def offer(request):
    params = await request.json()
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

    pc = RTCPeerConnection()
    pcs.add(pc)

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        logger.info("Connection state is %s", pc.connectionState)
        if pc.connectionState == "failed":
            await pc.close()
            pcs.discard(pc)

    # Add video track
    video = VideoStreamTrack()
    pc.addTrack(video)

    await pc.setRemoteDescription(offer)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    return web.Response(
        content_type="application/json",
        text=json.dumps(
            {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
        ),
    )

async def cors_middleware(app, handler):
    async def middleware(request):
        if request.method == 'OPTIONS':
            response = web.Response()
        else:
            response = await handler(request)
        
        response.headers['Access-Control-Allow-Origin'] = '*'
        response.headers['Access-Control-Allow-Methods'] = 'POST, GET, OPTIONS'
        response.headers['Access-Control-Allow-Headers'] = 'Content-Type'
        return response
    return middleware

async def start_webrtc_server():
    app = web.Application(middlewares=[cors_middleware])
    app.router.add_post("/offer", offer)
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "0.0.0.0", 8081)
    logger.info("WebRTC Signal Server running on 8081")
    await site.start()

async def start_grpc_server():
    server = grpc.aio.server()
    uav_pb2_grpc.add_FlightControlServicer_to_server(DroneService(), server)
    server.add_insecure_port('[::]:50051')
    logger.info("gRPC Server running on 50051")
    await server.start()
    await server.wait_for_termination()

if __name__ == "__main__":
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    
    # Run both servers
    loop.create_task(start_webrtc_server())
    loop.run_until_complete(start_grpc_server())
