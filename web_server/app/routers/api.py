from fastapi import APIRouter, HTTPException, WebSocket, WebSocketDisconnect
from pydantic import BaseModel
from typing import List
import asyncio
import json
import logging

from ..services.grpc_client import drone_client
from ..grpc_generated import uav_pb2

router = APIRouter()
logger = logging.getLogger("API")

# ── Data Models ───────────────────────────────────────────────
class CommandResponse(BaseModel):
    success: bool
    message: str

class Position(BaseModel):
    x: float
    y: float
    z: float
    yaw: float = 0.0

# ── REST Endpoints ────────────────────────────────────────────

@router.post("/arm", response_model=CommandResponse)
async def arm_drone():
    """Arm the drone"""
    res = await drone_client.arm()
    return {"success": res.success, "message": res.message}

@router.post("/disarm", response_model=CommandResponse)
async def disarm_drone():
    """Disarm the drone"""
    res = await drone_client.disarm()
    return {"success": res.success, "message": res.message}

@router.post("/takeoff", response_model=CommandResponse)
async def takeoff(height: float = 5.0):
    """Takeoff to specified height"""
    res = await drone_client.takeoff(height)
    return {"success": res.success, "message": res.message}

@router.post("/land", response_model=CommandResponse)
async def land():
    """Land the drone"""
    res = await drone_client.land()
    return {"success": res.success, "message": res.message}

@router.post("/return_home", response_model=CommandResponse)
async def return_home():
    """Return to home"""
    res = await drone_client.return_to_home()
    return {"success": res.success, "message": res.message}

@router.post("/goto", response_model=CommandResponse)
async def go_to(pos: Position):
    """Move to local position"""
    res = await drone_client.go_to(pos.x, pos.y, pos.z, pos.yaw)
    return {"success": res.success, "message": res.message}

# ── WebSocket for Status ──────────────────────────────────────

@router.websocket("/ws/status")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        # Connect to gRPC stream
        # This is a bit tricky with async generator in a loop
        # For simplicity, we poll or use the stream directly
        if not drone_client.channel:
            await drone_client.connect()
        
        stub = drone_client.stub
        stream = stub.StreamState(uav_pb2.Empty())
        
        async for state in stream:
            data = {
                "timestamp": state.timestamp,
                "mode": state.mode,
                "flight_phase": state.flight_phase,
                "battery": state.battery,
                "armed": state.armed,
                "connected": state.connected,
                "position": {
                    "x": state.position.x,
                    "y": state.position.y,
                    "z": state.position.z,
                    "yaw": state.position.yaw
                }
            }
            await websocket.send_json(data)
            
    except Exception as e:
        logger.error(f"WebSocket Error: {e}")
    finally:
        await websocket.close()

# ── Test Mission Logic ────────────────────────────────────────

async def run_test_mission_sequence():
    """
    Executes the sequence:
    1. Arm & Takeoff
    2. Go to Pickup (0, 20)
    3. Land (Simulate loading)
    4. Takeoff & Go to Dropoff (0, 0)
    5. Land & Disarm
    """
    logger.info("STARTING AUTOMATED TEST MISSION")
    
    # 1. Arm & Takeoff
    await drone_client.arm()
    await asyncio.sleep(1)
    await drone_client.takeoff(height=5.0)
    logger.info("Taking off...")
    await asyncio.sleep(6) # Wait for takeoff simulation

    # 2. Go to Pickup (North 20m)
    # Matches Lat: 47.3981508 (approx 20m North of origin)
    logger.info("Flying to Pickup Point (0, 20)...")
    await drone_client.go_to(x=0.0, y=20.0, z=5.0, yaw=0.0)
    await asyncio.sleep(8) # Wait for travel

    # 3. Simulate Loading (Land -> Wait -> Takeoff)
    logger.info("Arrived at Pickup. Landing...")
    await drone_client.land()
    await asyncio.sleep(6) # Wait for landing
    
    logger.info("Loading Cargo...")
    await asyncio.sleep(2)
    
    logger.info("Taking off with Cargo...")
    await drone_client.arm() # Re-arm if needed
    await drone_client.takeoff(height=5.0)
    await asyncio.sleep(6)

    # 4. Go to Dropoff (Origin 0, 0)
    # Matches Lat: 47.3979711
    logger.info("Flying to Dropoff Point (0, 0)...")
    await drone_client.go_to(x=0.0, y=0.0, z=5.0, yaw=0.0)
    await asyncio.sleep(8)

    # 5. Land & Finish
    logger.info("Arrived at Dropoff. Landing...")
    await drone_client.land()
    await asyncio.sleep(6)
    
    await drone_client.disarm()
    logger.info("MISSION COMPLETE")

@router.post("/test_mission", response_model=CommandResponse)
async def trigger_test_mission():
    """Trigger the automated test mission"""
    # Run in background so we don't block the response
    asyncio.create_task(run_test_mission_sequence())
    return {"success": True, "message": "Test Mission Started"}
