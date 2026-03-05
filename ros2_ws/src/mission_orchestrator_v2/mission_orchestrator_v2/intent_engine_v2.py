from dataclasses import dataclass, field
from typing import List, Optional, Deque
from collections import deque
import time

# --- Data Structures ---

@dataclass
class Task:
    type: str  # GOTO, LAND, TAKEOFF, WAIT
    params: dict = field(default_factory=dict)
    status: str = "PENDING" # PENDING, RUNNING, COMPLETED, FAILED

@dataclass
class MissionContext:
    mission_id: str = ""
    tasks: Deque[Task] = field(default_factory=deque)
    current_task: Optional[Task] = None
    state: str = "IDLE" # IDLE, RUNNING, PAUSED, COMPLETED, FAILED

# --- Intent Definitions ---
# Must match what uav_bt_agent expects
INTENT_IDLE = "IDLE"
INTENT_TAKEOFF = "TAKEOFF"
INTENT_LAND = "LAND" 
INTENT_MOVE = "MOVE_TO"
INTENT_RETURN = "RETURN_HOME"


class IntentEngineV2:
    def __init__(self):
        self.ctx = MissionContext()

    def load_mission(self, mission_id: str, tasks: List[Task]):
        self.ctx.mission_id = mission_id
        self.ctx.tasks = deque(tasks)
        self.ctx.current_task = None
        self.ctx.state = "RUNNING"
    
    def abort(self):
        self.ctx.tasks.clear()
        self.ctx.current_task = None
        self.ctx.state = "IDLE"

    def update(self, bt_status: str, uav_state: dict) -> tuple[str, dict]:
        """
        Decides the next Intent based on current task and BT feedback.
        
        Returns:
            (intent_type, intent_params)
        """
        if self.ctx.state != "RUNNING":
            return INTENT_IDLE, {}

        # 1. Check if current task is finished
        if self.ctx.current_task:
            if bt_status == "SUCCESS":
                self.ctx.current_task.status = "COMPLETED"
                print(f"Task {self.ctx.current_task.type} COMPLETED")
                self.ctx.current_task = None
            elif bt_status == "FAILURE":
                self.ctx.current_task.status = "FAILED"
                print(f"Task {self.ctx.current_task.type} FAILED")
                # Simple retry logic or abort could go here
                self.ctx.state = "FAILED"
                return INTENT_IDLE, {"reason": "Task Failed"}

        # 2. Pick next task if needed
        if not self.ctx.current_task:
            if not self.ctx.tasks:
                self.ctx.state = "COMPLETED"
                return INTENT_IDLE, {"reason": "Mission Completed"}
            
            self.ctx.current_task = self.ctx.tasks.popleft()
            self.ctx.current_task.status = "RUNNING"
            print(f"Starting Task: {self.ctx.current_task.type}")

        # 3. Translate Task -> Intent
        task = self.ctx.current_task
        
        if task.type == "TAKEOFF":
            return INTENT_TAKEOFF, {"height": task.params.get("height", 5.0)}
        
        elif task.type == "LAND":
            return INTENT_LAND, {}
            
        elif task.type == "GOTO":
            return INTENT_MOVE, {
                "x": task.params.get("x", 0.0),
                "y": task.params.get("y", 0.0),
                "z": task.params.get("z", 5.0),
                "yaw": task.params.get("yaw", 0.0)
            }
            
        elif task.type == "RETURN_HOME":
            return INTENT_RETURN, {}

        return INTENT_IDLE, {}
