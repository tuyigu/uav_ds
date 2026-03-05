from dataclasses import dataclass, field
from typing import List, Optional, Deque
from collections import deque
import json
import os
import time

# --- Data Structures ---

@dataclass
class Task:
    type: str  # GOTO, LAND, TAKEOFF, WAIT, DELIVER, RETURN_HOME
    params: dict = field(default_factory=dict)
    status: str = "PENDING" # PENDING, RUNNING, COMPLETED, FAILED

    def to_dict(self) -> dict:
        return {"type": self.type, "params": self.params, "status": self.status}

    @classmethod
    def from_dict(cls, d: dict) -> 'Task':
        return cls(type=d["type"], params=d.get("params", {}), status=d.get("status", "PENDING"))


@dataclass
class MissionContext:
    mission_id: str = ""
    tasks: Deque[Task] = field(default_factory=deque)
    current_task: Optional[Task] = None
    state: str = "IDLE" # IDLE, RUNNING, PAUSED, COMPLETED, FAILED

    def to_dict(self) -> dict:
        """Serialize mission state for persistence."""
        d = {
            "mission_id": self.mission_id,
            "state": self.state,
            "tasks": [t.to_dict() for t in self.tasks],
        }
        if self.current_task:
            d["current_task"] = self.current_task.to_dict()
        return d

    @classmethod
    def from_dict(cls, d: dict) -> 'MissionContext':
        ctx = cls()
        ctx.mission_id = d.get("mission_id", "")
        ctx.state = d.get("state", "IDLE")
        ctx.tasks = deque(Task.from_dict(t) for t in d.get("tasks", []))
        if "current_task" in d:
            ctx.current_task = Task.from_dict(d["current_task"])
        return ctx

# --- Intent Definitions ---
# Must match what uav_bt_agent expects
INTENT_IDLE = "IDLE"
INTENT_DELIVER = "DELIVER"
INTENT_TAKEOFF = "TAKEOFF"
INTENT_LAND = "LAND" 
INTENT_MOVE = "MOVE_TO"
INTENT_RETURN = "RETURN_HOME"
INTENT_HOLD = "HOLD"

# State file path
STATE_FILE = os.path.expanduser("~/.uav_ds/mission_state.json")


class IntentEngineV2:
    def __init__(self):
        self.ctx = MissionContext()
        self._try_restore()

    def _try_restore(self):
        """Attempt to restore a previously interrupted mission on startup."""
        if not os.path.exists(STATE_FILE):
            return

        try:
            with open(STATE_FILE, 'r') as f:
                data = json.load(f)
            restored = MissionContext.from_dict(data)

            # Only restore if mission was actively RUNNING or PAUSED
            if restored.state in ("RUNNING", "PAUSED"):
                self.ctx = restored
                self.ctx.state = "PAUSED"  # Always start paused for safety
                print(f"[IntentEngine] ✅ Restored mission '{restored.mission_id}' "
                      f"with {len(restored.tasks)} remaining tasks (PAUSED, send RESUME to continue)")
            else:
                self._clear_state_file()
        except Exception as e:
            print(f"[IntentEngine] ⚠️ Failed to restore state: {e}")
            self._clear_state_file()

    def _save_state(self):
        """Persist current mission state to disk."""
        try:
            os.makedirs(os.path.dirname(STATE_FILE), exist_ok=True)
            with open(STATE_FILE, 'w') as f:
                json.dump(self.ctx.to_dict(), f, indent=2)
        except Exception as e:
            print(f"[IntentEngine] ⚠️ Failed to save state: {e}")

    def _clear_state_file(self):
        """Remove the state file (mission completed or aborted)."""
        try:
            if os.path.exists(STATE_FILE):
                os.remove(STATE_FILE)
        except Exception:
            pass

    def load_mission(self, mission_id: str, tasks: List[Task]):
        """Load a new mission and start executing."""
        self.ctx.mission_id = mission_id
        self.ctx.tasks = deque(tasks)
        self.ctx.current_task = None
        self.ctx.state = "RUNNING"
        self._save_state()
        print(f"[IntentEngine] Loaded mission '{mission_id}' with {len(tasks)} tasks")

    def pause(self):
        """Pause the current mission. The BT will receive HOLD intent."""
        if self.ctx.state == "RUNNING":
            self.ctx.state = "PAUSED"
            self._save_state()
            print(f"[IntentEngine] ⏸ Mission '{self.ctx.mission_id}' PAUSED")

    def resume(self):
        """Resume a paused mission."""
        if self.ctx.state == "PAUSED":
            self.ctx.state = "RUNNING"
            self._save_state()
            print(f"[IntentEngine] ▶ Mission '{self.ctx.mission_id}' RESUMED")

    def abort(self):
        """Abort the mission and clear state."""
        self.ctx.tasks.clear()
        self.ctx.current_task = None
        self.ctx.state = "IDLE"
        self._clear_state_file()
        print(f"[IntentEngine] ❌ Mission ABORTED")

    def update(self, bt_status: str, uav_state: dict) -> tuple[str, dict]:
        """
        Decides the next Intent based on current task and BT feedback.
        
        Returns:
            (intent_type, intent_params)
        """
        if self.ctx.state == "PAUSED":
            return INTENT_HOLD, {}

        if self.ctx.state != "RUNNING":
            return INTENT_IDLE, {}

        # 1. Check if current task is finished
        if self.ctx.current_task:
            if bt_status == "SUCCESS":
                self.ctx.current_task.status = "COMPLETED"
                print(f"[IntentEngine] Task {self.ctx.current_task.type} COMPLETED")
                self.ctx.current_task = None
                self._save_state()
            elif bt_status == "FAILURE":
                self.ctx.current_task.status = "FAILED"
                print(f"[IntentEngine] Task {self.ctx.current_task.type} FAILED")
                self.ctx.state = "FAILED"
                self._save_state()
                return INTENT_IDLE, {"reason": "Task Failed"}

        # 2. Pick next task if needed
        if not self.ctx.current_task:
            if not self.ctx.tasks:
                self.ctx.state = "COMPLETED"
                self._clear_state_file()
                print(f"[IntentEngine] 🎉 Mission '{self.ctx.mission_id}' COMPLETED")
                return INTENT_IDLE, {"reason": "Mission Completed"}
            
            self.ctx.current_task = self.ctx.tasks.popleft()
            self.ctx.current_task.status = "RUNNING"
            self._save_state()
            print(f"[IntentEngine] Starting Task: {self.ctx.current_task.type}")

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

        elif task.type == "DELIVER":
            return INTENT_DELIVER, {
                "x": task.params.get("x", 0.0),
                "y": task.params.get("y", 0.0),
                "z": task.params.get("z", 5.0),
                "marker_id": task.params.get("marker_id", 0)
            }
            
        elif task.type == "RETURN_HOME":
            return INTENT_RETURN, {}

        return INTENT_IDLE, {}
