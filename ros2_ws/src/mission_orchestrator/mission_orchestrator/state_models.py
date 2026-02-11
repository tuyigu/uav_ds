"""
4 State Models for the Mission Orchestrator.

These are pure data classes — no ROS dependencies.
They represent the Orchestrator's internal understanding of the world.
"""
from dataclasses import dataclass, field
import time
import math


@dataclass
class MissionState:
    """Tracks what the mission is doing right now."""

    # Identity
    mission_id: str = ""
    uav_id: str = ""

    # Phase tracking
    phase: str = "IDLE"        # IDLE / PICKUP / DELIVERY / RETURN
    status: str = "IDLE"       # IDLE / ACCEPTED / EN_ROUTE / APPROACHING / WAITING / COMPLETED / ABORTED / FAILED_SAFE
    flight_phase: str = "GROUND"  # GROUND / TAKEOFF / CRUISE / TRANSITION / APPROACH / SEARCH / PRECISION_LAND

    # Target coordinates (current leg)
    target_x: float = 0.0
    target_y: float = 0.0
    target_z: float = 0.0
    target_marker_id: int = 0

    # Pickup / dropoff stored from original mission
    pickup_x: float = 0.0
    pickup_y: float = 0.0
    pickup_z: float = 0.0
    dropoff_x: float = 0.0
    dropoff_y: float = 0.0
    dropoff_z: float = 0.0
    home_x: float = 0.0
    home_y: float = 0.0
    home_z: float = 0.0

    # Flight altitude parameters
    cruise_alt: float = 30.0      # High-altitude cruise (GoTo, no A*)
    approach_alt: float = 15.0    # Transition → A* planning starts
    search_alt: float = 5.0       # YOLO/ArUco search altitude

    # State flags
    has_cargo: bool = False
    retry_count: int = 0
    max_retries: int = 3

    # Pending operator command (consumed after processing)
    pending_command: str = ""
    pending_command_reason: str = ""

    # Priority
    priority: str = "normal"

    def is_active(self) -> bool:
        return self.phase != "IDLE"

    def reset(self):
        """Reset to idle state for next mission."""
        self.mission_id = ""
        self.uav_id = ""
        self.phase = "IDLE"
        self.status = "IDLE"
        self.flight_phase = "GROUND"
        self.has_cargo = False
        self.retry_count = 0
        self.pending_command = ""
        self.pending_command_reason = ""


@dataclass
class SelfHealth:
    """
    Tracks drone self-health. NOT pre-flight checks.
    This answers: "Am I in a safe state to continue what I'm doing?"
    """
    battery_level: float = 1.0       # 0.0 - 1.0
    slam_ok: bool = True
    bt_alive: bool = True
    connected: bool = True
    armed: bool = False

    # Current position (from UavState)
    pos_x: float = 0.0
    pos_y: float = 0.0
    pos_z: float = 0.0

    # Flight phase from flight_core
    flight_phase: str = "IDLE"

    # Heartbeat timestamps
    last_uav_state_time: float = 0.0
    last_bt_heartbeat_time: float = 0.0

    # Battery thresholds
    CRITICAL_BATTERY: float = 0.10
    LOW_BATTERY: float = 0.25

    def is_battery_critical(self) -> bool:
        return self.battery_level < self.CRITICAL_BATTERY

    def is_battery_low(self) -> bool:
        return self.battery_level < self.LOW_BATTERY

    def estimated_return_cost(self, home_x: float, home_y: float) -> float:
        """
        Rough estimate of battery fraction needed to return home.
        Uses simple distance-based heuristic.
        Assumes ~5% battery per 100m at cruise speed.
        """
        dist = math.sqrt(
            (self.pos_x - home_x) ** 2 +
            (self.pos_y - home_y) ** 2 +
            self.pos_z ** 2  # vertical component to descend
        )
        return (dist / 100.0) * 0.05  # 5% per 100m

    def can_afford_return(self, home_x: float, home_y: float, safety_margin: float = 0.15) -> bool:
        """Can we make it home with safety margin?"""
        cost = self.estimated_return_cost(home_x, home_y)
        return self.battery_level >= (cost + safety_margin)

    def is_stale(self, timeout: float = 5.0) -> bool:
        """Has UavState stopped updating?"""
        if self.last_uav_state_time == 0.0:
            return False  # Never received, not stale yet
        return (time.time() - self.last_uav_state_time) > timeout


@dataclass
class SLATracker:
    """Tracks time constraints and SLA compliance."""
    created_at: float = 0.0          # Unix timestamp
    deadline_ts: int = 0             # 0 = no deadline
    mission_accepted_at: float = 0.0

    def elapsed_seconds(self) -> float:
        if self.created_at == 0.0:
            return 0.0
        return time.time() - self.created_at

    def is_delayed(self, expected_duration: float = 300.0) -> bool:
        """Is the mission taking longer than expected? (default: 5 min)"""
        return self.elapsed_seconds() > expected_duration

    def has_deadline(self) -> bool:
        return self.deadline_ts > 0

    def seconds_until_deadline(self) -> float:
        if not self.has_deadline():
            return float('inf')
        return self.deadline_ts - time.time()

    def is_deadline_breached(self) -> bool:
        return self.has_deadline() and self.seconds_until_deadline() < 0


@dataclass
class UserPerception:
    """
    What the user should see right now.
    Computed by the Orchestrator, consumed by the Web UI.
    """
    user_facing_msg: str = "系统就绪"
    severity: str = "normal"       # normal / warning / critical
    eta_seconds: int = 0
    progress: float = 0.0         # 0.0 - 1.0

    # Phase → Message mapping
    _PHASE_MESSAGES: dict = field(default_factory=lambda: {
        "IDLE": "系统就绪",
        "ACCEPTED": "任务已接收，准备起飞",
        "TAKING_OFF": "正在起飞",
        "CRUISING": "正在高空巡航中",
        "DESCENDING": "正在下降至目标区域",
        "APPROACHING": "正在接近目标，A* 避障导航中",
        "SEARCHING": "正在搜索降落标记",
        "LANDING_PRECISION": "已识别标记，精准降落中",
        "LANDING_GPS": "未找到标记，GPS 辅助降落",
        "WAITING_LOAD": "已到达商家，等待装货确认",
        "WAITING_UNLOAD": "已到达目的地，等待签收",
        "RETURNING": "配送完成，正在返航",
        "COMPLETED": "任务完成",
        "ABORTED": "任务已中止",
        "FAILED_SAFE": "任务异常，已安全着陆",
        "HOLDING": "悬停等待中",
        "ABORTING": "正在紧急处理",
    })

    def update_from_phase(self, status: str):
        self.user_facing_msg = self._PHASE_MESSAGES.get(status, f"状态: {status}")

    def set_delayed(self):
        self.severity = "warning"
        self.user_facing_msg += "（稍有延迟）"

    def set_critical(self, reason: str):
        self.severity = "critical"
        self.user_facing_msg = f"异常: {reason}"
