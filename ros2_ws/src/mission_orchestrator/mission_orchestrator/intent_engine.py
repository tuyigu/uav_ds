"""
Intent Engine: Pure decision logic.

Takes the 4 state models as input and outputs an intent string.
No ROS dependencies — easy to unit test.
"""
from .state_models import MissionState, SelfHealth, SLATracker, UserPerception


# Valid intents
INTENT_DELIVER = "DELIVER"
INTENT_HOLD = "HOLD"
INTENT_ABORT = "ABORT"
INTENT_REROUTE = "REROUTE"
INTENT_RETURN_HOME = "RETURN_HOME"
INTENT_WAIT_CONFIRM = "WAIT_CONFIRM"


def decide_intent(
    mission: MissionState,
    health: SelfHealth,
    sla: SLATracker,
    user: UserPerception,
) -> tuple[str, str]:
    """
    Core decision function.

    Returns:
        (intent, reason) tuple.
    """

    # ── Priority 1: Critical Self-Protection ──────────────────────
    if health.is_battery_critical():
        return INTENT_ABORT, "CRITICAL_BATTERY"

    if health.is_stale(timeout=5.0):
        return INTENT_HOLD, "UAV_STATE_TIMEOUT"

    # ── Priority 2: Operator Override ─────────────────────────────
    cmd = mission.pending_command
    if cmd:
        if cmd == "CANCEL":
            if mission.has_cargo:
                # Have cargo → return to pickup (merchant) for safe return
                return INTENT_REROUTE, "ORDER_CANCELLED_WITH_CARGO"
            else:
                return INTENT_RETURN_HOME, "ORDER_CANCELLED"

        if cmd == "PAUSE":
            return INTENT_HOLD, "OPERATOR_PAUSE"

        if cmd == "RESUME":
            # Clear the command and continue normal flow
            pass  # Fall through to normal logic

        if cmd == "CONFIRM_LOADED":
            # Cargo loaded → advance to delivery phase
            mission.has_cargo = True
            mission.phase = "DELIVERY"
            mission.status = "EN_ROUTE_DROPOFF"
            mission.target_x = mission.dropoff_x
            mission.target_y = mission.dropoff_y
            mission.target_z = mission.dropoff_z
            return INTENT_DELIVER, "CARGO_LOADED"

        if cmd == "CONFIRM_DELIVERED":
            # Cargo delivered → return home
            mission.has_cargo = False
            mission.phase = "RETURN"
            mission.status = "RETURNING"
            mission.target_x = mission.home_x
            mission.target_y = mission.home_y
            mission.target_z = mission.home_z
            return INTENT_RETURN_HOME, "CARGO_DELIVERED"

    # ── Priority 3: Energy-Aware Self-Protection ──────────────────
    if health.is_battery_low():
        if not health.can_afford_return(mission.home_x, mission.home_y):
            return INTENT_ABORT, "CANNOT_AFFORD_RETURN"
        return INTENT_RETURN_HOME, "LOW_BATTERY"

    # ── Priority 4: Normal Mission Flow ───────────────────────────
    if not mission.is_active():
        return INTENT_HOLD, "NO_ACTIVE_MISSION"

    # Determine intent based on current phase
    if mission.phase == "PICKUP":
        if mission.status == "WAITING_LOAD":
            return INTENT_WAIT_CONFIRM, "WAITING_FOR_LOADING"
        return INTENT_DELIVER, "EN_ROUTE_PICKUP"

    if mission.phase == "DELIVERY":
        if mission.status == "WAITING_UNLOAD":
            return INTENT_WAIT_CONFIRM, "WAITING_FOR_UNLOADING"
        return INTENT_DELIVER, "EN_ROUTE_DROPOFF"

    if mission.phase == "RETURN":
        return INTENT_RETURN_HOME, "RETURNING_HOME"

    # Fallback: hold position
    return INTENT_HOLD, "UNKNOWN_STATE"


def reinterpret_bt_failure(
    mission: MissionState,
    health: SelfHealth,
) -> tuple[str, str]:
    """
    Called when BT reports FAILURE.
    Instead of ending the mission, decide what to do next.

    Returns:
        (intent, reason) tuple.
    """
    # Can we retry?
    if mission.retry_count < mission.max_retries:
        if health.is_battery_low():
            return INTENT_RETURN_HOME, "BT_FAILED_LOW_BATTERY"

        mission.retry_count += 1
        return INTENT_REROUTE, f"BT_FAILED_RETRY_{mission.retry_count}"

    # Max retries exhausted
    if health.can_afford_return(mission.home_x, mission.home_y):
        return INTENT_RETURN_HOME, "BT_FAILED_MAX_RETRIES"

    return INTENT_ABORT, "BT_FAILED_CANNOT_RETURN"


def compute_user_perception(
    mission: MissionState,
    health: SelfHealth,
    sla: SLATracker,
    user: UserPerception,
) -> UserPerception:
    """Update user-facing perception based on current state."""
    user.update_from_phase(mission.status)
    user.severity = "normal"

    # Progress estimation (flight-phase-aware)
    phase_progress = {
        "IDLE": 0.0,
        "ACCEPTED": 0.05,
        "TAKING_OFF": 0.08,
        "CRUISING": 0.20,
        "DESCENDING": 0.30,
        "APPROACHING": 0.40,
        "SEARCHING": 0.50,
        "LANDING_PRECISION": 0.60,
        "LANDING_GPS": 0.60,
        "WAITING_LOAD": 0.30,
        "EN_ROUTE_DROPOFF": 0.50,
        "WAITING_UNLOAD": 0.75,
        "RETURNING": 0.90,
        "COMPLETED": 1.0,
    }
    user.progress = phase_progress.get(mission.status, user.progress)

    # SLA check
    if sla.is_delayed():
        user.set_delayed()

    # Health warnings
    if health.is_battery_low():
        user.severity = "warning"
        user.user_facing_msg += "（电量较低）"

    if health.is_battery_critical():
        user.set_critical("电量严重不足，正在紧急处理")

    return user
