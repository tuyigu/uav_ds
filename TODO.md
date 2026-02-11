# Project Roadmap & TODO

## 📅 Immediate Next Steps (Tomorrow)

### 1. Web Backend Integration (Full Stack)
- [ ] **Infrastructure**: Add `rosbridge_server` to `uav_system.launch.py`.
- [ ] **Python Backend**:
    - Update `RobotGateway` (roslibpy) to connect to `rosbridge`.
    - Implement `/uav/command` publisher (PAUSE, ABORT, CONFIRM_LOADED, CONFIRM_DELIVERED).
    - Implement status subscriber to map BT states to Frontend UI.
- [ ] **Frontend**: Add "Confirm Loading" / "Confirm Delivery" buttons that light up based on UAV status.

### 2. Behavior Tree Refinement (v4)
- [ ] **Robust Mission Logic**:
    - **Order Cancellation**: Smart Return (if carrying goods -> Merchant; else -> Home).
    - **Dynamic Geofence**: Subscribe to `/uav/geofence_updates` and trigger Re-plan.
    - **Battery Management**: Implement `LowBattery` (Return) vs `CriticalBattery` (Land Now).
- [ ] **Standard Logic**:
    - Implement `WaitUserConfirmation` plugin.
    - Implement `CheckUserCommand` plugin.

---

## 🚀 Future Milestones

### Phase 2: Advanced Reliability
- [ ] **Connection Loss**: Auto-hover if Web/ROS connection lost > 5s.
- [ ] **Dynamic Obstacle Avoidance**: Re-plan path if person walking into flight path.

### Phase 4: Multi-UAV
- [ ] **Fleet Management**: Scheduling multiple UAVs from the same Web backend.
- [ ] **Collision Avoidance**: Inter-UAV communication.
