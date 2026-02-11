from datetime import datetime
from typing import List
import uuid

from fastapi import APIRouter

from ..models import DeliveryOrderCreate, DeliveryOrder, OperatorCommandCreate
from ..services import scheduler
from ..services.robot_gateway import gateway

router = APIRouter()

# TODO: 仅用于原型阶段，后续应替换为数据库存储
ORDERS: list[DeliveryOrder] = []


@router.post("", response_model=DeliveryOrder)
def create_order(payload: DeliveryOrderCreate) -> DeliveryOrder:
    """
    创建外卖订单。

    - Web 前端调用此接口创建订单
    - 后端会尝试自动分配一架无人机，并生成 DeliveryMission
    - 通过 RobotGateway 将任务发送给 Orchestrator
    """
    order = DeliveryOrder(
        order_id=str(uuid.uuid4()),
        created_at=datetime.utcnow(),
        pickup=payload.pickup,
        dropoff=payload.dropoff,
        priority=payload.priority,
        deadline_ts=payload.deadline_ts,
        note=payload.note,
        status="pending",
        assigned_uav_id=None,
        mission_id=None,
    )

    # 1. 选择无人机（极简策略）
    uav_id = scheduler.pick_uav_for_order(order)
    if uav_id is not None:
        # 2. 构造 DeliveryMission
        mission = scheduler.build_mission_from_order(order, uav_id)
        # 3. 发送任务给 Orchestrator (via web_agent_node)
        ok = gateway.send_mission(mission)
        if ok:
            order.status = "assigned"
            order.assigned_uav_id = uav_id
            order.mission_id = mission.mission_id

    ORDERS.append(order)
    return order


@router.get("", response_model=List[DeliveryOrder])
def list_orders() -> list[DeliveryOrder]:
    """列出当前所有订单（内存版）。"""
    return ORDERS


@router.post("/commands")
def send_command(payload: OperatorCommandCreate):
    """
    发送操作指令给 Orchestrator。

    支持的指令:
    - PAUSE: 暂停当前任务
    - RESUME: 恢复任务
    - CANCEL: 取消订单
    - CONFIRM_LOADED: 确认装货完成
    - CONFIRM_DELIVERED: 确认签收完成
    """
    ok = gateway.send_command(payload)
    if ok:
        return {"status": "ok", "command": payload.command, "mission_id": payload.mission_id}
    return {"status": "error", "message": "Failed to send command to ROS"}


def update_local_order_status(msg):
    """
    处理来自 Orchestrator (ROS) 的状态更新。
    msg 格式: {'mission_id': '...', 'status': '...', 'reason': '...', ...}
    """
    m_id = msg.get('mission_id')
    new_status = msg.get('status')

    for order in ORDERS:
        if order.mission_id == m_id:
            print(f"[Orders] Updating order {order.order_id} status: {order.status} -> {new_status}")
            order.status = new_status.lower()
            break

# 注册回调
gateway.set_status_callback(update_local_order_status)
