import uuid
from typing import Optional

from ..models import DeliveryOrder, DeliveryMission


# TODO: 后续可以改为从数据库读取可用无人机列表
AVAILABLE_UAVS = ["uav-001"]


def pick_uav_for_order(order: DeliveryOrder) -> Optional[str]:
    """
    极简调度策略：当前仅选择固定的一架无人机。

    后续可以在这里接入：
    - 基于位置的最近邻选择
    - 考虑电量 / 载重的约束
    - 优先级与任务队列等因素
    """
    return AVAILABLE_UAVS[0] if AVAILABLE_UAVS else None


def build_mission_from_order(order: DeliveryOrder, uav_id: str) -> DeliveryMission:
    """从订单构造 DeliveryMission，高层任务描述。"""
    return DeliveryMission(
        mission_id=str(uuid.uuid4()),
        uav_id=uav_id,
        pickup=order.pickup,
        dropoff=order.dropoff,
        priority=order.priority,
        deadline_ts=order.deadline_ts,
        note=order.note,
    )

