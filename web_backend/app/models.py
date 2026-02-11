from datetime import datetime
from enum import Enum
from typing import Optional

from pydantic import BaseModel, Field


class Priority(str, Enum):
    normal = "normal"
    high = "high"


class GeoPoint(BaseModel):
    lat: float = Field(..., description="纬度")
    lon: float = Field(..., description="经度")
    alt: float = Field(..., description="高度（米）")


class DeliveryOrderCreate(BaseModel):
    pickup: GeoPoint
    dropoff: GeoPoint
    priority: Priority = Priority.normal
    deadline_ts: Optional[int] = Field(
        None, description="可选，Unix 时间戳"
    )
    note: Optional[str] = None


class DeliveryOrder(BaseModel):
    order_id: str
    created_at: datetime
    pickup: GeoPoint
    dropoff: GeoPoint
    priority: Priority
    deadline_ts: Optional[int]
    note: Optional[str]
    status: str  # pending / assigned / running / completed / failed
    assigned_uav_id: Optional[str] = None
    mission_id: Optional[str] = None


class DeliveryMission(BaseModel):
    """Web 输出给 ROS2 / 无人机侧的高层任务描述。"""

    mission_id: str
    uav_id: str
    pickup: GeoPoint
    dropoff: GeoPoint
    priority: Priority
    deadline_ts: Optional[int] = None
    note: Optional[str] = None


class OperatorCommandCreate(BaseModel):
    """Web 前端发送的操作指令。"""
    command: str = Field(..., description="PAUSE / RESUME / CANCEL / CONFIRM_LOADED / CONFIRM_DELIVERED")
    mission_id: str = Field(..., description="目标任务 ID")
    reason: Optional[str] = Field("", description="可选原因说明")


