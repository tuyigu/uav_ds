# 仿真任务设计方法：详解

我是如何确定任务参数的？这是一个**从仿真世界到控制指令的映射过程**。

### 第一步：读取世界原点 (World Origin)
在文件 `uav_simulation/worlds/urban_delivery.sdf` 中，我首先找到了 `<spherical_coordinates>` 字段。
这是整个仿真世界的 **(0, 0, 0) 本地坐标** 对应的真实 GPS 经纬度：
```xml
<spherical_coordinates>
  <latitude_deg>47.397971057728974</latitude_deg> <!-- 世界中心纬度 -->
  <longitude_deg> 8.546163739800146</longitude_deg> <!-- 世界中心经度 -->
  <elevation>0</elevation>
</spherical_coordinates>
```
这意味着：如果无人机在仿真中位于 (0, 0, 0) 处，FlightCore 和 Orchestrator 会认为它就在这个经纬度上。

### 第二步：确定目标点 (Landing Pads)
接着，我在同一个 SDF 文件中查找 **降落场 (Landing Pads)** 的位置。它们是任务的关键点。

1.  **取货点 (landing_pad_0)**
    ```xml
    <include>
      <name>landing_pad_0</name>
      ...
      <pose>0 0 0.5 0 0 0</pose> <!-- 本地坐标: (0, 0) -->
    </include>
    ```
    - **位置**：(0, 0)
    - **GPS**：等于世界原点 -> **Lat: 47.3979711, Lon: 8.5461637**

2.  **送货点 (landing_pad_1)**
    ```xml
    <include>
      <name>landing_pad_1</name>
      ...
      <pose>0 20 0.5 0 0 0</pose> <!-- 本地坐标: (0, 20) -->
    </include>
    ```
    - **位置**：(0, 20)。这里的 `y=20` 意味着向**正北方**移动 20 米。
    - **GPS 计算**：
      - 纬度 (Latitude) 管南北。在 47度纬度附近，1度纬度约等于 111,111 米。
      - 20米对应的纬度增量 = 20 / 111111 ≈ **0.0001798**
      - **新纬度** = 47.3979711 + 0.0001798 = **47.3981509**
    - **GPS 结果** -> **Lat: 47.3981509, Lon: 8.5461637** (经度不变)

### 第三步：验证 ArUco 识别码
在 `uav_simulation/models/arucotag/materials/textures` 中，我确认了纹理文件名是 `aruco_marker_0.png`。
这意味着这两个降落场上的二维码 ID 都是 **0**。这就在任务中不需要额外指定特殊的 `target_marker_id`（或者默认为 0 即可）。

### 第四步：构造任务 (The Mission)
现在我们有了两个坐标：
- **Point A (Pad 0 - 起点)**: `47.3979711, 8.5461637`
- **Point B (Pad 1 - 北向20米)**: `47.3981509, 8.5461637`

为了让无人机**飞起来**（而不是原地起降），任务必须利用这两个不同的点。
因此，我设计的任务是：
- **Pickup（取货）**: 去 Point B (Pad 1)。让无人机起飞后，向北飞 20 米。
- **Dropoff（送货）**: 回 Point A (Pad 0)。取货后，飞回原点。

这就是为什么最终的 JSON 是这样的：
```json
{
  "pickup": {
    "lat": 47.3981508,  <-- 北向20米 (Pad 1)
    "lon": 8.5461637,
    "alt": 0.5
  },
  "dropoff": {
    "lat": 47.3979711,  <-- 原点 (Pad 0)
    "lon": 8.5461637,
    "alt": 0.5
  }
}
```
通过这个过程，我确保了发布的任务坐标与仿真世界中的物理模型是严格对应的。
