# Topic Whitelist 使用說明

## 概述

這個 MAVROS 橋接器支援 **Topic 白名單 (Whitelist)** 機制，讓您可以根據需求動態選擇要啟用或關閉的 ROS2 topics。這對於城市導航、資源優化和模組化部署非常有用。

## 支援的 Topics

### Publishers (發布者)
- `status` - 基本載具狀態 (Drone 訊息)
- `gps` - GPS 定位資訊 (NavSatFix)
- `attitude` - 載具姿態 (QuaternionStamped) 
- `battery` - 電池狀態 (BatteryState)
- `obstacle` - 障礙物警告 (String)

### Subscribers (訂閱者)
- `mission` - 任務指令 (Mission)
- `velocity` - 速度控制 (Twist)
- `waypoint` - 航點導航 (PoseStamped)
- `emergency` - 緊急停止 (Bool)

## 使用方式

### 1. Launch 檔案參數控制

```bash
# 城市導航配置 (啟用所有導航相關 topics)
ros2 launch drone_control urban_navigation.launch.py \
  enable_gps_pub:=true \
  enable_obstacle_pub:=true \
  enable_waypoint_sub:=true \
  enable_emergency_sub:=true

# 基本飛行配置 (最小資源使用)
ros2 launch drone_control drone_control.launch.py \
  enable_gps_pub:=false \
  enable_attitude_pub:=false \
  enable_battery_pub:=false \
  enable_obstacle_pub:=false \
  enable_waypoint_sub:=false \
  enable_emergency_sub:=false
```

### 2. 配置檔案控制

使用預設的配置檔案：

```bash
# 城市導航配置
ros2 launch drone_control drone_control.launch.py --ros-args --params-file config/urban_navigation.yaml

# 基本飛行配置
ros2 launch drone_control drone_control.launch.py --ros-args --params-file config/basic_flight.yaml

# 完整功能配置
ros2 launch drone_control drone_control.launch.py --ros-args --params-file config/full_features.yaml
```

## 配置範例

### 城市導航配置
適用於城市環境的無人機導航，需要完整的感測器資訊和安全功能：

```yaml
enable_status_pub: true      # 基本狀態監控
enable_mission_sub: true     # 任務控制
enable_velocity_sub: true    # 速度控制
enable_gps_pub: true         # ⭐ GPS 定位
enable_attitude_pub: true    # ⭐ 姿態控制
enable_battery_pub: true     # ⭐ 電池監控
enable_obstacle_pub: true    # ⭐ 障礙物警告
enable_waypoint_sub: true    # ⭐ 航點導航
enable_emergency_sub: true   # ⭐ 緊急停止
```

### 基本飛行配置
適用於簡單的飛行任務，最小化資源使用：

```yaml
enable_status_pub: true      # 基本狀態
enable_mission_sub: true     # 任務控制
enable_velocity_sub: true    # 速度控制
# 其他功能關閉
```

## Topic 命名空間

Topics 會根據載具配置自動生成命名空間：

```
/{vehicle_type}/swarm{swarm_id}/{vehicle_name}/
```

範例：
- `/drone/swarm1/urban_navigator/status`
- `/drone/swarm1/urban_navigator/gps/fix`
- `/drone/swarm1/urban_navigator/cmd_vel`
- `/drone/swarm1/urban_navigator/waypoint`
- `/drone/swarm1/urban_navigator/emergency_stop`

## 使用場景

### 🏙️ 城市導航
```bash
ros2 launch drone_control urban_navigation.launch.py
```
- 啟用 GPS、障礙物偵測、航點導航
- 適用於複雜城市環境

### 🛩️ 基本飛行
```bash
ros2 launch drone_control drone_control.launch.py \
  enable_gps_pub:=false \
  enable_obstacle_pub:=false
```
- 只保留基本控制功能
- 節省系統資源

### 🔧 開發測試
```bash
ros2 launch drone_control drone_control.launch.py --ros-args --params-file config/full_features.yaml
```
- 啟用所有功能
- 適用於系統開發和測試

## 動態檢查

系統會在啟動時顯示啟用的 topics：

```
[INFO] ✓ Enabled publisher: /drone/swarm1/leader1/status
[INFO] ✓ Enabled publisher: /drone/swarm1/leader1/gps/fix  
[INFO] ✓ Enabled subscriber: /drone/swarm1/leader1/cmd_vel
[INFO] ✓ Enabled subscriber: /drone/swarm1/leader1/waypoint
```

這讓您可以輕鬆確認系統配置是否正確。
