# Drone Control - MAVLink Bridge

一個基於 ROS 2 的多執行緒 MAVLink 橋接器，支援多載具、多集群的無人系統控制與通訊。

## 功能特色

- 🚁 **多載具支援**：支援無人機(drone)、無人地面載具(ugv)、無人水面載具(usv)等
- 🔗 **MAVLink 橋接**：ROS 2 與 MAVLink 協議之間的雙向通訊
- 🌐 **GCS 轉發**：支援地面控制站的訊息轉發
- 👥 **集群控制**：支援多架載具的協調控制
- ⚡ **多執行緒架構**：高效率的並行處理
- 🎮 **速度控制**：支援 Twist 訊息的機體速度控制
- 🛡️ **安全機制**：內建速度限制和錯誤處理

## 系統架構

```
┌─────────────┐    MAVLink     ┌──────────────────┐    ROS 2      ┌─────────────┐
│   無人載具   │ ◄──────────── │  MAVLink Bridge  │ ◄─────────── │   ROS 2     │
│  (ArduPilot) │               │                  │              │ Applications │
└─────────────┘               └──────────────────┘              └─────────────┘
                                       │
                                       │ MAVLink
                                       ▼
                               ┌─────────────┐
                               │    GCS      │
                               │ (QGroundControl) │
                               └─────────────┘
```

## 支援的指令

### 基本控制指令
- **ARM** - 解鎖載具
- **DISARM** - 鎖定載具  
- **TAKEOFF** - 起飛指令
- **GOTO** - 導航到指定座標
- **LAND** - 降落指令

### 飛行模式切換
- **STABILIZE** - 穩定模式
- **GUIDED** - 導引模式
- **AUTO** - 自動模式
- **RTL** - 返航模式

### 速度控制
- **Body NED 速度控制** - 透過 Twist 訊息控制載具速度

## 安裝與編譯

### 前置需求
- ROS 2 (Humble/Iron/Rolling)
- Python 3.8+
- pymavlink

### 編譯步驟

```bash
# 1. 建立工作空間
mkdir -p ~/gcs_ws/src
cd ~/gcs_ws/src

# 2. 複製專案
git clone <your-repo-url> drone_control

# 3. 安裝相依套件
cd ~/gcs_ws
rosdep install --from-paths src --ignore-src -r -y

# 4. 編譯
colcon build --packages-select drone_control

# 5. 設定環境
source install/setup.bash
```

## 使用方法

### 1. 基本啟動

```bash
# 啟動單一無人機
ros2 launch drone_control drone_control.launch.py
```

### 2. 自定義參數啟動

```bash
# 指定載具類型和集群
ros2 launch drone_control drone_control.launch.py \
    vehicle_type:=drone \
    swarm_id:=1 \
    vehicle_name:=leader1 \
    mavlink_uri:=udp:0.0.0.0:14550 \
    gcs_uri:=udpout:192.168.1.100:14550
```

### 3. 多載具啟動範例

```bash
# 終端機 1 - 無人機領導者
ros2 launch drone_control drone_control.launch.py \
    vehicle_type:=drone \
    swarm_id:=1 \
    vehicle_name:=leader1 \
    mavlink_uri:=udp:0.0.0.0:14550

# 終端機 2 - 無人機跟隨者
ros2 launch drone_control drone_control.launch.py \
    vehicle_type:=drone \
    swarm_id:=1 \
    vehicle_name:=follower1 \
    mavlink_uri:=udp:0.0.0.0:14551

# 終端機 3 - 無人地面載具
ros2 launch drone_control drone_control.launch.py \
    vehicle_type:=ugv \
    swarm_id:=1 \
    vehicle_name:=scout1 \
    mavlink_uri:=udp:0.0.0.0:14552
```

## 參數說明

| 參數名稱 | 類型 | 預設值 | 說明 |
|----------|------|--------|------|
| `vehicle_type` | string | `drone` | 載具類型 (drone/ugv/usv) |
| `swarm_id` | int | `1` | 集群 ID |
| `vehicle_name` | string | `leader1` | 載具名稱 |
| `mavlink_uri` | string | `udp:0.0.0.0:14550` | MAVLink 連線 URI |
| `gcs_uri` | string | `udpout:192.168.2.44:14550` | GCS 連線 URI |

## ROS 2 介面

### 發布的主題
- `/{vehicle_type}_swarm_{swarm_id}/{vehicle_type}/swarm{swarm_id}/gcs/sub` (Drone) - 載具狀態資訊

### 訂閱的主題
- `/{vehicle_type}_swarm_{swarm_id}/{vehicle_type}/swarm{swarm_id}/gcs/pub` (Mission) - 任務指令
- `/{vehicle_type}_swarm_{swarm_id}/{vehicle_type}/swarm{swarm_id}/gcs/cmd_vel` (Twist) - 速度控制

### 訊息格式

#### Mission 訊息範例
```bash
# 解鎖指令
ros2 topic pub /{vehicle_type}_swarm_1/{vehicle_type}/swarm1/gcs/pub drone_interfaces/Mission "
mission: 'ARM'
data:
  latitude: 0.0
  longitude: 0.0
  altitude: 0.0"

# 起飛指令
ros2 topic pub /{vehicle_type}_swarm_1/{vehicle_type}/swarm1/gcs/pub drone_interfaces/Mission "
mission: 'TAKEOFF'
data:
  latitude: 0.0
  longitude: 0.0
  altitude: 5.0"
```

#### Twist 速度控制範例
```bash
# 前進飛行
ros2 topic pub /{vehicle_type}_swarm_1/{vehicle_type}/swarm1/gcs/cmd_vel geometry_msgs/Twist "
linear:
  x: 2.0  # 前進 2 m/s
  y: 0.0  # 右移 0 m/s
  z: 0.0  # 下降 0 m/s
angular:
  x: 0.0
  y: 0.0
  z: 0.5  # 逆時針旋轉 0.5 rad/s"
```

## 安全限制

- **線性速度限制**：±10 m/s (水平)，±5 m/s (垂直)
- **角速度限制**：±2 rad/s (偏航)
- **強制解鎖**：使用魔術數字 21196 進行強制解鎖

## 故障排除

### 常見問題

1. **無法解鎖 (ARM 失敗)**
   - 檢查 MAVLink 連線是否正常
   - 確認飛控系統狀態
   - 檢查安全檢查項目

2. **主題無資料**
   - 確認節點是否正常啟動
   - 檢查主題名稱是否正確
   - 驗證 QoS 設定

3. **GCS 連線失敗**
   - 確認 GCS URI 設定
   - 檢查網路連線
   - 驗證防火牆設定

### 除錯指令

```bash
# 檢查節點狀態
ros2 node list

# 檢查主題
ros2 topic list

# 監控載具狀態
ros2 topic echo /{vehicle_type}_swarm_1/{vehicle_type}/swarm1/gcs/sub

# 檢查節點日誌
ros2 launch drone_control drone_control.launch.py --ros-args --log-level DEBUG
```

## 開發

### 專案結構
```
drone_control/
├── drone_control/
│   ├── __init__.py
│   └── mavlink_bridge.py          # 主要橋接器程式
├── launch/
│   └── drone_control.launch.py    # 啟動檔案
├── package.xml                    # 套件描述
├── setup.py                       # Python 套件設定
└── README.md                      # 說明文件
```

### 貢獻指南
1. Fork 此專案
2. 建立功能分支 (`git checkout -b feature/新功能`)
3. 提交變更 (`git commit -am '新增某功能'`)
4. 推送到分支 (`git push origin feature/新功能`)
5. 建立 Pull Request

## 授權

MIT License

## 作者

**CY** - AESIL 團隊

## 更新日誌

### v1.0.0 (2025-08-11)
- 初始版本發布
- 支援基本 MAVLink 橋接功能
- 多載具、多集群支援
- GCS 轉發功能
- Twist 速度控制

