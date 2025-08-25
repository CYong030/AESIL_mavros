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
- drone_interfaces (自定義訊息套件，包含在此專案中)

### 編譯步驟

```bash
# 1. 建立工作空間
mkdir -p ~/gcs_ws/src
cd ~/gcs_ws/src

# 2. 複製專案
git clone git@github.com:CYong030/AESIL_mavros.git drone_control

# 3. 安裝相依套件
cd ~/gcs_ws
rosdep install --from-paths src --ignore-src -r -y

# 4. 編譯 (需要先編譯 drone_interfaces)
colcon build --packages-select drone_interfaces
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
| `mavlink_uri` | string | `udp:0.0.0.0:14550` | MAVLink 連線 URI (詳見連線方式) |
| `gcs_uri` | string | `udpout:192.168.2.44:14550` | GCS 連線 URI (詳見連線方式) |

## 連線方式詳解

### MAVLink URI 連線方式 (`mavlink_uri`)

MAVLink 通訊協定支援多種連線方式，用於連接飛控板或模擬器：

#### 1. 串口連線 (Serial)
```bash
# USB 串口 (最常見)
mavlink_uri:=/dev/ttyACM0            # USB 連接的飛控板
mavlink_uri:=/dev/ttyACM1            # 第二個 USB 裝置
mavlink_uri:=/dev/ttyUSB0            # USB 轉串口裝置

# 樹莓派 GPIO 串口
mavlink_uri:=/dev/ttyAMA0            # 樹莓派硬體串口
mavlink_uri:=/dev/ttyS0              # 第一個串口

```

#### 2. UDP 連線
```bash
# UDP 伺服器模式 (監聽連入連線)
mavlink_uri:=udp:0.0.0.0:14550       # 監聽所有介面的 14550 埠
mavlink_uri:=udp:127.0.0.1:14551     # 僅監聽本機的 14551 埠
mavlink_uri:=udp:192.168.1.100:14552 # 監聽特定 IP 的指定埠

# UDP 客戶端模式 (主動連線)
mavlink_uri:=udpout:127.0.0.1:14550  # 連線到本機模擬器
mavlink_uri:=udpout:192.168.1.10:14550 # 連線到遠端飛控
mavlink_uri:=udpout:10.0.0.1:14550   # 連線到特定網段裝置
```

#### 3. TCP 連線
```bash
# TCP 伺服器模式
mavlink_uri:=tcp:0.0.0.0:5760        # 監聽 TCP 5760 埠
mavlink_uri:=tcp:127.0.0.1:5761      # 監聽本機 5761 埠

# TCP 客戶端模式
mavlink_uri:=tcpout:127.0.0.1:5760   # 連線到本機 TCP 服務
mavlink_uri:=tcpout:192.168.1.10:5760 # 連線到遠端 TCP 服務
```

#### 4. 檔案重播模式
```bash
# 重播 MAVLink 日誌檔案 (用於測試)
mavlink_uri:=file:/path/to/mavlink.log
mavlink_uri:=file:/home/user/flight_logs/mission_001.tlog
```

### GCS URI 連線方式 (`gcs_uri`)

GCS (地面控制站) 連線用於將 MAVLink 訊息轉發給 QGroundControl 等地面站軟體：

#### 1. UDP 輸出連線
```bash
# 單一 GCS 連線
gcs_uri:=udpout:192.168.1.100:14550  # 傳送到特定 IP 的 GCS
gcs_uri:=udpout:127.0.0.1:14550      # 傳送到本機 GCS
gcs_uri:=udpout:10.0.0.5:14551       # 傳送到指定網段的 GCS

# 多重 GCS 連線 (使用逗號分隔)
gcs_uri:=udpout:192.168.1.100:14550,udpout:192.168.1.101:14550
gcs_uri:=udpout:127.0.0.1:14550,udpout:192.168.1.10:14551
```

#### 2. UDP 廣播連線
```bash
# 區域網路廣播 (所有 GCS 都能收到)
gcs_uri:=udpbcast:192.168.1.255:14550  # 廣播到 192.168.1.x 網段
gcs_uri:=udpbcast:10.0.0.255:14550     # 廣播到 10.0.0.x 網段
gcs_uri:=udpbcast:255.255.255.255:14550 # 全域廣播 (較少使用)

# 本機廣播
gcs_uri:=udpbcast:127.255.255.255:14550
```

#### 3. TCP 連線
```bash
# TCP 輸出連線
gcs_uri:=tcpout:192.168.1.100:5760   # TCP 連線到 GCS
gcs_uri:=tcpout:127.0.0.1:5760       # TCP 連線到本機 GCS

# TCP 伺服器模式 (等待 GCS 連入)
gcs_uri:=tcp:0.0.0.0:5760            # 開放 TCP 5760 給 GCS 連入
gcs_uri:=tcp:127.0.0.1:5761          # 僅允許本機 GCS 連入
```

#### 4. 無 GCS 連線
```bash
# 不轉發給 GCS (僅 ROS 2 通訊)
gcs_uri:=none
gcs_uri:=""                          # 空字串也表示無 GCS
```

### 常用連線組合範例

#### SITL 模擬器連線
```bash
# ArduPilot SITL + QGroundControl
mavlink_uri:=udp:127.0.0.1:14550
gcs_uri:=udpout:127.0.0.1:14551

# PX4 SITL + QGroundControl  
mavlink_uri:=udp:127.0.0.1:14540
gcs_uri:=udpout:127.0.0.1:14550
```

#### 實體硬體連線
```bash
# USB 連接飛控 + 網路 GCS
mavlink_uri:=/dev/ttyACM0:57600
gcs_uri:=udpout:192.168.1.100:14550

# 串口飛控 + 廣播 GCS
mavlink_uri:=/dev/ttyAMA0:115200
gcs_uri:=udpbcast:192.168.1.255:14550
```

#### 遠端飛控連線
```bash
# WiFi 連接遠端飛控 + 本機 GCS
mavlink_uri:=udpout:192.168.4.1:14550
gcs_uri:=udpout:127.0.0.1:14550

# 4G/LTE 連接 + 多重 GCS
mavlink_uri:=tcpout:drone.example.com:5760
gcs_uri:=udpout:192.168.1.100:14550,udpout:192.168.1.101:14550
```

#### 多載具系統連線
```bash
# 載具 1
mavlink_uri:=/dev/ttyACM0:57600
gcs_uri:=udpbcast:192.168.1.255:14550

# 載具 2  
mavlink_uri:=/dev/ttyACM1:57600
gcs_uri:=udpbcast:192.168.1.255:14550

# 載具 3 (網路連接)
mavlink_uri:=udpout:192.168.1.10:14550
gcs_uri:=udpbcast:192.168.1.255:14550
```

## 連線方式詳細說明

### MAVLink URI 連線方式 (mavlink_uri)

#### 1. 串口連線 (Serial Connection)
```bash
# USB 串口連線
mavlink_uri:=/dev/ttyUSB0:57600
mavlink_uri:=/dev/ttyACM0:115200

# Windows 系統
mavlink_uri:=COM3:57600
mavlink_uri:=COM4:115200

# macOS 系統
mavlink_uri:=/dev/cu.usbserial-XXXXXXXX:57600
mavlink_uri:=/dev/cu.usbmodem-XXXXXXXX:115200
```

#### 2. UDP 連線
```bash
# UDP 伺服器模式 (監聽指定埠)
mavlink_uri:=udp:0.0.0.0:14550      # 監聽所有介面的 14550 埠
mavlink_uri:=udp:192.168.1.100:14550 # 監聽特定 IP 的 14550 埠

# UDP 客戶端模式 (連線到遠端)
mavlink_uri:=udpout:192.168.1.10:14550  # 連線到遠端 IP
mavlink_uri:=udpout:127.0.0.1:14550     # 連線到本機
```

#### 3. TCP 連線
```bash
# TCP 伺服器模式
mavlink_uri:=tcp:0.0.0.0:5760       # TCP 伺服器監聽 5760 埠
mavlink_uri:=tcp:192.168.1.100:5760 # TCP 伺服器監聽特定 IP

# TCP 客戶端模式
mavlink_uri:=tcpout:192.168.1.10:5760   # 連線到 TCP 伺服器
mavlink_uri:=tcpout:mavlink.server.com:5760  # 連線到網域名稱
```

#### 4. 檔案重播 (Log Playback)
```bash
# 從 MAVLink 日誌檔案重播
mavlink_uri:=file:/path/to/logfile.tlog
mavlink_uri:=file:/home/user/flight_logs/mission_001.tlog
```

### GCS URI 連線方式 (gcs_uri)

#### 1. UDP 輸出 (UDP Out)
```bash
# 單點傳送到 GCS
gcs_uri:=udpout:192.168.1.100:14550  # 傳送到特定 IP
gcs_uri:=udpout:10.0.0.50:14550      # 傳送到無線網路 GCS
gcs_uri:=udpout:127.0.0.1:14550      # 傳送到本機 GCS
```

#### 2. UDP 廣播 (UDP Broadcast)
```bash
# 區域網路廣播
gcs_uri:=udpbroadcast:192.168.1.255:14550  # 特定子網路廣播
gcs_uri:=udpbroadcast:255.255.255.255:14550 # 全域廣播
gcs_uri:=udpbroadcast:10.0.0.255:14550     # 無線網路廣播
```

#### 3. TCP 輸出 (TCP Out)
```bash
# TCP 客戶端連線到 GCS
gcs_uri:=tcpout:192.168.1.100:5760   # 連線到 TCP GCS
gcs_uri:=tcpout:gcs.example.com:5760 # 連線到遠端 GCS 伺服器
```

#### 4. TCP 伺服器 (TCP Server)
```bash
# TCP 伺服器等待 GCS 連線
gcs_uri:=tcp:0.0.0.0:5760           # 監聽所有介面
gcs_uri:=tcp:192.168.1.100:5760     # 監聽特定介面
```

#### 5. 多重 GCS 連線
```bash
# 同時支援多個 GCS (在程式中可設定多個 gcs_uri)
# 範例：同時廣播和點對點傳送
gcs_uri:=udpbroadcast:192.168.1.255:14550,udpout:10.0.0.100:14550
```

### 常用連線組合範例

#### 範例 1：SITL 模擬器連線
```bash
ros2 launch drone_control drone_control.launch.py \
    mavlink_uri:=udp:0.0.0.0:14550 \
    gcs_uri:=udpout:127.0.0.1:14551
```

#### 範例 2：實體無人機 USB 連線
```bash
ros2 launch drone_control drone_control.launch.py \
    mavlink_uri:=/dev/ttyACM0:115200 \
    gcs_uri:=udpbroadcast:192.168.1.255:14550
```

#### 範例 3：Wi-Fi 遙測連線
```bash
ros2 launch drone_control drone_control.launch.py \
    mavlink_uri:=udp:0.0.0.0:14550 \
    gcs_uri:=udpbroadcast:192.168.1.255:14550
```

#### 範例 4：多載具網路連線
```bash
# 載具 1
ros2 launch drone_control drone_control.launch.py \
    vehicle_name:=drone1 \
    mavlink_uri:=udp:0.0.0.0:14550 \
    gcs_uri:=udpout:192.168.1.100:14550

# 載具 2  
ros2 launch drone_control drone_control.launch.py \
    vehicle_name:=drone2 \
    mavlink_uri:=udp:0.0.0.0:14551 \
    gcs_uri:=udpout:192.168.1.100:14550
```

### 連線注意事項

1. **埠號衝突**：確保不同載具使用不同的埠號
2. **防火牆設定**：開放相對應的埠號
3. **網路延遲**：Wi-Fi 連線可能有較高延遲
4. **頻寬限制**：多載具時注意網路頻寬
5. **MAVLink 版本**：確保版本相容性 (MAVLink 1.0/2.0)

### 連線狀態檢查

```bash
# 檢查 MAVLink 連線狀態
ros2 topic echo /{vehicle_type}/swarm{swarm_id}/{vehicle_name}/status

# 檢查網路連線
ping 192.168.1.100

# 檢查埠號占用
netstat -tulpn | grep 14550

# 檢查串口設備
ls -la /dev/tty*
```

