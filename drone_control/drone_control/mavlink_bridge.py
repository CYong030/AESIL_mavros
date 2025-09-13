#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mavlink_bridge_node.py  (multithread, Drone/Mission msgs)

Author : CY
License: MIT
"""
import time
from threading import Thread, Event
from queue import Queue, Empty
from math import isfinite

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor


from drone_interfaces.msg import Drone, Mission
from sensor_msgs.msg import NavSatFix, BatteryState
from std_msgs.msg import Float64, Bool, String
from geometry_msgs.msg import Twist, PoseStamped

from pymavlink import mavutil


class MavBridge(Node):
    def __init__(self,node:str=None):

        super().__init__(f"mavlink_bridge")
        # ────── 參數 ──────
        self.declare_parameter('mavlink_uri', 'udp:0.0.0.0:14550')
        self.declare_parameter('gcs_uri', '')

        self.declare_parameter('vehicle_type', 'drone')  
        self.declare_parameter('swarm_id', 1)
        self.declare_parameter('vehicle_name', 'leader1')
        
        # ────── Topic 白名單參數 ──────
        self.declare_parameter('enable_status_pub', True)
        self.declare_parameter('enable_mission_sub', True)
        self.declare_parameter('enable_velocity_sub', True)
        self.declare_parameter('enable_gps_pub', False)
        self.declare_parameter('enable_attitude_pub', False)
        self.declare_parameter('enable_battery_pub', False)
        self.declare_parameter('enable_obstacle_pub', False)
        self.declare_parameter('enable_waypoint_sub', False)
        self.declare_parameter('enable_emergency_sub', False)
        
        # Topic 白名單字典
        self.topic_whitelist = {
            'status': self.get_parameter('enable_status_pub').value,
            'mission': self.get_parameter('enable_mission_sub').value,
            'velocity': self.get_parameter('enable_velocity_sub').value,
            'gps': self.get_parameter('enable_gps_pub').value,
            'attitude': self.get_parameter('enable_attitude_pub').value,
            'battery': self.get_parameter('enable_battery_pub').value,
            'obstacle': self.get_parameter('enable_obstacle_pub').value,
            'waypoint': self.get_parameter('enable_waypoint_sub').value,
            'emergency': self.get_parameter('enable_emergency_sub').value,
        }

        mav_uri  = self.get_parameter('mavlink_uri').value
        gcs_uri  = self.get_parameter('gcs_uri').value

        self.swarmid = int(self.get_parameter('swarm_id').value)
        self.vehicle_type = self.get_parameter('vehicle_type').value
        self.vehicle_name = self.get_parameter('vehicle_name').value

        # ────── MAVLink 連線 ──────
        self.master = mavutil.mavlink_connection(mav_uri)
        self.master.wait_heartbeat()
        self.get_logger().warn(f'⬅  MAVLink in  @{mav_uri} at sysid={self.master.target_system}, compid={self.master.target_component}')

        self.sysid  = self.master.target_system
        self.compid = self.master.target_component

        self.tx_link = self.master
        self.gcs_link = None
        self.gcs_connected = False
        if gcs_uri:
            try:
                self.gcs_link = mavutil.mavlink_connection(
                    gcs_uri)
                self.gcs_connected = True
                self.get_logger().warn(f'➡  GCS connected @{gcs_uri}')
            except Exception as e:
                self.get_logger().error(f'Failed to connect to GCS: {e}')
                self.gcs_link = None

        # ────── QoS ──────
        qos_best_effort = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        qos_reliable = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1)

        # ────── ROS Pub / Sub (根據白名單創建) ──────
        ns = f'/{self.vehicle_type}/swarm{self.swarmid}/{self.vehicle_name}'
        
        # Publishers (根據白名單)
        self.topic_publishers = {}
        if self.topic_whitelist['status']:
            self.topic_publishers['status'] = self.create_publisher(Drone, f'{ns}/status', qos_best_effort)
            self.get_logger().info(f"✓ Enabled publisher: {ns}/status")
            
        if self.topic_whitelist['gps']:
            self.topic_publishers['gps'] = self.create_publisher(NavSatFix, f'{ns}/gps/fix', qos_best_effort)
            self.get_logger().info(f"✓ Enabled publisher: {ns}/gps/fix")
            
        if self.topic_whitelist['attitude']:
            self.topic_publishers['attitude'] = self.create_publisher(PoseStamped, f'{ns}/attitude', qos_best_effort)
            self.get_logger().info(f"✓ Enabled publisher: {ns}/attitude")
            
        if self.topic_whitelist['battery']:
            self.topic_publishers['battery'] = self.create_publisher(BatteryState, f'{ns}/battery', qos_best_effort)
            self.get_logger().info(f"✓ Enabled publisher: {ns}/battery")
            
        if self.topic_whitelist['obstacle']:
            self.topic_publishers['obstacle'] = self.create_publisher(String, f'{ns}/obstacle_warning', qos_best_effort)
            self.get_logger().info(f"✓ Enabled publisher: {ns}/obstacle_warning")

        # Subscribers (根據白名單)
        self.topic_subscribers = {}
        if self.topic_whitelist['mission']:
            self.topic_subscribers['mission'] = self.create_subscription(Mission, f'{ns}/mission', self._mission_cb, qos_reliable)
            self.get_logger().info(f"✓ Enabled subscriber: {ns}/mission")
            
        if self.topic_whitelist['velocity']:
            self.topic_subscribers['velocity'] = self.create_subscription(Twist, f'{ns}/cmd_vel', self._velocity_cb, qos_reliable)
            self.get_logger().info(f"✓ Enabled subscriber: {ns}/cmd_vel")
            
        if self.topic_whitelist['waypoint']:
            self.topic_subscribers['waypoint'] = self.create_subscription(PoseStamped, f'{ns}/waypoint', self._waypoint_cb, qos_reliable)
            self.get_logger().info(f"✓ Enabled subscriber: {ns}/waypoint")
            
        if self.topic_whitelist['emergency']:
            self.topic_subscribers['emergency'] = self.create_subscription(Bool, f'{ns}/emergency_stop', self._emergency_cb, qos_reliable)
            self.get_logger().info(f"✓ Enabled subscriber: {ns}/emergency_stop")

        # 保持向後相容性
        self.pub_status = self.topic_publishers.get('status', None)
        self.sub_mission = self.topic_subscribers.get('mission', None)
        self.sub_velocity = self.topic_subscribers.get('velocity', None)

        # ────── 執行緒 ──────
        self.tx_queue: Queue[Mission] = Queue(maxsize=50)
        self.velocity_queue: Queue[Twist] = Queue(maxsize=10)  # 速度指令佇列
        self.gcs_tx_queue: Queue = Queue(maxsize=100)  # GCS轉發佇列
        self._stop_evt = Event()
        Thread(target=self._mav_recv_loop, daemon=True).start()
        Thread(target=self._mav_tx_loop,   daemon=True).start()
        if self.gcs_link:
            Thread(target=self._gcs_recv_loop, daemon=True).start()
            Thread(target=self._gcs_tx_loop,   daemon=True).start()

        # ────── Housekeeping ──────
        self.create_timer(5.0, self._heartbeat_check)

    # ==================================================
    #  ROS → 佇列
    # ==================================================
    def _mission_cb(self, msg: Mission):
        try:
            self.tx_queue.put_nowait(msg)
        except:
            self.get_logger().warning('TX queue overflow')
            
    def _velocity_cb(self, msg: Twist):
        """處理 Twist 速度指令"""
        try:
            self.velocity_queue.put_nowait(msg)
        except:
            self.get_logger().warning('Velocity queue overflow')
            
    def _waypoint_cb(self, msg: PoseStamped):
        """處理航點導航指令 (城市導航用)"""
        self.get_logger().info(f'Received waypoint: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}')
        # 可以將航點轉換為 MAVLink 的 MISSION_ITEM 指令
        # 這裡可以實現具體的航點導航邏輯
        
    def _emergency_cb(self, msg: Bool):
        """處理緊急停止指令"""
        if msg.data:
            self.get_logger().warn('🚨 EMERGENCY STOP ACTIVATED!')
            # 發送 MAVLink 緊急停止指令
            # 可以發送 COMMAND_LONG 與 MAV_CMD_DO_PAUSE_CONTINUE
        else:
            self.get_logger().info('Emergency stop deactivated')
            
    def publish_topic_status(self):
        """發布當前啟用的 topic 狀態 (除錯用)"""
        enabled_topics = [k for k, v in self.topic_whitelist.items() if v]
        self.get_logger().info(f'Active topics: {", ".join(enabled_topics)}')

    # ==================================================
    #  Thread-3 : TX
    # ==================================================
    def _mav_tx_loop(self):
        while not self._stop_evt.is_set():
            try:
                # 處理任務指令
                try:
                    m: Mission = self.tx_queue.get(timeout=0.05)
                    self._dispatch_mission(m)
                except Empty:
                    pass
                    
                # 處理速度指令
                try:
                    twist: Twist = self.velocity_queue.get(timeout=0.05)
                    self._dispatch_velocity(twist)
                except Empty:
                    pass
                    
            except Exception as e:
                self.get_logger().error(f'TX error: {e}')

    def _dispatch_mission(self, mis: Mission):
        """Mission → MAVLink 指令"""
        cmd = mis.mission.upper()
        if cmd == 'ARM':
            # ARM 指令：param1=1 表示解鎖，param2=21196 是強制解鎖的魔術數字
            self.tx_link.mav.command_long_send(
                self.sysid, self.compid,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,       # confirmation
                1,       # param1: 1=arm, 0=disarm
                0,   # param2: 強制解鎖魔術數字 (忽略安全檢查)
                0,0,0,0,0)  # param3-7: 未使用
            self.get_logger().info('Sending ARM command')
            
        elif cmd == 'DISARM':
            # DISARM 指令：param1=0 表示上鎖
            self.tx_link.mav.command_long_send(
                self.sysid, self.compid,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,       # confirmation
                0,       # param1: 0=disarm
                0,       # param2: 0=正常上鎖
                0,0,0,0,0)  # param3-7: 未使用
            self.get_logger().info('Sending DISARM command')
            
        elif cmd == 'TAKEOFF':
            # TAKEOFF 指令：使用正確的參數格式
            alt = mis.data.altitude if mis.data.altitude > 0 else 2.0
            self.tx_link.mav.command_long_send(
                self.sysid, self.compid,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,       # confirmation
                0,       # param1: 最小起飛俯仰角 (度)
                0,       # param2: 未使用
                0,       # param3: 未使用
                0,       # param4: 偏航角 (度)
                0,       # param5: 緯度 (deg*10^7, 0 表示當前位置)
                0,       # param6: 經度 (deg*10^7, 0 表示當前位置)
                alt)     # param7: 起飛高度 (m)
            self.get_logger().info(f'Sending TAKEOFF command with altitude {alt}m')
        elif cmd == 'GOTO':
            # GOTO 指令：導航到指定座標
            self.tx_link.mav.set_position_target_global_int_send(
                int(time.time()*1e3),  # time_boot_ms
                self.sysid, self.compid,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                0b0000111111111000,    # 忽略除 lat/lon/alt 之外
                int(mis.data.latitude*1e7),
                int(mis.data.longitude*1e7),
                mis.data.altitude,
                0,0,0, 0,0,0, 0,0)
            self.get_logger().info(f'Sending GOTO command to lat={mis.data.latitude:.6f}, '
                                 f'lon={mis.data.longitude:.6f}, alt={mis.data.altitude:.1f}m')
            
        # ────── 模式切換指令 ──────
        elif cmd == 'STABILIZE':
            self._set_flight_mode('STABILIZE')
        elif cmd == 'GUIDED':
            self._set_flight_mode('GUIDED')
        elif cmd == 'AUTO':
            self._set_flight_mode('AUTO')
        elif cmd == 'RTL':
            self._set_flight_mode('RTL')
        elif cmd == 'LAND':
            self._set_flight_mode('LAND')
        else:
            self.get_logger().warning(f'Unknown mission command: {cmd}')

    def _set_flight_mode(self, mode_name: str):
        """設定飛行模式"""
        try:
            # 取得模式對應的數值
            mode_mapping = self.master.mode_mapping()
            
            if mode_name.upper() not in mode_mapping:
                self.get_logger().error(f'Unknown flight mode: {mode_name}')
                return
                
            mode_id = mode_mapping[mode_name.upper()]
            
            # 發送模式切換指令
            self.tx_link.mav.set_mode_send(
                self.sysid,  # target_system
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # base_mode
                mode_id)     # custom_mode
            
            self.get_logger().info(f'Setting flight mode to {mode_name} (ID: {mode_id})')
            
        except Exception as e:
            self.get_logger().error(f'Failed to set flight mode {mode_name}: {e}')

    def _dispatch_velocity(self, twist: Twist):
        """處理 Twist 速度指令並轉換為 MAVLink Body NED 速度控制
        
        Twist 訊息對應到 Body NED 座標系：
        - linear.x: 前進速度 (m/s) - 正值向前，負值向後
        - linear.y: 右移速度 (m/s) - 正值向右，負值向左  
        - linear.z: 下降速度 (m/s) - 正值向下，負值向上
        - angular.z: 偏航角速度 (rad/s) - 正值逆時針，負值順時針
        """
        try:
            vx = float(twist.linear.x)   # 前進速度
            vy = float(twist.linear.y)   # 右移速度  
            vz = float(twist.linear.z)   # 下降速度
            yaw_rate = float(twist.angular.z)  # 偏航角速度
            
            # 限制線性速度範圍 (-10 ~ 10 m/s)
            vx = max(-10.0, min(10.0, vx))
            vy = max(-10.0, min(10.0, vy))
            vz = max(-5.0, min(5.0, vz))   # 垂直速度限制較小
            
            # 限制角速度範圍 (-2 ~ 2 rad/s)
            yaw_rate = max(-2.0, min(2.0, yaw_rate))
            
            # 設定 type_mask: 控制速度和偏航角速度，忽略位置、加速度和偏航角
            type_mask = (
                0b0000111111000000 |  # 忽略位置 x,y,z 和加速度 ax,ay,az
                0b0000000000100000    # 忽略偏航角，但保留偏航角速度
            )
            
            self.tx_link.mav.set_position_target_local_ned_send(
                int(time.time() * 1000),  # time_boot_ms
                self.sysid, self.compid,
                mavutil.mavlink.MAV_FRAME_BODY_NED,  # 機體座標系
                type_mask,        # 控制遮罩
                0, 0, 0,          # x, y, z 位置 (忽略)
                vx, vy, vz,       # vx, vy, vz 速度
                0, 0, 0,          # afx, afy, afz 加速度 (忽略)
                0,                # yaw 偏航角 (忽略)
                yaw_rate)         # yaw_rate 偏航角速度
                
            self.get_logger().debug(f'Body NED velocity: vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}, yaw_rate={yaw_rate:.2f}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to dispatch velocity command: {e}')

    # ==================================================
    #  Thread-2 : RX
    # ==================================================
    def _mav_recv_loop(self):
        d = Drone()
        d.id = f"{self.vehicle_type}/swarm{self.swarmid}/{self.vehicle_name}"
        d.heading = Float64()
        d.battery = Float64()
        d.airspeed = Float64()
        d.data = NavSatFix()
        last_pub = 0.0

        while not self._stop_evt.is_set():
            m = self.master.recv_match(blocking=True, timeout=1.0)
            if m is None:
                continue
            t = time.time()
            mt = m.get_type()

            # 轉發消息到GCS
            if self._should_forward_to_gcs(mt):
                self._forward_to_gcs(m.get_msgbuf())

            if mt == 'HEARTBEAT':
                d.mode = mavutil.mode_string_v10(m)
                # ARM bit 7 (128)
                d.arm = bool(m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            elif mt == 'GLOBAL_POSITION_INT':
                d.data.latitude  = m.lat / 1e7
                d.data.longitude = m.lon / 1e7
                d.data.altitude  = m.relative_alt / 1000.0
                if m.hdg != 65535:
                    d.heading.data = m.hdg / 100.0
            elif mt == 'VFR_HUD':
                d.airspeed.data = m.groundspeed
            elif mt == 'SYS_STATUS':
                if m.battery_remaining != -1:
                    d.battery.data = float(m.battery_remaining)

            # 10 Hz 發佈 (根據白名單)
            if t - last_pub > 0.1 and self._msg_complete(d):
                d.data.header.stamp = self.get_clock().now().to_msg()
                
                # Status topic (原有的)
                if 'status' in self.topic_publishers and self.pub_status:
                    self.pub_status.publish(d)
                
                # GPS topic (新增)
                if 'gps' in self.topic_publishers and mt == 'GLOBAL_POSITION_INT':
                    gps_msg = NavSatFix()
                    gps_msg.header.stamp = self.get_clock().now().to_msg()
                    gps_msg.header.frame_id = "gps"
                    gps_msg.latitude = m.lat / 1e7
                    gps_msg.longitude = m.lon / 1e7
                    gps_msg.altitude = m.alt / 1000.0
                    self.topic_publishers['gps'].publish(gps_msg)
                
                # Attitude topic (新增)
                if 'attitude' in self.topic_publishers and mt == 'ATTITUDE':
                    att_msg = PoseStamped()
                    att_msg.header.stamp = self.get_clock().now().to_msg()
                    att_msg.header.frame_id = "base_link"
                    # 將歐拉角轉換為四元數 (簡化版本)
                    att_msg.pose.orientation.x = m.roll
                    att_msg.pose.orientation.y = m.pitch
                    att_msg.pose.orientation.z = m.yaw
                    att_msg.pose.orientation.w = 1.0
                    self.topic_publishers['attitude'].publish(att_msg)
                
                # Battery topic (新增)
                if 'battery' in self.topic_publishers and mt == 'SYS_STATUS':
                    battery_msg = BatteryState()
                    battery_msg.header.stamp = self.get_clock().now().to_msg()
                    battery_msg.percentage = float(m.battery_remaining) / 100.0 if m.battery_remaining != -1 else 0.0
                    battery_msg.voltage = float(m.voltage_battery) / 1000.0 if hasattr(m, 'voltage_battery') else 0.0
                    self.topic_publishers['battery'].publish(battery_msg)
                
                last_pub = t

    @staticmethod
    def _msg_complete(msg: Drone) -> bool:
        return (msg.mode and isfinite(msg.data.latitude)
                and isfinite(msg.data.longitude) and isfinite(msg.data.altitude))

    # ==================================================
    #  GCS 連線狀態檢查
    # ==================================================
    def _heartbeat_check(self):
        """定期檢查連線狀態並發送心跳"""
        if self.gcs_link and self.gcs_connected:
            try:
                # 發送心跳到GCS
                self.gcs_link.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
            except Exception as e:
                self.get_logger().warning(f'GCS heartbeat failed: {e}')
                self.gcs_connected = False

    # ==================================================
    #  Thread-4 : GCS RX (接收GCS指令)
    # ==================================================
    def _gcs_recv_loop(self):
        """接收來自GCS的指令並轉發到無人機"""
        self.get_logger().info('GCS receive loop started')
        
        while not self._stop_evt.is_set() and self.gcs_link:
            try:
                msg = self.gcs_link.recv_match(blocking=True, timeout=1.0)
                if msg is None:
                    continue
                    
                msg_type = msg.get_type()
                
                # 過濾需要轉發的消息類型
                if self._should_forward_to_drone(msg_type):
                    # 轉發到無人機
                    self.master.write(msg.get_msgbuf())
                    self.get_logger().debug(f'Forwarded {msg_type} from GCS to drone')
                    
            except Exception as e:
                self.get_logger().error(f'GCS receive error: {e}')
                self.gcs_connected = False
                break

    # ==================================================
    #  Thread-5 : GCS TX (轉發到GCS)
    # ==================================================
    def _gcs_tx_loop(self):
        """將佇列中的消息發送到GCS"""
        self.get_logger().info('GCS transmit loop started')
        
        while not self._stop_evt.is_set():
            try:
                msg_buf = self.gcs_tx_queue.get(timeout=0.1)
                if self.gcs_link and self.gcs_connected:
                    self.gcs_link.write(msg_buf)
            except Empty:
                pass
            except Exception as e:
                self.get_logger().error(f'GCS transmit error: {e}')
                self.gcs_connected = False

    # ==================================================
    #  消息過濾和轉發邏輯
    # ==================================================
    def _should_forward_to_drone(self, msg_type: str) -> bool:
        """判斷是否應該轉發GCS消息到無人機"""
        forward_types = [
            'COMMAND_LONG',
            'COMMAND_INT', 
            'SET_POSITION_TARGET_GLOBAL_INT',
            'SET_POSITION_TARGET_LOCAL_NED',
            'MISSION_REQUEST_LIST',
            'MISSION_REQUEST',
            'MISSION_ACK',
            'MISSION_ITEM',
            'MISSION_ITEM_INT',
            'MISSION_SET_CURRENT',
            'MISSION_CLEAR_ALL',
            'PARAM_REQUEST_READ',
            'PARAM_REQUEST_LIST',
            'PARAM_SET',
            'RC_CHANNELS_OVERRIDE'
        ]
        return msg_type in forward_types

    def _should_forward_to_gcs(self, msg_type: str) -> bool:
        """判斷是否應該轉發無人機消息到GCS"""
        # 大部分狀態消息都應該轉發到GCS
        skip_types = [
            'BAD_DATA',
            'MEMINFO'
        ]
        return msg_type not in skip_types

    def _forward_to_gcs(self, msg_buf: bytes):
        """將消息添加到GCS轉發佇列"""
        if self.gcs_link and self.gcs_connected:
            try:
                self.gcs_tx_queue.put_nowait(msg_buf)
            except:
                self.get_logger().warning('GCS TX queue overflow')

    # ==================================================
    def destroy_node(self):
        """正確關閉所有連線和執行緒"""
        self.get_logger().info('Shutting down MAVLink bridge...')
        self._stop_evt.set()
        
        # 關閉MAVLink連線
        if hasattr(self, 'master') and self.master:
            try:
                self.master.close()
            except:
                pass
                
        # 關閉GCS連線
        if hasattr(self, 'gcs_link') and self.gcs_link:
            try:
                self.gcs_link.close()
                self.get_logger().info('GCS connection closed')
            except:
                pass
                
        super().destroy_node()


# =======================================================
def main(args=None):
    rclpy.init(args=args)
    node = MavBridge()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
