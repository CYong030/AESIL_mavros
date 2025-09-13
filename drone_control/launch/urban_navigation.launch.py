#!/usr/bin/env python3
"""
城市導航 Launch 檔案範例
展示如何使用 topic whitelist 來啟用城市導航相關功能
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 基本參數
    declare_vehicle_type = DeclareLaunchArgument(
        'vehicle_type', 
        default_value='drone',
        description='Vehicle type'
    )
    
    declare_swarm_id = DeclareLaunchArgument(
        'swarm_id', 
        default_value='1',
        description='Swarm ID'
    )
    
    declare_vehicle_name = DeclareLaunchArgument(
        'vehicle_name', 
        default_value='urban_navigator',
        description='Vehicle name'
    )
    
    declare_mavlink_uri = DeclareLaunchArgument(
        'mavlink_uri', 
        default_value='/dev/ttyACM0',
        description='MAVLink connection URI'
    )
    
    declare_gcs_uri = DeclareLaunchArgument(
        'gcs_uri', 
        default_value='udpout:192.168.2.44:14550',
        description='GCS connection URI'
    )

    # 取得參數
    vehicle_type = LaunchConfiguration('vehicle_type')
    swarm_id = LaunchConfiguration('swarm_id')
    vehicle_name = LaunchConfiguration('vehicle_name')
    mavlink_uri = LaunchConfiguration('mavlink_uri')
    gcs_uri = LaunchConfiguration('gcs_uri')

    return LaunchDescription([
        declare_vehicle_type,
        declare_swarm_id,
        declare_vehicle_name,
        declare_mavlink_uri,
        declare_gcs_uri,

        Node(
            package='drone_control',
            executable='mavlink_bridge',
            name='urban_navigation_bridge',
            namespace='urban_drone',
            output='screen',
            parameters=[
                {
                    'mavlink_uri': mavlink_uri,
                    'gcs_uri': gcs_uri,
                    'vehicle_type': vehicle_type,
                    'swarm_id': swarm_id,
                    'vehicle_name': vehicle_name,
                    
                    # 城市導航配置 (啟用所有城市導航相關 topics)
                    'enable_status_pub': True,      # 基本狀態
                    'enable_mission_sub': True,     # 任務指令
                    'enable_velocity_sub': True,    # 速度控制
                    'enable_gps_pub': True,         # GPS 定位 ⭐ 城市導航必須
                    'enable_attitude_pub': True,    # 姿態資訊 ⭐ 導航控制用
                    'enable_battery_pub': True,     # 電池監控 ⭐ 安全考量
                    'enable_obstacle_pub': True,    # 障礙物警告 ⭐ 城市環境重要
                    'enable_waypoint_sub': True,    # 航點導航 ⭐ 城市路線規劃
                    'enable_emergency_sub': True,   # 緊急停止 ⭐ 安全考量
                }
            ],
        )
    ])
