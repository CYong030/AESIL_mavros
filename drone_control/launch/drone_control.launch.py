from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 宣告啟動參數
    declare_vehicle_type = DeclareLaunchArgument(
        'vehicle_type', 
        default_value='drone',
        description='Vehicle type (drone, ugv, usv, etc.)'
    )
    
    declare_swarm_id = DeclareLaunchArgument(
        'swarm_id', 
        default_value='1',
        description='Swarm ID for the vehicle'
    )
    
    declare_vehicle_name = DeclareLaunchArgument(
        'vehicle_name', 
        default_value='leader1',
        description='Vehicle name identifier'
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
    
    # ────── Topic 白名單參數 ──────
    declare_enable_status = DeclareLaunchArgument(
        'enable_status_pub', 
        default_value='True',
        description='Enable status topic publication'
    )
    
    declare_enable_mission = DeclareLaunchArgument(
        'enable_mission_sub', 
        default_value='True',
        description='Enable mission topic subscription'
    )
    
    declare_enable_velocity = DeclareLaunchArgument(
        'enable_velocity_sub', 
        default_value='True',
        description='Enable velocity command subscription'
    )
    
    declare_enable_gps = DeclareLaunchArgument(
        'enable_gps_pub', 
        default_value='False',
        description='Enable GPS topic publication (for urban navigation)'
    )
    
    declare_enable_attitude = DeclareLaunchArgument(
        'enable_attitude_pub', 
        default_value='False',
        description='Enable attitude topic publication'
    )
    
    declare_enable_battery = DeclareLaunchArgument(
        'enable_battery_pub', 
        default_value='False',
        description='Enable battery status publication'
    )
    
    declare_enable_obstacle = DeclareLaunchArgument(
        'enable_obstacle_pub', 
        default_value='False',
        description='Enable obstacle warning publication (for urban navigation)'
    )
    
    declare_enable_waypoint = DeclareLaunchArgument(
        'enable_waypoint_sub', 
        default_value='False',
        description='Enable waypoint navigation subscription (for urban navigation)'
    )
    
    declare_enable_emergency = DeclareLaunchArgument(
        'enable_emergency_sub', 
        default_value='False',
        description='Enable emergency stop subscription'
    )
    
    # 取得啟動參數
    vehicle_type = LaunchConfiguration('vehicle_type')
    swarm_id = LaunchConfiguration('swarm_id')
    vehicle_name = LaunchConfiguration('vehicle_name')
    mavlink_uri = LaunchConfiguration('mavlink_uri')
    gcs_uri = LaunchConfiguration('gcs_uri')
    
    # Topic 白名單參數
    enable_status_pub = LaunchConfiguration('enable_status_pub')
    enable_mission_sub = LaunchConfiguration('enable_mission_sub')
    enable_velocity_sub = LaunchConfiguration('enable_velocity_sub')
    enable_gps_pub = LaunchConfiguration('enable_gps_pub')
    enable_attitude_pub = LaunchConfiguration('enable_attitude_pub')
    enable_battery_pub = LaunchConfiguration('enable_battery_pub')
    enable_obstacle_pub = LaunchConfiguration('enable_obstacle_pub')
    enable_waypoint_sub = LaunchConfiguration('enable_waypoint_sub')
    enable_emergency_sub = LaunchConfiguration('enable_emergency_sub')

    return LaunchDescription([
        declare_vehicle_type,
        declare_swarm_id,
        declare_vehicle_name,
        declare_mavlink_uri,
        declare_gcs_uri,
        declare_enable_status,
        declare_enable_mission,
        declare_enable_velocity,
        declare_enable_gps,
        declare_enable_attitude,
        declare_enable_battery,
        declare_enable_obstacle,
        declare_enable_waypoint,
        declare_enable_emergency,

        Node(
            package='drone_control',
            executable='mavlink_bridge',
            name=[vehicle_type, '_bridge_swarm', swarm_id, '_', vehicle_name],  # 動態節點名稱包含載具類型
            namespace=[vehicle_type, '_swarm_', swarm_id],  # 動態命名空間包含載具類型
            output='screen',
            parameters=[
                {
                    'mavlink_uri': mavlink_uri,
                    'gcs_uri': gcs_uri,
                    'vehicle_type': vehicle_type,
                    'swarm_id': swarm_id,
                    'vehicle_name': vehicle_name,
                    # Topic 白名單參數
                    'enable_status_pub': enable_status_pub,
                    'enable_mission_sub': enable_mission_sub,
                    'enable_velocity_sub': enable_velocity_sub,
                    'enable_gps_pub': enable_gps_pub,
                    'enable_attitude_pub': enable_attitude_pub,
                    'enable_battery_pub': enable_battery_pub,
                    'enable_obstacle_pub': enable_obstacle_pub,
                    'enable_waypoint_sub': enable_waypoint_sub,
                    'enable_emergency_sub': enable_emergency_sub,
                }
            ],
        )
    ])
## 現在支援多種載具類型的動態命名空間，完全避免節點名稱衝突
## 支援載具類型：drone, ugv, usv, uav 等
