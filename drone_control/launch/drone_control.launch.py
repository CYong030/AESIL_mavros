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
    
    # 取得啟動參數
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
                }
            ],
        )
    ])
## 現在支援多種載具類型的動態命名空間，完全避免節點名稱衝突
## 支援載具類型：drone, ugv, usv, uav 等
