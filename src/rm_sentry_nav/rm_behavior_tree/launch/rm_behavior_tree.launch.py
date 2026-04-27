import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction

def generate_launch_description():
    bt_config_dir = os.path.join(get_package_share_directory('rm_behavior_tree'), 'config')
    
    # 声明参数，增加 robot_namespace
    style = LaunchConfiguration('style')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_namespace = LaunchConfiguration('robot_namespace')

    bt_xml_dir = [PathJoinSubstitution([bt_config_dir, style]), ".xml"]

    bringup_cmd_group = GroupAction(
        [
            # 使用变量 robot_namespace
            PushRosNamespace(namespace=robot_namespace),
            
            Node(
                package='rm_behavior_tree',
                executable='rm_behavior_tree',
                respawn=True,
                respawn_delay=3,
                parameters=[
                    {
                        'style': bt_xml_dir,
                        'use_sim_time': use_sim_time,
                    }
                ]
            )
        ]
    )

    ld = LaunchDescription()
    
    # 注册参数
    ld.add_action(DeclareLaunchArgument('style', default_value='full'))
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='False'))
    ld.add_action(DeclareLaunchArgument('robot_namespace', default_value='')) # 默认空命名空间

    ld.add_action(bringup_cmd_group)

    return ld