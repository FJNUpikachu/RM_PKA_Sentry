import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch_ros.actions import Node, PushRosNamespace, SetRemap

def generate_launch_description():
    bt_config_dir = os.path.join(get_package_share_directory('rm_behavior_tree'), 'config')
    
    style = LaunchConfiguration('style', default='full')
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')

    bt_xml_dir = PathJoinSubstitution([bt_config_dir, style]), ".xml"



    # rm_behavior_tree_node = Node(
    #     package='rm_behavior_tree',
    #     executable='rm_behavior_tree',
    #     respawn=True,
    #     respawn_delay=3,
    #     parameters=[
    #         {
    #           'style': bt_xml_dir,
    #           'use_sim_time': use_sim_time,
    #         }
    #     ]
    # )

    # return LaunchDescription([rm_behavior_tree_node])



    bringup_cmd_group = GroupAction(
        [
            PushRosNamespace(namespace="red_standard_robot1"),
            SetRemap("/navigate_to_pose", "navigate_to_pose"),
            SetRemap("/tf", "tf"),
            SetRemap("/tf_static", "tf_static"),
            Node(
                package='rm_behavior_tree',
                executable='rm_behavior_tree',
                respawn=True,
                respawn_delay=3,
                parameters=[
                    {
                    'style': bt_xml_dir,
                    'use_sim_time': use_sim_time,
                    'namespace': "red_standard_robot1",
                    }
                ]
            )
        ]
    )
    ld = LaunchDescription()
    ld.add_action(bringup_cmd_group)

    return ld
