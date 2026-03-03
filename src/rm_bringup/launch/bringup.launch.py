import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
sys.path.append(os.path.join(get_package_share_directory('rm_bringup'), 'launch'))

def generate_launch_description():

    from launch_ros.actions import ComposableNodeContainer, Node, PushRosNamespace
    from launch_ros.descriptions import ComposableNode
    from launch.actions import TimerAction
    from launch import LaunchDescription

    launch_params = yaml.safe_load(open(os.path.join(
        get_package_share_directory('rm_bringup'), 'config', 'launch_params.yaml')))

    robot_gimbal_description = Command(['xacro ', os.path.join(
        get_package_share_directory('rm_robot_description'), 'urdf', 'rm_gimbal.urdf.xacro'),
        ' xyz:=', launch_params['odom2camera']['xyz'], ' rpy:=', launch_params['odom2camera']['rpy']])
     
    # 修改点：将 robot_state_publisher 改为普通节点，不再放入容器
    robot_gimbal_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_gimbal_description,
                    'publish_frequency': 1000.0}]
    )

    def get_params(name):
        return os.path.join(get_package_share_directory('rm_bringup'), 'config', 'node_params', '{}_params.yaml'.format(name))

    # 检查是否为虚拟串口模式
    virtual_serial = launch_params.get('virtual_serial', 0)
    
    # 启动列表以 robot_state_publisher 节点开始（不再使用容器）
    launch_description_list = [robot_gimbal_publisher_node]
    
    # 如果不是虚拟串口模式，则启动所有视觉节点
    if virtual_serial != 1:
        # 图像ComposableNode
        hik_camera_node = ComposableNode(
            package='rm_hik_camera_driver',
            plugin='pka::hik_camera::HikCameraNode',
            name='hik_camera_driver',
            parameters=[get_params('hik_camera_driver')],
            extra_arguments=[{'use_intra_process_comms': True}]
        )
         
        dahua_camera_node = ComposableNode(
            package='rm_dahua_camera_driver',
            plugin='pka::dahua_camera::DahuaCameraNode',
            name='dahua_camera_driver',
            parameters=[get_params('dahua_camera_driver')],
            extra_arguments=[{'use_intra_process_comms': True}]
        )

        # 装甲板识别 ComposableNode
        armor_detector_node = ComposableNode(
            package='armor_detector', 
            plugin='pka::auto_aim::ArmorDetectorNode',
            name='armor_detector',
            parameters=[get_params('armor_detector')],
            extra_arguments=[{'use_intra_process_comms': True}]
        )

        # 装甲板解算 ComposableNode
        armor_solver_node = ComposableNode(
            package='armor_solver', 
            plugin='pka::auto_aim::ArmorSolverNode',
            name='armor_solver',
            parameters=[get_params('armor_solver')],
            extra_arguments=[{'use_intra_process_comms': True}]
        )
        
        # 2. 相机容器（根据配置选择相机）
        camera_node = hik_camera_node if launch_params.get('camera') == "hik" else dahua_camera_node
        camera_container = ComposableNodeContainer(
            name='camera_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[camera_node],
            output='both',
            emulate_tty=True,
            ros_arguments=['--ros-args', ],
        )
        
        # 3. 检测器容器
        detector_container = ComposableNodeContainer(
            name='detector_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[armor_detector_node],
            output='both',
            emulate_tty=True,
            ros_arguments=['--ros-args', ],
        )
        
        # 4. 解算器容器
        solver_container = ComposableNodeContainer(
            name='solver_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[armor_solver_node],
            output='both',
            emulate_tty=True,
            ros_arguments=['--ros-args', ],
        )
        
        # 设置延迟启动
        delay_camera_container = TimerAction(
            period=2.0,
            actions=[camera_container],
        )
        
        delay_detector_container = TimerAction(
            period=2.2,
            actions=[detector_container],
        )
        
        delay_solver_container = TimerAction(
            period=2.4,
            actions=[solver_container],
        )
        
        launch_description_list.extend([
            delay_camera_container,
            delay_detector_container,
            delay_solver_container,
        ])
    
    # 串口 ComposableNode（在虚拟串口模式下也会启动）
    serial_driver_node = ComposableNode(
        package='rm_serial_driver',
        plugin='pka::serial_driver::UARTNode',
        name='serial_driver',
        parameters=[get_params('serial_driver')],
        extra_arguments=[{'use_intra_process_comms': True}]
    )
    
    # 5. 串口容器
    serial_container = ComposableNodeContainer(
        name='serial_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[serial_driver_node],
        output='both',
        emulate_tty=True,
        ros_arguments=['--ros-args', ],
    )
    
    # 串口容器延迟启动（根据模式调整延迟时间）
    if virtual_serial == 1:
        delay_serial_container = TimerAction(
            period=1.0,
            actions=[serial_container],
        )
    else:
        delay_serial_container = TimerAction(
            period=2.6,
            actions=[serial_container],
        )
    
    push_namespace = PushRosNamespace(launch_params['namespace'])
     
    launch_description_list.append(push_namespace)
    launch_description_list.append(delay_serial_container)
     
    return LaunchDescription(launch_description_list)