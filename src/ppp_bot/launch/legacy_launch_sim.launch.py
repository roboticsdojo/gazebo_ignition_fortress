import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, LogInfo, ExecuteProcess, EmitEvent, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart, OnExecutionComplete, OnProcessExit
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, TextSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch.events import Shutdown

from launch_ros.actions import Node
import xacro


package_name='ppp_bot'


def launch_ign(context: LaunchContext, world_name):
    world_name_str = context.perform_substitution(world_name)
    world_file = os.path.join(pkg_path, 'worlds', f'{world_name_str}.sdf')

    ignition_launcher = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                              'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('gz_args', [f' -r -v 4 {world_file}'])])

    return [ignition_launcher]


def spawn_entity(context: LaunchContext, world_name, urdf_file):
    world_name_str = context.perform_substitution(world_name)

    node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', world_name_str,
            '-file', urdf_file,
            '-name', package_name,
            '-z', '1.0'
        ],
        output='screen'
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_drive_base_controller'],
        output='screen'
    )

    process_event_handler = RegisterEventHandler(
                                event_handler=OnProcessExit(
                                    target_action=node,
                                    on_exit=[load_joint_state_controller]
                                )
                            )
    joint_state_controller_event_handler = RegisterEventHandler(
                                                event_handler=OnProcessExit(
                                                    target_action=load_joint_state_controller,
                                                    on_exit=[load_joint_trajectory_controller],
                                                )
                                            )
    return [process_event_handler, joint_state_controller_event_handler, node]


def parameter_bridge(context: LaunchContext, world_name):
    
    world_name_str = context.perform_substitution(world_name)


    args = [
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            f'/world/{world_name_str}/model/ppp_bot/link/base_link/sensor/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            f'/world/{world_name_str}/model/ppp_bot/link/base_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            # f'/world/{world_name_str}/model/ppp_bot/link/base_link/sensor/depth_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            # f'/world/{world_name_str}/model/ppp_bot/link/base_link/sensor/depth_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            f'/world/{world_name_str}/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']

    node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=args,
        remappings=[(f'/world/{world_name_str}/clock', '/clock')],
        output='screen'
    )

    return [node]



def generate_launch_description():
    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!


    # Launch Configuration
    world_name = LaunchConfiguration('world_name')


    # Launch Arguments
    world_name_launch_arg = DeclareLaunchArgument(
        'world_name',
        default_value='cones'
    )

    
    
    # Check if we're told to use sim time

    # Process the URDF file
    global pkg_path
    pkg_path = os.path.join(get_package_share_directory('ppp_bot'))
    models_dir = os.path.join(pkg_path, 'models')
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    slam_config_file = os.path.join(pkg_path, 'config', 'mapper_params_online_async.yaml')
    rviz_config_file = os.path.join(pkg_path, 'config', 'default.rviz')
    twist_mux_config_file = os.path.join(pkg_path, 'config', 'twist_mux.yaml')


    urdf_file = os.path.join(pkg_path, 'description', 'robot.urdf')
    robot_description_config = xacro.process_file(xacro_file)
    with open(urdf_file, 'w') as f:
        f.write(robot_description_config.toxml())
    
    # Create a robot_state_publisher node
    node_robot_state_publisher_params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': True}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[node_robot_state_publisher_params]
    )



    node_teleop = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        remappings=[('/cmd_vel', '/cmd_vel_keyboard')],
        prefix='xterm -e',
        output='screen',
    )


    # static tf2 broadcasters for model sensors
    lidar_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_transform_publisher',
        arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'lidar_frame', '--child-frame-id', 'ppp_bot/base_link/gpu_lidar']
    )

    camera_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_static_transform_publisher',
        arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'camera_link', '--child-frame-id', 'ppp_bot/base_link/camera']
    )

    # depth_camera_broadcaster = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='depth_camera_static_transform_publisher',
    #     arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'depth_camera_link', '--child-frame-id', 'ppp_bot/base_link/depth_camera']
    # )

    slam_launcher = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            [os.path.join(get_package_share_directory('slam_toolbox'),
                                        'launch', 'online_async_launch.py')]),
                        launch_arguments={
                            'params_file': slam_config_file,
                            'use_sim_time': 'true'
                        }.items()
                    )

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': True}]
        )

    twist_mux_node = Node(
            package='twist_mux',
            executable='twist_mux',
            parameters=[twist_mux_config_file, {'use_sim_time': True}],
            remappings=[('cmd_vel_out', '/diff_drive_base_controller/cmd_vel_unstamped')]
        )

    nav2_bringup_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('nav2_bringup'),
                              'launch', 'navigation_launch.py')]),
            launch_arguments=[('use_sim_time', 'true')])

    # Launch!
    return LaunchDescription([
        # Launch Arguments
        world_name_launch_arg,
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', models_dir),
        SetEnvironmentVariable('GTK_PATH', ''),
        rviz_node,


        OpaqueFunction(function=launch_ign, args=[world_name]),
        

        node_robot_state_publisher,
        OpaqueFunction(function=spawn_entity, args=[world_name, urdf_file]),
        OpaqueFunction(function=parameter_bridge, args=[world_name]),

        # tf2 nodes
        lidar_broadcaster,
        camera_broadcaster,
        # depth_camera_broadcaster,


        node_teleop,
        slam_launcher,
        twist_mux_node,
        # nav2_bringup_node,

    ])