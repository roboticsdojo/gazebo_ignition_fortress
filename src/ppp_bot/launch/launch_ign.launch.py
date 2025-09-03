from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

import os
import xacro


package_name='ppp_bot'


def launch_ign(context: LaunchContext, world_name):
    world_name_str = context.perform_substitution(world_name)
    pkg_path = os.path.join(get_package_share_directory('ppp_bot'))
    world_file = os.path.join(pkg_path, 'worlds', f'{world_name_str}.sdf')
    ignition_launcher = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                              'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('gz_args', [f' -r {world_file}'])])
    return [ignition_launcher]


def spawn_entity(context: LaunchContext, world_name, robot_description):
    world_name_str = context.perform_substitution(world_name)
    node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', world_name_str,
            '-string', robot_description.toxml(),
            '-name', 'ppp_bot',
            '-z', '1.0'
        ]
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster']
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_drive_base_controller']
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
            f'/world/{world_name_str}/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']
    node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=args,
        remappings=[(f'/world/{world_name_str}/clock', '/clock')]
    )
    return [node]



def generate_launch_description():
    pkg_path = os.path.join(get_package_share_directory('ppp_bot'))

    # Launch Configuration
    world_name = LaunchConfiguration('world_name')

    # Launch Arguments
    world_name_launch_arg = DeclareLaunchArgument(
        'world_name',
        default_value='cones'
    )
    
    models_dir = os.path.join(pkg_path, 'models')
    xacro_path = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_path)

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_path, 'launch', 'rsp.launch.py')
        ]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # static tf2 broadcasters for model sensors
    # These transforms map from the sensor frames to the Gazebo sensor frames
    # The lidar is positioned at (0.15, 0, 0.15) relative to chassis, which is at (-0.1, 0, 0) relative to base_link
    # So lidar is at (0.05, 0, 0.15) relative to base_link
    lidar_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_transform_publisher',
        arguments = ['--x', '0.05', '--y', '0', '--z', '0.15', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'lidar_frame', '--child-frame-id', 'ppp_bot/base_link/gpu_lidar']
    )

    # The camera is positioned at (0.305, 0, 0.08) relative to chassis, which is at (-0.1, 0, 0) relative to base_link
    # So camera is at (0.205, 0, 0.08) relative to base_link
    camera_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_static_transform_publisher',
        arguments = ['--x', '0.205', '--y', '0', '--z', '0.08', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'camera_link', '--child-frame-id', 'ppp_bot/base_link/camera']
    )

    return LaunchDescription([
        world_name_launch_arg,
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
SetEnvironmentVariable('MESA_GL_VERSION_OVERRIDE', '3.3'),
SetEnvironmentVariable('MESA_GLSL_VERSION_OVERRIDE', '330'),
SetEnvironmentVariable('IGN_RENDERING_ENGINE_PATH', '/usr/lib/x86_64-linux-gnu/ign-rendering-6/engine-plugins'),
SetEnvironmentVariable('IGN_RENDERING_ENGINE', 'ogre2'),
SetEnvironmentVariable('DISPLAY', ':0'),
SetEnvironmentVariable('QT_X11_NO_MITSHM', '1'),
        OpaqueFunction(function=launch_ign, args=[world_name]),
        robot_state_publisher,
        OpaqueFunction(function=spawn_entity, args=[world_name, robot_description]),
        OpaqueFunction(function=parameter_bridge, args=[world_name]),
        lidar_broadcaster,
        camera_broadcaster,
    ])