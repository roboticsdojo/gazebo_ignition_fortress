import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='ppp_bot' #<--- CHANGE ME

    launch_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','launch_sim.launch.py'
        )])
    )

    parameter_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist']
    )

    teleop_twist_keyboard = Node(
        package='teleop_tiwst_keyboard',
        executable='teleop_twist_keyboard'
    )

    # Launch them all!
    return LaunchDescription([
        launch_sim,
        TimerAction(period=2.0, actions=[parameter_bridge]),
        RegisterEventHandler(
            OnProcessStart(
                target_action=parameter_bridge,
                on_start=[
                    LogInfo(msg='Parameter bridge established, running teleop_twist_keyboard...'),
                    teleop_twist_keyboard
                ]
            )
        )
    ])