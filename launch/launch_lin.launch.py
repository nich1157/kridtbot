## Launch three packages to run Gazebo
## From template: https://github.com/joshnewans/articubot_one/blob/adb08202d3dafeeaf8a3691ddd64aa8551c40f78/launch/launch_sim.launch.py

import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node



def generate_launch_description():

    package_name='kridtbot' 

    #### Launch 1: Launch the robot state publisher
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control':'true'}.items()
    )

    imu_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','launch_imu.launch.py'
                )])
    )

    robot_description = Command([
        'ros2 param get --hide-type /robot_state_publisher robot_description'
    ])
    controller_params = os.path.join(get_package_share_directory(package_name), 'config', 'my_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description':robot_description},
                    controller_params],
    )
    delayed_controller_manager = TimerAction(period=3.0,actions=[controller_manager])
    
    # configure, inactive and activate controllers - diff_drive and joint broadcaster
    lin_control_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["linear_position_control"],
    )

    delayed_lin_control_spwaner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[lin_control_spawner]
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner]
        )
    )

    imu_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor"],
    )

    delayed_imu_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[imu_broad_spawner]
        )
    )
#    # ros2 jazzy update. no use of unstamped 
#    twist_stamper = Node(
#        package='twist_stamper',
#        executable='twist_stamper',
#        parameters=[{'use_sim_time': False}],
#        remappings=[('/cmd_vel_in', 'diff_cont/cmd_vel_unstamped'),
#                    ('/cmd_vel_out','/diff_cont/cmd_vel')]
#    )


    # Launch all
    return LaunchDescription([
        #imu_launch,
        rsp,
        delayed_controller_manager,
        delayed_lin_control_spwaner,
        delayed_joint_broad_spawner,
        #delayed_imu_broad_spawner,
    ])