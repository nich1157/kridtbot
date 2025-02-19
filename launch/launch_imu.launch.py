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


    config = os.path.join(
        get_package_share_directory('bno055'),
        'config',
        'bno055_params_i2c.yaml'
        )
        
    imu_launch =Node(
        package = 'bno055',
        executable = 'bno055',
        parameters = [config]
    )

    package_name = 'kridtbot'
    
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control':'true'}.items()
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


    # Launch all
    return LaunchDescription([
        rsp,
        imu_launch,
        delayed_controller_manager,
        delayed_imu_broad_spawner,
    ])