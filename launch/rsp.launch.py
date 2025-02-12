import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # sim time argument
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    
    # Process URDF 
    pkg_path = os.path.join(get_package_share_directory('kridtbot'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro') # main xacro
    # robot_description_config = xacro.process_file(xacro_file).toxml()
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])
    
    # robot_state_publisher node on topic "robot_description"
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    #params = {'robot_description': robot_description_config}
    #node_joint_state_publisher_gui = Node(
    #    package='joint_state_publisher_gui',
    #    executable='joint_state_publisher_gui',
    #    output='screen',
    #    parameters=[params]
    #)


    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        node_robot_state_publisher,
        #node_joint_state_publisher_gui
    ])
