from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.actions import IncludeLaunchDescription
import os
from ament_index_python.packages import get_package_share_path
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    urdf_path = os.path.join(get_package_share_path('robot_description'),
                             'urdf', 'robot.urdf.xacro')
    rviz_config_path = os.path.join(get_package_share_path('robot_description'),
                             'rviz', 'rviz2.config.rviz')
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_path('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
    )
     
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}]
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'Inverted_Pendulum'],
                    output='screen')

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo,
        spawn_entity
    ])