import os
import logging
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro

from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    return
    
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=["-topic","/robot_description","-entity","kapibara","-timeout","240"],
                    output='screen')



    # Run the node
    return LaunchDescription([
        spawn,
        #diff_drive_spawner,
        #joint_broad_spawner
    ])


