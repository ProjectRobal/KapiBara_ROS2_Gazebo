import os
import logging
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from launch_ros.actions import Node
import xacro

from launch.actions import SetEnvironmentVariable

def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'kapibara'
    file_subpath = 'description/kapibara.urdf.xacro'


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file,mappings={'sim_mode' : 'true'}).toxml()
    
    xacro_file = os.path.join(get_package_share_directory(pkg_name),'description/landmine.urdf.xacro')
    landmine_description = xacro.process_file(xacro_file).toxml()

    gazebo_env = SetEnvironmentVariable("GAZEBO_MODEL_PATH", os.path.join(get_package_prefix("kapibara"), "share"))

    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}] # add other parameters here if required
    )
    
    # Configure the node
    node_landmine_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': landmine_description,
        'use_sim_time': True}], # add other parameters here if required
        remappings=[
            ('/robot_description','/mine_description')
        ]
    )
    
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=["-topic","/mine_description","-entity","mine","-timeout","240","-z","1"],
                    output='screen')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py',)])
        )
    
   
    # Run the node
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        node_landmine_state_publisher,
        spawn
    ])

