import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'kapibara'
    file_subpath = 'description/kapibara.urdf.xacro'


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory("kapibara"),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}] # add other parameters here if required
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )
    
    state_publisher = Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
                    arguments=[],
                    output='screen')


    rviz = Node(package='rviz2', executable='rviz2',
                    arguments=[],
                    output='screen')
    
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=["-topic","robot_description","-entity","kapibara"],
                    output='screen')
    
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )


    # Run the node
    return LaunchDescription([
        node_robot_state_publisher,
        rviz,
        #gazebo,
        state_publisher,
        #spawn,
        #diff_drive_spawner,
        #joint_broad_spawner
    ])


