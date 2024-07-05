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
    pkg_name = 'gym'
    
    robot_pkg_name = "kapibara"
    file_subpath = 'description/kapibara.urdf.xacro'

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(robot_pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file,mappings={'sim_mode' : 'true'}).toxml()
    
    
    xacro_file = os.path.join(get_package_share_directory(pkg_name),'description/landmine.urdf.xacro')
    landmine_description = xacro.process_file(xacro_file).toxml()


    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}] # add other parameters here if required
    )
    
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
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py',)]),
            launch_arguments={
                #'world': '/app/src/rviz/playground.sdf',
                'params_file': os.path.join(get_package_share_directory(pkg_name),"config/gazebo.yaml")
                }.items()
        )
    
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=["-topic","/robot_description","-entity","kapibara","-timeout","240","-Y","-1.57"],
                    output='screen')
    
    spawn_mine = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=["-topic","/mine_description","-entity","mine","-timeout","240","-x","-4.29","-y","4.22"],
                    output='screen')
    
    spawn_maze = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=["-file",os.path.join(get_package_share_directory(pkg_name),"props/Simple_Maze/model.sdf"),"-entity","Maze","-timeout","240","-x","0.844","-y","4.16"],
                    output='screen')
    
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["motors",'--controller-manager-timeout','240'],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad",'--controller-manager-timeout','240'],
    )
    
    ears_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ears_controller",'--controller-manager-timeout','240'],
    )
    
    fusion = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        parameters=[
            {"use_mag":False}
        ],
        remappings=[
            ('/imu/data_raw','/Gazebo/imu'),
            ('/imu/data','/Gazebo/orientation')
        ]
    )

    
    # Run the node
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        node_landmine_state_publisher,
        spawn_maze,
        spawn,
        spawn_mine,
        fusion,
        diff_drive_spawner,
        joint_broad_spawner,
        ears_controller_spawner
        #mind
    ])

