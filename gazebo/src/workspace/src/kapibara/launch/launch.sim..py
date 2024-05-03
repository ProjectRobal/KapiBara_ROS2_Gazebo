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

    gazebo_env = SetEnvironmentVariable("GAZEBO_MODEL_PATH", os.path.join(get_package_prefix("kapibara"), "share"))

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
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py',)]),
            launch_arguments={'world': '/app/src/rviz/playground.sdf'}.items()
        )
    
    state_publisher = Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
                    arguments=[],
                    output='screen')


    rviz = Node(package='rviz2', executable='rviz2',
                    arguments=[],
                    output='screen')
    
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=["-topic","/robot_description","-entity","kapibara","-timeout","240","-z","1"],
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

    emotions = Node(
        package="emotion_estimer",
        executable="estimator.py",
        parameters=[{
            "tofs":["/Gazebo/front_left",
                    "/Gazebo/front_right",
                    "/Gazebo/side_left",
                    "/Gazebo/side_right"
                    ],
            "imu":'/Gazebo/orientation'
        }]
    )
    # Run the node
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        #rviz,
        #state_publisher,
        spawn,
        fusion,
        emotions,
        diff_drive_spawner,
        joint_broad_spawner,
        ears_controller_spawner
    ])


