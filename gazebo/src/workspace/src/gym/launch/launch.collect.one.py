import os
import logging
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from launch import LaunchDescription,substitutions
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument,GroupAction
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from launch_ros.actions import Node
import xacro

from launch.actions import SetEnvironmentVariable

'''
 
 Position for all collectables:

# 1:
 -1.86847, 2.12847, 0.5
# 2:
 0.109086, -1.9246, 0.5
# 3: 
 -3.04748, -3.18496, 0.5
# 4:
 -4.68853, 2.104755, 0.5

'''

POINTS_POSITIONS=[
    (-1.86847, 2.12847, 0.4),
    (0.109086, -1.9246, 0.4),
    (-3.04748, -3.18496, 0.4),
    (-4.68853, 2.104755, 0.4)
    ]

def generate_launch_description():

    logger = substitutions.LaunchConfiguration("log_level")
    
    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'gym'
    
    robot_pkg_name = "kapibara"
    file_subpath = 'description/kapibara.urdf.xacro'

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(robot_pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file,mappings={'sim_mode' : 'true'}).toxml()    
    
    xacro_file = os.path.join(get_package_share_directory(pkg_name),'description/landmine.urdf.xacro')
    landmine_description = xacro.process_file(xacro_file,mappings={'topic' : 'trigger1'}).toxml()
    
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
            launch_arguments={
                'world': os.path.join(get_package_share_directory(pkg_name),'worlds/world.sdf'),
                'params_file': os.path.join(get_package_share_directory(pkg_name),"config/gazebo.yaml"),
                'verbose': 'true'
                }.items()
        )
    
    spawn_maze = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=["-file",os.path.join(get_package_share_directory(pkg_name),"props/Collect_Maze_1/model.sdf"),"-entity","Maze","-timeout","240","-x","0","-y","0"],
                    output='screen')
    
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
    
    actions = []
    
    for i,(x,y,z) in enumerate(POINTS_POSITIONS):
        landmine_description = xacro.process_file(xacro_file,mappings={'topic' : 'trigger'+str(i)}).toxml()
        
        node_landmine_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': landmine_description,
        'use_sim_time': True}] # add other parameters here if required
        )
        
        spawn_mine = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=["-topic",f"/point{i}/robot_description","-entity","point"+str(i),"-timeout","240","-x",str(x),"-y",str(y),"-z",str(z)],
                    output='screen')
        actions.append(GroupAction(
            actions=[
                PushRosNamespace('point'+str(i)),
                node_landmine_state_publisher,
                spawn_mine
            ]
        ))

    
    # Run the node
    return LaunchDescription([
        DeclareLaunchArgument(
            "log_level",
            default_value=["debug"],
            description="Logging level",
      ),
        gazebo,
        node_robot_state_publisher,
        *actions,
        spawn_maze,
        fusion
    ])


