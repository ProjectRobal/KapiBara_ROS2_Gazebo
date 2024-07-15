
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    
    position_x = LaunchConfiguration('x')
    position_y = LaunchConfiguration('y')
    position_z = LaunchConfiguration('z')
    
    rot_roll = LaunchConfiguration('roll')
    rot_pitch = LaunchConfiguration('pitch')
    rot_yaw = LaunchConfiguration('yaw')
    
    position_x_arg = DeclareLaunchArgument(
        'x',
        default_value='0.0'
    )
    
    position_y_arg = DeclareLaunchArgument(
        'y',
        default_value='0.0'
    )
    
    position_z_arg = DeclareLaunchArgument(
        'z',
        default_value='0.0'
    )
    
    position_roll_arg = DeclareLaunchArgument(
        'roll',
        default_value='0.0'
    )
    
    position_pitch_arg = DeclareLaunchArgument(
        'pitch',
        default_value='0.0'
    )
    
    position_yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='0.0'
    )
   
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=["-topic","/robot_description","-entity","kapibara","-timeout","240","-x",position_x,"-y",position_y,"-z",position_z,"-R",rot_roll,"-P",rot_pitch,"-Y",rot_yaw],
                    output='screen')
    
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["motors",'--controller-manager-timeout','240','--ros-args']
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad",'--controller-manager-timeout','240','--ros-args']
    )
    
    ears_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ears_controller",'--controller-manager-timeout','240']
    )

    
    # Run the node
    return LaunchDescription([
        DeclareLaunchArgument(
            "log_level",
            default_value=["debug"],
            description="Logging level",
      ),
        position_x_arg,
        position_y_arg,
        position_z_arg,
        position_roll_arg,
        position_pitch_arg,
        position_yaw_arg,
        spawn,
        diff_drive_spawner,
        joint_broad_spawner,
        ears_controller_spawner
    ])


