from launch.launch_service import LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

from multiprocessing import Process

import os

def launch(launch_path:str)->Process:
    
    service = LaunchService()
    
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_path]))
    
    service.include_launch_description(description)
    
    process = Process(target=service.run)
    process.start()
    
    return process
    

def launch_environment(env_name:str)->Process:
    
    launch_file = os.path.join(get_package_share_directory("gym"),"launch/launch."+env_name+".py")
    
    return launch(launch_file)

def launch_other(launch_filename:str)->Process:
    
    launch_file = os.path.join(get_package_share_directory("gym"),"launch",launch_filename+".py")
    
    return launch(launch_file)