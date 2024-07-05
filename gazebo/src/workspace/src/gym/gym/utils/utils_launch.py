from launch.launch_service import LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from multiprocessing import Process

def launch(launch_path:str)->Process:
    
    service = LaunchService()
    
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_path]))
    
    service.include_launch_description(description)
    
    process = Process(target=service.run)
    process.start()
    
    return process
    

