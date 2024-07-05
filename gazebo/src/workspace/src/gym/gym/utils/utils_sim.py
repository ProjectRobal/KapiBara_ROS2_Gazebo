'''

    A module with class for simulation control.

'''

import numpy as np

import rclpy
from rclpy.node import Node

from std_srvs.srv import Empty

class SimulationControl:
    def __init__(self,parent_node:Node):
        self._node = parent_node
        
        self._reset_env_srv = self._node.create_client(Empty,"/reset_simulation")
        
        # wait 60 seconds for service ready
        if not self._reset_env_srv.wait_for_service(60):
            raise TimeoutError("Cannot connect to service: /reset_simulation")
        
        self._pause_env_srv = self._node.create_client(Empty,"/pause_physics")
        
        # wait 60 seconds for service ready
        if not self._pause_env_srv.wait_for_service(60):
            raise TimeoutError("Cannot connect to service: /pause_physics")
        
        self._unpause_env_srv = self._node.create_client(Empty,"/unpause_physics")
        
        # wait 60 seconds for service ready
        if not self._unpause_env_srv.wait_for_service(60):
            raise TimeoutError("Cannot connect to service: /unpause_physics")

    def reset(self):
        '''
            Reset simulation
        '''    
        self._reset_env_srv.call(Empty.Request())
        
    def pause(self):
        '''
            Pause simulation
        '''    
        self._pause_env_srv.call(Empty.Request())
        
    def unpause(self):
        '''
            Unpause simulation
        '''    
        self._unpause_env_srv.call(Empty.Request())
