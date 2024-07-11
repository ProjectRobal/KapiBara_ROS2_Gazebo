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
        
        self._reset_world_srv = self._node.create_client(Empty,"/reset_world")
        
        # wait 60 seconds for service ready
        if not self._reset_world_srv.wait_for_service(60):
            raise TimeoutError("Cannot connect to service: /reset_world")
        
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
        self.pause()
        
        # future = self._reset_env_srv.call_async(Empty.Request())
        
        # while rclpy.ok():
        #     rclpy.spin_once(self._node)
        #     if future.done():
        #         break
            
        future = self._reset_world_srv.call_async(Empty.Request())
        
        while rclpy.ok():
            rclpy.spin_once(self._node)
            if future.done():
                break
            
        self.unpause()
        
    def pause(self):
        '''
            Pause simulation
        '''    
        future = self._pause_env_srv.call_async(Empty.Request())
        
        while rclpy.ok():
            rclpy.spin_once(self._node)
            if future.done():
                break
        
    def unpause(self):
        '''
            Unpause simulation
        '''    
        future = self._unpause_env_srv.call_async(Empty.Request())
        
        while rclpy.ok():
            rclpy.spin_once(self._node)
            if future.done():
                break
