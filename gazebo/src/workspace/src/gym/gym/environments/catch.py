from typing import Literal
import numpy as np

import gymnasium as gym
from gymnasium import spaces

from gym.utils.utils_launch import launch_environment

import rclpy
from rclpy.node import Node

from gazebo_msgs.msg import ContactsState

from gym.agents.step_agnet import KapiBaraStepAgent
from gym.utils.utils_sim import SimulationControl

from sensor_msgs.msg import Imu,LaserScan
from geometry_msgs.msg import Twist

from threading import Thread

from copy import copy

import time

class Catch(gym.Env):
    metadata = {"render_modes": ["human"]}
    
    def mouse_collison_callback(self,contacts:ContactsState):
        for contact in contacts.states:
            if contact.collision1_name.find("kapibara") > -1:
                self._mouse_has_been_catched = True
                return
            if contact.collision2_name.find("kapibara") > -1:
                self._mouse_has_been_catched = True
                return
            
    def mouse_tof_callback(self,tof_msg:LaserScan):
        range = min(tof_msg.ranges)
        
        if range > tof_msg.range_max:
            range = tof_msg.range_max
        
        self._mouse_distance = range
        
        self._node.get_logger().debug(f"Got range id: {id}")
            
    
    
    def __init__(self, render_mode=None,sequence_length=1):

        # Observations are dictionaries with the agent's and the target's location.
        # Each location is encoded as an element of {0, ..., `size`}^2, i.e. MultiDiscrete([size, size]).
        # Inputs are 4 laser sensors distance, quanterion
        
        high = np.array([1.0,1.0,1.0,1.0 , 1.0,1.0,1.0,1.0 , *([1.0]*40*30*3) ]*sequence_length,dtype=np.float32)
        low = np.array([0.0,0.0,0.0,0.0 , -1.0,-1.0,-1.0,-1.0 , *([0.0]*40*30*3)]*sequence_length,dtype=np.float32)
        
        self._mouse_has_been_catched = False       
        self._mouse_distance = 10 
        
        self._max_angular_speed = 2.0
        self._max_linear_speed = 1.0
                
        self.observation_space = spaces.Box(low, high, dtype=np.float32)
        # We have 4 actions, corresponding to "right", "up", "left", "down"
        self.action_space = spaces.Discrete(4)

        """
        The following dictionary maps abstract actions from `self.action_space` to
        the direction we will walk in if that action is taken.
        I.e. 0 corresponds to "right", 1 to "up" etc.
        """
        self._action_to_direction = {
            0: np.array([1, 0]),
            1: np.array([0, 1]),
            2: np.array([-1, 0]),
            3: np.array([0, -1]),
        }

        self.render_mode = render_mode
        
        self._robot_data = np.zeros(high.shape,dtype = np.float32)
        self._last_robot_data = self._robot_data
        
        rclpy.init()

        self._node=Node("maze_env")

        self._mouse_contact_topic = self._node.create_subscription(ContactsState,"/mouse/collision",self.mouse_collison_callback,10)
        self._mouse_tof_sensor = self._node.create_subscription(LaserScan,"/mouse/front",self.mouse_tof_callback,10)
        
        self._mouse_twist_output = self._node.create_publisher(Twist, "/mouse/mouse_motors/cmd_vel_unstamped", 10)
        
        # Start an Gazbo using proper launch file
        self._env = launch_environment("mouse.trap.one")
        self._env.start()
        
        # create client for step control service for KapiBara robot
        self._robot = KapiBaraStepAgent(self._node,position=[-1.0,0.0,0.0],rotation=[0.0,0.0,0],reload_agent=False,use_camera=True)
        # create client for service to control gazebo environment
        
        self._sim = SimulationControl(self._node)
        
        
        self._sim.pause()
        self._sim.reset()
        
    def move_mouse(self,linear,angular):
        
        twist = Twist()
        
        linear = np.clip(int(linear),-1,1)
        angular = np.clip(int(angular),-1,1)
                
        twist.angular.z = angular * self._max_angular_speed
        twist.linear.x = linear * self._max_linear_speed
        
        self._mouse_twist_output.publish(twist)
    
    def _get_obs(self):
        return self._robot_data

    def _get_info(self):
        # get information about distance between mouse and kapibara
        return {}
        
    def reset(self, seed=None, options=None):
        # We need the following line to seed self.np_random
        super().reset(seed=seed)
        
        # reset gazebo
        self._node.get_logger().info("Resetting simulation")
        self._sim.reset()
        self._node.get_logger().info("Resetting robot")
        self._robot.reset_agent()
        
        self._robot_data = np.zeros(self._robot_data.shape,dtype = np.float32)
        
        self._mouse_has_been_catched = False       
        self._mouse_distance = 10

        observation = self._get_obs()
        info = self._get_info()

        return observation, info
    
    def append_observations(self,observation):
        self._robot_data = np.roll(self._robot_data,-len(observation))    
        self._robot_data[-len(observation):] = observation[:]
    
    # that doesn't work as it should
    def step(self, action):
        # Map the action (element of {0,1,2,3}) to the direction we walk in
        
        
        self._robot.move(action)
        
        if self._mouse_distance < 0.5:
            self.move_mouse(0.0,-1.0)
        else:
            self.move_mouse(1.0,0.0)
        
        self._sim.unpause()
        # wait couple of steps
        self._robot.wait_for_steps()
        
        self._sim.pause()
        
        observation = self._robot.get_observations()
        
        frame = self._robot.get_camera_frame()[-1]
        
        # self._robot_data[:8] = observation[:8]
        # self._robot_data[8:] = frame[:]
        
        self.append_observations(np.concatenate((observation[:8],frame)))
        
        #print(self._robot_data.shape)
        # We use `np.clip` to make sure we don't leave the grid
        
        # An episode is done iff the agent has reached the target
        terminated = False
        done = False
        
        # We want agent to do as few steps as possible
        #
        # Each step will give reward -0.04
        # It will get 1.0 for reaching it is goal, finding all points
        # It will get 0.5 for finding one point
        # When robot hits the wall it will get -0.5 reward and environment is terminated
        
        # default reward for every step
        reward = 0.0
        
        # check sensor data
        # id = 0
        # for distance in observation[0:4]:
        #     id+=1
        #     if distance < 0.1:
        #         reward = -0.5
        #         self._node.get_logger().info(f"Robot hits the wall, terminated!, sensor id: {id}")
        #         terminated = True
        #         break
        
        # if self._robot_data[11] < 0.1:
        #     reward = -0.5
        #     self._node.get_logger().info("Robot hits the wall, terminated!, back sensor!")
        #     terminated = True
                
        info = self._get_info()
            
                
        return self._get_obs(), reward, terminated, done, info
    
    def render(self):
        return None
    
    def close(self):
        self._env.kill()
        self._env.join()
        self._env.close()