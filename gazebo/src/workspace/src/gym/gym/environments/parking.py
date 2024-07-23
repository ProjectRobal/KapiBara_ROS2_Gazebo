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

from threading import Thread

from copy import copy

from timeit import default_timer as timer

class Parking(gym.Env):
    metadata = {"render_modes": ["human"]}
    
    stages = ["searching","parking"]
        
    def point_callback(self,contacts:ContactsState):
        for contact in contacts.states:
            if contact.collision1_name.find("kapibara") > -1:
                self._point_id_triggered = contact.collision2_name.split("::")[0]
                return
            if contact.collision2_name.find("kapibara") > -1:
                self._point_id_triggered = contact.collision1_name.split("::")[0]
                return
    
    def __init__(self, render_mode=None):

        # Observations are dictionaries with the agent's and the target's location.
        # Each location is encoded as an element of {0, ..., `size`}^2, i.e. MultiDiscrete([size, size]).
        # Inputs are 4 laser sensors distance, quanterion
        
        high = np.array([1.0,1.0,1.0,1.0 , 1.0,1.0,1.0,1.0  ],dtype=np.float32)
        low = np.array([0.0,0.0,0.0,0.0 , -1.0,-1.0,-1.0,-1.0 ],dtype=np.float32)
        
        
        print("High shape: ",high.shape)
        
        self._stall_timer = timer()
        
        self._point_id_triggered = ""
                
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
        
        self._stage_number = 0
        
        rclpy.init()

        self._node=Node("maze_env")
        
        # run spin in seaparated thread
        #self._spin_thread = Thread(target=rclpy.spin,args=(self._node,))
        #self._spin_thread.start()
        # Start an Gazbo using proper launch file
        self._env = launch_environment("parking.one")
        self._env.start()
                
        self._point_topics = {}
        
        for i in range(2):
            self._node.get_logger().info(f"Created subscription for point{i}")
            self._point_topics["point"+str(i)]=self._node.create_subscription(ContactsState,"/trigger"+str(i),self.point_callback,10)
        
                
        # create client for step control service for KapiBara robot
        self._robot = KapiBaraStepAgent(self._node,position=[-0.2,0.0,0.0],rotation=[0.0,0.0,0],reload_agent=False,use_camera=False,max_linear_speed=0.25)
        # create client for service to control gazebo environment
        
        self._sim = SimulationControl(self._node)
        
        self._sim.pause()
        self._sim.reset()
    
    def _get_obs(self):
        return self._robot_data

    def _get_info(self):
        
        # get information abut stage:
        return {"stage":self.stages[self._stage_number]}
        
    def reset(self, seed=None, options=None):
        # We need the following line to seed self.np_random
        super().reset(seed=seed)
        
        # reset gazebo
        self._node.get_logger().info("Resetting simulation")
        self._sim.reset()
        self._node.get_logger().info("Resetting robot")
        self._robot.reset_agent()
        
        self._robot_data = np.zeros(self._robot_data.shape,dtype = np.float32)
        
        self._stage_number = 0
        self._point_id_triggered = ""
        
        del self._point_topics

        observation = self._get_obs()
        info = self._get_info()
        
        self._stall_timer = timer()

        return observation, info
    
    # that doesn't work as it should
    def step(self, action):
        # Map the action (element of {0,1,2,3}) to the direction we walk in
        
        if not hasattr(self,"_point_topics"):
            self._point_topics = {}
            
            for i in range(2):
                self._node.get_logger().info(f"Created subscription for point{i}")
                self._point_topics["point"+str(i)]=self._node.create_subscription(ContactsState,"/trigger"+str(i),self.point_callback,10)
                
            rclpy.spin_once(self._node)
            self._point_id_triggered = ""
            
            
        
        
        self._robot.move(action)
        self._sim.unpause()
        # wait couple of steps
        self._robot.wait_for_steps()
        
        self._sim.pause()
        
        observation = self._robot.get_observations()
                
        self._robot_data[:8] = observation[:8]
        
        #print(self._robot_data.shape)
        # We use `np.clip` to make sure we don't leave the grid
        
        # An episode is done iff the agent has reached the target
        terminated = False
        done = False
        
        # We want agent to do as few steps as possible
        #
        # Each step will give reward -0.04
        # Robot should stick to a wall so it will get -0.25 for moving away from it and 
        # It will get 1.0 for find proper parking spot
        # It will get -1.0 for hitting wall and environment is terminated
        # When robot park properly simulation finish and robot recive 1.0
        
                

        # if self._robot_data[11] < 0.1:
        #     reward = -0.5
        #     self._node.get_logger().info("Robot hits the wall, terminated!, back sensor!")
        #     terminated = True
                
        info = self._get_info()
        
        if self._stage_number == 0:
            reward = -0.04
            
            if min(observation[2],observation[3]) > 0.4:
                reward = -0.25
                
            if len(self._point_id_triggered) > 0 :
                reward = 1.0
                self._node.get_logger().info("Robot found a proper parking spot! "+self._point_id_triggered)
                
                #self._sim.remove_entity(self._point_id_triggered)
                
                #del self._point_topics[self._point_id_triggered]
                            
                self._point_id_triggered = ""
                
                self._stage_number = 1
                
            id = 0
            for distance in observation[0:4]:
                id+=1
                if distance < 0.1:
                    reward = -1.0
                    self._node.get_logger().info(f"Robot hits the wall, terminated!, sensor id: {id}")
                    terminated = True
                    break
        else:
            reward = -0.25
            # robot parked properly!
            dist = abs(observation[2] - observation[3])
            self._node.get_logger().info("Robot parking spot size: "+str(dist))
            
            if dist <= 0.1 and observation[2]<0.3 and observation[3]<0.3:
                reward = 1.0
                self._node.get_logger().info("Robot has parked properly!")
                done = True
                
            # if len(self._point_id_triggered) == 0 :
            #     reward = -1.0
            #     terminated = True
            #     self._node.get_logger().info("Robot has escaped from parking!")
                
            id = 0
            for distance in observation[0:2]:
                id+=1
                if distance < 0.1:
                    reward = -1.0
                    self._node.get_logger().info(f"Robot hits the wall, terminated!, sensor id: {id}")
                    terminated = True
                    break
                
        # check sensor data
        
        
        if observation[11] < 0.1:
            reward = - 1.0
            self._node.get_logger().info("Robot hits the wall, terminated!, back sensor!")
            terminated = True
            
        if timer() - self._stall_timer > 60*2:
            terminated =  True
            reward = -2.0
            self._stall_timer = timer()
            
        self._node.get_logger().info("Reward: "+str(reward))
        self._node.get_logger().info("Observations: "+str(observation))
         
                
        return self._get_obs(), reward, terminated, done, info
    
    def render(self):
        return None
    
    def close(self):
        self._env.kill()
        self._env.join()
        self._env.close()