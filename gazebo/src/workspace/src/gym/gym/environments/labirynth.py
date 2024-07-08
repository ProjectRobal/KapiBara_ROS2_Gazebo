import numpy as np

import gymnasium as gym
from gymnasium import spaces

from gym.utils.utils_launch import launch_environment

import rclpy
from rclpy.node import Node

from std_srvs.srv import Empty

from gym.agents.step_agnet import KapiBaraStepAgent
from gym.utils.utils_sim import SimulationControl

from threading import Thread

import time

class Labirynth(gym.Env):
    metadata = {"render_modes": ["human"]}
    
    def __init__(self, render_mode=None):

        # Observations are dictionaries with the agent's and the target's location.
        # Each location is encoded as an element of {0, ..., `size`}^2, i.e. MultiDiscrete([size, size]).
        # Inputs are 4 laser sensors distance, quanterion , position
        
        high = np.array([1.0,1.0,1.0,1.0 , 1.0,1.0,1.0,1.0 , np.inf,np.inf,np.inf ],dtype=np.float32)
        low = np.array([0.0,0.0,0.0,0.0 , -1.0,-1.0,-1.0,-1.0 , -np.inf,-np.inf,-np.inf ],dtype=np.float32)
        
        # Position of a target in labirynth
        target_high = np.array([np.inf,np.inf,np.inf],dtype=np.float32)
        
        self.observation_space = spaces.Dict(
            {
                "robot": spaces.Box(low, high, dtype=np.float32),
                "target": spaces.Box(-target_high, target_high, dtype=np.float32),
            }
        )

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
        
        # since target is immovable the position is constant for entire simulation
        self._target_data = np.array([-4.290007,4.220005,0.499995],dtype=np.float32)
        
        rclpy.init()

        self._node=Node("maze_env")
        
        # run spin in seaparated thread
        #self._spin_thread = Thread(target=rclpy.spin,args=(self._node,))
        #self._spin_thread.start()
        # Start an Gazbo using proper launch file
        
        self._env = launch_environment("labirynth")
                
        # create client for step control service for KapiBara robot
        
        self._robot = KapiBaraStepAgent(self._node)
        
        # create client for service to control gazebo environment
        
        self._sim = SimulationControl(self._node)
        
        self._sim.pause()
        self._sim.reset()
    
    def _get_obs(self):
        return {"robot": self._robot_data, "target": self._target_data}

    
    def _get_info(self):
        return {
            "distance": np.linalg.norm(
                self._robot_data[6:9] - self._target_data, ord=1
            )
        }
        
    def reset(self, seed=None, options=None):
        # We need the following line to seed self.np_random
        super().reset(seed=seed)
        
        # reset gazebo
        self._sim.reset()
        
        self._robot_data = np.zeros(self._robot_data.shape,dtype = np.float32)

        observation = self._get_obs()
        info = self._get_info()

        return observation, info
    
    # that doesn't work as it should
    def step(self, action):
        # Map the action (element of {0,1,2,3}) to the direction we walk in
        self._robot.move(action)
        self._sim.unpause()
        # wait couple of steps
        self._robot.wait_for_steps()
        
        self._sim.pause()
        # We use `np.clip` to make sure we don't leave the grid
        
        # An episode is done iff the agent has reached the target
        terminated = np.array_equal(self._robot_data[6:9], self._target_data)
        reward = 1 if terminated else 0  # Binary sparse rewards
        observation = self._get_obs()
        info = self._get_info()
        done = False

        return observation, reward, terminated, done, info
    
    def render(self):
        return None
    
    def close(self):
        self._env.kill()
        self._env.join()
        self._env.close()