import numpy as np

import gymnasium as gym
from gymnasium import spaces

from gym.utils.utils_launch import launch_environment


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
        
        # Start an Gazbo using proper launch file
        
        self._env = launch_environment("labirynth")

    
    def _get_obs(self):
        return {"agent": self._agent_location, "target": self._target_location}

    
    def _get_info(self):
        return {
            "distance": np.linalg.norm(
                self._agent_location - self._target_location, ord=1
            )
        }
        
    def reset(self, seed=None, options=None):
        # We need the following line to seed self.np_random
        super().reset(seed=seed)

        # Choose the agent's location uniformly at random
        self._agent_location = self.np_random.integers(0, self.size, size=2, dtype=int)

        # We will sample the target's location randomly until it does not coincide with the agent's location
        self._target_location = self._agent_location
        while np.array_equal(self._target_location, self._agent_location):
            self._target_location = self.np_random.integers(
                0, self.size, size=2, dtype=int
            )

        observation = self._get_obs()
        info = self._get_info()

        return observation, info
    
    def step(self, action):
        # Map the action (element of {0,1,2,3}) to the direction we walk in
        direction = self._action_to_direction[action]
        # We use `np.clip` to make sure we don't leave the grid
        self._agent_location = np.clip(
            self._agent_location + direction, 0, self.size - 1
        )
        # An episode is done iff the agent has reached the target
        terminated = np.array_equal(self._agent_location, self._target_location)
        reward = 1 if terminated else 0  # Binary sparse rewards
        observation = self._get_obs()
        info = self._get_info()

        return observation, reward, terminated, False, info
    
    def render(self):
        return None
    
    def close(self):
        pass