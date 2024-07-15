#!/usr/bin/env python3

import gym
from gym.environments.labirynth import Labirynth

import gymnasium

from random import randint

def main():
    env = gymnasium.make("gym/SimpleMaze-v0")
    
    env.reset()
        
    while True:
                
        observation, reward, terminated, done, info = env.step(env.action_space.sample())
        
        print("Observation: ",observation)
        print("Reward: ",reward)
        print("Done: ",done)
        print("Terminated: ",terminated)
        print("Info: ",info)
        
        if terminated:
            env.reset()


if __name__ == "__main__":
    main()
