#!/usr/bin/env python3

from gym.environments.labirynth import Labirynth

from random import randint

def main():
    env = Labirynth()
        
    while True:
        # it doesn't work somehow
        observation, reward, terminated, done, info = env.step(randint(0,3))
        
        print("Observation: ",observation)
        print("Reward: ",reward)
        print("Done: ",done)
        print("Terminated: ",terminated)
        print("Info: ",info)
        #env.step(0)


if __name__ == "__main__":
    main()
