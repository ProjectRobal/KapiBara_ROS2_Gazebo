#!/usr/bin/env python3

from gym.environments.labirynth import Labirynth

from random import randint

def main():
    env = Labirynth()
        
    while True:
        # it doesn't work somehow
        pass
        #env.step(0)


if __name__ == "__main__":
    main()
