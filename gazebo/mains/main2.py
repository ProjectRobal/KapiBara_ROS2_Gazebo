#!/usr/bin/env python3
from protorl.policies.discrete import DiscretePolicy
# from protorl.utils.network_utils import make_dqn_networks
from protorl.wrappers.common import make_env
from protorl.learner.genetic import GeneticLearner as Learner

from protorl.agents.genetic import GeneticAgent as Agent
from protorl.actor.genetic import GeneticActor as Actor
from protorl.loops.simple import EpisodeLoop
from protorl.utils.network_utils import make_genetic_networks
# from protorl.utils.initializers import he



import gym
import gymnasium

from random import randint

import os

def main():
    
    if not os.path.exists("./models"):
        os.mkdir("./models")
        
    if not os.path.exists("/app/models"):
        os.mkdir("/app/models")
    
    env_name = 'gym/Maze-v0'
   
    env = make_env(env_name,sequence_length=1)
    
    n_games = 200
    bs = 64
    # 0.3, 0.5 works okay for cartpole
    # 0.25, 0.25 doesn't seem to work
    # 0.25, 0.75 doesn't work

    policy = DiscretePolicy()

    networks = make_genetic_networks(env,count=10)
    
    genetic_actor = Actor(networks, policy)
    
    genetic_learner = Learner(0.25,5)

    agent = Agent(genetic_actor, genetic_learner)
    
    ep_loop = EpisodeLoop(agent, env,load_checkpoint=False,filename="/app/models/genetic_simple_maze.csv")
    scores, steps_array = ep_loop.run(n_games)
            

# def main():
#     env = gymnasium.make("gym/SimpleMaze-v0")
    
#     env.reset()
        
#     while True:
                
#         observation, reward, terminated, done, info = env.step(env.action_space.sample())
        
#         print("Observation: ",observation)
#         print("Reward: ",reward)
#         print("Done: ",done)
#         print("Terminated: ",terminated)
#         print("Info: ",info)
        
#         if terminated:
#             env.reset()


if __name__ == "__main__":
    main()
