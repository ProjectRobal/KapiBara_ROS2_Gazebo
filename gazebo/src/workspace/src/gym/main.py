#!/usr/bin/env python3

# from protorl.agents.dqn import DQNAgent as Agent
# from protorl.actor.dqn import DQNActor as Actor
# from protorl.learner.dqn import DQNLearner as Learner
# from protorl.loops.single import EpisodeLoop
from protorl.policies.epsilon_greedy import EpsilonGreedyPolicy
from protorl.policies.discrete import DiscretePolicy
# from protorl.utils.network_utils import make_dqn_networks
from protorl.wrappers.common import make_env
from protorl.memory.generic import initialize_memory
from protorl.learner.esp import ESPLearner as Learner

from protorl.agents.esp import ESPAgent as Agent
from protorl.actor.esp import ESPActor as Actor
from protorl.loops.simple import EpisodeLoop
from protorl.utils.network_utils import make_genetic_networks,make_esp_networks
from protorl.utils.initializers import he



import gym
import gymnasium

from random import randint

import os

def main():
    
    if not os.path.exists("./models"):
        os.mkdir("./models")
    
    env_name = 'gym/Maze-v0'
    # env_name = 'PongNoFrameskip-v4'
    use_prioritization = True
    use_double = False
    use_dueling = False
    use_atari = False
    env = make_env(env_name,sequence_length=4)
    
    n_games = 1500
    bs = 64
    # 0.3, 0.5 works okay for cartpole
    # 0.25, 0.25 doesn't seem to work
    # 0.25, 0.75 doesn't work

    policy = DiscretePolicy()

    base,networks = make_esp_networks(env,count=10,hidden_layers=[4096,4096])
        
    actor = Actor(base,networks, policy)
    
    actor.shuttle()
    
    actor.reset()
        
    learner = Learner(mutation_probability=0.1,members_to_keep=4)

    agent = Agent(actor, learner,initializer=he)
    
    sample_mode = 'prioritized' if use_prioritization else 'uniform'
    ep_loop = EpisodeLoop(agent, env, sample_mode=sample_mode,
                          prioritized=use_prioritization,load_checkpoint = False)
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
