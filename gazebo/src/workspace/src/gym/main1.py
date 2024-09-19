#!/usr/bin/env python3
from protorl.agents.ddpg import DDPGAgent as Agent
from protorl.actor.ddpg import DDPGActor as Actor
from protorl.learner.ddpg import DDPGLearner as Learner
from protorl.loops.single import EpisodeLoop
from protorl.memory.generic import initialize_memory
from protorl.utils.network_utils import make_ddpg_networks
from protorl.policies.noisy_deterministic import NoisyDeterministicPolicy
from protorl.wrappers.common import make_env



import gym
import gymnasium

from random import randint

import os

def main():
    
    if not os.path.exists("./models"):
        os.mkdir("./models")
    
    env_name = 'gym/Maze-v0'
    # env_name = 'PongNoFrameskip-v4'
    use_prioritization = False
    use_double = False
    use_dueling = False
    use_atari = False
    env = make_env(env_name,sequence_length=1)
    
    n_games = 200
    bs = 64
    # 0.3, 0.5 works okay for cartpole
    # 0.25, 0.25 doesn't seem to work
    # 0.25, 0.75 doesn't work
    env = make_env(env_name, continuous=True)

    memory = initialize_memory(max_size=100_000,
                               obs_shape=env.observation_space.shape,
                               batch_size=bs,
                               n_actions=env.action_space.shape[0],
                               action_space='continuous',
                               )
    policy = NoisyDeterministicPolicy(n_actions=env.action_space.shape[0],
                                      min_action=env.action_space.low[0],
                                      max_action=env.action_space.high[0])

    actor, critic, target_actor, target_critic = make_ddpg_networks(env)

    ddpg_actor = Actor(actor, critic, target_actor, target_critic, policy)

    actor, critic, target_actor, target_critic = make_ddpg_networks(env)

    ddpg_learner = Learner(actor, critic, target_actor, target_critic)

    agent = Agent(ddpg_actor, ddpg_learner)
        
    ep_loop = EpisodeLoop(agent, env, memory,
                          prioritized=use_prioritization,
                          load_checkpoint=False,filename="/app/models/dqn_simple_maze.csv")
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
