#!/usr/bin/env python3
import numpy as np
from protorl.agents.ppo import PPOAgent as Agent
from protorl.actor.ppo import PPOActor as Actor
from protorl.learner.ppo import PPOLearner as Learner
from protorl.loops.ppo_episode import EpisodeLoop
from protorl.memory.generic import initialize_memory
from protorl.utils.network_utils import make_ppo_networks
from protorl.policies.discrete import DiscretePolicy
from protorl.wrappers.vec_env import make_vec_envs
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
    # env_name = 'PongNoFrameskip-v4'
    # use_prioritization = False
    
    # env = make_env(env_name,sequence_length=1)
    
    n_games = 200
    bs = 64
    n_threads = 1
    n_epochs = 10
    horizon = 16384
    T = int(horizon // n_threads)
    batch_size = int(T // bs)

    env = make_vec_envs(env_name, n_threads=n_threads, seed=0,sequence_length=1)

    fields = ['states', 'actions', 'rewards', 'states_',
              'mask', 'log_probs']
    state_shape = (T, n_threads, *env.observation_space.shape)
    action_shape = probs_shape = (T, n_threads)
    reward_shape = mask_shape = (T, n_threads)
    vals = [np.zeros(state_shape, dtype=np.float32),
            np.zeros(action_shape, dtype=np.float32),
            np.zeros(reward_shape, dtype=np.float32),
            np.zeros(state_shape, dtype=np.float32),
            np.zeros(mask_shape, dtype=np.float32),
            np.zeros(probs_shape, dtype=np.float32)]

    memory = initialize_memory(max_size=T,
                               obs_shape=env.observation_space.shape,
                               batch_size=bs,
                               n_actions=env.action_space.n,
                               action_space='discrete',
                               n_threads=n_threads,
                               fields=fields,
                               vals=vals
                               )

    policy = DiscretePolicy()

    actor_net, critic_net = make_ppo_networks(env, action_space='discrete')
    actor = Actor(actor_net, critic_net, 'discrete', policy)

    actor_net, critic_net = make_ppo_networks(env, action_space='discrete')
    learner = Learner(actor_net, critic_net, 'discrete', policy)

    agent = Agent(actor, learner)

    ep_loop = EpisodeLoop(agent, env, memory, n_epochs, T, batch_size,
                          n_threads=n_threads, clip_reward=True,
                          extra_functionality=[agent.anneal_policy_clip],filename="/app/models/ppo_simple_maze.csv")

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
