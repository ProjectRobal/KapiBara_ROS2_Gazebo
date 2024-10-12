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
from protorl.learner.dqn import DQNLearner as Learner

from protorl.agents.dqn import DQNAgent as Agent
from protorl.actor.dqn import DQNActor as Actor
from protorl.loops.single import EpisodeLoop
from protorl.utils.network_utils import make_genetic_networks,make_esp_networks,make_dqn_networks
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
    
    env_name = 'gym/Collect-v0'
    # env_name = 'PongNoFrameskip-v4'
    use_prioritization = False
    use_double = False
    use_dueling = False
    use_atari = False
    env = make_env(env_name,sequence_length=4)
    
    n_games = 8000
    bs = 64
    # 0.3, 0.5 works okay for cartpole
    # 0.25, 0.25 doesn't seem to work
    # 0.25, 0.75 doesn't work
    memory = initialize_memory(max_size=50_000,
                               obs_shape=env.observation_space.shape,
                               batch_size=bs,
                               n_actions=env.action_space.n,
                               action_space='discrete',
                               prioritized=use_prioritization,
                               alpha=0.25,
                               beta=0.25
                               )

    policy = EpsilonGreedyPolicy(n_actions=env.action_space.n, eps_dec=1e-4)

    q_eval, q_target = make_dqn_networks(env, use_double=use_double,
                                         use_dueling=use_dueling,
                                         hidden_layers=[4096*8],
                                         use_atari=use_atari)
    dqn_actor = Actor(q_eval, q_target, policy)
    q_eval, q_target = make_dqn_networks(env, use_double=use_double,
                                         use_dueling=use_dueling,
                                         hidden_layers=[4096*8],
                                         use_atari=use_atari)
    dqn_learner = Learner(q_eval, q_target,
                          prioritized=use_prioritization, lr=1e-4)

    agent = Agent(dqn_actor, dqn_learner, prioritized=use_prioritization)
    sample_mode = 'prioritized' if use_prioritization else 'uniform'
    ep_loop = EpisodeLoop(agent, env, memory, sample_mode=sample_mode,
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