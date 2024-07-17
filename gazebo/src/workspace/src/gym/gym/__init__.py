from gymnasium.envs.registration import register

register(
     id="gym/SimpleMaze-v0",
     entry_point="gym.environments.labirynth:Labirynth",
     kwargs={'maze': 'basic'}
)