from gymnasium.envs.registration import register

register(
     id="gym/SimpleMaze-v0",
     entry_point="gym.environments.labirynth:Labirynth",
     kwargs={'maze': 'basic'}
)

register(
     id="gym/PackCollect-v0",
     entry_point="gym.environments.collect:Collect"
)

register(
     id="gym/Parking-v0",
     entry_point="gym.environments.parking:Parking"
)