from gym_env.spring_creature_generation_test import SpringCreatureGenerationTest
from gym_env.box2d_func import *

import gym

gym.envs.register(
     id='SpringCreatureGenerationTest-v0',
     entry_point='gym_env:SpringCreatureGenerationTest',
     max_episode_steps=1000,
)
