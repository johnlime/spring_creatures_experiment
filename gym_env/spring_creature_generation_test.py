import gym
from gym import spaces
from Box2D import *
import pygame

from math import sin, pi, sqrt
import numpy as np
from copy import copy, deepcopy

from gym_env.box2d_func import *

class SpringCreatureGenerationTest(gym.Env):
    name = "SpringCreatureGenerationTest"
    description = 'SpringCreatureGenerationTest'
    count = 800

    def __init__(self, morphogen_function = None):
        self.action_space = spaces.Discrete(1) # no action space
        self.observation_space = spaces.Box(low = -np.inf,
                                            high = np.inf,
                                            shape = (1,),
                                            dtype = np.float32)

        self.world = b2World(gravity=(0, 0), doSleep = True)
        dim_x, dim_y = 5, 5
        assert dim_x > 0 and dim_y > 0

        spring_creature_generation(self.world,
                                   dim_x, dim_y,
                                   morphogen_function)

        self.go = False
        self.time = 0.0

    def _get_obs(self):
        obs = self.observation_space.sample() # this will be changed later

        assert self.observation_space.contains(obs)
        return obs

    def _get_info(self):
        info = {}
        return info

    def reset(self):
        return self._get_obs(), self._get_info()

    def step(self, action = 0):
        assert self.action_space.contains(action)
        obs = self._get_obs()
        reward = 0
        done = True
        info = {}
        return obs, reward, done, info
        # return obs, reward, done, False, info # there is an optional truncated segment

    def render(self):
        if self.render_mode == "rgb_array":
            return self._render_frame()

    def _render_frame(self):
        if self.window is None and self.render_mode == "human":
            pygame.init()
            pygame.display.init()
            self.window = pygame.display.set_mode((self.window_size, self.window_size))
        if self.clock is None and self.render_mode == "human":
            self.clock = pygame.time.Clock()

        canvas = pygame.Surface((self.window_size, self.window_size))
        canvas.fill((255, 255, 255))

    def close(self):
        try:
            if self.window is not None:
                pygame.display.quit()
                pygame.quit()
        except:
            pass
