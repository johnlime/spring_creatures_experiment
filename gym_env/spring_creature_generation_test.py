import gym
from gym import spaces
import pygame

from math import sin, pi, sqrt
import numpy as np
from copy import copy, deepcopy

class SpringCreatureGenerationTest(gym.Env):
    """
    Generate spring creature limbs recursively
    """
    name = "SpringCreatureGenerationTest"
    description = 'g to stop/go'
    count = 800
    def __init__(self):
        self.action_space
        self.observation_space

    def _get_obs(self):
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
        return obs, reward, done, False, info # there is an optional truncated segment

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
        if self.window is not None:
            pygame.display.quit()
            pygame.quit()
