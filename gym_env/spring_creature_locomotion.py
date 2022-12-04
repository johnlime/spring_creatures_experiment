import gym
from gym import spaces
from Box2D import *
import pygame

from math import sin, cos, pi, sqrt
import numpy as np

from gym_env.box2d_func import *

class SpringCreatureLocomotion(gym.Env):
    name = "SpringCreatureLocomotion"
    description = 'SpringCreatureLocomotion'
    count = 800
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 60}

    def __init__(self, morphogen_function = None, render_mode = "human"):
        self.action_space = spaces.Discrete(1) # no action space
        self.observation_space = spaces.Box(low = -np.inf,
                                            high = np.inf,
                                            shape = (2,),
                                            dtype = np.float32)

        self.world = b2World(gravity=(0, 0), doSleep = True)
        dim_x, dim_y = 5, 5
        assert dim_x > 0 and dim_y > 0

        self.base_body = spring_creature_generation(self.world,
                                                    dim_x, dim_y,
                                                    morphogen_function)
        self.go = False
        self.time = 0.0

        # for oscillations
        self.cycle = 100.0
        self.cycle_time = 0.0

        # for reward
        self.prev_pos_x, self.prev_pos_y = 0, 0

        # rendering
        assert render_mode is None or \
            render_mode in self.metadata["render_modes"]
        self.render_mode = render_mode
        self.window = None
        self.clock = None
        self.window_size = 512  # The size of the PyGame window
        self.scale = 2

    def _get_obs(self):
        obs = [self.base_body.position[0], self.base_body.position[1]]
        assert self.observation_space.contains(obs)
        return obs

    def _get_info(self):
        info = {}
        return info

    def reset(self):
        return self._get_obs()

    def step(self, action = 0):
        err_msg = f"{action!r} ({type(action)}) invalid"
        assert self.action_space.contains(action), err_msg

        # oscillation of springs
        self.cycle_time += 1
        if self.cycle_time >= self.cycle:
            self.cycle_time = 0.0

        for joint in self.world.joints:
            if type(joint) == b2PrismaticJoint:
                joint.motorSpeed = 30 * sin(2 * pi * self.cycle_time / self.cycle)

        self.world.Step(TIMESTEP, VEL_ITERS, POS_ITERS)
        obs = self._get_obs()

        # only positive x axis movement is encouraged
        reward = (obs[0] - self.prev_pos_x) ** 3 / (obs[0] - self.prev_pos_x) - \
                 (obs[1] - self.prev_pos_y) ** 2    # y axis movement should be avoided


        done = True
        info = {}
        return obs, reward, done, info
        # return obs, reward, done, False, info # there is an optional truncated segment

    def render(self):
        spring_creature_render(self)

    def close(self):
        if self.window is not None:
            pygame.display.quit()
            pygame.quit()
