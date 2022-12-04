import gym
from gym import spaces
from Box2D import *
import pygame

from math import sin, cos, pi, sqrt
import numpy as np
from copy import deepcopy

from gym_env.box2d_func import *

class SpringCreatureGenerationTest(gym.Env):
    name = "SpringCreatureGenerationTest"
    description = 'SpringCreatureGenerationTest'
    count = 800
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 60}

    def __init__(self, morphogen_function = None, render_mode = "human"):
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

        # rendering
        assert render_mode is None or \
            render_mode in self.metadata["render_modes"]
        self.render_mode = render_mode
        self.window = None
        self.clock = None
        self.window_size = 512  # The size of the PyGame window
        self.scale = 2

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
        self.world.Step(TIMESTEP, VEL_ITERS, POS_ITERS)
        obs = self._get_obs()
        reward = 0
        done = True
        info = {}
        return obs, reward, done, info
        # return obs, reward, done, False, info # there is an optional truncated segment

    def render(self):
        ####
        # Generate window and clock
        ####
        if self.window is None and self.render_mode == "human":
            pygame.init()
            pygame.display.init()
            self.window = pygame.display.set_mode((self.window_size, self.window_size))
        if self.clock is None and self.render_mode == "human":
            self.clock = pygame.time.Clock()

        canvas = pygame.Surface((self.window_size, self.window_size))
        canvas.fill((255, 255, 255))

        ####
        # Render items here
        ####
        for body in self.world.bodies:
            for fixture in body.fixtures:
                if type(fixture.shape) == b2PolygonShape:
                    fixture_vertices = []
                    # rotate + translate vertices
                    for i in range(len(fixture.shape.vertices)):
                        x = fixture.shape.vertices[i][0] * cos(-body.angle) - \
                            fixture.shape.vertices[i][1] * sin(-body.angle) + \
                            body.position[0]

                        y = fixture.shape.vertices[i][0] * sin(body.angle) - \
                            fixture.shape.vertices[i][1] * cos(body.angle) + \
                            body.position[1]

                        x = self.scale * x + self.window_size // 2 # half the window size
                        y = -self.scale * y + self.window_size // 2

                        fixture_vertices.append(tuple((x, y)))

                    pygame.draw.polygon(
                        surface = canvas,
                        color = (0, 255, 150),
                        points = fixture_vertices,
                    )

                elif type(fixture.shape) == b2CircleShape:
                    pygame.draw.circle(
                        surface = canvas,
                        color = (0, 0, 255),
                        center = (self.scale * (body.position[0] + fixture.shape.pos[0]) + \
                                    self.window_size // 2,
                                  - self.scale * (body.position[1] + fixture.shape.pos[1]) + \
                                    self.window_size // 2),
                        radius = self.scale * fixture.shape.radius
                    )
                else:
                    pass

        for joint in self.world.joints:
            if type(joint) == b2DistanceJoint:
                anchor_a = (self.scale * joint.anchorA[0] + \
                                self.window_size // 2,
                            - self.scale * joint.anchorA[1] + \
                                self.window_size // 2)
                anchor_b = (self.scale * joint.anchorB[0] + \
                                self.window_size // 2,
                            - self.scale * joint.anchorB[1] + \
                                self.window_size // 2)
                pygame.draw.line(
                    surface = canvas,
                    color = (255, 0, 0),
                    start_pos = anchor_a,
                    end_pos = anchor_b
                )


        ####
        # Display in a window or return an rgb array
        ####
        if self.render_mode == "human":
            # The following line copies our drawings from `canvas` to the visible window
            self.window.blit(canvas, canvas.get_rect())
            pygame.event.pump()
            pygame.display.update()

            # We need to ensure that human-rendering occurs at the predefined framerate.
            # The following line will automatically add a delay to keep the framerate stable.
            self.clock.tick(self.metadata["render_fps"])
        else:  # if self.render_mode == "rgb_array":
            return np.transpose(
                np.array(pygame.surfarray.pixels3d(canvas)), axes=(1, 0, 2)
            )

    def close(self):
        if self.window is not None:
            pygame.display.quit()
            pygame.quit()
