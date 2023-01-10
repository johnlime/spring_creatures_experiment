import os
import pickle
import neat
import gym
import numpy as np
import configparser

from Box2D import *
from Box2D.examples.framework import (Framework, Keys)
from Box2D.examples.settings import fwSettings
from settings import *
from gym_env.box2d_func import *

RANDOM_GENOME = True

# config file is assumed to live in the same directory as this script.
local_dir = os.path.dirname(__file__)
config_path = os.path.join(local_dir, 'config')

class SpringCreaturePygame (Framework):
    name = "SpringCreaturePygame"
    description = 'g to stop/go'
    count = 800

    def __init__(self, morphogen_function = None):
        Framework.__init__(self)
        self.world.gravity = (0, 0)

        dim_x, dim_y = 5, 5
        assert dim_x > 0 and dim_y > 0

        spring_creature_generation(self.world,
                                   dim_x, dim_y,
                                   morphogen_function)

        self.go = False
        self.time = 0.0

    def Keyboard(self, key):
        if key == Keys.K_g:
            self.go = not self.go

    def Step(self, settings = default_settings):
        Framework.Step(self, settings)
        if self.go and settings.hz > 0.0:
            self.time += 1.0 / settings.hz

        # print(len(self.world.contacts))
        # for body in self.world.bodies:
        #     for fixture in body.fixtures:
        #         print(fixture)

        renderer = self.renderer
        renderer.DrawPoint(renderer.to_screen((0, 0)),
                           4,
                           b2Color(0.9, 0.9, 0.9)
                           )

if __name__ == "__main__":
    """
    Genome generation
    """
    # random or winner genome
    genome = None
    if RANDOM_GENOME == False:
        # load the winner
        with open('winner', 'rb') as f:
            genome = pickle.load(f)
        print('Loaded genome:')
    else:
        config = configparser.ConfigParser()
        config.read(config_path)
        param_dict = {}
        for key in config["DefaultGenome"]:
            param_dict[key] = str(config["DefaultGenome"][key])
        config = neat.DefaultGenome.parse_config(param_dict)
        genome = neat.DefaultGenome(key = 0)
        genome.configure_new(config)
        print('Random genome:')

    # Load the config file
    config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                         neat.DefaultSpeciesSet, neat.DefaultStagnation,
                         config_path)
    print(genome)
    net = neat.nn.FeedForwardNetwork.create(genome, config)

    """
    Run Pygame Testbed
    """
    test = SpringCreaturePygame(net.activate)
    if fwSettings.onlyInit:
        pass
    test.run()
