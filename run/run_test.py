import os
import pickle
import neat
import gym
import numpy as np
import argparse
import configparser

from gym_env import SpringCreatureLocomotion
from gym_env import SpringCreatureGenerationTest#SpringCreatureGenerationTest

# config file is assumed to live in the same directory as this script.
local_dir = os.path.dirname(__file__)
config_path = os.path.join(local_dir, 'config')

def test(genome):
    # Load the config file
    config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                         neat.DefaultSpeciesSet, neat.DefaultStagnation,
                         config_path)
    print(genome)
    net = neat.nn.FeedForwardNetwork.create(genome, config)
    # print(net.node_evals)

    #env = gym.make("CartPole-v1")
    env = SpringCreatureLocomotion(net.activate)
    observation = env.reset()

    for _ in range(500):
        observation, reward, done, info = env.step()
        env.render()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-r', '--random', action = 'store_true')
    arg = parser.parse_args()

    # random or winner genome
    c = None
    if arg.random == False:
        # load the winner
        with open('winner', 'rb') as f:
            c = pickle.load(f)
        print('Loaded genome:')
    else:
        config = configparser.ConfigParser()
        config.read(config_path)
        param_dict = {}
        for key in config["DefaultGenome"]:
            param_dict[key] = str(config["DefaultGenome"][key])
        config = neat.DefaultGenome.parse_config(param_dict)
        c = neat.DefaultGenome(key = 0)
        c.configure_new(config)
        print('Random genome:')

    test(c)
