import multiprocessing
import os
import pickle

import neat
import numpy as np

import gym

from gym_env import SpringCreatureLocomotion
from gym_env import SpringCreatureGenerationTest

import time

runs_per_net = 2
# def cppn 
# Use the NN network phenotype and the discrete actuator force function.
def eval_genome(genome, config):
    genome_return = genome
    net = neat.nn.FeedForwardNetwork.create(genome, config)

    fitnesses = []

    for runs in range(runs_per_net):
        env = SpringCreatureLocomotion(net.activate)
        #env = gym.make("CartPole-v1")
        obs = env.reset()
        fitness = 0.0
        for i in range(500):
            #action = np.argmax(net.activate(obs))#env.action_space.sample())
            obs, reward, done, info = env.step()
            fitness += reward
        fitnesses.append(fitness)
    return np.mean(fitnesses)



def eval_genomes(genomes, config):
    for genome_id, genome in genomes:
        genome.fitness = eval_genome(genome, config)


def run():
    # Load the config file, which is assumed to live in
    # the same directory as this script.
    local_dir = os.path.dirname(__file__)
    config_path = os.path.join(local_dir, 'config')
    config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                         neat.DefaultSpeciesSet, neat.DefaultStagnation,
                         config_path)

    pop = neat.Population(config)
    stats = neat.StatisticsReporter()
    pop.add_reporter(stats)
    pop.add_reporter(neat.StdOutReporter(True))

    pe = neat.ParallelEvaluator(multiprocessing.cpu_count(), eval_genome)
    winner = pop.run(pe.evaluate)

    # Save the winner.
    with open('winner', 'wb') as f:
        pickle.dump(winner, f)

    print(winner)




if __name__ == '__main__':
    run()