from gym_env import SpringCreatureLocomotion
from gym_env import SpringCreatureGenerationTest#SpringCreatureGenerationTest
import gym

#env = SpringCreatureGenerationTest()
env = SpringCreatureLocomotion()

obs = env.reset()
for _ in range(1000):
        actions = net.activate(obs)
        obs, reward, done, info = env.step(actions)
        print(obs, reward)
        env.render()

env.close()
