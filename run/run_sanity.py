from gym_env import SpringCreatureLocomotion#SpringCreatureGenerationTest
import gym

# env = SpringCreatureGenerationTest()
env = SpringCreatureLocomotion()

obs = env.reset()
for _ in range(1000):
        actions = env.action_space.sample()
        obs, reward, done, info = env.step(actions)
        print(obs, reward)
        env.render()

env.close()
