from gym_env import SpringCreatureGenerationTest
import gym

env = SpringCreatureGenerationTest()

obs = env.reset()
for _ in range(80):
        actions = env.action_space.sample()
        obs, reward, done, info = env.step(actions)
        # env.render()

env.close()
