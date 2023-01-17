from gym_env import SpringCreatureLocomotion
from gym_env import SpringCreatureGenerationTest
import gym

# env = SpringCreatureGenerationTest()
env = SpringCreatureLocomotion()

obs = env.reset()
cumul_reward = 0
for _ in range(500):
    actions = env.action_space.sample()
    obs, reward, done, info = env.step(actions)
    print(obs, reward)
    cumul_reward += reward
    env.render()
print(cumul_reward)

env.close()
