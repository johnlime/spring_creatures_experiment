from gym_env import SpringCreatureGenerationTest
import gym

from neat_gym import read_file, eval_net
import time


# it will be useful to have this when we implement the CPPN
# for not I commented out because I cannot test it



# # Load genome and configuration from pickled file
# net, env_name, record_dir, seed, nodisplay, csvfilename = \
#         read_file(allow_record=True, allow_seed=True)

# # Run the network on the environment
# eval_net(net,
#          gym.make(SpringCreatureGenerationTest),
#          render=(not nodisplay),
#          record_dir=record_dir,
#          seed=seed,
#          csvfilename=csvfilename,
#          report=True)



env = gym.make('CartPole-v1')

# env = SpringCreatureGenerationTest()

policy = lambda obs: 0

obs = env.reset()
for _ in range(80):
        actions = policy(obs)
        obs, reward, done, info = env.step(actions)
        env.render()
        time.sleep(0.05)

env.close()
