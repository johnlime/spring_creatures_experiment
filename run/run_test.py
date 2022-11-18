from gym_env import SpringCreatureGenerationTest
import gym

from neat_gym import read_file, eval_net

# Load genome and configuration from pickled file
net, env_name, record_dir, seed, nodisplay, csvfilename = \
        read_file(allow_record=True, allow_seed=True)

# Run the network on the environment
eval_net(net,
         gym.make(SpringCreatureGenerationTest),
         render=(not nodisplay),
         record_dir=record_dir,
         seed=seed,
         csvfilename=csvfilename,
         report=True)