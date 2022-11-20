# Plan
1. We need to write spring_creature_generation_test.py
2. run_test.py does not work because ```observation_space.contains(obs)``` in the environment; it says we cannot run it because we do not have contain
3. I just copied neat_evolve.py from the levy's github, and I think after we finalize our environment we should modify the config file in run/config/spring_creature_generation_test - we can discuss about this next week through MS team?


## summary
Jay - I did try implementing CPPN algorithms in the code, but it did not work. For now, I will leave it, but as soon as the spring_creature_generation is complete, I will update the tester. For trainer, I think only thing we need to do is to change config file, so let's discuss about this next week at some points. The earlier the better.

## command
1. running trainer would look like (not working for now)
```
python3 run_train.py ../run/config/spring_creature_generation_test
```
2. running tester would look like (not working for now)
```
python3 run_test.py ../run/models/sample.dat
```