# Spring Creatures
An virtual creature environment trained with NEAT made for CS/CSYS 352: Evolutionary Computation by Prof. Nick Cheney at UVM.

Uses NEAT-Python library.

# Environment Settings
Create Anaconda environment.
```
conda env create -f conda_env/mac-11-6-1.yml
conda activate spring_creatures
```

Add working directory to `PYTHONPATH`.
```
export PYTHONPATH=$PWD
```

Run sanity check to see if the Spring Creatures environment runs.
```
python run/run_sanity.py
```

Visualize and play with the Spring Creatures environment using Pygame.
```
python run/run_pygame.py
```

# Run NEAT on Spring Creatures
Run evolutionary algorithm on the Spring Creatures environment (overwrites `winner` file).
```
python run/run_train.py
```

Run and visualize the final solution.
```
python run/run_test.py
```

# Video Demonstration
[Video](https://youtube.com/shorts/T8a8Qa22M5Q)

# Collaborators
NEAT-Python implementation \
[marszzibros](https://github.com/marszzibros)
