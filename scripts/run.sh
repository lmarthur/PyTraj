# This script runs the program, using an input file that is currently specified in main.py.
#!/bin/bash

# Activate the environment
mamba activate pytraj_env

# Run the program
python ./src/main.py

# Generate the impact plot
python ./src/impact_plot.py

