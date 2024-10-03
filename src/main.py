import sys
import os
from ctypes import *

# Specify the input file name (without the extension)
config_file = "run_0"

# Check for the existence of the input file
config_path = f"./input/{config_file}.toml"
if not os.path.isfile(config_path):
    print(f"Error: The input file {config_file}.toml does not exist.")
    sys.exit()

# Check for the existence of the output directory
if not os.path.isdir(f"./output/{config_file}"):
    # Create the output directory if it does not exist
    os.makedirs(f"./output/{config_file}")
    sys.exit()

# Copy the input file to the output directory
os.system(f"cp {config_path} ./output/{config_file}")

# Import the necessary functions from the Python library
sys.path.append('.')
from src.pylib import *
so_file = "./build/libPyTraj.so"
pytraj = CDLL(so_file)

# Code block to run the Monte Carlo simulation
if __name__ == "__main__":
    # Read the configuration file
    print("Reading configuration file...")
    run_params = read_config(config_path)
    print("Configuration file read.")

    impact_data_pointer = pytraj.mc_run(run_params)
    print("Monte Carlo simulation complete.")
