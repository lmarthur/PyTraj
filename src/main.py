import sys
from ctypes import *


sys.path.append('.')
from src.pylib import *
so_file = "./build/libPyTraj.so"
pytraj = CDLL(so_file)

# Code block to run the Monte Carlo simulation
config_file = "./test/test_input.toml"
run_params = read_config(config_file)

