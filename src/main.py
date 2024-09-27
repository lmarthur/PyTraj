import sys
from ctypes import *


sys.path.append('.')
from src.pylib import *
so_file = "./build/libPyTraj.so"
pytraj = CDLL(so_file)

# Code block to run the Monte Carlo simulation
config_file = "./test/test_input.toml"
run_params = read_config(config_file)
run_params.traj_output = c_int(1)
run_params.num_runs = c_int(10)
run_params.initial_pos_error = c_double(1.0)
mc_run(run_params)