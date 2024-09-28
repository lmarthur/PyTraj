import pytest
import sys
from ctypes import *


sys.path.append('.')
from src.pylib import *
so_file = "./build/libPyTraj.so"
pytraj = CDLL(so_file)

def test_read_config():
    """
    Test suite for the read_config function
    """
    run_params = read_config("./test/test_input.toml")

    assert run_params.num_runs == 2
    assert run_params.time_step == 1.0
    assert run_params.traj_output == 0
    assert run_params.x_aim == 6371e3
    assert run_params.y_aim == 0
    assert run_params.z_aim == 0
    
    assert run_params.grav_error == 0
    assert run_params.atm_error == 0
    assert run_params.gnss_nav == 0
    assert run_params.ins_nav == 0
    assert run_params.filter_type == 0

    assert run_params.rv_type == 0

    assert run_params.initial_x_error == 0.0
    assert run_params.initial_pos_error == 0.0
    assert run_params.initial_vel_error == 0.0
    assert run_params.initial_angle_error == 0.0


# def test_mc_run():
#     """
#     Test suite for the mc_run function
#     """
#     
#     # Turn off all random errors and verify that the first two runs are identical
#     run_params = read_config("./test/test_input.toml")
#     run_results = mc_run(run_params)
# 
#     assert run_results[0].all() == run_results[1].all()
#     assert len(run_results) == run_params.num_runs
# 
#     # Turn on initial position error and verify that the first two runs are different
#     run_params.initial_pos_error = c_double(1.0)
#     run_params.traj_output = c_int(1)
#     run_results = mc_run(run_params)
# 
#     assert run_results[0, 2] != run_results[1, 2]
#     assert run_results[0, 3] != run_results[1, 3]