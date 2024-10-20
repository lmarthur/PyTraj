import pytest
import sys
from ctypes import *
import numpy as np


sys.path.append('.')
from src.pylib import *
so_file = "./build/libPyTraj.so"
pytraj = CDLL(so_file)

def test_read_config():
    """
    Test suite for the read_config function
    """
    run_params = read_config("test")

    #assert run_params.run_name == b"test"
    assert run_params.num_runs == 2
    assert run_params.time_step_main == 1.0
    assert run_params.time_step_reentry == 0.01
    assert run_params.traj_output == 0
    assert run_params.x_aim == 6371e3
    assert run_params.y_aim == 0
    assert run_params.z_aim == 0
    
    assert run_params.grav_error == 0
    assert run_params.atm_error == 0
    assert run_params.gnss_nav == 0
    assert run_params.ins_nav == 0
    assert run_params.boost_guidance == 1
    assert run_params.filter_type == 0

    assert run_params.rv_type == 0

    assert run_params.initial_x_error == 0.0
    assert run_params.initial_pos_error == 0.0
    assert run_params.initial_vel_error == 0.0
    assert run_params.initial_angle_error == 0.0
    assert run_params.acc_scale_stability == 0.0
    assert run_params.gyro_bias_stability == 0.0
    assert run_params.gyro_noise == 0.0
    assert run_params.gnss_noise == 0.0


def test_mc_run():
    """
    Test suite for the mc_run function
    """
    
    # Turn off all random errors and verify that the first two runs are identical
    run_params = read_config("test")
    impact_data_pointer = pytraj.mc_run(run_params)

    # Get the impact data from the pointer
    assert impact_data_pointer != None
    
    # Read the impact data
    run_path = "./output/test/"
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    assert np.allclose(impact_data[0,:], impact_data[1,:], atol=1e-6)

    # Turn on random errors and verify that the first two runs are different
    run_params.initial_pos_error = c_double(1.0)

    impact_data_pointer = pytraj.mc_run(run_params)

    # Get the impact data from the pointer
    assert impact_data_pointer != None

    # Read the impact data
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    assert not np.allclose(impact_data[0,:], impact_data[1,:], atol=1e-6)


