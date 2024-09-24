import pytest
import sys
from ctypes import *


sys.path.append('.')
from src.main import *
so_file = "./build/libPyTraj.so"
pytraj = CDLL(so_file)

def test_read_config():
    """
    Test suite for the read_config function
    """
    run_params = read_config("./test/test_input.toml")

    assert run_params.num_runs == 100
    assert run_params.time_step == 1.0
    assert run_params.traj_output == 0
    assert run_params.rv_type == 0
    assert run_params.grav_error == 0
    assert run_params.atm_error == 0
    assert run_params.gnss_nav == 0
    assert run_params.ins_nav == 0
    assert run_params.filter_type == 0

def test_fly():
    """
    Test suite for the fly function
    """
    pytraj.init_state.restype = state
    pytraj.init_mmiii_booster.restype = booster
    pytraj.init_ballistic_rv.restype = rv
    run_params = read_config("./test/test_input.toml")
    print("Run parameters:")
    print("num_runs: ", run_params.num_runs, " time_step: ", run_params.time_step, " traj_output: ", run_params.traj_output, " rv_type: ", run_params.rv_type, " grav_error: ", run_params.grav_error, " atm_error: ", run_params.atm_error, " gnss_nav: ", run_params.gnss_nav, " ins_nav: ", run_params.ins_nav, " filter_type: ", run_params.filter_type)
    # Initialize the state
    initial_state = pytraj.init_state()
    initial_state.x += 10
    
    final_state = fly(run_params, initial_state, pytraj.init_mmiii_booster(), pytraj.init_ballistic_rv())
    print("Final state:")
    print("t: ", final_state.t, " x: ", final_state.x, " y: ", final_state.y, " z: ", final_state.z, " vx: ", final_state.vx, " vy: ", final_state.vy, " vz: ", final_state.vz, " ax_total: ", final_state.ax_total, " ay_total: ", final_state.ay_total, " az_total: ", final_state.az_total)
    assert final_state.x == 6371e3
    assert final_state.t > 0
    assert final_state.vx < 0
    assert final_state.vy == 0
    assert final_state.vz == 0

    
def test_mc_run():
    """
    Test suite for the mc_run function
    """
    
    # Turn off all random errors and verify that the first two runs are identical

    # Turn on all random errors and verify that the first two runs are different