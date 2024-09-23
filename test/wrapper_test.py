import pytest
import sys
from ctypes import *

sys.path.append('.')
from src.main import *
so_file = "./build/libPyTraj.so"
pytraj = CDLL(so_file)

def test_fly():
    """
    Test suite for the fly function
    """
    pytraj.init_state.restype = state
    pytraj.init_mmiii_booster.restype = booster
    pytraj.init_ballistic_rv.restype = rv

    # Initialize the state
    initial_state = pytraj.init_state()
    initial_state.x += 10
    
    final_state = fly(initial_state, pytraj.init_mmiii_booster(), pytraj.init_ballistic_rv(), 1.0, 0)

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