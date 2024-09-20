import numpy as np
from ctypes import *

so_file = "./build/pytrajlib.so"
pytraj = CDLL(so_file)

# define the state struct
class state(Structure):
    _fields_ = [
        ("t", c_double),
        ("theta_long",c_double),
        ("theta_lat", c_double),
        ("x", c_double),
        ("y", c_double),
        ("z", c_double),
        ("vx", c_double),
        ("vy", c_double),
        ("vz", c_double),
        ("ax_grav", c_double),
        ("ay_grav", c_double),
        ("az_grav", c_double),
        ("ax_drag", c_double),
        ("ay_drag", c_double),
        ("az_drag", c_double),
        ("ax_lift", c_double),
        ("ay_lift", c_double),
        ("az_lift", c_double),
        ("ax_thrust", c_double),
        ("ay_thrust", c_double),
        ("az_thrust", c_double),
        ("ax_total", c_double),
        ("ay_total", c_double),
        ("az_total", c_double),
    ]

# define the booster struct
class booster(Structure):
    _fields_ = [
        ("name", c_char_p),
        ("num_stages", c_int),
        ("maxdiam", c_double),
        ("area", c_double),
        ("total_burn_time", c_double),
        ("bus_mass", c_double),
        ("total_mass", c_double),
        ("c_d_0", c_double),
        ("wet_mass", c_double * 3),
        ("fuel_mass", c_double * 3),
        ("isp0", c_double * 3),
        ("burn_time", c_double * 3),
        ("fuel_burn_rate", c_double * 3),
    ]

# define the rv struct
class rv(Structure):
    _fields_ = [
        ("name", c_char_p),
        ("maneuverability_flag", c_int),
        ("rv_mass", c_double),
        ("rv_length", c_double),
        ("rv_radius", c_double),
        ("rv_area", c_double),
        ("c_d_0", c_double),
        ("c_d_alpha", c_double),
        ("c_m_alpha", c_double),
        ("c_m_q", c_double),
        ("c_l_alpha", c_double),
        ("flap_area", c_double),
        ("x_flap", c_double),
        ("x_com", c_double),
        ("Iyy", c_double),
    ]

# define the vehicle struct
class vehicle(Structure):
    _fields_ = [
        ("booster", POINTER(booster)),
        ("rv", POINTER(rv)),
        ("total_mass", c_double),
        ("current_mass", c_double),
    ]

def fly(initial_state, booster_type, rv_type, time_step=1.0, traj_output=False):
    """
    Python wrapper for the fly function in the C library. This function flies the vehicle.
    INPUTS:
    ----------
        initial_state: state * (pointer to a state struct)
            The initial state of the vehicle.
        vehicle: vehicle * (pointer to a vehicle struct)
            The vehicle to fly. Default is MMIII.
        time_step: float
            The time step to use in the simulation. Default is 1.0.
        traj_output: bool
            Whether or not to output the trajectory. Default is False.
    OUTPUTS:
    ----------
        final_state: state
            The final state of the vehicle.
    """
    # set the return types
    pytraj.fly.restype = state

    # set the argument types
    pytraj.fly.argtypes = [POINTER(state), POINTER(booster), POINTER(rv), c_double]

    final_state = pytraj.fly(initial_state, booster_type, rv_type, c_double(time_step))

    return final_state

pytraj.init_state.restype = state
pytraj.init_mmiii_booster.restype = booster
pytraj.init_ballistic_rv.restype = rv

print("Initializing...")
initial_state = pytraj.init_state()
print("State initialized.")
booster_type = pytraj.init_mmiii_booster()
print("Booster initialized.")
rv_type = pytraj.init_ballistic_rv()
print("Vehicle initialized.")

print("Flying...")
final_state = fly(initial_state, booster_type, rv_type)

print("Final state:")
print("t: ", final_state.t)
print("theta_long: ", final_state.theta_long)
print("theta_lat: ", final_state.theta_lat)
print("x: ", final_state.x)
print("y: ", final_state.y)
print("z: ", final_state.z)