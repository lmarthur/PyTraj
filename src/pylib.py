import numpy as np
from ctypes import *
import configparser

so_file = "./build/libPyTraj.so"
pytraj = CDLL(so_file)

# define the runparam struct
class runparams(Structure):
    _fields_ = [
        ("num_runs", c_int),
        ("time_step", c_double),
        ("traj_output", c_int),
        ("x_aim", c_double),
        ("y_aim", c_double),
        ("z_aim", c_double),
        
        ("grav_error", c_int),
        ("atm_error", c_int),
        ("gnss_nav", c_int),
        ("ins_nav", c_int),
        ("filter_type", c_int), # filter type (0: None, 1: KF, 2: EKF)

        ("rv_type", c_int), # 0 for ballistic, 1 for maneuverable

        ("initial_x_error", c_double),
        ("initial_pos_error", c_double),
        ("initial_vel_error", c_double),
        ("initial_angle_error", c_double),
        ("acc_scale_stability", c_double),
        ("gyro_bias_stability", c_double),
        ("gyro_noise", c_double),
        ("gnss_noise", c_double),
    ]

def read_config(config_file):
    """
    Function to read the .toml configuration file and return the run parameters.

    INPUTS:
    ----------
        config_file: str
            The path to the configuration file.
    OUTPUTS:
    ----------
        run_params: runparams
            The run parameters.
    """
    # read the configuration file
    config = configparser.ConfigParser()
    config.read(config_file)

    # create the run parameters struct
    run_params = runparams()

    # set the run parameters
    run_params.num_runs = c_int(int(config['RUN']['num_runs']))
    run_params.time_step = c_double(float(config['RUN']['time_step']))
    run_params.traj_output = c_int(int(config['RUN']['traj_output']))
    run_params.x_aim = c_double(float(config['RUN']['x_aim']))
    run_params.y_aim = c_double(float(config['RUN']['y_aim']))
    run_params.z_aim = c_double(float(config['RUN']['z_aim']))

    # set the flight parameters
    run_params.grav_error = c_int(int(config['FLIGHT']['grav_error']))
    run_params.atm_error = c_int(int(config['FLIGHT']['atm_error']))
    run_params.gnss_nav = c_int(int(config['FLIGHT']['gnss_nav']))
    run_params.ins_nav = c_int(int(config['FLIGHT']['ins_nav']))
    run_params.filter_type = c_int(int(config['FLIGHT']['filter_type']))

    # set the vehicle parameters
    run_params.rv_type = c_int(int(config['VEHICLE']['rv_type']))
    
    # set the error parameters
    run_params.initial_x_error = c_double(float(config['ERRORPARAMS']['initial_x_error']))
    run_params.initial_pos_error = c_double(float(config['ERRORPARAMS']['initial_pos_error']))
    run_params.initial_vel_error = c_double(float(config['ERRORPARAMS']['initial_vel_error']))
    run_params.initial_angle_error = c_double(float(config['ERRORPARAMS']['initial_angle_error']))
    run_params.acc_scale_stability = c_double(float(config['ERRORPARAMS']['acc_scale_stability']))
    run_params.gyro_bias_stability = c_double(float(config['ERRORPARAMS']['gyro_bias_stability']))
    run_params.gyro_noise = c_double(float(config['ERRORPARAMS']['gyro_noise']))
    run_params.gnss_noise = c_double(float(config['ERRORPARAMS']['gnss_noise']))

    return run_params
