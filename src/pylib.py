import numpy as np
from ctypes import *
import configparser
import os

so_file = "./build/libPyTraj.so"
pytraj = CDLL(so_file)

# define the runparam struct
class runparams(Structure):
    _fields_ = [
        ("run_name", c_char_p),
        ("output_path", c_char_p),
        ("impact_data_path", c_char_p),
        ("trajectory_path", c_char_p),
        ("num_runs", c_int),
        ("time_step_main", c_double),
        ("time_step_reentry", c_double),
        ("traj_output", c_int),
        ("x_aim", c_double),
        ("y_aim", c_double),
        ("z_aim", c_double),
        ("theta_long", c_double),
        ("theta_lat", c_double),
        
        ("grav_error", c_int),
        ("atm_error", c_int),
        ("gnss_nav", c_int),
        ("ins_nav", c_int),
        ("rv_maneuv", c_int),

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

class cart_vector(Structure):
    _fields_ = [
        ("x", c_double),
        ("y", c_double),
        ("z", c_double),
    ]
    
def read_config(run_name):
    """
    Function to read the .toml configuration file and return the run parameters.

    INPUTS:
    ----------
        run_name: str
            The name of the configuration file.
    OUTPUTS:
    ----------
        run_params: runparams
            The run parameters.
    """
    # read the configuration file
    config_file = "./input/" + run_name + ".toml"
    config = configparser.ConfigParser()
    config.read(config_file)

    # Make the output directory if it does not exist
    if not os.path.isdir(f"./output/{run_name}"):
        os.makedirs(f"./output/{run_name}")

    # create the run parameters struct
    run_params = runparams()

    # set the run parameters
    run_params.run_name = c_char_p(config['RUN']['run_name'].encode('utf-8'))
    run_params.output_path = c_char_p(config['RUN']['output_path'].encode('utf-8'))
    run_params.impact_data_path = run_params.output_path + b"/" + run_params.run_name + b"/impact_data.txt"
    run_params.trajectory_path = run_params.output_path + b"/" + run_params.run_name + b"/trajectory.txt"

    run_params.num_runs = c_int(int(config['RUN']['num_runs']))
    run_params.time_step_main = c_double(float(config['RUN']['time_step_main']))
    run_params.time_step_reentry = c_double(float(config['RUN']['time_step_reentry']))
    run_params.traj_output = c_int(int(config['RUN']['traj_output']))
    run_params.x_aim = c_double(float(config['RUN']['x_aim']))
    run_params.y_aim = c_double(float(config['RUN']['y_aim']))
    run_params.z_aim = c_double(float(config['RUN']['z_aim']))
    run_params.theta_long = c_double(float(config['RUN']['theta_long']))
    run_params.theta_lat = c_double(float(config['RUN']['theta_lat']))

    # set the flight parameters
    run_params.grav_error = c_int(int(config['FLIGHT']['grav_error']))
    run_params.atm_error = c_int(int(config['FLIGHT']['atm_error']))
    run_params.gnss_nav = c_int(int(config['FLIGHT']['gnss_nav']))
    run_params.ins_nav = c_int(int(config['FLIGHT']['ins_nav']))
    run_params.rv_maneuv = c_int(int(config['FLIGHT']['rv_maneuv']))

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

def get_cep(impact_data, run_params):
    """
    Function to calculate the circular error probable (CEP) from the impact data.

    INPUTS:
    ----------
        impact_data: numpy.ndarray
            The impact data.
    OUTPUTS:
    ----------
        cep: double
            The circular error probable.
    """
    # get longitude and latitude of aimpoint
    aimpoint_lon = np.arctan2(run_params.y_aim, run_params.x_aim)
    aimpoint_lat = np.arctan2(run_params.z_aim, np.sqrt(run_params.x_aim**2 + run_params.y_aim**2))

    impact_x = impact_data[:,1]
    impact_y = impact_data[:,2]
    impact_z = impact_data[:,3]

    # get vector relative to aimpoint
    impact_x = impact_x - run_params.x_aim
    impact_y = impact_y - run_params.y_aim
    impact_z = impact_z - run_params.z_aim

    # convert impact data to local tangent plane coordinates
    impact_x_local = -np.sin(aimpoint_lon)*impact_x + np.cos(aimpoint_lon)*impact_y
    impact_y_local = -np.sin(aimpoint_lat)*np.cos(aimpoint_lon)*impact_x - np.sin(aimpoint_lat)*np.sin(aimpoint_lon)*impact_y + np.cos(aimpoint_lat)*impact_z

    # get the miss distances
    miss_distance = np.sqrt(impact_x_local**2 + impact_y_local**2)
    cep = np.percentile(miss_distance, 50)

    return cep

def update_aimpoint(run_params, config_path):
    """
    Function to update the aimpoint based on the current run parameters.

    INPUTS:
    ----------
        run_params: runparams
            The run parameters.
    OUTPUTS:
    ----------
        aimpoint: cart_vector
            The updated aimpoint.
    """
    # Set the output of update_aimpoint to be a cart_vector struct
    pytraj.update_aimpoint.restype = cart_vector

    aimpoint = pytraj.update_aimpoint(run_params, c_double(run_params.theta_long))
    run_params.x_aim = aimpoint.x
    run_params.y_aim = aimpoint.y
    run_params.z_aim = aimpoint.z

    config = configparser.ConfigParser()
    config.read(config_path)
    config['RUN']['x_aim'] = str(aimpoint.x)
    config['RUN']['y_aim'] = str(aimpoint.y)
    config['RUN']['z_aim'] = str(aimpoint.z)

    return aimpoint