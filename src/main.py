import sys
import os
from ctypes import *
from traj_plot import *
from impact_plot import *
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

# Import the necessary functions from the Python library
sys.path.append('.')
from src.pylib import *
so_file = "./build/libPyTraj.so"
pytraj = CDLL(so_file)

# Code block to run the Monte Carlo simulation
if __name__ == "__main__":
    # Read the configuration file
    print("Reading configuration file " + config_file + ".toml...")
    run_params = read_config(config_file)
    print("Configuration file read.")

    # Read the atmosphere data
    print("Reading atmosphere data...")
    atm_data = read_atm_data()
    print("Atmosphere data read.")
    pytraj.print_atm_data(byref(atm_data))

    # Set the output of update_aimpoint to be a cart_vector struct
    pytraj.update_aimpoint.restype = cart_vector

    aimpoint = pytraj.update_aimpoint(run_params, c_double(run_params.theta_long), byref(atm_data))
    run_params.x_aim = aimpoint.x
    run_params.y_aim = aimpoint.y
    run_params.z_aim = aimpoint.z
    # Set the configuration file aimpoint to the updated aimpoint
    config = configparser.ConfigParser()
    config.read(config_path)
    config['RUN']['x_aim'] = str(aimpoint.x)
    config['RUN']['y_aim'] = str(aimpoint.y)
    config['RUN']['z_aim'] = str(aimpoint.z)
    print(f"Aimpoint: ({aimpoint.x}, {aimpoint.y}, {aimpoint.z})")

    impact_data_pointer = pytraj.mc_run(run_params, byref(atm_data))
    print("Monte Carlo simulation complete.")

    # Copy the input file to the output directory
    os.system(f"cp {config_path} ./output/{config_file}")
    
    # Plot the trajectory
    if run_params.traj_output:
        print("Plotting trajectory...")
        traj_plot("./output/" + config_file + "/")
        print("Trajectory plotted.")

    # Plot the impact data
    print("Plotting impact data...")
    impact_plot("./output/" + config_file + "/", run_params)
    print("Impact data plotted.")
