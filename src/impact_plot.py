# This script contains code to generate scatter plots and histograms of the impact data.

import matplotlib.pyplot as plt
import sys
sys.path.append('.')
import numpy as np
from src.pylib import *

# define the run path -- this provides the path to the data files and the output directory for the plots
run_path = "./output/"

print("Reading impact data...")
impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = " ")
print("Reading config file...")
run_params = read_config("./test/test_input.toml")

impact_t = impact_data[:,0]
impact_x = impact_data[:,1]
impact_y = impact_data[:,2]
impact_z = impact_data[:,3]

def impact_plot():
    # get longitude and latitude of aimpoint
    aimpoint_lon = np.arctan2(run_params.y_aim, run_params.x_aim)
    aimpoint_lat = np.arctan2(run_params.z_aim, np.sqrt(run_params.x_aim**2 + run_params.y_aim**2))
    print("Aimpoint longitude: ", aimpoint_lon)
    print("Aimpoint latitude: ", aimpoint_lat)
    # print aimpoint coordinates
    print("Aimpoint x: ", run_params.x_aim)
    print("Aimpoint y: ", run_params.y_aim)
    print("Aimpoint z: ", run_params.z_aim)

    impact_t = impact_data[:,0]
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

    print("Impact data:")
    print("Impact x: ", impact_x)
    print("Impact y: ", impact_y)
    print("Impact z: ", impact_z)
    print("Impact x local: ", impact_x_local)
    print("Impact y local: ", impact_y_local)

    # scatter plot of impact data
    plt.figure(figsize=(10,10))
    plt.scatter(impact_x_local, impact_y_local)
    plt.xlabel("Impact x (m)")
    plt.ylabel("Impact y (m)")
    plt.title("Impact Location")
    plt.grid()
    plt.savefig(run_path + "impact_location.pdf")
    plt.close()

impact_plot()