# This script contains code to generate scatter plots and histograms of the impact data.

import matplotlib.pyplot as plt
import numpy as np
from main import *

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
    # get local impact scatter data
    impact_x_local = impact_x - run_params.x_aim
    impact_y_local = impact_y - run_params.y_aim
    impact_z_local = impact_z - run_params.z_aim

    print("Impact data:")
    print("Impact x: ", impact_x)
    print("Impact y: ", impact_y)
    print("Impact z: ", impact_z)

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