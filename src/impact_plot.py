# This script contains code to generate scatter plots and histograms of the impact data.
import scipy.stats as stats
import os
import matplotlib.pyplot as plt
import sys
sys.path.append('.')
import numpy as np
from src.pylib import *

# TODO: Add a calculation of the range to the aimpoint

def impact_plot(run_path, run_params):

    # print error if the paths are not found
    if not os.path.exists(run_path + "impact_data.txt"):
        print("Error: impact data not found.")
        sys.exit()

    print("Reading impact data...")
    impact_data = np.loadtxt(run_path + "impact_data.txt", delimiter = ",", skiprows=1)

    impact_t = impact_data[:,0]
    impact_x = impact_data[:,1]
    impact_y = impact_data[:,2]
    impact_z = impact_data[:,3]

    # get longitude and latitude of aimpoint
    aimpoint_lon = np.arctan2(run_params.y_aim, run_params.x_aim)
    aimpoint_lat = np.arctan2(run_params.z_aim, np.sqrt(run_params.x_aim**2 + run_params.y_aim**2))

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

    # get the miss distances
    miss_distance = np.sqrt(impact_x_local**2 + impact_y_local**2)
    cep = np.percentile(miss_distance, 50)
    print('CEP: ', cep)
    cep = round(np.percentile(miss_distance, 50), 2)
    plotrange = 4*cep

    # Plot the data
    params = {
        'axes.labelsize': 8,
        'font.size': 8,
        'font.family': 'serif',
        'legend.fontsize': 10,
        'xtick.labelsize': 10,
        'ytick.labelsize': 10,
    }
    plt.rcParams.update(params)

    fig = plt.figure(figsize=(5,5))
    # plot a circle of radius CEP m centered on (0,0)
    N = 400
    t = np.linspace(0, 2 * np.pi, N)
    x, y = cep * np.cos(t), cep * np.sin(t)

    # gridspec
    gs = fig.add_gridspec(2, 1, height_ratios=(6, 1), hspace=0.18, bottom=0.1, top=0.9, left=0.02, right=0.98)
    
    a0 = fig.add_subplot(gs[0, 0])
    a1 = fig.add_subplot(gs[1, 0])

    a0.plot(x, y, c='r', label='CEP', linestyle='--')
    a0.scatter(impact_x_local, impact_y_local, c='k', marker='x', label='True Impact Points', s=25, alpha=0.5)
    a0.legend(['CEP', 'True Impact Points'])

    # center the plot on (0,0)
    a0.set_xlim(-plotrange, plotrange)
    a0.set_ylim(-plotrange, plotrange)
    a0.set_aspect('equal')

    # add N=len(guided_r) to the top left of the plot
    a0.text(-0.8*plotrange, 0.8*plotrange, 'N=' + str(len(miss_distance)), fontsize=10, verticalalignment='top', horizontalalignment='left')

    # add a grid
    # plt.grid()

    # add label to a0
    a0.set_xlabel('Downrange (m)', labelpad=-1)
    a0.set_ylabel('Crossrange (m)', labelpad=-1)
    # For a0 axis (if you have operations on it that require adjustments)
    a0.tick_params(axis='x', which='major', pad=1)  # Adjust pad for x-axis ticks
    a0.tick_params(axis='y', which='major', pad=1)  # Adjust pad for y-axis ticks


    # a0.set_title(' Impacts at ' + str(int(round(range/1000, -3))) + ' km. ' + 'CEP: ' + str(cep) + ' m.')
    # plot the histogram of the miss distances

    cep = round(np.percentile(miss_distance, 50), 2)

    # Fit a Nakagami distribution to the data
    x = np.linspace(0, 5*cep, 100)
    shape, loc, scale = stats.nakagami.fit(miss_distance, floc=0)
    nakagamipdf = stats.nakagami.pdf(x, shape, loc, scale)

    bins = 50
    # plot histogram up to 5 times the CEP, with no y axis
    a1.hist(miss_distance, bins=bins, range=(0, 5*cep), color='b', edgecolor='black', alpha=0.7)
    # renormalize the pdfs to the histogram
    nakagamipdf = nakagamipdf * len(miss_distance) * 5*cep / bins
    a1.plot(x, nakagamipdf, 'r', linewidth=2, label = 'Nakagami Distribution')
    # Add a vertical line at the CEP
    a1.axvline(x=cep, color='g', linestyle='--', linewidth=2, label='CEP')

    a1.grid()
    a1.yaxis.set_visible(False)
    a1.legend()
    a1.tick_params(axis='x', which='major', pad=1)  # Adjust pad for x-axis ticks
    a1.tick_params(axis='y', which='major', pad=1)  # Adjust pad for y-axis ticks (if y-axis is used)
    a1.set_xlabel('Miss Distance Histogram (m)', labelpad=1)  # Adjust labelpad for x-axis label
    plt.savefig(run_path + "impact_plot.pdf")
    plt.close()