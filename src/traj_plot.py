# This script contains code to generate plots of the vehicle's trajectory and control surfaces. 

import matplotlib.pyplot as plt
import numpy as np

def traj_plot(run_path):
    """
    Function to plot the trajectory of the vehicle.
    """
    # load the trajectory data from the .txt file, skipping the first row
    traj_data = np.loadtxt(run_path + "trajectory.txt", delimiter = ",", skiprows=1)

    true_t = traj_data[:,0]
    true_x = traj_data[:,1]
    true_y = traj_data[:,2]
    true_z = traj_data[:,3]
    true_vx = traj_data[:,4]
    true_vy = traj_data[:,5]
    true_vz = traj_data[:,6]
    # true_ax_grav = traj_data[:,7]
    # true_ay_grav = traj_data[:,8]
    # true_az_grav = traj_data[:,9]
    # true_ax_drag = traj_data[:,10]
    # true_ay_drag = traj_data[:,11]
    # true_az_drag = traj_data[:,12]
    # true_ax_lift = traj_data[:,13]
    # true_ay_lift = traj_data[:,14]
    # true_az_lift = traj_data[:,15]
    true_ax_thrust = traj_data[:,16]
    true_ay_thrust = traj_data[:,17]
    true_az_thrust = traj_data[:,18]
    # true_ax_total = traj_data[:,19]
    # true_ay_total = traj_data[:,20]
    # true_az_total = traj_data[:,21]
    true_mass = traj_data[:,22]

    true_altitude = np.sqrt(np.square(true_x) + np.square(true_y) + np.square(true_z)) - 6371e3
    true_thrust_mag = true_ax_thrust + true_ay_thrust + true_az_thrust

    # position vs. time
    plt.figure(figsize=(10,10))
    plt.plot(true_t, true_x, label="x")
    plt.plot(true_t, true_y, label="y")
    plt.plot(true_t, true_z, label="z")
    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")
    plt.title("Position")
    plt.legend()
    plt.grid()
    plt.savefig(run_path + "position.pdf")
    plt.close()

    # orbit plot
    earth_radius = 6371e3
    plt.figure(figsize=(10,10))
    # plot the Earth
    earth = plt.Circle((0, 0), earth_radius, color='blue', label="Earth")
    plt.gca().add_artist(earth)
    # set range for x and y axes to 2*earth_radius
    plt.xlim(-2*earth_radius, 2*earth_radius)
    plt.ylim(-2*earth_radius, 2*earth_radius)

    # plot the vehicle's trajectory in the x-y plane
    plt.plot(true_x, true_y, 'r', label="Trajectory")
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title("Position (x-y plane)")
    plt.grid()
    plt.savefig(run_path + "orbit.pdf")
    plt.close()

    # altitude vs. time
    plt.figure(figsize=(10,10))
    plt.plot(true_t, true_altitude)
    plt.xlabel("Time (s)")
    plt.ylabel("Altitude (m)")
    plt.title("Altitude")
    plt.grid()
    plt.savefig(run_path + "altitude.pdf")
    plt.close()

    # velocity vs. time
    plt.figure(figsize=(10,10))
    plt.plot(true_t, true_vx, label="vx")
    plt.plot(true_t, true_vy, label="vy")
    plt.plot(true_t, true_vz, label="vz")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.title("Velocity")
    plt.legend()
    plt.grid()
    plt.savefig(run_path + "velocity.pdf")
    plt.close()

    # thrust vs. time
    plt.figure(figsize=(10,10))
    plt.plot(true_t, true_thrust_mag)
    plt.xlabel("Time (s)")
    plt.ylabel("Thrust Acceleration (m/s^2)")
    plt.title("Thrust")
    plt.grid()
    plt.savefig(run_path + "thrust.pdf")
    plt.close()

    # mass vs. time
    plt.figure(figsize=(10,10))
    plt.plot(true_t, true_mass)
    plt.xlabel("Time (s)")
    plt.ylabel("Mass (kg)")
    plt.title("Mass")
    plt.grid()
    plt.savefig(run_path + "mass.pdf")
    plt.close()
