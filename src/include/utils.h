#ifndef UTILS_H
#define UTILS_H

#include <math.h>
#include <stdio.h>

typedef struct runparams{
    int num_runs; // number of Monte Carlo runs
    double time_step; // time step in seconds
    int traj_output; // flag to output trajectory data
    double x_aim; // target x-coordinate in meters
    double y_aim; // target y-coordinate in meters
    double z_aim; // target z-coordinate in meters

    int grav_error; // flag to include gravitational perturbations
    int atm_error; // flag to include atmospheric perturbations
    int gnss_nav; // flag to include GNSS navigation
    int ins_nav; // flag to include INS navigation
    int filter_type; // filter type (0: None, 1: KF, 2: EKF)

    int rv_type; // reentry vehicle type (0: ballistic, 1: maneuverable)

    double initial_x_error; // initial x-error in meters
    double initial_pos_error; // initial position error in meters
    double initial_vel_error; // initial velocity error in meters per second
    double initial_angle_error; // initial angle error in radians
    double acc_scale_stability; // accelerometer scale stability in ppm
    double gyro_bias_stability; // gyro bias stability in rad/s
    double gyro_noise; // gyro noise in rad/s/sqrt(s)
    double gnss_noise; // GNSS error in meters

} runparams;

void cartcoords_to_sphercoords(double *cart_coords, double *spher_coords){
    /*
    Converts Cartesian coordinates to spherical coordinates

    INPUTS:
    ----------
        cart_coords: double *
            pointer to Cartesian coordinates [x, y, z]
        spher_coords: double *
            pointer to spherical coordinates [r, long, lat]
    */

    // Calculate the radial coordinate
    spher_coords[0] = sqrt(cart_coords[0]*cart_coords[0] + cart_coords[1]*cart_coords[1] + cart_coords[2]*cart_coords[2]);

    // Calculate the longitudinal coordinate
    spher_coords[1] = atan2(cart_coords[1], cart_coords[0]);

    // Calculate the latitudinal coordinate
    spher_coords[2] = atan(cart_coords[2] / sqrt(cart_coords[0]*cart_coords[0] + cart_coords[1]*cart_coords[1]));
}

void sphercoords_to_cartcoords(double *spher_coords, double *cart_coords){
    /*
    Converts spherical coordinates to Cartesian coordinates

    INPUTS:
    ----------
        spher_coords: double *
            pointer to spherical coordinates [r, long, lat]
        cart_coords: double *
            pointer to Cartesian coordinates [x, y, z]
    */

    // Calculate the x-coordinate
    cart_coords[0] = spher_coords[0] * cos(spher_coords[1]) * cos(spher_coords[2]);

    // Calculate the y-coordinate
    cart_coords[1] = spher_coords[0] * sin(spher_coords[1]) * cos(spher_coords[2]);

    // Calculate the z-coordinate
    cart_coords[2] = spher_coords[0] * sin(spher_coords[2]);

}

void sphervec_to_cartvec(double *sphervec, double *cartvec, double *spher_coords){
    /*
    Converts a spherical vector to a Cartesian vector at a given set of spherical coordinates

    INPUTS:
    ----------
        sphervec: double *
            pointer to spherical vector [r, long, lat]
        cartvec: double *
            pointer to Cartesian vector [x, y, z]
        spher_coords: double *
            pointer to spherical coordinates [r, long, lat]
    */

    // Get th x-component of the spherical vector
    cartvec[0] = sphervec[0] * cos(spher_coords[1]) * cos(spher_coords[2]);

    // Get the y-component of the spherical vector
    cartvec[1] = sphervec[0] * sin(spher_coords[1]) * cos(spher_coords[2]);

    // Get the z-component of the spherical vector
    cartvec[2] = sphervec[0] * sin(spher_coords[2]);
    
}

void print_config(runparams *run_params){
    /*
    Prints the run parameters to the console at runtime

    INPUTS:
    ----------
        run_params: runparams *
            pointer to the run parameters struct
    */

    printf("Number of Monte Carlo runs: %d\n", run_params->num_runs);
    printf("Time step: %f\n", run_params->time_step);
    printf("Trajectory output: %d\n", run_params->traj_output);
    printf("Target x-coordinate: %f\n", run_params->x_aim);
    printf("Target y-coordinate: %f\n", run_params->y_aim);
    printf("Target z-coordinate: %f\n", run_params->z_aim);

    printf("Gravitational perturbations: %d\n", run_params->grav_error);
    printf("Atmospheric perturbations: %d\n", run_params->atm_error);
    printf("GNSS navigation: %d\n", run_params->gnss_nav);
    printf("INS navigation: %d\n", run_params->ins_nav);
    printf("Filter type: %d\n", run_params->filter_type);

    printf("Reentry vehicle type: %d\n", run_params->rv_type);

    printf("Initial x-error: %f\n", run_params->initial_x_error);
    printf("Initial position error: %f\n", run_params->initial_pos_error);
    printf("Initial velocity error: %f\n", run_params->initial_vel_error);
    printf("Initial angle error: %f\n", run_params->initial_angle_error);
    printf("Accelerometer scale stability: %f\n", run_params->acc_scale_stability);
    printf("Gyro bias stability: %f\n", run_params->gyro_bias_stability);
    printf("Gyro noise: %f\n", run_params->gyro_noise);
    printf("GNSS noise: %f\n", run_params->gnss_noise);

}
#endif