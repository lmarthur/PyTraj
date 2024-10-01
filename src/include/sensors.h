#ifndef SENSORS_H
#define SENSORS_H

#include "utils.h"
#include "trajectory.h"
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>

// Define an inertial measurement unit struct
typedef struct imu{
    // Accelerometer parameters
    double acc_scale_stability; // Scale stability (ppm)

    double acc_scale_x; // Scale factor for x-axis (ppm)
    double acc_scale_y; // Scale factor for y-axis (ppm)
    double acc_scale_z; // Scale factor for z-axis (ppm)

    // Gyroscope parameters
    double gyro_bias_stability; // Gyro bias (rad/s)
    double gyro_noise; // Gyro noise/random walk (rad/s/sqrt(s))

    double gyro_bias_lat; // Gyro bias in the latitude direction (rad/s)
    double gyro_bias_long; // Gyro bias in the longitude direction (rad/s)

    double gyro_error_lat; // Gyro error in the latitude direction (rad/s, defined recursively)
    double gyro_error_long; // Gyro error in the longitude direction (rad/s, defined recursively)

} imu;

imu imu_init(runparams *run_params, gsl_rng *rng){
    /*
    Initializes an accelerometer struct

    INPUTS:
    ----------
        run_params: runparams *
            pointer to the run parameters struct
        rng: gsl_rng *
            pointer to the random number generator

    OUTPUTS:
    ----------
        imu: imu
            pointer to the inertial measurement unit struct
    */

    imu imu;
    imu.acc_scale_stability = run_params->acc_scale_stability;
    imu.acc_scale_x = imu.acc_scale_stability * gsl_ran_gaussian(rng, 1); // ppm
    imu.acc_scale_y = imu.acc_scale_stability * gsl_ran_gaussian(rng, 1); // ppm
    imu.acc_scale_z = imu.acc_scale_stability * gsl_ran_gaussian(rng, 1); // ppm

    imu.gyro_bias_stability = run_params->gyro_bias_stability;
    imu.gyro_noise = run_params->gyro_noise;

    imu.gyro_bias_lat = imu.gyro_bias_stability * gsl_ran_gaussian(rng, 1); // rad/s
    imu.gyro_bias_long = imu.gyro_bias_stability * gsl_ran_gaussian(rng, 1); // rad/s

    imu.gyro_error_lat = 0;
    imu.gyro_error_long = 0;

    return imu;

}

state imu_measurement(imu *imu, state *true_state, gsl_rng *rng){
    /*
    Simulates an accelerometer measurement

    INPUTS:
    ----------
        imu: imu *
            pointer to the inertial measurement unit struct
        true_state: state *
            pointer to the true state of the vehicle
        rng: gsl_rng *
            pointer to the random number generator

    OUTPUTS:
    ----------
        meas_state: state
            pointer to the measured state of the vehicle
    */

    state meas_state;
    // Set timestamp
    meas_state.t = true_state->t;

    // TODO: Account for the orientation error in the accelerometer measurement
    // Accelerometer measurements
    // TODO: Double check this
    meas_state.ax_total = true_state->ax_total * (1 + imu->acc_scale_x);
    meas_state.ay_total = true_state->ay_total * (1 + imu->acc_scale_y);
    meas_state.az_total = true_state->az_total * (1 + imu->acc_scale_z);

    // Gyroscope measurements
    meas_state.theta_long = true_state->theta_long + imu->gyro_error_long;
    meas_state.theta_lat = true_state->theta_lat + imu->gyro_error_lat;

    return meas_state;
}

void update_imu(imu *imu, runparams *run_params, gsl_rng *rng){
    /*
    Updates the accelerometer parameters

    INPUTS:
    ----------
        imu: imu *
            pointer to the accelerometer struct
        run_params: runparams *
            pointer to the run parameters struct
        rng: gsl_rng *
            pointer to the random number generator
    */

    // Update the gyro error by recursively adding noise and bias drift
    imu->gyro_error_long = imu->gyro_error_long + imu->gyro_noise * gsl_ran_gaussian(rng, 1) + imu->gyro_bias_long * run_params->time_step;
    imu->gyro_error_lat = imu->gyro_error_lat + imu->gyro_noise * gsl_ran_gaussian(rng, 1) + imu->gyro_bias_lat * run_params->time_step;

}
#endif