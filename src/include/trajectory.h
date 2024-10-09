#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <stdio.h>
#include <math.h>
#include "utils.h"
#include "vehicle.h"
#include "gravity.h"
#include "atmosphere.h"
#include "physics.h"
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>

// Define a constant upper limit for the number of Monte Carlo runs
#define MAX_RUNS 1000

// Define a struct to store impact data
typedef struct impact_data{
    // Impact data
    state impact_states[MAX_RUNS];

} impact_data;

state init_state(runparams *run_params, gsl_rng *rng){
    /*
    Initializes a state struct at the launch site with zero velocity and acceleration

    OUTPUTS:
    ----------
        state: state
            initial state of the vehicle
    */

    state state;
    state.t = 0;
    state.x = 6371e3 + run_params->initial_x_error * gsl_ran_gaussian(rng, 1);
    state.y = run_params->initial_pos_error * gsl_ran_gaussian(rng, 1);
    state.z = run_params->initial_pos_error * gsl_ran_gaussian(rng, 1);
    state.theta_long = run_params->theta_long + run_params->initial_angle_error * gsl_ran_gaussian(rng, 1);
    state.theta_lat = run_params->theta_lat + run_params->initial_angle_error * gsl_ran_gaussian(rng, 1);
    state.vx = run_params->initial_vel_error * gsl_ran_gaussian(rng, 1);
    state.vy = run_params->initial_vel_error * gsl_ran_gaussian(rng, 1);
    state.vz = run_params->initial_vel_error * gsl_ran_gaussian(rng, 1);
    state.ax_grav = 0;
    state.ay_grav = 0;
    state.az_grav = 0;
    state.ax_drag = 0;
    state.ay_drag = 0;
    state.az_drag = 0;
    state.ax_lift = 0;
    state.ay_lift = 0;
    state.az_lift = 0;
    state.ax_thrust = 0;
    state.ay_thrust = 0;
    state.az_thrust = 0;
    state.ax_total = 0;
    state.ay_total = 0;
    state.az_total = 0;

    return state;
}

state impact_linterp(state *state_0, state *state_1){
    /*
    Performs a spatial linear interpolation between two states to find the impact point, velocity, and time

    INPUTS:
    ----------
        state_0: state *
            pointer to initial state of the vehicle
        state_1: state *
            pointer to final state of the vehicle
    OUTPUTS:
    ----------
        impact_state: state
            state of the vehicle at impact
    */

    // Calculate the interpolation factor
    double altitude_0 = sqrt(state_0->x*state_0->x + state_0->y*state_0->y + state_0->z*state_0->z) - 6371e3;
    double altitude_1 = sqrt(state_1->x*state_1->x + state_1->y*state_1->y + state_1->z*state_1->z) - 6371e3;
    double interp_factor = altitude_0 / (altitude_0 - altitude_1);

    // Perform the interpolation
    state impact_state = *state_0;
    impact_state.t = state_0->t + interp_factor * (state_1->t - state_0->t);
    impact_state.x = state_0->x + interp_factor * (state_1->x - state_0->x);
    impact_state.y = state_0->y + interp_factor * (state_1->y - state_0->y);
    impact_state.z = state_0->z + interp_factor * (state_1->z - state_0->z);
    impact_state.vx = state_0->vx + interp_factor * (state_1->vx - state_0->vx);
    impact_state.vy = state_0->vy + interp_factor * (state_1->vy - state_0->vy);
    impact_state.vz = state_0->vz + interp_factor * (state_1->vz - state_0->vz);

    return impact_state;
}

void output_impact(FILE *impact_file, impact_data *impact_data, int num_runs){
    /*
    Function that outputs the impact data struct to the impact file
    
    INPUTS:
    ----------
        impact_file: * FILE
            Pointer to the impact file stream
        impact_data: * impact_data
            Pointer to the impact data struct
        num_runs: int
            Number of Monte Carlo runs
    */
    printf("Outputting the impact data...");

    // Iterate through the number of runs and output the impact data
    for (int i = 0; i < num_runs; i++){
        fprintf(impact_file, "%f, %f, %f, %f, %f, %f, %f\n", impact_data->impact_states[i].t, impact_data->impact_states[i].x, impact_data->impact_states[i].y, impact_data->impact_states[i].z, impact_data->impact_states[i].vx, impact_data->impact_states[i].vy, impact_data->impact_states[i].vz);
    }

    // Close the impact file
    fclose(impact_file);
    
}

state fly(runparams *run_params, state *initial_state, vehicle *vehicle){
    /*
    Function that simulates the flight of a vehicle, updating the state of the vehicle at each time step
    
    INPUTS:
    ----------
        run_params: runparams *
            pointer to the run parameters struct
        initial_state: state *
            pointer to the initial state of the vehicle
        vehicle: vehicle *
            pointer to the vehicle struct

    OUTPUTS:
    ----------
        final_state: state
            final state of the vehicle (impact point)
    */

    // Initialize the variables and structures
    int max_steps = 10000;

    // Initialize the random number generator
    const gsl_rng_type *T;
    gsl_rng *rng;
    gsl_rng_env_setup();
    T = gsl_rng_default;
    rng = gsl_rng_alloc(T);

    grav grav = init_grav(run_params, rng);
    state old_state = *initial_state;
    state new_state = *initial_state;

    int traj_output = run_params->traj_output;
    double time_step = run_params->time_step;

    // Create a .txt file to store the trajectory data
    FILE *traj_file;
    if (traj_output == 1){
        traj_file = fopen("./output/run_0/trajectory.txt", "w");
        fprintf(traj_file, "t, x, y, z, vx, vy, vz, ax_grav, ay_grav, az_grav, ax_drag, ay_drag, az_drag, ax_lift, ay_lift, az_lift, ax_thrust, ay_thrust, az_thrust, ax_total, ay_total, az_total, current_mass\n");
        // Write the initial state to the trajectory file
        fprintf(traj_file, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", old_state.t, old_state.x, old_state.y, old_state.z, old_state.vx, old_state.vy, old_state.vz, old_state.ax_grav, old_state.ay_grav, old_state.az_grav, old_state.ax_drag, old_state.ay_drag, old_state.az_drag, old_state.ax_lift, old_state.ay_lift, old_state.az_lift, old_state.ax_thrust, old_state.ay_thrust, old_state.az_thrust, old_state.ax_total, old_state.ay_total, old_state.az_total, vehicle->current_mass);
    }

    // Begin the integration loop
    for (int i = 0; i < max_steps; i++){
        // Get the atmospheric conditions
        double old_altitude = sqrt(old_state.x*old_state.x + old_state.y*old_state.y + old_state.z*old_state.z) - 6371e3;
        atm_cond atm_cond = get_exp_atm_cond(old_altitude);
        // Update the thrust of the vehicle
        update_thrust(vehicle, &new_state);
        // Update the gravity acceleration components
        update_gravity(&grav, &new_state);
        // Update the drag acceleration components
        update_drag(vehicle, &atm_cond, &new_state);
        // Calculate the total acceleration components
        new_state.ax_total = new_state.ax_grav + new_state.ax_drag + new_state.ax_lift + new_state.ax_thrust;
        new_state.ay_total = new_state.ay_grav + new_state.ay_drag + new_state.ay_lift + new_state.ay_thrust;
        new_state.az_total = new_state.az_grav + new_state.az_drag + new_state.az_lift + new_state.az_thrust;
        
        // Perform a Runge-Kutta step
        rk4step(&new_state, time_step);
        // Update the mass of the vehicle
        update_mass(vehicle, old_state.t);

        // Check if the vehicle has impacted the Earth
        double new_altitude = sqrt(new_state.x*new_state.x + new_state.y*new_state.y + new_state.z*new_state.z) - 6371e3;
        if (new_altitude < 0){
            state final_state = impact_linterp(&old_state, &new_state);
            if (traj_output == 1){
                // Write the final state to the trajectory file
                fprintf(traj_file, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", final_state.t, final_state.x, final_state.y, final_state.z, final_state.vx, final_state.vy, final_state.vz, final_state.ax_grav, final_state.ay_grav, final_state.az_grav, final_state.ax_drag, final_state.ay_drag, final_state.az_drag, final_state.ax_lift, final_state.ay_lift, final_state.az_lift, final_state.ax_thrust, final_state.ay_thrust, final_state.az_thrust, final_state.ax_total, final_state.ay_total, final_state.az_total, vehicle->current_mass);
                fclose(traj_file);
            }

            return final_state;
        }
        // output the trajectory data
        if (traj_output == 1){
            fprintf(traj_file, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", new_state.t, new_state.x, new_state.y, new_state.z, new_state.vx, new_state.vy, new_state.vz, new_state.ax_grav, new_state.ay_grav, new_state.az_grav, new_state.ax_drag, new_state.ay_drag, new_state.az_drag, new_state.ax_lift, new_state.ay_lift, new_state.az_lift, new_state.ax_thrust, new_state.ay_thrust, new_state.az_thrust, new_state.ax_total, new_state.ay_total, new_state.az_total, vehicle->current_mass);
        }
        // Update the old state
        old_state = new_state;
    }
    
    printf("Warning: Maximum number of steps reached with no impact\n");

    // Close the trajectory file
    if (traj_output == 1){
        fclose(traj_file);
    }

    return new_state;
}

cart_vector update_aimpoint(runparams *run_params, double thrust_angle_long){
    /*
    Updates the aimpoint based on the thrust angle and other run parameters

    INPUTS:
    ----------
        run_params: runparams *
            pointer to the run parameters struct
        thrust_angle_long: double
            thrust angle in the longitudinal direction
    OUTPUTS:
    ----------
        cart_vector: aimpoint
            Cartesian vector to the updated aimpoint
    */

    cart_vector aimpoint;
    
    // Set output to zero
    run_params->traj_output = 0;
    // Set all error parameters to zero
    run_params->grav_error = 0;
    run_params->atm_error = 0;
    run_params->initial_x_error = 0;
    run_params->initial_pos_error = 0;
    run_params->initial_vel_error = 0;
    run_params->initial_angle_error = 0;
    run_params->acc_scale_stability = 0;
    run_params->gyro_bias_stability = 0;
    run_params->gyro_noise = 0;
    run_params->gnss_noise = 0;

    // Initialize the random number generator (unused in this case, but still required)
    const gsl_rng_type *T;
    gsl_rng *rng;
    gsl_rng_env_setup();
    T = gsl_rng_default;
    rng = gsl_rng_alloc(T);

    // Initialize the vehicle 
    vehicle vehicle = init_mmiii_ballistic();
    state initial_state = init_state(run_params, rng);
    initial_state.theta_long = thrust_angle_long;

    // Call the fly function to get the final state
    state final_state = fly(run_params, &initial_state, &vehicle);

    // Update the aimpoint based on the final state
    aimpoint.x = final_state.x;
    aimpoint.y = final_state.y;
    aimpoint.z = final_state.z;

    return aimpoint;
}

void mc_run(runparams run_params){
    /*
    Function that runs a Monte Carlo simulation of the vehicle flight
    
    INPUTS:
    ----------
        run_params: runparams
            run parameters struct
    */

    // Print the run parameters to the console
    // print_config(&run_params);

    // Initialize the variables
    int num_runs = run_params.num_runs;
    printf("Simulating %d Monte Carlo runs...\n", num_runs);
    if (num_runs > MAX_RUNS){
        printf("Error: Number of runs exceeds the maximum limit. Increase MAX_RUNS in src/include/trajectory.h \n");
        printf("num_runs: %d, MAX_RUNS: %d\n", num_runs, MAX_RUNS);
        exit(1);
    }
    // state initial_state = init_state();
    // vehicle vehicle = init_mmiii_ballistic();
    impact_data impact_data;

    // Create a .txt file to store the impact data
    FILE *impact_file;
    impact_file = fopen("./output/run_0/impact_data.txt", "w");
    fprintf(impact_file, "t, x, y, z, vx, vy, vz\n");
    
    // Initialize the random number generator
    const gsl_rng_type *T;
    gsl_rng *rng;
    gsl_rng_env_setup();
    T = gsl_rng_default;
    rng = gsl_rng_alloc(T);

    // Run the Monte Carlo simulation
    for (int i = 0; i < num_runs; i++){

        vehicle vehicle = init_mmiii_ballistic();
        state initial_state = init_state(&run_params, rng);
        
        impact_data.impact_states[i] = fly(&run_params, &initial_state, &vehicle);

    }

    // Output the impact data
    output_impact(impact_file, &impact_data, num_runs);


}

#endif