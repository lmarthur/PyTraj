#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <stdio.h>
#include <math.h>
#include "utils.h"
#include "vehicle.h"
#include "gravity.h"
#include "atmosphere.h"
#include "physics.h"



state init_state(){
    /*
    Initializes a state struct at the launch site with zero velocity and acceleration

    OUTPUTS:
    ----------
        state: state
            initial state of the vehicle
    */

    state state;
    state.t = 0;
    state.x = 6371e3;
    state.y = 0;
    state.z = 0;
    state.theta_long = 0;
    state.theta_lat = 0;
    state.vx = 0;
    state.vy = 0;
    state.vz = 0;
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

state fly(state *initial_state, booster *booster, rv *rv, double time_step, int traj_output){
    /*
    Function that simulates the flight of a vehicle, updating the state of the vehicle at each time step
    
    INPUTS:
    ----------
        initial_state: state *
            pointer to the initial state of the vehicle
        booster: booster *
            pointer to the booster struct
        rv: rv *
            pointer to the reentry vehicle struct
        time_step: double
            time step in seconds
        traj_output: int
            flag to output trajectory data

    OUTPUTS:
    ----------
        final_state: state
            final state of the vehicle (impact point)
    */

    // Initialize the variables and structures
    int max_steps = 10000;
    grav grav = init_grav();
    state old_state = *initial_state;
    state new_state = *initial_state;

    // Initialize the vehicle
    // TODO: Replace this with an init_vehicle(booster, rv) function
    vehicle vehicle;
    vehicle.booster = *booster;
    vehicle.rv = *rv;
    vehicle.total_mass = vehicle.booster.total_mass + vehicle.rv.rv_mass;
    vehicle.current_mass = vehicle.total_mass;

    // Create a .txt file to store the trajectory data
    FILE *traj_file;
    if (traj_output == 1){
        traj_file = fopen("./output/trajectory.txt", "w");
        fprintf(traj_file, "t, x, y, z, vx, vy, vz, ax_grav, ay_grav, az_grav, ax_drag, ay_drag, az_drag, ax_lift, ay_lift, az_lift, ax_thrust, ay_thrust, az_thrust, ax_total, ay_total, az_total, current_mass\n");
        // Write the initial state to the trajectory file
        fprintf(traj_file, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", old_state.t, old_state.x, old_state.y, old_state.z, old_state.vx, old_state.vy, old_state.vz, old_state.ax_grav, old_state.ay_grav, old_state.az_grav, old_state.ax_drag, old_state.ay_drag, old_state.az_drag, old_state.ax_lift, old_state.ay_lift, old_state.az_lift, old_state.ax_thrust, old_state.ay_thrust, old_state.az_thrust, old_state.ax_total, old_state.ay_total, old_state.az_total, vehicle.current_mass);
    }

    // Begin the integration loop
    for (int i = 0; i < max_steps; i++){
        // Get the atmospheric conditions
        double old_altitude = sqrt(old_state.x*old_state.x + old_state.y*old_state.y + old_state.z*old_state.z) - 6371e3;
        atm_cond atm_cond = get_exp_atm_cond(old_altitude);
        // Update the thrust of the vehicle
        update_thrust(&vehicle, &new_state);
        // Update the gravity acceleration components
        update_gravity(&grav, &new_state);
        // Update the drag acceleration components
        update_drag(&vehicle, &atm_cond, &new_state);
        // Calculate the total acceleration components
        new_state.ax_total = new_state.ax_grav + new_state.ax_drag + new_state.ax_lift + new_state.ax_thrust;
        new_state.ay_total = new_state.ay_grav + new_state.ay_drag + new_state.ay_lift + new_state.ay_thrust;
        new_state.az_total = new_state.az_grav + new_state.az_drag + new_state.az_lift + new_state.az_thrust;
        
        // Perform a Runge-Kutta step
        rk4step(&new_state, time_step);
        // Update the mass of the vehicle
        update_mass(&vehicle, old_state.t);

        // Check if the vehicle has impacted the Earth
        double new_altitude = sqrt(new_state.x*new_state.x + new_state.y*new_state.y + new_state.z*new_state.z) - 6371e3;
        if (new_altitude < 0){
            state final_state = impact_linterp(&old_state, &new_state);
            if (traj_output == 1){
                // Write the final state to the trajectory file
                fprintf(traj_file, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", final_state.t, final_state.x, final_state.y, final_state.z, final_state.vx, final_state.vy, final_state.vz, final_state.ax_grav, final_state.ay_grav, final_state.az_grav, final_state.ax_drag, final_state.ay_drag, final_state.az_drag, final_state.ax_lift, final_state.ay_lift, final_state.az_lift, final_state.ax_thrust, final_state.ay_thrust, final_state.az_thrust, final_state.ax_total, final_state.ay_total, final_state.az_total, vehicle.current_mass);
                fclose(traj_file);
            }

            return final_state;
        }
        // output the trajectory data
        if (traj_output == 1){
            fprintf(traj_file, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", new_state.t, new_state.x, new_state.y, new_state.z, new_state.vx, new_state.vy, new_state.vz, new_state.ax_grav, new_state.ay_grav, new_state.az_grav, new_state.ax_drag, new_state.ay_drag, new_state.az_drag, new_state.ax_lift, new_state.ay_lift, new_state.az_lift, new_state.ax_thrust, new_state.ay_thrust, new_state.az_thrust, new_state.ax_total, new_state.ay_total, new_state.az_total, vehicle.current_mass);
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

#endif