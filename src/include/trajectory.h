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

state fly(state *initial_state, vehicle *vehicle, double time_step){
    /*
    Function that simulates the flight of a vehicle, updating the state of the vehicle at each time step
    
    INPUTS:
    ----------
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
    int max_steps = 500000;
    grav grav = init_grav();
    state old_state = *initial_state;
    state new_state = *initial_state;

    // Begin the integration loop
    for (int i = 0; i < max_steps; i++){
        // Get the atmospheric conditions
        double old_altitude = sqrt(old_state.x*old_state.x + old_state.y*old_state.y + old_state.z*old_state.z) - 6371e3;
        atm_cond atm_cond = get_exp_atm_cond(old_altitude);
        // Update the thrust of the vehicle
        update_thrust(vehicle, &old_state);
        // Update the gravity acceleration components
        update_gravity(&grav, &old_state);
        // Update the drag acceleration components
        update_drag(vehicle, &atm_cond, &old_state);
        // Calculate the total acceleration components
        new_state.ax_total = old_state.ax_grav + old_state.ax_drag + old_state.ax_lift + old_state.ax_thrust;
        new_state.ay_total = old_state.ay_grav + old_state.ay_drag + old_state.ay_lift + old_state.ay_thrust;
        new_state.az_total = old_state.az_grav + old_state.az_drag + old_state.az_lift + old_state.az_thrust;
        // Perform a Runge-Kutta step
        rk4step(&new_state, time_step);
        // Update the mass of the vehicle
        update_mass(vehicle, old_state.t);
        // Check if the vehicle has impacted the Earth
        double new_altitude = sqrt(new_state.x*new_state.x + new_state.y*new_state.y + new_state.z*new_state.z) - 6371e3;
        if (new_altitude < 0){
            state final_state = impact_linterp(&old_state, &new_state);

            return final_state;
        }
        
        // Update the old state
        old_state = new_state;
    }
    
    printf("Warning: Maximum number of steps reached with no impact\n");
    // printf("x_min: %f\n", x_min);
    // printf("x_max: %f\n", x_max);
    return new_state;
}

#endif