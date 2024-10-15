#ifndef MANEUVERABILITY_H
#define MANEUVERABILITY_H

#include "trajectory.h"
#include "guidance.h"

state instant_maneuv(state *true_state, cart_vector *a_command){
    /*
    Simulates instantaneous maneuverability of the vehicle by applying a commanded acceleration vector with no time delay

    INPUTS:
    ----------
        true_state: state *
            pointer to the true state of the vehicle
        a_command: cart_vector *
            pointer to the commanded acceleration vector
    
    OUTPUTS:
    ----------
        state: updated_state
            state of the vehicle after the maneuver
    */

    // Initialize the new state
    state updated_state = *true_state;

    // Update the acceleration components
    updated_state.ax_lift = a_command->x;
    updated_state.ay_lift = a_command->y;
    updated_state.az_lift = a_command->z;

    // Update the total acceleration components
    updated_state.ax_total = updated_state.ax_grav + updated_state.ax_drag + updated_state.ax_lift + updated_state.ax_thrust;
    updated_state.ay_total = updated_state.ay_grav + updated_state.ay_drag + updated_state.ay_lift + updated_state.ay_thrust;
    updated_state.az_total = updated_state.az_grav + updated_state.az_drag + updated_state.az_lift + updated_state.az_thrust;

    return updated_state;
}

state perfect_maneuv(state *true_state, state *estimated_state, state *desired_state){
    /*
    Simulates perfect maneuverability by shifting the true state by the difference between the estimated and desired states

    INPUTS:
    ----------
        true_state: state *
            pointer to the true state of the vehicle
        estimated_state: state *
            pointer to the estimated state of the vehicle
        desired_state: state *
            pointer to the desired state of the vehicle

    OUTPUTS:
    ----------
        state: updated_state
            true state of the vehicle after the maneuver
    */

    // Initialize the new state
    state updated_state = *true_state;

    // Calculate the difference between the desired and estimated states
    updated_state.x = true_state->x + (desired_state->x - estimated_state->x);
    updated_state.y = true_state->y + (desired_state->y - estimated_state->y);
    updated_state.z = true_state->z + (desired_state->z - estimated_state->z);
    updated_state.vx = true_state->vx + (desired_state->vx - estimated_state->vx);
    updated_state.vy = true_state->vy + (desired_state->vy - estimated_state->vy);
    updated_state.vz = true_state->vz + (desired_state->vz - estimated_state->vz);
    updated_state.theta_lat = true_state->theta_lat + (desired_state->theta_lat - estimated_state->theta_lat);
    updated_state.theta_long = true_state->theta_long + (desired_state->theta_long - estimated_state->theta_long);
    updated_state.ax_total = true_state->ax_total + (desired_state->ax_total - estimated_state->ax_total);
    updated_state.ay_total = true_state->ay_total + (desired_state->ay_total - estimated_state->ay_total);
    updated_state.az_total = true_state->az_total + (desired_state->az_total - estimated_state->az_total);

    return updated_state;
}

double rv_time_constant(vehicle *vehicle, state *true_state, atm_cond *atm_cond){
    /*
    Calculates the time constant of the reentry vehicle based on the current state

    INPUTS:
    ----------
        vehicle: vehicle *
            pointer to the vehicle struct
        true_state: state *
            pointer to the true state of the vehicle
        atm_cond: atm_cond *
            pointer to the atmospheric conditions

    OUTPUTS:
    ----------
        double: time_constant
            time constant of the vehicle
    */

    // Get the current velocity
    double velocity = sqrt(true_state->vx*true_state->vx + true_state->vy*true_state->vy + true_state->vz*true_state->vz);
    
    // Calculate the time constant
    double time_constant = sqrt(-2 * vehicle->rv.Iyy / (vehicle->rv.c_m_alpha * vehicle->rv.rv_area * atm_cond->density * pow(velocity, 2) * vehicle->rv.rv_length));

    return time_constant;
}

void update_lift(state *state, cart_vector *a_command, atm_cond *atm_cond, vehicle *vehicle, double time_step){
    /*
    Simulates maneuverability of a reentry vehicle by applying a commanded acceleration vector with a time delay and realistic atmospheric model

    INPUTS:
    ----------
        true_state: state *
            pointer to the state of the vehicle
        a_command: cart_vector *
            pointer to the commanded acceleration vector
        atm_cond: atm_cond *
            pointer to the atmospheric conditions
        vehicle: vehicle *
            pointer to the vehicle struct
        time_step: double
            time step for the simulation
    */

    // Calculate the time constant of the vehicle
    double time_constant = rv_time_constant(vehicle, state, atm_cond);

    state->ax_lift = state->ax_lift + (a_command->x - state->ax_lift) * time_step / time_constant;
    state->ay_lift = state->ay_lift + (a_command->y - state->ay_lift) * time_step / time_constant;
    state->az_lift = state->az_lift + (a_command->z - state->az_lift) * time_step / time_constant;

}

#endif