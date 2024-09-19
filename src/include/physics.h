#ifndef PHYSICS_H
#define PHYSICS_H

#include <math.h>
#include "vehicle.h"
#include "gravity.h"
#include "atmosphere.h"
#include "utils.h"

// Define a struct to store the state of a vehicle in 3D space
typedef struct state{
    // State parameters
    double t; // time in seconds since launch
    double x; // x-coordinate in meters
    double y; // y-coordinate in meters
    double z; // z-coordinate in meters
    double vx; // x-velocity in meters per second
    double vy; // y-velocity in meters per second
    double vz; // z-velocity in meters per second
    double ax_grav; // x-acceleration due to gravity in meters per second squared
    double ay_grav; // y-acceleration due to gravity in meters per second squared
    double az_grav; // z-acceleration due to gravity in meters per second squared
    double ax_drag; // x-acceleration due to drag in meters per second squared
    double ay_drag; // y-acceleration due to drag in meters per second squared
    double az_drag; // z-acceleration due to drag in meters per second squared
    double ax_lift; // x-acceleration due to lift in meters per second squared
    double ay_lift; // y-acceleration due to lift in meters per second squared
    double az_lift; // z-acceleration due to lift in meters per second squared
    double ax_thrust; // x-acceleration due to thrust in meters per second squared
    double ay_thrust; // y-acceleration due to thrust in meters per second squared
    double az_thrust; // z-acceleration due to thrust in meters per second squared
    double ax_total; // total x-acceleration in meters per second squared
    double ay_total; // total y-acceleration in meters per second squared
    double az_total; // total z-acceleration in meters per second squared

} state;

// Define a series of functions to calculate acceleration components


void update_gravity(grav *grav, state *state){
    /*
    Updates the gravitational acceleration components

    INPUTS:
    ----------
        grav: grav *
            pointer to the grav struct
        state: state *
            pointer to the state struct
    */

    // Non-perturbed gravity model
    if (grav->perturb_flag == 0){
        // Calculate the gravitational acceleration components
        double r = sqrt(state->x*state->x + state->y*state->y + state->z*state->z);
        double ar_grav = -grav->grav_const * grav->earth_mass / (r*r);
        state->ax_grav = ar_grav * state->x / r;
        state->ay_grav = ar_grav * state->y / r;
        state->az_grav = ar_grav * state->z / r;
    }
    else{
        printf("Perturbations not yet implemented\n");
    }

}

void update_drag(vehicle *vehicle, atm_cond *atm_cond, state *state){
    /*
    Updates the drag acceleration components

    INPUTS:
    ----------
        vehicle: vehicle *
            pointer to the vehicle struct
        state: state *
            pointer to the state struct
    */

    // Get the relative airspeed 
    double cart_wind[3];
    double spher_wind[3] = {atm_cond->vertical_wind, atm_cond->zonal_wind, atm_cond->meridional_wind};
    double spher_coords[3];
    double cart_coords[3] = {state->x, state->y, state->z};
    cartcoords_to_sphercoords(cart_coords, spher_coords);

    sphervec_to_cartvec(spher_wind, cart_wind, spher_coords);

    double v_rel[3] = {state->vx - cart_wind[0], state->vy - cart_wind[1], state->vz - cart_wind[2]};

    double v_rel_mag = sqrt(v_rel[0]*v_rel[0] + v_rel[1]*v_rel[1] + v_rel[2]*v_rel[2]);
    if (v_rel_mag < 1e-2){
        state->ax_drag = 0;
        state->ay_drag = 0;
        state->az_drag = 0;
        return;
    }

    // Calculate the drag acceleration components for a booster or reentry vehicle
    if (state->t > vehicle->booster.total_burn_time){
        // Calculate the drag acceleration components for a reentry vehicle
        double a_drag_mag = 0.5 * atm_cond->density * v_rel_mag * v_rel_mag * vehicle->rv.rv_area * vehicle->rv.c_d_0 / vehicle->current_mass;
        state->ax_drag = -a_drag_mag * v_rel[0] / v_rel_mag;
        state->ay_drag = -a_drag_mag * v_rel[1] / v_rel_mag;
        state->az_drag = -a_drag_mag * v_rel[2] / v_rel_mag;

        return;
    }
    else{
        // Calculate the drag acceleration components for a booster
        double a_drag_mag = 0.5 * atm_cond->density * v_rel_mag * v_rel_mag * vehicle->booster.area * vehicle->booster.c_d_0 / vehicle->current_mass;
        state->ax_drag = -a_drag_mag * v_rel[0] / v_rel_mag;
        state->ay_drag = -a_drag_mag * v_rel[1] / v_rel_mag;
        state->az_drag = -a_drag_mag * v_rel[2] / v_rel_mag;

        return;
    }

}

void update_thrust(vehicle *vehicle, state *state){
    /*
    Updates the thrust acceleration components

    INPUTS:
    ----------
        vehicle: vehicle *
            pointer to the vehicle struct
        state: state *
            pointer to the state struct
    */
    double a_thrust_mag;

    if (state->t > vehicle->booster.total_burn_time){
        state->ax_thrust = 0;
        state->ay_thrust = 0;
        state->az_thrust = 0;
        return;
    }
    
    // Get the current stage
    int stage = 0;
    if (state->t > vehicle->booster.burn_time[0]){
        stage = 1;
    }
    if (state->t > vehicle->booster.burn_time[0] + vehicle->booster.burn_time[1]){
        stage = 2;
    }

    // Calculate the thrust acceleration components
    a_thrust_mag = vehicle->booster.isp0[stage] * vehicle->booster.fuel_burn_rate[stage] / vehicle->current_mass;
    double v_mag = sqrt(state->vx*state->vx + state->vy*state->vy + state->vz*state->vz);

    if (v_mag < 1e-2){
        state->ax_thrust = a_thrust_mag;
        state->ay_thrust = 0;
        state->az_thrust = 0;
        return;
    }

    state->ax_thrust = a_thrust_mag * state->vx / v_mag;
    state->ay_thrust = a_thrust_mag * state->vy / v_mag;
    state->az_thrust = a_thrust_mag * state->vz / v_mag;
    printf("Thrust: %f\n", a_thrust_mag);
    printf("Thrust components: %f, %f, %f\n", state->ax_thrust, state->ay_thrust, state->az_thrust);
    
}

#endif