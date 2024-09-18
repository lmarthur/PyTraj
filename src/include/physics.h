#ifndef PHYSICS_H
#define PHYSICS_H

#include <math.h>
#include "vehicle.h"
#include "gravity.h"
#include "atmosphere.h"

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

    // Calculate the drag acceleration components


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

    // Get the current stage
    int stage = 0;
    for(int i = 0; i < vehicle->booster.num_stages; i++){
        if(state->t < vehicle->booster.burn_time[i]){
            stage = i;
            break;
        }
    }

    // Calculate the thrust acceleration components, assuming the thrust is along the velocity vector
    double thrust = vehicle->booster.fuel_burn_rate[stage] * vehicle->booster.isp0[stage];

    state->ax_thrust = thrust * state->vx / vehicle->current_mass;
    state->ay_thrust = thrust * state->vy / vehicle->current_mass;
    state->az_thrust = thrust * state->vz / vehicle->current_mass;

}

#endif