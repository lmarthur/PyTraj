#ifndef GRAVITY_H
#define GRAVITY_H

#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include "utils.h"

// Note that the update_gravity function is in the physics.h file

// Define a grav struct to store gravity parameters
typedef struct grav{
    double earth_mass; // mass of the Earth in kg
    double earth_radius; // radius of the Earth in meters
    double grav_const; // gravitational constant in m^3/kg/s^2
    double grav_g0; // acceleration due to gravity at the geoid surface in m/s^2
    int perturb_flag; // flag to indicate if perturbations are enabled (1) or not (0)
    double geoid_height_error;
    double geoid_height_std;

} grav;

// Define a function to initialize gravity parameters
grav init_grav(runparams *run_params, gsl_rng *rng){
    /*
    Initializes gravity parameters

    INPUTS:
    ----------
        run_params: *runparams
            Pointer to the runparams struct
        rng: *gsl_rng
            Pointer to the GSL random number generator
    OUTPUTS:
    ----------
        grav: grav
            gravity struct
    */

    grav grav;
    // Define parameters for gravity
    grav.earth_mass = 5.972e24;
    grav.earth_radius = 6371e3;
    grav.grav_const = 6.67408e-11;
    grav.grav_g0 = - grav.grav_const * grav.earth_mass / (grav.earth_radius * grav.earth_radius);
    grav.geoid_height_std = 0.05;
    if (run_params->grav_error != 0){
        // Set nonzero geoid height error
        grav.geoid_height_error = gsl_ran_gaussian(rng, grav.geoid_height_std);
    }
    else {
        grav.geoid_height_error = 0;
    }
    
    grav.perturb_flag = run_params->grav_error;

    return grav;
}

#endif