#ifndef ATMOSPHERE_H
#define ATMOSPHERE_H

#include <math.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include "utils.h"

// Define an atm_cond struct to store local atmospheric conditions
typedef struct atm_cond{
    double altitude; // altitude in meters
    double density; // density in kg/m^3
    double meridional_wind; // meridional wind in m/s
    double zonal_wind; // zonal wind in m/s
    double vertical_wind; // vertical wind in m/s

    // General atmospheric parameters
} atm_cond;

// Define an atm_model struct to store the atmospheric model
typedef struct atm_model{
    // Constants
    double scale_height; // scale height in meters
    double sea_level_density; // sea level density in kg/m^3

    // Standard deviations
    double std_densities[4];
    double std_winds[4];
    double std_vert_winds[4];

    // Perturbations
    double pert_densities[4];
    double pert_zonal_winds[4];
    double pert_meridional_winds[4];
    double pert_vert_winds[4];

} atm_model;

atm_model init_atm(runparams *run_params, gsl_rng *rng){
    /*
    Initializes the atmospheric model

    INPUTS:
    ----------
        run_params: runparams *
            pointer to the run parameters struct
        rng: gsl_rng *
            pointer to the random number generator
    OUTPUT:
    ----------
        atm_model: atm_model
            atmospheric model
    */

    atm_model atm_model;
    
    // Define constants
    atm_model.scale_height = 8000; // scale height in meters
    atm_model.sea_level_density = 1.225; // sea level density in kg/m^3

    // Non-perturbed branch
    if (run_params->atm_error == 0){

        for (int i = 0; i < 4; i++){
            atm_model.std_densities[i] = 0;
            atm_model.std_winds[i] = 0;
            atm_model.std_vert_winds[i] = 0;
            atm_model.pert_densities[i] = 0;
            atm_model.pert_zonal_winds[i] = 0;
            atm_model.pert_meridional_winds[i] = 0;
            atm_model.pert_vert_winds[i] = 0;
        }

    }
    else{
        // Density standard deviations
        atm_model.std_densities[0] = 0.005; // 0.5% error for h<50km
        atm_model.std_densities[1] = 0.05; // 5% error for 50km<h<100
        atm_model.std_densities[2] = 0.1; // 10% error for 100km<h<250
        atm_model.std_densities[3] = 0.03; // 3% error for h>250km

        // Wind standard deviations
        atm_model.std_winds[0] = 2; // 2 m/s error for h<5km
        atm_model.std_winds[1] = 8; // 8 m/s error for 5km<h<50km
        atm_model.std_winds[2] = 50; // 50 m/s error for 50km<h<200km
        atm_model.std_winds[3] = 100; // 100 m/s error for h>200km

        // Vertical wind standard deviations
        atm_model.std_vert_winds[0] = 0.1; // 0.1 m/s error for h<5km
        atm_model.std_vert_winds[1] = 0.5; // 0.5 m/s error for 5km<h<50km
        atm_model.std_vert_winds[2] = 2; // 2 m/s error for 50km<h<150km
        atm_model.std_vert_winds[3] = 10; // 5 m/s error for h>150km

        for (int i = 0; i < 4; i++){
            // Generate perturbations, which are then used by the get_atm_cond function to generate the true conditions
            atm_model.pert_densities[i] = atm_model.std_densities[i] * gsl_ran_gaussian(rng, 1);
            atm_model.pert_zonal_winds[i] = atm_model.std_winds[i] * gsl_ran_gaussian(rng, 1);
            atm_model.pert_meridional_winds[i] = atm_model.std_winds[i] * gsl_ran_gaussian(rng, 1);
            atm_model.pert_vert_winds[i] = atm_model.std_vert_winds[i] * gsl_ran_gaussian(rng, 1);
        }

    }

    return atm_model;
}


atm_cond get_exp_atm_cond(double altitude, atm_model *atm_model){
    /*
    Calculates the atmospheric conditions at a given altitude using an exponential model

    INPUTS:
    ----------
        altitude: double
            altitude in meters
        atm_model: atm_model *
            pointer to the atmospheric model
    OUTPUT:
    ----------
        atm_conditions: atm_cond
            local atmospheric conditions
    */

    atm_cond atm_conditions;
    if (altitude < 0){
        altitude = 0;
    }
    atm_conditions.altitude = altitude;
    atm_conditions.density = atm_model->sea_level_density * exp(-altitude/atm_model->scale_height);
    atm_conditions.meridional_wind = 0;
    atm_conditions.zonal_wind = 0;
    atm_conditions.vertical_wind = 0;

    return atm_conditions;
}


atm_cond get_pert_atm_cond(double altitude, atm_model *atm_model){
    /*
    Calculates the atmospheric conditions at a given altitude using a model based on EarthGRAM 2016 results

    INPUTS:
    ----------
        altitude: double
            altitude in meters
        atm_model: atm_model *
            pointer to the atmospheric model
    OUTPUT:
    ----------
        atm_conditions: atm_cond
            local atmospheric conditions
    */

    atm_cond atm_conditions;
    if (altitude < 0){
        altitude = 0;
    }
    atm_conditions.altitude = altitude;

    // Use if statements to determine the standard deviations to use
    
    // Density
    if (altitude < 50000 && altitude >= 0){
        atm_conditions.density = atm_model->sea_level_density * exp(-altitude/atm_model->scale_height);
        atm_conditions.density += atm_model->pert_densities[0] * atm_conditions.density;
    }
    else if (altitude < 100000){
        atm_conditions.density = atm_model->sea_level_density * exp(-altitude/atm_model->scale_height);
        atm_conditions.density += atm_model->pert_densities[1] * atm_conditions.density;
    }
    else if (altitude < 250000){
        atm_conditions.density = atm_model->sea_level_density * exp(-altitude/atm_model->scale_height);
        atm_conditions.density += atm_model->pert_densities[2] * atm_conditions.density;
    }
    else{
        atm_conditions.density = atm_model->sea_level_density * exp(-altitude/atm_model->scale_height);
        atm_conditions.density += atm_model->pert_densities[3] * atm_conditions.density;
    }

    // Wind
    if (altitude < 5000 && altitude >= 0){
        atm_conditions.meridional_wind = atm_model->pert_meridional_winds[0];
        atm_conditions.zonal_wind = atm_model->pert_zonal_winds[0];
        atm_conditions.vertical_wind = atm_model->pert_vert_winds[0];
    }
    else if (altitude < 50000){
        atm_conditions.meridional_wind = atm_model->pert_meridional_winds[1];
        atm_conditions.zonal_wind = atm_model->pert_zonal_winds[1];
        atm_conditions.vertical_wind = atm_model->pert_vert_winds[1];
    }
    else if (altitude < 200000){
        atm_conditions.meridional_wind = atm_model->pert_meridional_winds[2];
        atm_conditions.zonal_wind = atm_model->pert_zonal_winds[2];
        atm_conditions.vertical_wind = atm_model->pert_vert_winds[2];
    }
    else{
        atm_conditions.meridional_wind = atm_model->pert_meridional_winds[3];
        atm_conditions.zonal_wind = atm_model->pert_zonal_winds[3];
        atm_conditions.vertical_wind = atm_model->pert_vert_winds[3];
    }

    return atm_conditions;
}
#endif