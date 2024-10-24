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

// Define an atmprofile struct to store the atmospheric profile
typedef struct atmprofile{
    double alt_array[100]; // altitude array
    double density_array[100]; // density array
    double meridional_wind_array[100]; // meridional wind array
    double zonal_wind_array[100]; // zonal wind array
    double vertical_wind_array[100]; // vertical wind array
} atmprofile;

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

typedef struct atmdata{
    int n_profiles;
    int n_altitudes;
    double atm_array[100000][10];

} atmdata;

void print_atm_data(atmdata *atm_data){
    /*
    Prints portions of the atmospheric data to the console--useful for debugging purposes

    INPUTS:
    ----------
        atm_data: atmdata *
            pointer to the atmospheric data struct
    */

    printf("Number of profiles: %d\n", atm_data->n_profiles);
    printf("Number of altitudes: %d\n", atm_data->n_altitudes);
    printf("First 10 altitudes: ");
    for (int i = 0; i < 10; i++){
        printf("%f ", atm_data->atm_array[i][1]);
    }
    printf("\n");
}

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
        atm_model.std_densities[0] = 0.00005;
        atm_model.std_densities[1] = 0.00006;
        atm_model.std_densities[2] = 0.0018;
        atm_model.std_densities[3] = 0.0001;

        // Wind standard deviations
        atm_model.std_winds[0] = 0.05;
        atm_model.std_winds[1] = 0.12;
        atm_model.std_winds[2] = 0.75;
        atm_model.std_winds[3] = 1.5;

        // Vertical wind standard deviations
        atm_model.std_vert_winds[0] = 0.05;
        atm_model.std_vert_winds[1] = 0.015;
        atm_model.std_vert_winds[2] = 0.075;
        atm_model.std_vert_winds[3] = 0.3;

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

atmprofile init_atm_profile(atmdata *atm_data, int pert_flag, int profile){
    /*
    Initializes the atmospheric model by reading in the atmospheric data to an atmprofile struct
    
    INPUTS:
    ----------
        atm_data: atmdata *
            pointer to the atmospheric data struct
        pert_flag: int
            flag to include atmospheric perturbations (0: no, 1: yes)
        profile: int
            profile number to use
    OUTPUT:
    ----------
        atm_profile: atmprofile
            local atmospheric profile
    */

    atmprofile atm_profile;
    // Get starting index for the profile
    int start_index = profile * 100;

    // Branch for no perturbations
    if (pert_flag == 0){
        for (int i = 0; i < 100; i++){
            atm_profile.alt_array[i] = atm_data->atm_array[start_index + i][1];
            atm_profile.density_array[i] = atm_data->atm_array[start_index + i][2];
            atm_profile.meridional_wind_array[i] = atm_data->atm_array[start_index + i][4];
            atm_profile.zonal_wind_array[i] = atm_data->atm_array[start_index + i][6];
            atm_profile.vertical_wind_array[i] = atm_data->atm_array[start_index + i][8];
        }
    }
    // Branch for perturbations
    else{
        for (int i = 0; i < 100; i++){
            atm_profile.alt_array[i] = atm_data->atm_array[start_index + i][1];
            atm_profile.density_array[i] = atm_data->atm_array[start_index + i][3];
            atm_profile.meridional_wind_array[i] = atm_data->atm_array[start_index + i][5];
            atm_profile.zonal_wind_array[i] = atm_data->atm_array[start_index + i][7];
            atm_profile.vertical_wind_array[i] = atm_data->atm_array[start_index + i][9];
        }
    }
    
    return atm_profile;
}

atm_cond get_atm_cond_profile(double altitude, atmprofile *atm_profile){
    /*
    Calculates the atmospheric conditions at a given altitude using a model based on EarthGRAM 2016 results

    INPUTS:
    ----------
        altitude: double
            altitude in meters
        atm_profile: atmprofile *
            pointer to the atmospheric profile
    OUTPUT:
    ----------
        atm_conditions: atm_cond
            local atmospheric conditions
    */

    atm_cond atm_conditions;
    if (altitude >= 99000){
        atm_conditions.altitude = altitude;
        atm_conditions.density = 0;
        atm_conditions.meridional_wind = 0;
        atm_conditions.zonal_wind = 0;
        atm_conditions.vertical_wind = 0;
        return atm_conditions;

    }

    if (altitude < 0){
        altitude = 0;
    }

    atm_conditions.altitude = altitude;

    // Interpolate the density
    atm_conditions.density = array_linterp(atm_profile->alt_array, atm_profile->density_array, 100, altitude);

    // Interpolate the wind components
    atm_conditions.meridional_wind = array_linterp(atm_profile->alt_array, atm_profile->meridional_wind_array, 100, altitude);
    atm_conditions.zonal_wind = array_linterp(atm_profile->alt_array, atm_profile->zonal_wind_array, 100, altitude);
    atm_conditions.vertical_wind = array_linterp(atm_profile->alt_array, atm_profile->vertical_wind_array, 100, altitude);

    return atm_conditions;
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
    if (altitude < 5000 && altitude >= 0){
        atm_conditions.density = atm_model->sea_level_density * exp(-altitude/atm_model->scale_height);
        atm_conditions.density += atm_model->pert_densities[0] * atm_conditions.density;
    }
    else if (altitude < 50000){
        atm_conditions.density = atm_model->sea_level_density * exp(-altitude/atm_model->scale_height);
        atm_conditions.density += atm_model->pert_densities[1] * atm_conditions.density;
    }
    else if (altitude < 100000){
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
    else if (altitude < 100000){
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

atm_cond get_atm_cond(double altitude, atm_model *atm_model, runparams *run_params){
    /*
    Calculates the atmospheric conditions at a given altitude

    INPUTS:
    ----------
        altitude: double
            altitude in meters
        atm_model: atm_model *
            pointer to the atmospheric model
        run_params: runparams *
            pointer to the run parameters struct
    OUTPUT:
    ----------
        atm_conditions: atm_cond
            local atmospheric conditions
    */

    atm_cond atm_conditions;
    if (run_params->atm_error == 0){
        atm_conditions = get_exp_atm_cond(altitude, atm_model);
    }
    else{
        atm_conditions = get_pert_atm_cond(altitude, atm_model);
    }
    
    return atm_conditions;
}
#endif