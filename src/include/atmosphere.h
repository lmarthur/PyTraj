#ifndef ATMOSPHERE_H
#define ATMOSPHERE_H

#include <math.h>

// Define an atm_cond struct to store local atmospheric conditions
typedef struct atm_cond{
    double altitude; // altitude in meters
    double temperature; // temperature in Kelvin
    double pressure; // pressure in Pascals
    double density; // density in kg/m^3
    double meridional_wind; // meridional wind in m/s
    double zonal_wind; // zonal wind in m/s
    double vertical_wind; // vertical wind in m/s

} atm_cond;

atm_cond get_exp_atm_cond(double altitude){
    /*
    Calculates the atmospheric conditions at a given altitude using an exponential model

    INPUTS:
    ----------
        altitude: double
            altitude in meters
        wgs84: WGS84
            WGS84 structure
    OUTPUT:
    ----------
        primt_vertical_roc: double
            prime vertical radius of curvature
    */

    atm_cond atm_conditions;
    // Define constants
    double scale_height = 8000; // scale height in meters
    double sea_level_temperature = 288.15; // sea level temperature in Kelvin
    double sea_level_pressure = 101325; // sea level pressure in Pascals
    double sea_level_density = 1.225; // sea level density in kg/m^3

    atm_conditions.altitude = altitude;
    atm_conditions.temperature = sea_level_temperature * exp(-altitude/scale_height);
    atm_conditions.pressure = sea_level_pressure * exp(-altitude/scale_height);
    atm_conditions.density = sea_level_density * exp(-altitude/scale_height);
    atm_conditions.meridional_wind = 0;
    atm_conditions.zonal_wind = 0;
    atm_conditions.vertical_wind = 0;

    return atm_conditions;
}



#endif