#include <gsl/gsl_rng.h>

// Define a grav struct to store gravity parameters
typedef struct grav{
    double earth_mass; // mass of the Earth in kg
    double earth_radius; // radius of the Earth in meters
    double grav_const; // gravitational constant in m^3/kg/s^2

} grav;

// Define a function to initialize gravity parameters
grav init_grav(){
    /*
    Initializes gravity parameters

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

    return grav;
}