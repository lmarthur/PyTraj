#ifndef UTILS_H
#define UTILS_H

#include <math.h>
#include <stdio.h>

void cartcoords_to_sphercoords(double *cart_coords, double *spher_coords){
    /*
    Converts Cartesian coordinates to spherical coordinates

    INPUTS:
    ----------
        cart_coords: double *
            pointer to Cartesian coordinates [x, y, z]
        spher_coords: double *
            pointer to spherical coordinates [r, long, lat]
    */

    // Calculate the radial coordinate
    spher_coords[0] = sqrt(cart_coords[0]*cart_coords[0] + cart_coords[1]*cart_coords[1] + cart_coords[2]*cart_coords[2]);

    // Calculate the longitudinal coordinate
    spher_coords[1] = atan2(cart_coords[1], cart_coords[0]);

    // Calculate the latitudinal coordinate
    spher_coords[2] = atan(cart_coords[2] / sqrt(cart_coords[0]*cart_coords[0] + cart_coords[1]*cart_coords[1]));
}

void sphercoords_to_cartcoords(double *spher_coords, double *cart_coords){
    /*
    Converts spherical coordinates to Cartesian coordinates

    INPUTS:
    ----------
        spher_coords: double *
            pointer to spherical coordinates [r, long, lat]
        cart_coords: double *
            pointer to Cartesian coordinates [x, y, z]
    */

    // Calculate the x-coordinate
    cart_coords[0] = spher_coords[0] * cos(spher_coords[1]) * cos(spher_coords[2]);

    // Calculate the y-coordinate
    cart_coords[1] = spher_coords[0] * sin(spher_coords[1]) * cos(spher_coords[2]);

    // Calculate the z-coordinate
    cart_coords[2] = spher_coords[0] * sin(spher_coords[2]);

}

void sphervec_to_cartvec(double *sphervec, double *cartvec, double *spher_coords){
    /*
    Converts a spherical vector to a Cartesian vector at a given set of spherical coordinates

    INPUTS:
    ----------
        sphervec: double *
            pointer to spherical vector [r, long, lat]
        cartvec: double *
            pointer to Cartesian vector [x, y, z]
        spher_coords: double *
            pointer to spherical coordinates [r, long, lat]
    */

    // Get th x-component of the spherical vector
    cartvec[0] = sphervec[0] * cos(spher_coords[1]) * cos(spher_coords[2]);

    // Get the y-component of the spherical vector
    cartvec[1] = sphervec[0] * sin(spher_coords[1]) * cos(spher_coords[2]);

    // Get the z-component of the spherical vector
    cartvec[2] = sphervec[0] * sin(spher_coords[2]);
    

}

#endif