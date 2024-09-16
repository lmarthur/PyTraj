// includes
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// define pi constant
#define PI 3.14159265358979323846

// define the WGS84 structure
typedef struct WGS84{
    double semi_major_axis;
    double semi_minor_axis;
    double flattening;
    double e1sq; // first eccentricity squared

} WGS84;

WGS84 init_WGS84(){
    WGS84 wgs84;
    wgs84.semi_major_axis = 6378137.0;
    wgs84.flattening = 1.0 / 298.257223563;
    wgs84.semi_minor_axis = wgs84.semi_major_axis * (1 - wgs84.flattening);
    wgs84.e1sq = (pow(wgs84.semi_major_axis, 2) - pow(wgs84.semi_minor_axis, 2)) / pow(wgs84.semi_major_axis, 2);

    return wgs84;
}

double get_prime_vertical_radius_of_curvature(double geodetic_lat, WGS84 wgs84){
    /*
    Calculate the prime vertical radius of curvature

    INPUTS:
    ----------
        geodetic_lat: double
            geodetic latitude in radians
        wgs84: WGS84
            WGS84 structure
    OUTPUT:
    ----------
        primt_vertical_roc: double
            prime vertical radius of curvature
    */

    double prime_vertical_roc = wgs84.semi_major_axis / sqrt(1.0 - wgs84.e1sq * pow(sin(geodetic_lat), 2));

    return prime_vertical_roc;

}

double get_surface_radius(double geodetic_lat, WGS84 wgs84){
    /*
    Calculate the distance from the geometric center of the ellipsoid to the surface at the given geodetic latitude (surface radius)
    
    INPUTS:
    ----------
        geodetic_lat: double
            geodetic latitude in radians
        wgs84: WGS84
            WGS84 structure
    OUTPUT:
    ----------
        surface_radius: double
            distance from the geometric center of the ellipsoid to the surface at the given geodetic latitude
    */
    
    double surface_radius = sqrt((pow(wgs84.semi_major_axis*wgs84.semi_major_axis*cos(geodetic_lat), 2) + pow(wgs84.semi_minor_axis*wgs84.semi_minor_axis*sin(geodetic_lat), 2))/(pow(wgs84.semi_major_axis*cos(geodetic_lat), 2) + pow(wgs84.semi_minor_axis*sin(geodetic_lat), 2)));
    return surface_radius;

}

double get_height(double x, double y, double z, WGS84 wgs84){
    /*
    Calculate the height above the ellipsoid surface given the geocentric coordinates

    INPUTS:
    ----------
        x: double
            geocentric x coordinate
        y: double
            geocentric y coordinate
        z: double
            geocentric z coordinate
    OUTPUT:
    ----------
        height: double
            height above the ellipsoid surface
    */

    double r = sqrt(x*x + y*y + z*z);
    double lon = atan2(y, x);
    double geodetic_lat = atan2(z, sqrt(x*x + y*y)); // CHECK THIS
    double parametric_lat = atan((wgs84.semi_minor_axis / wgs84.semi_major_axis) * tan(geodetic_lat));
    
    // geocentric radius of point on ellipsoid surface
    double surface_radius = get_surface_radius(geodetic_lat, wgs84);

    double hz = z - wgs84.semi_minor_axis*sin(parametric_lat);
    double hx = x - wgs84.semi_major_axis*cos(parametric_lat)*cos(lon);
    double hy = y - wgs84.semi_major_axis*cos(parametric_lat)*sin(lon);
    double height = sqrt(hx*hx + hy*hy + hz*hz);

    if (r < surface_radius) {
        height = -height;
    } 

    return height;
}

double get_radius(double height, double geodetic_lat, double lon, WGS84 wgs84){
    /*
    Calculate the geocentric radius of a given point

    INPUTS:
    ----------
        height: double
            height above the ellipsoid surface
        geodetic_lat: double
            geodetic latitude in radians
        lon: double
            longitude in radians
        wgs84: WGS84
            WGS84 structure
    OUTPUT:
    ----------
        radius: double
            geocentric radius of the given point
    */
    double parametric_lat = atan((wgs84.semi_minor_axis / wgs84.semi_major_axis) * tan(geodetic_lat));

    double x = height*cos(geodetic_lat)*cos(lon) + wgs84.semi_major_axis*cos(parametric_lat)*cos(lon);
    double y = height*cos(geodetic_lat)*sin(lon) + wgs84.semi_major_axis*cos(parametric_lat)*sin(lon);
    double z = height*sin(geodetic_lat) + wgs84.semi_minor_axis*sin(parametric_lat);
    
    double radius = sqrt(x*x+y*y+z*z);

    return radius;
}

double geodetic_to_geocentric(double geodetic_lat, double height, WGS84 wgs84){
    /*
    Convert geodetic latitude to geocentric latitude

    INPUTS:
    ----------
        geodetic_lat: double
            geodetic latitude in radians
        height: double
            height above the ellipsoid surface
        wgs84: WGS84
            WGS84 structure
    OUTPUT:
    ----------
        geocentric_lat: double
            geocentric latitude in radians
    */
    double prime_vertical_roc = get_prime_vertical_radius_of_curvature(geodetic_lat, wgs84);
    double geocentric_lat = atan(tan(geodetic_lat)*(prime_vertical_roc*(1-wgs84.flattening)*(1-wgs84.flattening)+height)/(prime_vertical_roc+height));

    return geocentric_lat;
}

double geocentric_to_geodetic(double geocentric_lat, double radius, WGS84 wgs84){
    /*
    Convert geocentric latitude to geodetic latitude

    INPUTS:
    ----------
        geocentric_lat: double
            geocentric latitude in radians
        radius: double
            geocentric radius
        wgs84: WGS84
            WGS84 structure
    OUTPUT:
    ----------
        geodetic_lat: double
            geodetic latitude in radians
    */
    
    // get x and z coordinates
    double x = radius*cos(geocentric_lat);
    double z = radius*sin(geocentric_lat);
    double y = 0.0;

    // calculate geodetic latitude
    geodetic_lat = atan(z / sqrt(x*x + y*y));

    return geodetic_lat;
}