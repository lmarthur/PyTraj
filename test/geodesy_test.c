#include <tau/tau.h>
#include "../src/include/geodesy.h"

TAU_MAIN()

TEST(wgs84, prime_vertical_roc){
    WGS84 wgs84 = init_WGS84();
    double geodetic_lat, prime_vertical_roc, error, tolerance, expected;
    tolerance = 10;

    geodetic_lat = 0.0;
    expected = wgs84.semi_major_axis;
    prime_vertical_roc = get_prime_vertical_radius_of_curvature(geodetic_lat, wgs84);
    REQUIRE_LT(fabs(prime_vertical_roc - expected), tolerance);

    geodetic_lat = PI / 4;
    expected = 6388837.68116;
    prime_vertical_roc = get_prime_vertical_radius_of_curvature(geodetic_lat, wgs84);
    REQUIRE_LT(fabs(prime_vertical_roc - expected), tolerance);

    geodetic_lat = PI / 2;
    expected = 6399592.40167;
    prime_vertical_roc = get_prime_vertical_radius_of_curvature(geodetic_lat, wgs84);
    REQUIRE_LT(fabs(prime_vertical_roc - expected), tolerance);

}

TEST(wgs84, surface_radius){
    WGS84 wgs84 = init_WGS84();
    double geodetic_lat, surface_radius, error, tolerance, expected;
    tolerance = 1;

    geodetic_lat = 0.0;
    expected = wgs84.semi_major_axis;
    surface_radius = get_surface_radius(geodetic_lat, wgs84);
    error = fabs((surface_radius - expected));
    REQUIRE_LT(error, tolerance);

    geodetic_lat = PI / 4;
    expected = 6367490.1467;
    surface_radius = get_surface_radius(geodetic_lat, wgs84);
    error = fabs((surface_radius - expected));
    REQUIRE_LT(error, tolerance);

    geodetic_lat = PI / 2;
    expected = wgs84.semi_minor_axis;
    surface_radius = get_surface_radius(geodetic_lat, wgs84);
    error = fabs((surface_radius - expected));
    REQUIRE_LT(error, tolerance);

}

TEST(wgs84, height){
    WGS84 wgs84 = init_WGS84();
    double height, tolerance, expected;
    tolerance = 0.1;

    // north pole
    expected = 0.0;
    height = get_height(0.0, 0.0, wgs84.semi_minor_axis, wgs84);
    REQUIRE_LT(fabs(height - expected), tolerance);

    // south pole
    expected = 0.0;
    height = get_height(0.0, 0.0, -wgs84.semi_minor_axis, wgs84);
    REQUIRE_LT(fabs(height - expected), tolerance);

    // equator at 0 longitude
    expected = 0.0;
    height = get_height(wgs84.semi_major_axis, 0.0, 0.0, wgs84);
    REQUIRE_LT(fabs(height - expected), tolerance);

    // equator at 90 longitude
    expected = 0.0;
    height = get_height(0.0, wgs84.semi_major_axis, 0.0, wgs84);
    REQUIRE_LT(fabs(height - expected), tolerance);

    // 45 latitude, 45 longitude, 0 height
    expected = 0.0;
    height = get_height(wgs84.semi_major_axis / sqrt(2), wgs84.semi_major_axis / sqrt(2), 0.0, wgs84);
    REQUIRE_LT(fabs(height - expected), tolerance);

    // positive height
    expected = 1000.0;
    height = get_height(wgs84.semi_major_axis + 1000.0, 0.0, 0.0, wgs84);
    REQUIRE_LT(fabs(height - expected), tolerance);

    // negative height
    expected = -1000.0;
    height = get_height(wgs84.semi_major_axis - 1000.0, 0.0, 0.0, wgs84);
    REQUIRE_LT(fabs(height - expected), tolerance);

}

TEST(wgs84, radius){
    WGS84 wgs84 = init_WGS84();
    double radius, tolerance, expected;
    tolerance = 0.1;

    // north pole
    expected = wgs84.semi_minor_axis;
    radius = get_radius(0.0, PI / 2, 0.0, wgs84);
    REQUIRE_LT(fabs(radius - expected), tolerance);

    // south pole
    expected = wgs84.semi_minor_axis;
    radius = get_radius(0.0, -PI / 2, 0.0, wgs84);
    REQUIRE_LT(fabs(radius - expected), tolerance);

    // equator at 0 longitude
    expected = wgs84.semi_major_axis;
    radius = get_radius(0.0, 0.0, 0.0, wgs84);
    REQUIRE_LT(fabs(radius - expected), tolerance);

    // equator at 90 longitude
    expected = wgs84.semi_major_axis;
    radius = get_radius(0.0, 0.0, PI / 2, wgs84);
    REQUIRE_LT(fabs(radius - expected), tolerance);

    // positive height
    expected = wgs84.semi_major_axis + 1000.0;
    radius = get_radius(1000.0, 0.0, 0.0, wgs84);
    REQUIRE_LT(fabs(radius - expected), tolerance);

    // negative height
    expected = wgs84.semi_major_axis - 1000.0;
    radius = get_radius(-1000.0, 0.0, 0.0, wgs84);
    REQUIRE_LT(fabs(radius - expected), tolerance);
}

TEST(wgs84, geodetic_to_geocentric){
    WGS84 wgs84 = init_WGS84();
    double geodetic_lat, geocentric_lat, height, tolerance, expected;
    tolerance = 0.01;

    // north pole, zero height
    expected = PI / 2;
    geodetic_lat = PI / 2;
    height = 0.0;
    geocentric_lat = geodetic_to_geocentric(geodetic_lat, height, wgs84);
    REQUIRE_LT(fabs(geocentric_lat - expected), tolerance);

    // equator, zero height
    expected = 0.0;
    geodetic_lat = 0.0;
    height = 0.0;
    geocentric_lat = geodetic_to_geocentric(geodetic_lat, height, wgs84);

    // equator, 1000m height
    expected = 0.0;
    geodetic_lat = 0.0;
    height = 1000.0;
    geocentric_lat = geodetic_to_geocentric(geodetic_lat, height, wgs84);
    REQUIRE_LT(fabs(geocentric_lat - expected), tolerance);

    // north pole, 1000m height
    expected = PI / 2;
    geodetic_lat = PI / 2;
    height = 1000.0;
    geocentric_lat = geodetic_to_geocentric(geodetic_lat, height, wgs84);
    REQUIRE_LT(fabs(geocentric_lat - expected), tolerance);

}