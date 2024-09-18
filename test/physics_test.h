#include <tau/tau.h>
#include "../src/include/physics.h"

TEST(physics, update_mass){
    // Define a state struct
    state state;
    // Define a vehicle struct
    vehicle vehicle;

    // Initialize the vehicle
    vehicle.booster = init_mmiii_booster();
    vehicle.rv = init_ballistic_rv();
    double time_step = 1;

    vehicle.total_mass = vehicle.booster.total_mass + vehicle.rv.rv_mass;
    vehicle.current_mass = vehicle.total_mass;
    state.t = 0;
    
    // Loop through the burn time of the booster
    for (int i = 0; i <= vehicle.booster.total_burn_time + time_step; i++){
        state.t = i * time_step;

        update_mass(&vehicle, state.t);

        // Check that mass is always positive
        REQUIRE_GT(vehicle.current_mass, 0);
        
        // First time step
        if (state.t == time_step){
            REQUIRE_EQ(vehicle.current_mass, vehicle.total_mass - time_step * vehicle.booster.fuel_burn_rate[0]);
        }

        // First stage separation
        if (state.t == vehicle.booster.burn_time[0] + time_step){
            REQUIRE_EQ(vehicle.current_mass, vehicle.total_mass - vehicle.booster.wet_mass[0] - time_step*vehicle.booster.fuel_burn_rate[1]);
        }

        // Second stage separation
        if (state.t == vehicle.booster.burn_time[0] + vehicle.booster.burn_time[1] + time_step){
            REQUIRE_EQ(vehicle.current_mass, vehicle.total_mass - vehicle.booster.wet_mass[0] - vehicle.booster.wet_mass[1] - time_step*vehicle.booster.fuel_burn_rate[2]);
        }

        // After separation
        if (state.t == vehicle.booster.total_burn_time + time_step){
            REQUIRE_EQ(vehicle.current_mass, vehicle.rv.rv_mass);
        }
    }

}

TEST(physics, update_gravity){
    // Define a grav struct
    grav grav;
    // Define a state struct
    state state;

    // Initialize the grav struct
    grav = init_grav();

    // Initialize the state struct with the vehicle at one earth radius
    state.x = grav.earth_radius;
    state.y = 0;
    state.z = 0;

    // Update the gravity acceleration components
    update_gravity(&grav, &state);

    // Check that the gravitational acceleration components are correct
    REQUIRE_LT(state.ax_grav + 9.81, 0.01);
    REQUIRE_EQ(state.ay_grav, 0);
    REQUIRE_EQ(state.az_grav, 0);

    // Move the vehicle to a different location
    state.x = grav.earth_radius / sqrt(2);
    state.y = grav.earth_radius / sqrt(2);
    state.z = 0;

    // Update the gravity acceleration components
    update_gravity(&grav, &state);
    
    // Check that the gravitational acceleration components are correct
    double r = sqrt(state.x*state.x + state.y*state.y + state.z*state.z);
    double ar_grav = -grav.grav_const * grav.earth_mass / (r*r);
    double ar_grav_surface = ar_grav;
    REQUIRE_EQ(state.ax_grav, ar_grav * state.x / r);
    REQUIRE_EQ(state.ay_grav, ar_grav * state.y / r);
    REQUIRE_EQ(state.az_grav, ar_grav * state.z / r);
    REQUIRE_EQ(ar_grav, -sqrt(state.ax_grav*state.ax_grav + state.ay_grav*state.ay_grav + state.az_grav*state.az_grav));

    // Move the vehicle to a different height
    state.x = grav.earth_radius / sqrt(2) + 1000;
    state.y = grav.earth_radius / sqrt(2) + 1000;
    state.z = 1000;

    // Update the gravity acceleration components
    update_gravity(&grav, &state);

    // Check that the gravitational acceleration components are correct
    r = sqrt(state.x*state.x + state.y*state.y + state.z*state.z);
    ar_grav = -grav.grav_const * grav.earth_mass / (r*r);
    REQUIRE_LT(state.ax_grav, 0);
    REQUIRE_LT(state.ay_grav, 0);
    REQUIRE_LT(state.az_grav, 0);
    REQUIRE_GT(ar_grav, ar_grav_surface);
    REQUIRE_EQ(ar_grav, -sqrt(state.ax_grav*state.ax_grav + state.ay_grav*state.ay_grav + state.az_grav*state.az_grav));
}

TEST(physics, update_drag){

}

TEST(physics, update_thrust){

}