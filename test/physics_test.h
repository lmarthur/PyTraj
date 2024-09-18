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

}

TEST(physics, update_drag){

}

TEST(physics, update_thrust){

}