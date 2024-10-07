#include <tau/tau.h>
#include "../src/include/maneuverability.h"

TEST(maneuverability, instant_maneuv){
    // Get the true state and the aimpoint
    runparams run_params;
    run_params.x_aim = 6371e3;
    run_params.y_aim = 0;
    run_params.z_aim = 0;

    state true_state;
    true_state.x = 6371e3 + 10;
    true_state.y = 0;
    true_state.z = 0;
    true_state.vx = -1;
    true_state.vy = 1;
    true_state.vz = 0;
    true_state.ax_grav = 0;
    true_state.ay_grav = 0;
    true_state.az_grav = 0;
    true_state.ax_drag = 0;
    true_state.ay_drag = 0;
    true_state.az_drag = 0;
    true_state.ax_lift = 0;
    true_state.ay_lift = 0;
    true_state.az_lift = 0;
    true_state.ax_thrust = 0;
    true_state.ay_thrust = 0;
    true_state.az_thrust = 0;

    state estimated_state = true_state;

    // Get the commanded acceleration vector
    cart_vector a_command;
    a_command = prop_nav(&run_params, &estimated_state);

    // Get the updated state
    true_state = instant_maneuv(&true_state, &a_command);

    // Verify that the updated state is correct
    REQUIRE_EQ(true_state.ax_lift, a_command.x);
    REQUIRE_EQ(true_state.ay_lift, a_command.y);
    REQUIRE_EQ(true_state.az_lift, a_command.z);
    REQUIRE_EQ(true_state.ax_total, true_state.ax_grav + true_state.ax_drag + true_state.ax_lift + true_state.ax_thrust);
    REQUIRE_EQ(true_state.ay_total, true_state.ay_grav + true_state.ay_drag + true_state.ay_lift + true_state.ay_thrust);
    REQUIRE_EQ(true_state.az_total, true_state.az_grav + true_state.az_drag + true_state.az_lift + true_state.az_thrust);

}

TEST(maneuverability, perfect_maneuv){
    // initialize the true state
    state true_state;
    true_state.x = 6371e3 + 10;
    true_state.y = 0;
    true_state.z = 0;
    true_state.theta_lat = 0;
    true_state.vx = -1;
    true_state.vy = 1;
    true_state.vz = 0;
    true_state.ax_grav = 0;
    true_state.ay_grav = 0;
    true_state.az_grav = 0;
    true_state.ax_drag = 0;
    true_state.ay_drag = 0;
    true_state.az_drag = 0;
    true_state.ax_lift = 0;
    true_state.ay_lift = 0;
    true_state.az_lift = 0;
    true_state.ax_thrust = 0;
    true_state.ay_thrust = 0;
    true_state.az_thrust = 0;

    state estimated_state = true_state;
    state desired_state = true_state;

    // For desired = estimated, the true state should not change
    state new_true_state = perfect_maneuv(&true_state, &estimated_state, &desired_state);
    REQUIRE_EQ(new_true_state.x, true_state.x);
    REQUIRE_EQ(new_true_state.y, true_state.y);
    REQUIRE_EQ(new_true_state.z, true_state.z);
    REQUIRE_EQ(new_true_state.vx, true_state.vx);
    REQUIRE_EQ(new_true_state.vy, true_state.vy);
    REQUIRE_EQ(new_true_state.vz, true_state.vz);
    REQUIRE_EQ(new_true_state.ax_grav, true_state.ax_grav);
    REQUIRE_EQ(new_true_state.ay_grav, true_state.ay_grav);
    REQUIRE_EQ(new_true_state.az_grav, true_state.az_grav);
    REQUIRE_EQ(new_true_state.ax_drag, true_state.ax_drag);
    REQUIRE_EQ(new_true_state.ay_drag, true_state.ay_drag);
    REQUIRE_EQ(new_true_state.az_drag, true_state.az_drag);
    REQUIRE_EQ(new_true_state.ax_lift, true_state.ax_lift);
    REQUIRE_EQ(new_true_state.ay_lift, true_state.ay_lift);
    REQUIRE_EQ(new_true_state.az_lift, true_state.az_lift);
    REQUIRE_EQ(new_true_state.ax_thrust, true_state.ax_thrust);
    REQUIRE_EQ(new_true_state.ay_thrust, true_state.ay_thrust);
    REQUIRE_EQ(new_true_state.az_thrust, true_state.az_thrust);

    // For desired.x = estimated.x + 1, the true state should change by 1
    desired_state.x = estimated_state.x + 1;
    new_true_state = perfect_maneuv(&true_state, &estimated_state, &desired_state);
    REQUIRE_EQ(new_true_state.x, true_state.x + 1);
    REQUIRE_EQ(new_true_state.y, true_state.y);
    REQUIRE_EQ(new_true_state.z, true_state.z);

    // For desired.y = estimated.y - 1, the true state should change by 1
    desired_state.y = estimated_state.y - 1;
    new_true_state = perfect_maneuv(&true_state, &estimated_state, &desired_state);
    REQUIRE_EQ(new_true_state.y, true_state.y - 1);

}