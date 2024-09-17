#include <tau/tau.h>
#include "../src/include/vehicle.h"

TEST(vehicle, init_ballistic_rv){
    rv rv = init_ballistic_rv();

    REQUIRE_EQ(rv.maneuverability_flag, 0);
    REQUIRE_LT(rv.c_m_alpha, 0);
    REQUIRE_LT(rv.c_m_q, 0);
    REQUIRE_EQ(rv.flap_area, 0);

}

TEST(vehicle, init_swerve_rv){
    rv rv = init_swerve_rv();

    REQUIRE_EQ(rv.maneuverability_flag, 1);
    REQUIRE_LT(rv.c_m_alpha, 0);
    REQUIRE_LT(rv.c_m_q, 0);
    REQUIRE_GT(rv.flap_area, 0);
}

TEST(vehicle, init_mmiii_booster){
    booster booster = init_mmiii_booster();

    REQUIRE_EQ(booster.num_stages, 3);
    REQUIRE_GT(booster.maxdiam, 0);
    REQUIRE_GT(booster.area, 0);
    REQUIRE_EQ(booster.total_burn_time, 188);
    REQUIRE_EQ(booster.total_mass, 34210);
}