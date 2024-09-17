#include <tau/tau.h>
#include "../src/include/gravity.h"

TEST(gravity, init_grav){
    grav grav = init_grav();

    REQUIRE_GT(grav.earth_mass, 0);
    REQUIRE_GT(grav.earth_radius, 0);
    REQUIRE_GT(grav.grav_const, 0);
}