#include <tau/tau.h>
#include "../src/include/atmosphere.h"

TEST(atmosphere, get_exp_atm_cond){
    atm_cond atm_conditions = get_exp_atm_cond(0);
    double expected, tolerance;
    tolerance = 1e-2;

    expected = 288.15;
    REQUIRE_LT(fabs(atm_conditions.temperature - expected), tolerance);

    expected = 101325;
    REQUIRE_LT(fabs(atm_conditions.pressure - expected), tolerance);

    expected = 1.225;
    REQUIRE_LT(fabs(atm_conditions.density - expected), tolerance);

    expected = 0;
    REQUIRE_LT(fabs(atm_conditions.meridional_wind - expected), tolerance);

    expected = 0;
    REQUIRE_LT(fabs(atm_conditions.zonal_wind - expected), tolerance);

    expected = 0;
    REQUIRE_LT(fabs(atm_conditions.vertical_wind - expected), tolerance);
}