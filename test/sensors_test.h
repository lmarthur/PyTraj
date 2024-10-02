#include <tau/tau.h>
#include "../src/include/sensors.h"

TEST(sensors, imu_init){
    // Initialize the random number generator
    const gsl_rng_type *T;
    gsl_rng *rng;
    gsl_rng_env_setup();
    T = gsl_rng_default;
    rng = gsl_rng_alloc(T);

    // Initialize the run parameters
    runparams run_params;
    run_params.acc_scale_stability = 0;
    run_params.gyro_bias_stability = 0;
    run_params.gyro_noise = 0;

    // Initialize the imu
    imu imu = imu_init(&run_params, rng);

    // Check that the imu struct is initialized correctly
    REQUIRE_EQ(imu.acc_scale_stability, run_params.acc_scale_stability);
    REQUIRE_EQ(imu.gyro_bias_stability, run_params.gyro_bias_stability);
    REQUIRE_EQ(imu.gyro_noise, run_params.gyro_noise);
    REQUIRE_EQ(imu.gyro_error_lat, 0);
    REQUIRE_EQ(imu.gyro_error_long, 0);
}

TEST(sensors, imu_meas){
    // Initialize the random number generator
    const gsl_rng_type *T;
    gsl_rng *rng;
    gsl_rng_env_setup();
    T = gsl_rng_default;
    rng = gsl_rng_alloc(T);

    // // Initialize the run parameters
    runparams run_params;
    run_params.acc_scale_stability = 0;
    run_params.gyro_bias_stability = 0;
    run_params.gyro_noise = 0;

    // Initialize the state
    state true_state = init_state(&run_params, rng);
    true_state.ax_total = 10;
    true_state.ay_total = 10;
    true_state.az_total = 10;
    // Initialize the imu
    imu imu = imu_init(&run_params, rng);

    // Check that for zero scale stability, the accelerometer errors are zero
    state meas_state = imu_measurement(&imu, &true_state, rng);
    REQUIRE_EQ(fabs(meas_state.ax_total - true_state.ax_total), 0);
    REQUIRE_EQ(fabs(meas_state.ay_total - true_state.ay_total), 0);
    REQUIRE_EQ(fabs(meas_state.az_total - true_state.az_total), 0);

    // Check that for zero acceleration the accelerometer errors are zero
    true_state.ax_total = 0;
    true_state.ay_total = 0;
    true_state.az_total = 0;
    meas_state = imu_measurement(&imu, &true_state, rng);
    REQUIRE_EQ(meas_state.ax_total, 0);
    REQUIRE_EQ(meas_state.ay_total, 0);
    REQUIRE_EQ(meas_state.az_total, 0);

    // Check that for non-zero acceleration and non-zero scale stability the accelerometer errors are non-zero
    true_state.ax_total = 10;
    true_state.ay_total = 10;
    true_state.az_total = 10;
    run_params.acc_scale_stability = 1e-3;
    imu = imu_init(&run_params, rng);
    meas_state = imu_measurement(&imu, &true_state, rng);
    REQUIRE_NE(meas_state.ax_total, true_state.ax_total);

}

TEST(sensors, imu_update){
    // Initialize the random number generator
    const gsl_rng_type *T;
    gsl_rng *rng;
    gsl_rng_env_setup();
    T = gsl_rng_default;
    rng = gsl_rng_alloc(T);

    // Initialize the run parameters
    runparams run_params;
    run_params.time_step = 1;
    run_params.acc_scale_stability = 0;

    // Initialize the imu
    imu imu = imu_init(&run_params, rng);

    // Initialize the true state
    state true_state = init_state(&run_params, rng);
    
    // Check that the gyro error is constant with zero gyro random walk and zero gyro bias
    run_params.gyro_bias_stability = 0;
    run_params.gyro_noise = 0;

    true_state.theta_long = 1;
    true_state.theta_lat = 1;

    imu = imu_init(&run_params, rng);
    state meas_state_0 = imu_measurement(&imu, &true_state, rng);
    update_imu(&imu, &run_params, rng);
    state meas_state_1 = imu_measurement(&imu, &true_state, rng);

    REQUIRE_EQ(meas_state_0.theta_long, meas_state_1.theta_long);
    REQUIRE_EQ(meas_state_0.theta_lat, meas_state_1.theta_lat);

    // Check that the gyro error is increasing with non-zero gyro random walk and zero gyro bias
    run_params.gyro_bias_stability = 0;
    run_params.gyro_noise = 1e-3;
    imu = imu_init(&run_params, rng);
    meas_state_0 = imu_measurement(&imu, &true_state, rng);
    for (int i = 0; i < 10; i++){
        update_imu(&imu, &run_params, rng);
    }
    meas_state_1 = imu_measurement(&imu, &true_state, rng);
    
    REQUIRE_LT(fabs(true_state.theta_long - meas_state_0.theta_long), fabs(true_state.theta_long - meas_state_1.theta_long));
    REQUIRE_LT(fabs(true_state.theta_lat - meas_state_0.theta_lat), fabs(true_state.theta_lat - meas_state_1.theta_lat));


    // Check that the gyro error is increasing with zero gyro random walk and non-zero gyro bias
    run_params.gyro_bias_stability = 1e-3;
    run_params.gyro_noise = 0;
    imu = imu_init(&run_params, rng);
    meas_state_0 = imu_measurement(&imu, &true_state, rng);
    for (int i = 0; i < 10; i++){
        update_imu(&imu, &run_params, rng);
    }
    meas_state_1 = imu_measurement(&imu, &true_state, rng);

    REQUIRE_LT(fabs(true_state.theta_long - meas_state_0.theta_long), fabs(true_state.theta_long - meas_state_1.theta_long));
    REQUIRE_LT(fabs(true_state.theta_lat - meas_state_0.theta_lat), fabs(true_state.theta_lat - meas_state_1.theta_lat));

}

TEST(sensors, gnss_init){
    // Initialize the run parameters
    runparams run_params;
    run_params.gnss_noise = 0;

    gnss gnss = gnss_init(&run_params);
    REQUIRE_EQ(gnss.noise, run_params.gnss_noise);

}

TEST(sensors, gnss_meas){
    // Initialize the random number generator
    const gsl_rng_type *T;
    gsl_rng *rng;
    gsl_rng_env_setup();
    T = gsl_rng_default;
    rng = gsl_rng_alloc(T);

    // Initialize the run parameters
    runparams run_params;
    run_params.gnss_noise = 0;

    // Initialize the gnss
    gnss gnss = gnss_init(&run_params);

    // Initialize the true state
    state true_state = init_state(&run_params, rng);
    true_state.x = 10;
    true_state.y = 10;
    true_state.z = 10;

    // Check that for zero gnss noise the gnss errors are zero
    state meas_state = gnss_measurement(&gnss, &true_state, rng);
    REQUIRE_EQ(fabs(meas_state.x - true_state.x), 0);
    REQUIRE_EQ(fabs(meas_state.y - true_state.y), 0);
    REQUIRE_EQ(fabs(meas_state.z - true_state.z), 0);

    // Check that for non-zero gnss noise the gnss errors are non-zero
    run_params.gnss_noise = 1e-3;
    gnss = gnss_init(&run_params);
    meas_state = gnss_measurement(&gnss, &true_state, rng);
    REQUIRE_NE(meas_state.x, true_state.x);
    REQUIRE_NE(meas_state.y, true_state.y);
    REQUIRE_NE(meas_state.z, true_state.z);
}
