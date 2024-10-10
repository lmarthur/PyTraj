#include <tau/tau.h>
#include "../src/include/filters.h"

TEST(kalman, kalman_filter_alloc){
    KalmanFilter *kf = kalman_filter_alloc(2, 3);
    REQUIRE_EQ(kf->timestep, 0);
    REQUIRE_EQ(kf->state_dim, 2);
    REQUIRE_EQ(kf->measurement_dim, 3);
    REQUIRE_EQ(kf->predicted_state->size, 2);
    REQUIRE_EQ(kf->dynamic_matrix->size1, 2);
    REQUIRE_EQ(kf->dynamic_matrix->size2, 2);
    REQUIRE_EQ(kf->predicted_covariance->size1, 2);
    REQUIRE_EQ(kf->predicted_covariance->size2, 2);
    REQUIRE_EQ(kf->process_noise->size1, 2);
    REQUIRE_EQ(kf->process_noise->size2, 2);
    REQUIRE_EQ(kf->measurement_matrix->size1, 3);
    REQUIRE_EQ(kf->measurement_matrix->size2, 2);
    REQUIRE_EQ(kf->innovation_covariance->size1, 3);
    REQUIRE_EQ(kf->innovation_covariance->size2, 3);
    REQUIRE_EQ(kf->measurement_noise->size1, 3);
    REQUIRE_EQ(kf->measurement_noise->size2, 3);
    REQUIRE_EQ(kf->kalman_gain->size1, 2);
    REQUIRE_EQ(kf->kalman_gain->size2, 3);
    REQUIRE_EQ(kf->measured_state->size, 3);
    kalman_filter_free(kf);
}

TEST(kalman, kalman_filter_predict){
    // Initialize the Kalman filter
    KalmanFilter *kf = kalman_filter_alloc(2, 3);
    gsl_matrix_set(kf->dynamic_matrix, 0, 0, 1);
    gsl_matrix_set(kf->dynamic_matrix, 0, 1, 0);
    gsl_matrix_set(kf->dynamic_matrix, 1, 0, 0);
    gsl_matrix_set(kf->dynamic_matrix, 1, 1, 1);

    // With no dynamics or noise, the predicted state should be the same as the previous state
    gsl_vector_set(kf->predicted_state, 0, 1);
    gsl_vector_set(kf->predicted_state, 1, 2);

    // set noise matrices to zero
    gsl_matrix_set_zero(kf->process_noise);
    gsl_matrix_set_zero(kf->predicted_covariance);
    
    kalman_filter_predict(kf);

    REQUIRE_EQ(gsl_vector_get(kf->predicted_state, 0), 1);
    REQUIRE_EQ(gsl_vector_get(kf->predicted_state, 1), 2);

    // Free memory
    kalman_filter_free(kf);

}

TEST(kalman, kalman_filter_update){
    // Initialize the Kalman filter
    KalmanFilter *kf = kalman_filter_alloc(2, 3);
    gsl_matrix_set(kf->dynamic_matrix, 0, 0, 1);
    gsl_matrix_set(kf->dynamic_matrix, 0, 1, 0);
    gsl_matrix_set(kf->dynamic_matrix, 1, 0, 0);
    gsl_matrix_set(kf->dynamic_matrix, 1, 1, 1);

    // With no dynamics or noise, the predicted state should be the same as the previous state
    gsl_vector_set(kf->predicted_state, 0, 1);
    gsl_vector_set(kf->predicted_state, 1, 2);

    // set noise matrices to zero
    gsl_matrix_set_zero(kf->process_noise);
    gsl_matrix_set_zero(kf->predicted_covariance);

    // Set the measurement matrix to the identity
    gsl_matrix_set_identity(kf->measurement_matrix);

    // Set the measurement noise to zero
    gsl_matrix_set_zero(kf->measurement_noise);

    // Set the measured state
    gsl_vector_set(kf->measured_state, 0, 1);
    gsl_vector_set(kf->measured_state, 1, 2);
    gsl_vector_set(kf->measured_state, 2, 3);

    kalman_filter_predict(kf);
    kalman_filter_update(kf);

    REQUIRE_EQ(gsl_vector_get(kf->predicted_state, 0), 1);
    REQUIRE_EQ(gsl_vector_get(kf->predicted_state, 1), 2);

    // Free memory
    kalman_filter_free(kf);

}