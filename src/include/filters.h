#ifndef FILTERS_H
#define FILTERS_H

#include "linalg.h"

typedef struct KalmanFilter{
    /* 
    This struct defines a Kalman filter and holds the associated data structures.
    */
    int timestep; // The current time step t.
    int state_dim; // The dimension of the state space.
    int measurement_dim; // The dimension of the measurement space.
    gsl_vector *predicted_state; // The predicted state of the system at time t.
    gsl_matrix *dynamic_matrix; // The dynamic matrix of the system at time t.
    gsl_matrix *predicted_covariance; // The predicted covariance of the system at time t.
    gsl_matrix *process_noise; // The process noise of the system at time t.
    gsl_matrix *measurement_matrix; // The measurement matrix of the system at time t.
    gsl_matrix *innovation_covariance; // The innovation covariance of the system at time t.
    gsl_matrix *measurement_noise; // The measurement noise of the system at time t.
    gsl_matrix *kalman_gain; // The Kalman gain of the system at time t.
    gsl_vector *measured_state; // The measured state of the system at time t.
} KalmanFilter;

KalmanFilter *kalman_filter_alloc(int state_dim, int measurement_dim){
    /* 
    This function allocates memory for a Kalman filter.

    INPUTS:
    ----------------
        state_dim (int): The dimension of the state space.
        measurement_dim (int): The dimension of the measurement space.

    OUTPUTS:
    ----------------
        kf (KalmanFilter *): The Kalman filter.
    */

    KalmanFilter *kf = (KalmanFilter *)malloc(sizeof(KalmanFilter));
    kf->timestep = 0;
    kf->state_dim = state_dim;
    kf->measurement_dim = measurement_dim;
    kf->predicted_state = gsl_vector_alloc(state_dim);
    kf->dynamic_matrix = gsl_matrix_alloc(state_dim, state_dim);
    kf->predicted_covariance = gsl_matrix_alloc(state_dim, state_dim);
    kf->process_noise = gsl_matrix_alloc(state_dim, state_dim);
    kf->measurement_matrix = gsl_matrix_alloc(measurement_dim, state_dim);
    kf->innovation_covariance = gsl_matrix_alloc(measurement_dim, measurement_dim);
    kf->measurement_noise = gsl_matrix_alloc(measurement_dim, measurement_dim);
    kf->kalman_gain = gsl_matrix_alloc(state_dim, measurement_dim);
    kf->measured_state = gsl_vector_alloc(measurement_dim);
    return kf;
}

void kalman_filter_free(KalmanFilter *kf){
    /* 
    This function frees the memory associated with a Kalman filter.

    INPUTS:
    ----------------
        kf (KalmanFilter *): The Kalman filter.
    */

    gsl_vector_free(kf->predicted_state);
    gsl_matrix_free(kf->dynamic_matrix);
    gsl_matrix_free(kf->predicted_covariance);
    gsl_matrix_free(kf->process_noise);
    gsl_matrix_free(kf->measurement_matrix);
    gsl_matrix_free(kf->innovation_covariance);
    gsl_matrix_free(kf->measurement_noise);
    gsl_matrix_free(kf->kalman_gain);
    gsl_vector_free(kf->measured_state);
    free(kf);

}

void kalman_filter_predict(KalmanFilter *kf){
    /* 
    This function predicts the state of the system at time t.

    INPUTS:
    ----------------
        kf (KalmanFilter *): The Kalman filter.
    */

    // Predict the state
    kf->predicted_state = mv_multiply(kf->dynamic_matrix, kf->predicted_state);

    // Predict the covariance
    gsl_matrix *tmp = mm_multiply(kf->dynamic_matrix, kf->predicted_covariance);
    gsl_matrix *tmp_t = m_transpose(kf->dynamic_matrix);
    gsl_matrix *tmp2 = mm_multiply(tmp, tmp_t);
    kf->predicted_covariance = mm_add(tmp2, kf->process_noise);
    
    // Increment the timestep
    kf->timestep += 1;

    // Free memory
    gsl_matrix_free(tmp);
    gsl_matrix_free(tmp_t);
    gsl_matrix_free(tmp2);

}

void kalman_filter_update(KalmanFilter *kf){
    /* 
    This function updates the state of the system at time t.

    INPUTS:
    ----------------
        kf (KalmanFilter *): The Kalman filter.
    */

    // Compute the innovation covariance
    gsl_matrix *tmp = mm_multiply(kf->measurement_matrix, kf->predicted_covariance);
    gsl_matrix *tmp_t = m_transpose(kf->measurement_matrix);
    gsl_matrix *tmp2 = mm_multiply(tmp, tmp_t);
    kf->innovation_covariance = mm_add(tmp2, kf->measurement_noise);

    // Compute the Kalman gain
    gsl_matrix *tmp3 = m_pseudoinverse(kf->innovation_covariance);
    gsl_matrix *tmp4 = mm_multiply(kf->predicted_covariance, tmp_t);
    kf->kalman_gain = mm_multiply(tmp4, tmp3);

    // Update the state

}

#endif