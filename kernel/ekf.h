#ifndef EKF_H
#define EKF_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/* EKF State Vector: [q0, q1, q2, q3, bx, by, bz, vx, vy, vz, px, py, pz]^T
 * q: quaternion (attitude)
 * b: gyro bias
 * v: velocity in body frame
 * p: position in NED frame
 */
#define EKF_STATE_SIZE 13
#define EKF_MEASUREMENT_SIZE 6  /* Accel + Gyro + Mag */

typedef struct {
    /* State */
    float x[EKF_STATE_SIZE];
    float P[EKF_STATE_SIZE][EKF_STATE_SIZE];
    
    /* Covariance matrices */
    float Q[EKF_STATE_SIZE][EKF_STATE_SIZE];  /* Process noise */
    float R[EKF_MEASUREMENT_SIZE][EKF_MEASUREMENT_SIZE];  /* Measurement noise */
    
    /* Measurement */
    float z[EKF_MEASUREMENT_SIZE];
    float H[EKF_MEASUREMENT_SIZE][EKF_STATE_SIZE];
    float K[EKF_STATE_SIZE][EKF_MEASUREMENT_SIZE];  /* Kalman gain */
    
    /* Innovation */
    float innovation[EKF_MEASUREMENT_SIZE];
    float innovation_covariance[EKF_MEASUREMENT_SIZE][EKF_MEASUREMENT_SIZE];
    
    /* Status */
    bool initialized;
    uint32_t last_update;
    uint32_t update_count;
    
    /* Tuning parameters */
    float gyro_noise;
    float accel_noise;
    float mag_noise;
    float process_noise;
    
    /* Output */
    float roll;
    float pitch;
    float yaw;
    float altitude;
    float velocity[3];
    float position[3];
} EKFState;

/* Function Prototypes */
void ekf_init(EKFState *state);
void ekf_predict(EKFState *state, float *gyro, float *accel, float dt);
void ekf_update_mag(EKFState *state, float *mag);
void ekf_update_gps(EKFState *state, double lat, double lon, float alt);
void ekf_update_baro(EKFState *state, float pressure, float temperature);
void ekf_get_attitude(EKFState *state, float *roll, float *pitch, float *yaw);
void ekf_get_position(EKFState *state, float *x, float *y, float *z);
void ekf_get_velocity(EKFState *state, float *vx, float *vy, float *vz);
bool ekf_is_healthy(EKFState *state);

/* Helper functions */
float ekf_quaternion_to_roll(float q0, float q1, float q2, float q3);
float ekf_quaternion_to_pitch(float q0, float q1, float q2, float q3);
float ekf_quaternion_to_yaw(float q0, float q1, float q2, float q3);
void ekf_quaternion_from_euler(float roll, float pitch, float yaw, float *q);

#ifdef __cplusplus
}
#endif

#endif /* EKF_H */