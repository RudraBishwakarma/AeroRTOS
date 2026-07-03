#include "ekf.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* ============================================================================
 * Matrix Operations (3x3 and 13x13)
 * ============================================================================ */

static void mat13x13_scale(float A[EKF_STATE_SIZE][EKF_STATE_SIZE], float scale,
                           float B[EKF_STATE_SIZE][EKF_STATE_SIZE]) {
    for (int i = 0; i < EKF_STATE_SIZE; i++) {
        for (int j = 0; j < EKF_STATE_SIZE; j++) {
            B[i][j] = A[i][j] * scale;
        }
    }
}

static void mat13x13_add(float A[EKF_STATE_SIZE][EKF_STATE_SIZE],
                         float B[EKF_STATE_SIZE][EKF_STATE_SIZE],
                         float C[EKF_STATE_SIZE][EKF_STATE_SIZE]) {
    for (int i = 0; i < EKF_STATE_SIZE; i++) {
        for (int j = 0; j < EKF_STATE_SIZE; j++) {
            C[i][j] = A[i][j] + B[i][j];
        }
    }
}

/* ============================================================================
 * Quaternion Operations
 * ============================================================================ */

void ekf_quaternion_from_euler(float roll, float pitch, float yaw, float *q) {
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    
    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}

float ekf_quaternion_to_roll(float q0, float q1, float q2, float q3) {
    return atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
}

float ekf_quaternion_to_pitch(float q0, float q1, float q2, float q3) {
    return asinf(2.0f * (q0 * q2 - q3 * q1));
}

float ekf_quaternion_to_yaw(float q0, float q1, float q2, float q3) {
    return atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
}

/* ============================================================================
 * EKF Implementation
 * ============================================================================ */

void ekf_init(EKFState *state) {
    if (!state) return;
    
    memset(state, 0, sizeof(EKFState));
    
    /* Initialize quaternion to identity */
    state->x[0] = 1.0f;  /* q0 */
    state->x[1] = 0.0f;  /* q1 */
    state->x[2] = 0.0f;  /* q2 */
    state->x[3] = 0.0f;  /* q3 */
    
    /* Initialize covariance matrix (large uncertainty) */
    for (int i = 0; i < EKF_STATE_SIZE; i++) {
        for (int j = 0; j < EKF_STATE_SIZE; j++) {
            state->P[i][j] = 0.0f;
        }
        state->P[i][i] = 100.0f;  /* Large initial uncertainty */
    }
    /* Position uncertainty larger */
    for (int i = 10; i < 13; i++) {
        state->P[i][i] = 1000.0f;
    }
    
    /* Process noise (Q) - tuned for quadcopter */
    for (int i = 0; i < EKF_STATE_SIZE; i++) {
        for (int j = 0; j < EKF_STATE_SIZE; j++) {
            state->Q[i][j] = 0.0f;
        }
    }
    state->Q[0][0] = 0.0001f;   /* q0 */
    state->Q[1][1] = 0.0001f;   /* q1 */
    state->Q[2][2] = 0.0001f;   /* q2 */
    state->Q[3][3] = 0.0001f;   /* q3 */
    state->Q[4][4] = 0.001f;    /* gyro bias x */
    state->Q[5][5] = 0.001f;    /* gyro bias y */
    state->Q[6][6] = 0.001f;    /* gyro bias z */
    state->Q[7][7] = 0.01f;     /* vx */
    state->Q[8][8] = 0.01f;     /* vy */
    state->Q[9][9] = 0.01f;     /* vz */
    state->Q[10][10] = 0.1f;    /* px */
    state->Q[11][11] = 0.1f;    /* py */
    state->Q[12][12] = 0.1f;    /* pz */
    
    /* Measurement noise (R) */
    for (int i = 0; i < EKF_MEASUREMENT_SIZE; i++) {
        for (int j = 0; j < EKF_MEASUREMENT_SIZE; j++) {
            state->R[i][j] = 0.0f;
        }
    }
    state->R[0][0] = 0.01f;  /* accel x */
    state->R[1][1] = 0.01f;  /* accel y */
    state->R[2][2] = 0.01f;  /* accel z */
    state->R[3][3] = 0.001f; /* gyro x */
    state->R[4][4] = 0.001f; /* gyro y */
    state->R[5][5] = 0.001f; /* gyro z */
    
    state->initialized = true;
    state->last_update = 0;
    state->update_count = 0;
    
    printf("EKF Initialized\n");
}

void ekf_predict(EKFState *state, float *gyro, float *accel, float dt) {
    if (!state || !state->initialized) return;
    
    /* Get current quaternion */
    float q0 = state->x[0];
    float q1 = state->x[1];
    float q2 = state->x[2];
    float q3 = state->x[3];
    
    /* Remove gyro bias */
    float wx = gyro[0] - state->x[4];
    float wy = gyro[1] - state->x[5];
    float wz = gyro[2] - state->x[6];
    
    /* Quaternion derivative */
    float q0_dot = 0.5f * (-q1 * wx - q2 * wy - q3 * wz);
    float q1_dot = 0.5f * (q0 * wx + q2 * wz - q3 * wy);
    float q2_dot = 0.5f * (q0 * wy - q1 * wz + q3 * wx);
    float q3_dot = 0.5f * (q0 * wz + q1 * wy - q2 * wx);
    
    /* Update quaternion */
    state->x[0] += q0_dot * dt;
    state->x[1] += q1_dot * dt;
    state->x[2] += q2_dot * dt;
    state->x[3] += q3_dot * dt;
    
    /* Normalize quaternion */
    float norm = sqrtf(state->x[0]*state->x[0] + state->x[1]*state->x[1] +
                       state->x[2]*state->x[2] + state->x[3]*state->x[3]);
    if (norm > 0.001f) {
        state->x[0] /= norm;
        state->x[1] /= norm;
        state->x[2] /= norm;
        state->x[3] /= norm;
    }
    
    /* Update velocity (simple integration of acceleration) */
    /* Convert acceleration from body to world frame using quaternion */
    float a_body[3] = {accel[0], accel[1], accel[2]};
    float a_world[3];
    
    /* Body to world rotation using quaternion */
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;
    
    a_world[0] = (q0q0 + q1q1 - q2q2 - q3q3) * a_body[0] +
                 2.0f * (q1q2 - q0q3) * a_body[1] +
                 2.0f * (q1q3 + q0q2) * a_body[2];
    a_world[1] = 2.0f * (q1q2 + q0q3) * a_body[0] +
                 (q0q0 - q1q1 + q2q2 - q3q3) * a_body[1] +
                 2.0f * (q2q3 - q0q1) * a_body[2];
    a_world[2] = 2.0f * (q1q3 - q0q2) * a_body[0] +
                 2.0f * (q2q3 + q0q1) * a_body[1] +
                 (q0q0 - q1q1 - q2q2 + q3q3) * a_body[2];
    
    /* Remove gravity */
    a_world[2] += 9.81f;
    
    /* Update velocity and position */
    state->x[7] += a_world[0] * dt;  /* vx */
    state->x[8] += a_world[1] * dt;  /* vy */
    state->x[9] += a_world[2] * dt;  /* vz */
    state->x[10] += state->x[7] * dt;  /* px */
    state->x[11] += state->x[8] * dt;  /* py */
    state->x[12] += state->x[9] * dt;  /* pz */
    
    /* Update covariance matrix (simplified: add process noise) */
    float Q_scaled[EKF_STATE_SIZE][EKF_STATE_SIZE];
    mat13x13_scale(state->Q, dt, Q_scaled);
    mat13x13_add(state->P, Q_scaled, state->P);
    
    /* Clamp covariance to prevent divergence */
    for (int i = 0; i < EKF_STATE_SIZE; i++) {
        if (state->P[i][i] > 1000.0f) {
            state->P[i][i] = 1000.0f;
        }
    }
    
    state->last_update = 0;
    state->update_count++;
    
    /* Extract attitude */
    state->roll = ekf_quaternion_to_roll(state->x[0], state->x[1], 
                                         state->x[2], state->x[3]);
    state->pitch = ekf_quaternion_to_pitch(state->x[0], state->x[1], 
                                           state->x[2], state->x[3]);
    state->yaw = ekf_quaternion_to_yaw(state->x[0], state->x[1], 
                                       state->x[2], state->x[3]);
}

void ekf_update_gps(EKFState *state, double lat, double lon, float alt) {
    if (!state || !state->initialized) return;
    
    /* Convert lat/lon to local NED frame (simplified) */
    /* In real implementation, use reference point for conversion */
    state->position[0] = (float)(lat - 37.7749) * 111111.0f;
    state->position[1] = (float)(lon + 122.4194) * 111111.0f * cosf(37.7749 * 0.0174533f);
    state->position[2] = alt;
    
    /* Update state with GPS measurement (simplified Kalman gain) */
    float alpha = 0.1f;  /* Tuning parameter */
    state->x[10] += alpha * (state->position[0] - state->x[10]);
    state->x[11] += alpha * (state->position[1] - state->x[11]);
    state->x[12] += alpha * (state->position[2] - state->x[12]);
}

void ekf_update_baro(EKFState *state, float pressure, float temperature) {
    (void)temperature;
    if (!state || !state->initialized) return;
    
    /* Convert pressure to altitude (simplified) */
    float altitude = 44330.0f * (1.0f - powf(pressure / 101325.0f, 0.1903f));
    
    /* Update altitude with baro measurement */
    float alpha = 0.2f;
    state->x[12] += alpha * (altitude - state->x[12]);
}

void ekf_get_attitude(EKFState *state, float *roll, float *pitch, float *yaw) {
    if (!state || !state->initialized) return;
    if (roll) *roll = state->roll;
    if (pitch) *pitch = state->pitch;
    if (yaw) *yaw = state->yaw;
}

void ekf_get_position(EKFState *state, float *x, float *y, float *z) {
    if (!state || !state->initialized) return;
    if (x) *x = state->x[10];
    if (y) *y = state->x[11];
    if (z) *z = state->x[12];
}

void ekf_get_velocity(EKFState *state, float *vx, float *vy, float *vz) {
    if (!state || !state->initialized) return;
    if (vx) *vx = state->x[7];
    if (vy) *vy = state->x[8];
    if (vz) *vz = state->x[9];
}

bool ekf_is_healthy(EKFState *state) {
    if (!state) return false;
    if (!state->initialized) return false;
    if (state->update_count < 10) return false;
    
    /* Check if any covariance is too large (divergence) */
    for (int i = 0; i < EKF_STATE_SIZE; i++) {
        if (state->P[i][i] > 1000.0f) {
            return false;
        }
    }
    return true;
}