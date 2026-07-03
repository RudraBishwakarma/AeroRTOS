#ifndef CASCADED_PID_H
#define CASCADED_PID_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* PID Controller Structure */
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float output_min;
    float output_max;
    float integral_min;
    float integral_max;
    float alpha;  /* Low-pass filter for derivative */
    float derivative_filtered;
    bool initialized;
} PIDController;

/* Cascaded PID for Attitude Control */
typedef struct {
    /* Rate Controllers (Inner Loop) */
    PIDController rate_roll;
    PIDController rate_pitch;
    PIDController rate_yaw;
    
    /* Attitude Controllers (Outer Loop) */
    PIDController att_roll;
    PIDController att_pitch;
    PIDController att_yaw;
    
    /* Position Controllers */
    PIDController pos_xy;
    PIDController pos_z;
    PIDController vel_xy;
    PIDController vel_z;
    
    /* Tuning Parameters */
    float rate_roll_kp, rate_roll_ki, rate_roll_kd;
    float rate_pitch_kp, rate_pitch_ki, rate_pitch_kd;
    float rate_yaw_kp, rate_yaw_ki, rate_yaw_kd;
    float att_roll_kp, att_roll_ki, att_roll_kd;
    float att_pitch_kp, att_pitch_ki, att_pitch_kd;
    float att_yaw_kp, att_yaw_ki, att_yaw_kd;
    float pos_xy_kp, pos_xy_ki, pos_xy_kd;
    float pos_z_kp, pos_z_ki, pos_z_kd;
    float vel_xy_kp, vel_xy_ki, vel_xy_kd;
    float vel_z_kp, vel_z_ki, vel_z_kd;
    
    /* Output limits */
    float rate_output_limit;
    float att_output_limit;
    float pos_output_limit;
    float vel_output_limit;
    
    /* Status */
    bool initialized;
    uint32_t last_update;
} CascadedPID;

/* Function Prototypes */
void pid_init(PIDController *pid, float kp, float ki, float kd,
              float output_min, float output_max);
float pid_update(PIDController *pid, float setpoint, float measurement, float dt);
void pid_reset(PIDController *pid);
void pid_set_limits(PIDController *pid, float min, float max);
void pid_set_integral_limits(PIDController *pid, float min, float max);

void cascaded_pid_init(CascadedPID *pid);
void cascaded_pid_set_tuning(CascadedPID *pid, 
                             float rate_kp, float rate_ki, float rate_kd,
                             float att_kp, float att_ki, float att_kd,
                             float pos_kp, float pos_ki, float pos_kd);
void cascaded_pid_update(CascadedPID *pid,
                         float target_roll, float target_pitch, float target_yaw,
                         float target_altitude, float target_velocity,
                         float current_roll, float current_pitch, float current_yaw,
                         float current_altitude, float current_velocity,
                         float *motor_outputs, float dt);

#ifdef __cplusplus
}
#endif

#endif /* CASCADED_PID_H */