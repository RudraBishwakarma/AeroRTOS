#include "cascaded_pid.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

/* ============================================================================
 * PID Controller Implementation
 * ============================================================================ */

void pid_init(PIDController *pid, float kp, float ki, float kd,
              float output_min, float output_max) {
    if (!pid) return;
    
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->integral_min = output_min * 0.5f;
    pid->integral_max = output_max * 0.5f;
    pid->alpha = 0.1f;  /* Derivative filter coefficient */
    pid->derivative_filtered = 0.0f;
    pid->initialized = true;
}

float pid_update(PIDController *pid, float setpoint, float measurement, float dt) {
    if (!pid || !pid->initialized || dt < 0.0001f) {
        return 0.0f;
    }
    
    float error = setpoint - measurement;
    
    /* Proportional term */
    float p_term = pid->kp * error;
    
    /* Integral term with anti-windup */
    pid->integral += error * dt;
    if (pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if (pid->integral < pid->integral_min) {
        pid->integral = pid->integral_min;
    }
    float i_term = pid->ki * pid->integral;
    
    /* Derivative term with low-pass filter */
    float derivative = (error - pid->prev_error) / dt;
    pid->derivative_filtered = pid->alpha * derivative + 
                               (1.0f - pid->alpha) * pid->derivative_filtered;
    float d_term = pid->kd * pid->derivative_filtered;
    
    /* Update previous error */
    pid->prev_error = error;
    
    /* Compute output */
    float output = p_term + i_term + d_term;
    
    /* Clamp output */
    if (output > pid->output_max) {
        output = pid->output_max;
    } else if (output < pid->output_min) {
        output = pid->output_min;
    }
    
    return output;
}

void pid_reset(PIDController *pid) {
    if (!pid) return;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->derivative_filtered = 0.0f;
}

void pid_set_limits(PIDController *pid, float min, float max) {
    if (!pid) return;
    pid->output_min = min;
    pid->output_max = max;
    pid->integral_min = min * 0.5f;
    pid->integral_max = max * 0.5f;
}

void pid_set_integral_limits(PIDController *pid, float min, float max) {
    if (!pid) return;
    pid->integral_min = min;
    pid->integral_max = max;
}

/* ============================================================================
 * Cascaded PID Controller Implementation
 * ============================================================================ */

void cascaded_pid_init(CascadedPID *pid) {
    if (!pid) return;
    
    memset(pid, 0, sizeof(CascadedPID));
    
    /* Default tuning for a 250-size quadcopter */
    /* Rate controller (inner loop) */
    pid_init(&pid->rate_roll, 0.15f, 0.005f, 0.002f, -1.0f, 1.0f);
    pid_init(&pid->rate_pitch, 0.15f, 0.005f, 0.002f, -1.0f, 1.0f);
    pid_init(&pid->rate_yaw, 0.05f, 0.001f, 0.001f, -0.5f, 0.5f);
    
    /* Attitude controller (outer loop) */
    pid_init(&pid->att_roll, 1.0f, 0.01f, 0.05f, -45.0f, 45.0f);
    pid_init(&pid->att_pitch, 1.0f, 0.01f, 0.05f, -45.0f, 45.0f);
    pid_init(&pid->att_yaw, 0.8f, 0.005f, 0.02f, -180.0f, 180.0f);
    
    /* Position controller */
    pid_init(&pid->pos_xy, 0.5f, 0.01f, 0.1f, -5.0f, 5.0f);
    pid_init(&pid->pos_z, 1.0f, 0.02f, 0.2f, -2.0f, 2.0f);
    pid_init(&pid->vel_xy, 0.5f, 0.01f, 0.05f, -2.0f, 2.0f);
    pid_init(&pid->vel_z, 1.0f, 0.02f, 0.1f, -1.0f, 1.0f);
    
    /* Output limits */
    pid->rate_output_limit = 1.0f;
    pid->att_output_limit = 45.0f;
    pid->pos_output_limit = 5.0f;
    pid->vel_output_limit = 2.0f;
    
    pid->initialized = true;
    pid->last_update = 0;
    
    printf("Cascaded PID Controller Initialized\n");
}

void cascaded_pid_set_tuning(CascadedPID *pid, 
                             float rate_kp, float rate_ki, float rate_kd,
                             float att_kp, float att_ki, float att_kd,
                             float pos_kp, float pos_ki, float pos_kd) {
    if (!pid || !pid->initialized) return;
    
    /* Rate controller tuning */
    pid->rate_roll.kp = rate_kp;
    pid->rate_roll.ki = rate_ki;
    pid->rate_roll.kd = rate_kd;
    pid->rate_pitch.kp = rate_kp;
    pid->rate_pitch.ki = rate_ki;
    pid->rate_pitch.kd = rate_kd;
    pid->rate_yaw.kp = rate_kp * 0.3f;
    pid->rate_yaw.ki = rate_ki * 0.3f;
    pid->rate_yaw.kd = rate_kd * 0.3f;
    
    /* Attitude controller tuning */
    pid->att_roll.kp = att_kp;
    pid->att_roll.ki = att_ki;
    pid->att_roll.kd = att_kd;
    pid->att_pitch.kp = att_kp;
    pid->att_pitch.ki = att_ki;
    pid->att_pitch.kd = att_kd;
    pid->att_yaw.kp = att_kp * 0.5f;
    pid->att_yaw.ki = att_ki * 0.5f;
    pid->att_yaw.kd = att_kd * 0.5f;
    
    /* Position controller tuning */
    pid->pos_xy.kp = pos_kp;
    pid->pos_xy.ki = pos_ki;
    pid->pos_xy.kd = pos_kd;
    pid->pos_z.kp = pos_kp * 1.5f;
    pid->pos_z.ki = pos_ki * 1.5f;
    pid->pos_z.kd = pos_kd * 1.5f;
}

void cascaded_pid_update(CascadedPID *pid,
                         float target_roll, float target_pitch, float target_yaw,
                         float target_altitude, float target_velocity,
                         float current_roll, float current_pitch, float current_yaw,
                         float current_altitude, float current_velocity,
                         float *motor_outputs, float dt) {
    if (!pid || !pid->initialized || !motor_outputs || dt < 0.0001f) {
        return;
    }
    
    /* Position Control (if altitude mode) */
    float vel_target = 0.0f;
    if (target_altitude > 0) {
        vel_target = pid_update(&pid->pos_z, target_altitude, current_altitude, dt);
        vel_target = fmaxf(fminf(vel_target, pid->vel_output_limit), -pid->vel_output_limit);
    }
    
    /* Velocity Control */
    float accel_target = pid_update(&pid->vel_z, target_velocity + vel_target, 
                                    current_velocity, dt);
    
    /* Calculate target thrust (simplified) */
    float thrust = 0.5f + accel_target * 0.1f;
    thrust = fmaxf(fminf(thrust, 1.0f), 0.0f);
    
    /* Attitude Control (outer loop) */
    float att_roll_output = pid_update(&pid->att_roll, target_roll, current_roll, dt);
    float att_pitch_output = pid_update(&pid->att_pitch, target_pitch, current_pitch, dt);
    float att_yaw_output = pid_update(&pid->att_yaw, target_yaw, current_yaw, dt);
    
    /* Clamp attitude outputs */
    att_roll_output = fmaxf(fminf(att_roll_output, pid->att_output_limit), -pid->att_output_limit);
    att_pitch_output = fmaxf(fminf(att_pitch_output, pid->att_output_limit), -pid->att_output_limit);
    att_yaw_output = fmaxf(fminf(att_yaw_output, pid->att_output_limit), -pid->att_output_limit);
    
    /* Rate Control (inner loop) - Convert attitude errors to rate targets */
    /* In real implementation, rate_target = attitude_error * k */
    float rate_roll_target = att_roll_output * 0.1f;
    float rate_pitch_target = att_pitch_output * 0.1f;
    float rate_yaw_target = att_yaw_output * 0.1f;
    
    /* Rate controller outputs */
    float rate_roll_output = pid_update(&pid->rate_roll, rate_roll_target, 0, dt);
    float rate_pitch_output = pid_update(&pid->rate_pitch, rate_pitch_target, 0, dt);
    float rate_yaw_output = pid_update(&pid->rate_yaw, rate_yaw_target, 0, dt);
    
    /* Clamp rate outputs */
    rate_roll_output = fmaxf(fminf(rate_roll_output, pid->rate_output_limit), -pid->rate_output_limit);
    rate_pitch_output = fmaxf(fminf(rate_pitch_output, pid->rate_output_limit), -pid->rate_output_limit);
    rate_yaw_output = fmaxf(fminf(rate_yaw_output, pid->rate_output_limit), -pid->rate_output_limit);
    
    /* Convert to motor outputs (Quad-X configuration) */
    /* M1: Front-Right, M2: Back-Left, M3: Front-Left, M4: Back-Right */
    motor_outputs[0] = thrust + rate_roll_output + rate_pitch_output - rate_yaw_output;
    motor_outputs[1] = thrust - rate_roll_output - rate_pitch_output + rate_yaw_output;
    motor_outputs[2] = thrust + rate_roll_output - rate_pitch_output + rate_yaw_output;
    motor_outputs[3] = thrust - rate_roll_output + rate_pitch_output - rate_yaw_output;
    
    /* Clamp motor outputs */
    for (int i = 0; i < 4; i++) {
        motor_outputs[i] = fmaxf(fminf(motor_outputs[i], 1.0f), 0.0f);
    }
    
    pid->last_update = 0;
}