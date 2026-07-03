#include "motor_mixer.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

/* ============================================================================
 * Mixing Matrices
 * ============================================================================ */

/* Quad-X configuration (motors at 45 degrees) */
static float mix_quad_x[4][4] = {
    { 0.5f,  0.5f,  0.5f, -1.0f},  /* M1: Front-Right (CCW) */
    {-0.5f, -0.5f,  0.5f,  1.0f},  /* M2: Back-Left (CW) */
    { 0.5f, -0.5f, -0.5f, -1.0f},  /* M3: Front-Left (CCW) */
    {-0.5f,  0.5f, -0.5f,  1.0f}   /* M4: Back-Right (CW) */
};

/* Quad-Plus configuration (motors at 0, 90, 180, 270 degrees) */
static float mix_quad_plus[4][4] = {
    { 0.0f,  0.5f,  0.5f, -1.0f},  /* M1: Front (CCW) */
    { 0.0f, -0.5f,  0.5f,  1.0f},  /* M2: Back (CW) */
    { 0.5f,  0.0f, -0.5f, -1.0f},  /* M3: Right (CCW) */
    {-0.5f,  0.0f, -0.5f,  1.0f}   /* M4: Left (CW) */
};

/* Hexacopter (6 motors) */
static float mix_hexa[6][4] = {
    { 0.5f,  0.2f,  0.5f, -1.0f},
    {-0.5f, -0.2f,  0.5f,  1.0f},
    { 0.5f, -0.2f, -0.5f, -1.0f},
    {-0.5f,  0.2f, -0.5f,  1.0f},
    { 0.0f,  0.5f,  0.5f, -1.0f},
    { 0.0f, -0.5f,  0.5f,  1.0f}
};

/* ============================================================================
 * Motor Mixer Implementation
 * ============================================================================ */

void motor_mixer_init(MotorMixer *mixer, MixerType type) {
    if (!mixer) return;
    
    memset(mixer, 0, sizeof(MotorMixer));
    mixer->type = type;
    
    switch (type) {
        case MIXER_QUAD_X:
            mixer->motor_count = 4;
            memcpy(mixer->mixing_matrix, mix_quad_x, sizeof(mix_quad_x));
            mixer->motor_direction[0] = -1.0f;  /* CCW */
            mixer->motor_direction[1] = 1.0f;   /* CW */
            mixer->motor_direction[2] = -1.0f;  /* CCW */
            mixer->motor_direction[3] = 1.0f;   /* CW */
            break;
            
        case MIXER_QUAD_PLUS:
            mixer->motor_count = 4;
            memcpy(mixer->mixing_matrix, mix_quad_plus, sizeof(mix_quad_plus));
            mixer->motor_direction[0] = -1.0f;
            mixer->motor_direction[1] = 1.0f;
            mixer->motor_direction[2] = -1.0f;
            mixer->motor_direction[3] = 1.0f;
            break;
            
        case MIXER_HEXACOPTER:
            mixer->motor_count = 6;
            memcpy(mixer->mixing_matrix, mix_hexa, sizeof(mix_hexa));
            for (int i = 0; i < 6; i++) {
                mixer->motor_direction[i] = (i % 2 == 0) ? -1.0f : 1.0f;
            }
            break;
            
        default:
            mixer->motor_count = 4;
            memcpy(mixer->mixing_matrix, mix_quad_x, sizeof(mix_quad_x));
            break;
    }
    
    printf("Motor Mixer Initialized: %s (%d motors)\n",
           (type == MIXER_QUAD_X) ? "Quad-X" :
           (type == MIXER_QUAD_PLUS) ? "Quad-Plus" :
           (type == MIXER_HEXACOPTER) ? "Hexacopter" : "Custom",
           mixer->motor_count);
}

void motor_mixer_update(MotorMixer *mixer, float *motor_outputs,
                        float roll, float pitch, float yaw, float throttle) {
    if (!mixer || !motor_outputs) return;
    
    /* Clamp inputs */
    roll = fmaxf(fminf(roll, 1.0f), -1.0f);
    pitch = fmaxf(fminf(pitch, 1.0f), -1.0f);
    yaw = fmaxf(fminf(yaw, 1.0f), -1.0f);
    throttle = fmaxf(fminf(throttle, 1.0f), 0.0f);
    
    /* Scale throttle to usable range */
    float min_throttle = 0.1f;
    float max_throttle = 1.0f;
    float scaled_throttle = min_throttle + throttle * (max_throttle - min_throttle);
    
    /* Apply mixing matrix */
    for (int i = 0; i < mixer->motor_count; i++) {
        motor_outputs[i] = mixer->mixing_matrix[i][0] * roll +
                          mixer->mixing_matrix[i][1] * pitch +
                          mixer->mixing_matrix[i][2] * yaw +
                          mixer->mixing_matrix[i][3] * scaled_throttle;
        
        /* Apply motor direction */
        motor_outputs[i] *= mixer->motor_direction[i];
        
        /* Clamp to valid range */
        motor_outputs[i] = fmaxf(fminf(motor_outputs[i], 1.0f), 0.0f);
    }
}

void motor_mixer_set_custom(MotorMixer *mixer, int motor_count,
                            float *directions, float *matrix) {
    if (!mixer || motor_count < 1 || motor_count > 8) return;
    
    mixer->type = MIXER_CUSTOM;
    mixer->motor_count = motor_count;
    
    if (directions) {
        memcpy(mixer->motor_direction, directions, motor_count * sizeof(float));
    }
    if (matrix) {
        memcpy(mixer->mixing_matrix, matrix, motor_count * 4 * sizeof(float));
    }
}

void motor_mixer_print_config(MotorMixer *mixer) {
    if (!mixer) return;
    
    printf("Motor Mixer Configuration:\n");
    printf("  Type: %d\n", mixer->type);
    printf("  Motors: %d\n", mixer->motor_count);
    printf("  Directions: ");
    for (int i = 0; i < mixer->motor_count; i++) {
        printf("%.1f ", mixer->motor_direction[i]);
    }
    printf("\n  Matrix:\n");
    for (int i = 0; i < mixer->motor_count; i++) {
        printf("    M%d: %.2f %.2f %.2f %.2f\n", i+1,
               mixer->mixing_matrix[i][0],
               mixer->mixing_matrix[i][1],
               mixer->mixing_matrix[i][2],
               mixer->mixing_matrix[i][3]);
    }
}