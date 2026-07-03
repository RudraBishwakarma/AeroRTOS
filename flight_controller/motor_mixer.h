#ifndef MOTOR_MIXER_H
#define MOTOR_MIXER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Motor Mixer Types */
typedef enum {
    MIXER_QUAD_X = 0,
    MIXER_QUAD_PLUS,
    MIXER_HEXACOPTER,
    MIXER_OCTACOPTER,
    MIXER_CUSTOM
} MixerType;

/* Motor Mixer Configuration */
typedef struct {
    MixerType type;
    uint8_t motor_count;
    float motor_direction[8];  /* +1 for CCW, -1 for CW */
    float mixing_matrix[8][4]; /* [motor][roll, pitch, yaw, throttle] */
} MotorMixer;

/* Function Prototypes */
void motor_mixer_init(MotorMixer *mixer, MixerType type);
void motor_mixer_update(MotorMixer *mixer, float *motor_outputs,
                        float roll, float pitch, float yaw, float throttle);
void motor_mixer_set_custom(MotorMixer *mixer, int motor_count,
                            float *directions, float *matrix);
void motor_mixer_print_config(MotorMixer *mixer);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_MIXER_H */