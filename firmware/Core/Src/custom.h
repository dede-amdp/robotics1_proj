#include<stdint.h> /* used for types like uint8_t and similar */
#include "ringbuffer.h"
#include "main.h"

#ifndef CUSTOM_DEF
#define CUSTOM_DEF
/*
TRJ:q0:dq0:ddq0:q1:dq1:ddq1:penup

bytes:
3 for the command string (3 chars)
+7 for each ":" char (may be less)
+6*18 bytes (18 characters to represent "0x"+hex)
+1 for the penup value which is 1 or 0

= 119 (bytes) -> use 128 bytes just in case longer messages are needed
*/
#define DATA_SZ 128
/* CONTROL TIME */
#define T_C 0.01
/* SAMPLING TIME (encoder readings) -> htim10: prescaler = 2, ARR = 28000 */
#define T_S 0.002
/* PRESCALER */
#define PRESCALER_ENCODER 16
/* RESOLUTION OF THE STEPPER MOTOR (in rads) */
#define RESOLUTION 0.0314159
/* MAX SPEED of rotation of the motors (in rads/s) */
#define MAX_SPEED 4
/* PWM frequency first motor */
#define PWM_FREQ_1 1800
/* PWM frequency second motor */
#define PWM_FREQ_2 2000
/* Number of previous values to use for speed and acceleration estimation */
#define ESTIMATION_STEPS 3
/* Debounce delay macro */
#define DEBOUNCE_DELAY 50
/* Microstepping */
#define MICROSTEPS 16
/* THRESHOLD */
#define THRESHOLD 5*3.14159*(1/180)

/* matrix determinant macro */
#define DET(matrix) matrix[0]*matrix[3]-matrix[1]*matrix[2]

/* SIGN macro */
#define SIGN(A) (int8_t) ((A >= 0) - (A <= 0))

/* ABS macro */
#define ABS(A) (SIGN(A)*A)


/*
#@
@name: struct manipulator aka manipulator_t
@brief: struct representation of a 2Dof planar manipulator
@notes: it holds RBUF_SZ values of the reference and actual position, speed and acceleration of each motor via multiple ring/circular buffer; 
@inputs: 
- ringbuffer_t q0: holds up to RBUF_SZ values of position of the first motor;
- ringbuffer_t q1: holds up to RBUF_SZ values of position of the second motor;
- ringbuffer_t dq0: holds up to RBUF_SZ values of speed of the first motor;
- ringbuffer_t dq1: holds up to RBUF_SZ values of speed of the second motor;
- ringbuffer_t ddq0: holds up to RBUF_SZ values of acceleration of the first motor;
- ringbuffer_t ddq1: holds up to RBUF_SZ values of acceleration of the second motor;
- ringbuffer_t penup: holds up to RBUF_SZ values of position of the end-effector motor;
- ringbuffer_t q0_actual: holds up to RBUF_SZ values of actual position (encoder readings) of the first motor;
- ringbuffer_t q1_actual: holds up to RBUF_SZ values of actual position (encoder readings) of the second motor;
- ringbuffer_t dq0_actual: holds up to RBUF_SZ values of actual speed (estimation) of the first motor;
- ringbuffer_t dq1_actual: holds up to RBUF_SZ values of actual speed (estimation) of the second motor;
- ringbuffer_t ddq0_actual: holds up to RBUF_SZ values of actual acceleration (estimation) of the first motor;
- ringbuffer_t ddq1_actual: holds up to RBUF_SZ values of actual acceleration (estimation) of the second motor;
- ringbuffer_t penup_actual: holds up to RBUF_SZ values of actual position of the end-effector motor;
- float B[4]: array that represents a linearized (as in making a 2x2 become a 1x4) inertia matrix;
- float C[4]: array that represents a linearized (as in making a 2x2 become a 1x4) coriolis matrix;
- TIM_HandleTypeDef *htim_encoder1: pointer to the timer used to read the first encoder;
- TIM_HandleTypeDef *htim_encoder2: pointer to the timer used to read the second encoder;
@#
*/
typedef struct manipulator {
    /* desired values */
    ringbuffer_t q0;
    ringbuffer_t q1;
    ringbuffer_t dq0;
    ringbuffer_t dq1;
    ringbuffer_t ddq0;
    ringbuffer_t ddq1;
    ringbuffer_t penup;
    /* readings from sensors */
    ringbuffer_t q0_actual;
    ringbuffer_t q1_actual;
    ringbuffer_t dq0_actual;
    ringbuffer_t dq1_actual;
    ringbuffer_t ddq0_actual;
    ringbuffer_t ddq1_actual;
    ringbuffer_t penup_actual;
    float B[4];
    float C[4];
    TIM_HandleTypeDef *htim_encoder1;
    TIM_HandleTypeDef *htim_encoder2;
} man_t; /* 1.2 kB of data with RBUF_SZ = 10 -> total mC memory: 512 KB, remaining 510.8 KB */

typedef struct rate {
    uint32_t last_time;
    uint32_t delta_time;     /* in ms */
} rate_t;

extern uint8_t rx_data[DATA_SZ]; /* where the message will be saved for reception */
extern uint8_t tx_data[DATA_SZ]; /* where the message will be saved for transmission */
extern man_t manip;

extern uint32_t previous_trigger;
extern uint8_t triggered;

/* controller parameters */
extern const float Kp[4];
extern const float Kd[4];
/* reduction values for motors */
extern const uint8_t reduction1;
extern const uint8_t reduction2;

// SECTION DEBUG
extern float disp1, disp2;
extern float dq_actual0, dq_actual1;
extern float ddq_actual0, ddq_actual1;
extern ringbuffer_t timestamps;
extern uint32_t count;
// !SECTION DEBUG

void init_man(man_t *manip, TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2);
uint8_t dot(float *A, uint8_t nA, uint8_t mA, float* B, uint8_t nB, uint8_t mB, float* C);
void sum(float *A, float *B, uint8_t n, float *C);
void diff(float *A, float *B, uint8_t n, float *C);
uint8_t inv2x2(float *M, float *invM); 
void det(float *M, uint8_t n, float *d);
void adj(float *M, float *subM, uint8_t n, float *adjM);
void tr(float *M, uint8_t n, uint8_t m, float *trM);
uint8_t inv(float *M, float *adjM, float *subM, float *trM,  uint8_t n, float *invM);
void pseudo_inv(float *M, float *trM, float *tempM, float *adjM, float *subM, float *invM, float *dotM, uint8_t n, float *psinvM);

void B_calc(man_t *manip);
void C_calc(man_t *manip);
void controller(man_t *manip, float *u);
void rad2stepdir(float dq, float resolution, float frequency, uint32_t *steps, int8_t *dir);
void speed_estimation(ringbuffer_t *q_actual, ringbuffer_t *dq_actual, float reduction, float *v_est, float *a_est);

void init_rate(rate_t *rate, uint32_t ms);
void rate_sleep(rate_t *rate);

void read_encoders(man_t *manip);
void update_speeds(man_t *manip);
void apply_input(TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2, float *u);

void start_timers(TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3, TIM_HandleTypeDef *htim4);
void stop_timers(TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3, TIM_HandleTypeDef *htim4);

void log_data(UART_HandleTypeDef *huart, man_t *manip);

#endif
