#include<stdint.h> /* used for types like uint8_t and similar */
#include "ringbuffer.h"

/*
bytes:
3 for the command string (3 chars)
+8 for each ":" char (may be less)
+6*2 for each value sent via the trj command (6 values of 64 bits -> 48 bytes)
+1 for the penup value which is 1 or 0

(3+8+48+1) = 60 (bytes) -> use 64 bytes just in case longer messages are needed
*/
#define DATA_SZ 64

// matrix determinant macro
#define DET(matrix) matrix[0]*matrix[3]-matrix[1]*matrix[2]

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
- double B[4]: array that represents a linearized (as in making a 2x2 become a 1x4) inertia matrix;
- double C[4]: array that represents a linearized (as in making a 2x2 become a 1x4) coriolis matrix;
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
    double B[4];
    double C[4];
} man_t; /* 1.2 kB of data with RBUF_SZ = 10 -> total mC memory: 512 KB, remaining 510.8 KB */

uint8_t rx_data[DATA_SZ]; /* where the message will be saved for reception */
uint8_t tx_data[DATA_SZ]; /* where the message will be saved for transmission */
man_t manip;
//controller parameters
const double Kp[4] = {1,0,0,1}; 
const double Kd[4] = {1,0,0,1};

void init_man(man_t *manip);
uint8_t dot(double *A, uint8_t nA, uint8_t mA, double* B, uint8_t nB, uint8_t mB, double* C);
void sum(double *A, double *B, uint8_t n, double *C);
void diff(double *A, double *B, uint8_t n, double *C);
uint8_t inv2x2(double *M, double *invM); 
void B(man_t *manip);
void C(man_t *manip);
void controller(man_t *manip, double *u);