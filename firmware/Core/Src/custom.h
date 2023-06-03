#include<stdint.h> /* used for types like uint8_t and similar */

/*
bytes:
3 for the command string (3 chars)
+8 for each ":" char (may be less)
+6*2 for each value sent via the trj command (6 values of 64 bits -> 16 bytes)
+1 for the penup value which is 1 or 0
*10 buffer size

(3+8+6*2+1)*10 = 240 (bytes) -> use 320 bytes just in case longer messages are needed
*/

#define BUFFER_SZ 10
#define DATA_SZ 32

// matrix determinant macro
#define DET(matrix) matrix[0]*matrix[3]-matrix[1]*matrix[2]

typedef struct manipulator {
    // desired values
    double q[2];
    double dq[2];
    double ddq[2];
    uint8_t penup;
    // readings from sensors
    double q_actual[2];
    double dq_actual[2];
    double ddq_actual[2];
    uint8_t penup_actual;
    double B[4];
    double C[4];
} man_t;

uint8_t rx_data[DATA_SZ * BUFFER_SZ]; // where the message will be saved for reception
uint8_t tx_data[DATA_SZ * BUFFER_SZ]; // where the message will be saved for transmission
man_t manip;
//controller parameters
const double Kp[4] = {1,0,0,1}; 
const double Kd[4] = {1,0,0,1};

uint8_t cbi = 0; // circular buffer index

uint8_t dot(double *A, uint8_t nA, uint8_t mA, double* B, uint8_t nB, uint8_t mB, double* C);
void sum(double *A, double *B, uint8_t n, double *C);
void diff(double *A, double *B, uint8_t n, double *C);
uint8_t inv2x2(double *M, double *invM); 
void B(man_t *manip);
void C(man_t *manip);
void controller(man_t *manip, double *u);