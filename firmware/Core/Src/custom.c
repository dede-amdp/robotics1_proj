#include "custom.h"

#include<stdint.h>  /* used for types like uint8_t and similar */
#include<string.h>  /* used for string manipulation via serial */
#include<stdlib.h>  /* used for string manipulation via serial */
#include<math.h>    /* used for sin and cos */
#include<time.h>    /* used for fixing the while looping rate */
#include "main.h"
#include "ringbuffer.h"

uint8_t rx_data[DATA_SZ]; /* where the message will be saved for reception */
uint8_t tx_data[DATA_SZ]; /* where the message will be saved for transmission */
man_t manip;
//controller parameters
const double Kp[4] = {1,0,0,1}; 
const double Kd[4] = {1,0,0,1};


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    char *cmd, *data, *value;
    uint8_t i = 0;
    /* read the first characters */
    cmd = strtok((char*) &rx_data, ":");
    if(strcmp(cmd, "trj")){ /* trj case*/
        /* READ the data from the serial and populate the corresponding members of the man_t struct 
           these values will be used to set the reference value for the control loop */
        data = strtok(NULL, ":");
        while(data != NULL){
            if(i == 6) break; /* reading penup */
            value = "0x"; /* will contain the value extracted from the received string */
            strcat(value, data); /* string concatenation REF: https://www.programiz.com/c-programming/library-function/string.h/strcat */
            rbpush((((ringbuffer_t *) &manip)+i),  strtoull(value, NULL, 16)); /* convert from str to ull -> unsigned long long (uint64_t). REF: https://cplusplus.com/reference/cstdlib/strtoull/ */
            data = strtok(NULL, ":");
            i++;
        }
        rbpush(&manip.penup, (double) atoi(data));
    }else{ /* default case */

    }
    /* wait again for incoming data */
    HAL_UART_Receive_DMA(&huart, &rx_data, (uint8_t) DATA_SZ); /* DATA_SZ bytes of data for each reception */
}

/*
#@
@name: init_man
@brief: initializes the members of the man_t struct
@inputs: 
- man_t *manip: man_t obj. to initialize;
@outputs: 
- void;
@#
*/
void init_man(man_t *manip){
    uint8_t i;
    // initialize all the ring buffers
    for(i = 0; i < 14; i++){
        rbclear((((ringbuffer_t *) manip)+i));
    }
    // initialize the inertia and coriolis matrices
    for(i = 0; i < 4; i++){
        manip->B[i] = (double) 0;
        manip->C[i] = (double) 0;
    }
}

/*
#@
@name: dot
@brief: computes the dot product between two matrices, A and B, represented in vector form;
@inputs: 
- double *A: pointer to a vector of doubles of size nA*mA, which represents the first nAxmA matrix;
- uint8_t nA: number of rows of matrix A;
- uint8_t mA: number of columns of matrix A;
- double *B: pointer to a vector of doubles of size nB*mB, which represents the second nBxmB matrix;
- uint8_t nB: number of rows of matrix B;
- uint8_t mB: number of columns of matrix B;
- double *C: pointer to a vector of doubles of size nA*mB, which represents the resulting nAxmB matrix -> if the operation cannot be done, it will be NULL;
@outputs: 
- uint8_t: 0 or 1 that shows whether the operation completed successfully or not.
@#
*/
uint8_t dot(double *A, uint8_t nA, uint8_t mA, double* B, uint8_t nB, uint8_t mB, double* C){ /* nAxmA * nBxmB dot product */
    /*  why wasn't a matrix struct created to check if the matrices can be multiplied? 
        this firmware does not make heavy use of matrices to justify creating a struct,
        this method exists just to make the code more readable and understand what each
        operation actually does instead of having meaningless calculations */
    
    /* C[i, j] = \sum_k A[i, k]*B[k, j] */
    /* given n rows and m columns, the matrix indexes i, j correspond to j+i*n array index */
    if(mA != nB){
        C = NULL;
        return 0; /* matrix multiplication cannot be done */
    }
    uint8_t i, j, k;
    for(i = 0; i < 4; i++){
        C[i] = (double) 0.0;
    }
    for( i = 0; i < nA; i++){
        for( j = 0; j < mB; j++){
            for( k = 0; k < mA; k++){
                C[j+i*2] += (double) A[k+i*2]*B[j+k*2];
            }
        }
    }
    return 1; // matrix multiplication successfully completed
}

/*
#@
@name: inv2x2
@brief: Inverts (if possible) a 2x2 matrix
@inputs: 
- double *M: pointer to the matrix to invert;
- double *invM: pointer to the inverted matrix (NULL if inversion is not possible);
@outputs: 
- uint8_t: shows whether the inversion was completed or not
@#
*/
uint8_t inv2x2(double *M, double *invM){
    double d = DET(M);
    if(d == 0){
        invM = NULL;
        return 0; /* Inversion not possible */
    }
    invM[0] =  M[3]/d;
    invM[3] =  M[0]/d;
    invM[1] = -M[1]/d;
    invM[2] = -M[2]/d;
    return 1; /* Inversion completed successfully */
}

/*
#@
@name: sum
@brief: sums two matrices (with the same size)
@inputs: 
- double *A: pointer to the first matrix to sum;
- double *B: pointer to the second matrix to sum;
- uint8_t n: number of elements in the matrix;
- double *C: pointer to the resulting matrix;
@outputs: 
- void;
@#
*/
void sum(double *A, double *B, uint8_t n, double *C){
    uint8_t i;
    for(i = 0; i < n; i++){
        C[i] = A[i] + B[i];
    }
}

/*
#@
@name: diff
@brief: subtracts two matrices (with the same size)
@inputs: 
- double *A: pointer to the first matrix to subtract;
- double *B: pointer to the second matrix to subtract;
- uint8_t n: number of elements in the matrix;
- double *C: pointer to the resulting matrix;
@outputs: 
- void;
@#
*/
void diff(double *A, double *B, uint8_t n, double *C){
    uint8_t i;
    for(i = 0; i < n; i++){
        C[i] = A[i] - B[i];
    }
}

/*
#@
@name: B
@brief: computes the inertia matrix of the manipulator
@notes: the inertia matrix B of the manipulator depends on its current configuration: 
it is computed analytically with the dynamic model found via the  Matlab Peter Corke toolbox 
@inputs: 
- man_t *manip: pointer to the manipulator struct that olds the reference and actual values of the position, speed and acceleration of the motors;
@outputs:
- void;
@#
*/
void B(man_t *manip){
    double q1,q2;
    rblast(&manip->q0_actual, &q1);
    rblast(&manip->q1_actual, &q2);
    manip->B[0] = (double) (0.024938*cos(q1 + 2*q2) + 0.12469*cos(q1 + q2) + 0.26194*cos(q1) + 0.074812*cos(q2) + 0.16349);
    manip->B[1] = (double) (0.012469*cos(q1 + 2*q2) + 0.09975*cos(q1 + q2) + 0.14962*cos(q1) + 0.049875*cos(q2) + 0.058307);
    manip->B[2] = manip->B[1]; // the matrix is symmetrical
    manip->B[3] = (double) (0.074812*cos(q1 + q2) + 0.14962*cos(q1) + 0.024938*cos(q2) + 0.058309);
    /*  manip::B is actually a vector, but it can be seen as follows: 
        [B[0], B[1]]
        [B[2], B[3]] */
}


/*
#@
@name: C
@brief: computes the coriolis matrix of the manipulator
@notes: the coriolis matrix C of the manipulator depends on its current configuration and joint speed: 
it is computed analytically with the dynamic model found via the  Matlab Peter Corke toolbox 
@inputs: 
- man_t *manip: pointer to the manipulator struct that olds the reference and actual values of the position, speed and acceleration of the motors;
@outputs:
- void;
@#
*/
void C(man_t *manip){
    double q1, q2, dq1, dq2;
    rblast(&manip->q0_actual, &q1);
    rblast(&manip->q1_actual, &q2);
    rblast(&manip->dq0_actual, &dq1);
    rblast(&manip->dq1_actual, &dq2);
    manip->C[0] = (double) ( - 0.5*dq2*(0.024938*sin(q1 + 2*q2) + 0.049875*sin(q1 + q2) + 0.049875*sin(q2)));
    manip->C[1] = (double) ( - 0.012469*(dq1 + dq2)*(sin(q1 + 2*q2) + 2*sin(q1 + q2) + 2*sin(q2)));
    manip->C[2] = (double) (dq1*(0.012469*sin(q1 + 2*q2) + 0.024938*sin(q1 + q2) + 0.024938*sin(q2)));
    manip->C[3] = (double) 0.0;
    /*  manip::C is actually a vector, but it can be seen as follows: 
        [C[0], C[1]]
        [C[2], C[3]] */
}

/*
#@
@name: controller
@brief: implements the control law
@notes: it implements the "Inverse Dynamics" control in the joint space.
@inputs: 
- man_t *manip: pointer to the manipulator struct that holds all the current motors' position, speed and acceleration and their reference values;
- double *u: double[2] vector pointer that holds the control input to apply to motors (speed control);
@outputs: 
- void;
@#
*/
void controller(man_t *manip, double *u){
    /* Access the manip variable and use its values to implement the control law */
    /* 
    e_p = q_d-q
    e_d = dq_d-dq
    y = Kp*e_p + Kd*e_d + ddq_d
    tau = B*y+C*dq 
    //---//
    dynamics of the system:
    B*ddq+C*dq+F*dq+g=tau -> dq = inv(C)*(tau-B*ddq)
    u = dq = inv(C)*(tau-B*ddq)
    */
    double q[2], dq[2], ddq[2], q_actual[2], dq_actual[2], ddq_actual[2];
    double ep[2], ed[2], y[2], tau[2], Kpep[2], Kded[2], By[2], Cdq[2];
    double Bddq[2], invC[4], result[2];

    /* data preparation */
    rbpop(&manip->q0, &q[0]);
    rbpop(&manip->q1, &q[1]);
    rbpop(&manip->dq0, &dq[0]);
    rbpop(&manip->dq1, &dq[1]);
    rbpop(&manip->ddq0, &ddq[0]);
    rbpop(&manip->ddq1, &ddq[1]);
    rblast(&manip->q0_actual, &q_actual[0]);
    rblast(&manip->q1_actual, &q_actual[1]);
    rblast(&manip->dq0_actual, &dq_actual[0]);
    rblast(&manip->dq1_actual, &dq_actual[1]);
    rblast(&manip->ddq0_actual, &ddq_actual[0]);
    rblast(&manip->ddq1_actual, &ddq_actual[1]);
    B(manip);
    C(manip);

    diff(q, q_actual, 2, ep); /* q - q_d */
    diff(dq, dq_actual, 2, ed); /* dq - dq_d */

    dot(Kp, 2, 2, ep, 2, 1, Kpep); /* Kp*ep */
    dot(Kd, 2, 2, ed, 2, 1, Kded); /* Kd*ed */

    /* y = Kp*e_p + Kd*e_d + ddq */
    sum(Kpep, Kded, 2, y);
    sum(y, ddq, 2, y);

    dot(manip->B, 2, 2, y, 2, 1, By); /* B*y */
    dot(manip->C, 2, 2, dq_actual, 2, 1, Cdq); /* C*dq */
    sum(By, Cdq, 2, tau); /* tau = B*y+C*dq  */

    dot(manip->B, 2, 2, ddq_actual, 2, 1, Bddq); /* B*ddq */
    diff(tau, Bddq, 2, result); /* tau - B*ddq */
    inv2x2(manip->C, invC); /* inv(C) */
    dot(invC, 2, 2, result, 2, 1, u); /* u = inv(C) * (tau - B*ddq) */
}

/*
#@
@name: rad2stepdir
@brief: converts velocity (rad/s) to step and direction (step dir);
@inputs: 
- float dq: velocity (rad/s);
- float resolution: resolution of the motor (how many radians is a single step?) -> expressed in radians;
- float frequency: reciprocal of delta_t -> time period in which the change of position happens;
- uint8_t *steps: pointer to the variable that will hold the number of steps;
- int8_t *dir: pointer to the variable that will hold the direction (+1 means counterclockwise, -1 means clockwise);
@outputs: 
- void;
@#
*/
void rad2stepdir(float dq, float resolution, float frequency, uint8_t *steps, int8_t *dir){
    /* 
    Given the velocity dq (discretized as delta_q/delta_t), it can be rewritten in terms of resolution and number of steps:
    dq = delta_q/delta_t = delta_q*f -> stepdir*Resolution*f
    stepdir = dq/(Resolution*f)
    dir = sign(stepdir)
    step = abs(stepdir)
    */
    uint8_t stepdir = (uint8_t) dq/(resolution*frequency);
   *dir = sign(stepdir);
   *steps = abs(stepdir);
}

/*
#@
@name: init_rate
@brief: initializes the rate struct
@inputs: 
- rate_t *rate: pointer to the rate struct to initialize;
- uint16_t ms: number of millisecond that define the rate;
@outputs: 
- void;
@#
*/
void init_rate(rate_t *rate, uint16_t ms){
    rate->last_time = clock()/CLOCKS_PER_SEC;
    rate->delta_time = ms;
}

/*
#@
@name: rate_sleep
@brief: stops the process to maintain a fixed framerate (useful in while loops to implement fixed time control loops)
@inputs: 
- rate_t *rate: pointer to the rate struct;
@outputs: 
- void;
@#
*/
void rate_sleep(rate_t *rate){
    clock_t now, interval;
    now = clock()/CLOCKS_PER_SEC; /* timestamp of this instant */
    interval = now - rate->last_time; /* time passed from the last rate_sleep call */
    /* wait until enough time has passed from the last rate_sleep call */
    while((uint16_t) interval < rate->delta_time){
        now = clock()/CLOCKS_PER_SEC;
        interval = now - rate->last_time;
    }
    /* if enough time has passed, save the time stamp and go on with the process */
    rate->last_time = now;
    return;

}




























