#include "custom.h"

#include<stdint.h>  /* used for types like uint8_t and similar */
#include<string.h>  /* used for string manipulation via serial */
#include<stdlib.h>  /* used for string manipulation via serial */
#include<stdio.h>
#include<math.h>    /* used for sin and cos */
#include<time.h>    /* used for fixing the while looping rate */
#include "main.h"
#include "ringbuffer.h"

uint8_t rx_data[DATA_SZ]; /* where the message will be saved for reception */
uint8_t tx_data[DATA_SZ]; /* where the message will be saved for transmission */
man_t manip;

uint32_t previous_trigger = 0;
uint8_t triggered = 0;

/* controller parameters */
const double Kp[4] = {0.5,0,0,0.5};
const double Kd[4] = {0.5,0,0,0.5};
/* reduction values for motors */
const uint8_t reduction1 = 10;
const uint8_t reduction2 = 5;

uint8_t dir2_global;
float global_var;
uint16_t cnt;


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    char *cmd, *data;
    double value;
    unsigned long long encoding;
    uint8_t i = 0;
    /* read the first characters */
    cmd = strtok((char*) rx_data, ":");
    if(strcmp(cmd, "TRJ")){ /* trj case*/
        /* READ the data from the serial and populate the corresponding members of the man_t struct 
           these values will be used to set the reference value for the control loop */
        data = strtok(NULL, ":");
        while(data != NULL){
            if(i == 6) break; /* reading penup */
            // value = "0x"; /* will contain the value extracted from the received string */
            encoding = strtoull(data, NULL, 16);
            memcpy(&value, &encoding, sizeof value);
            // value = strcat(value, data); /* string concatenation REF: https://www.programiz.com/c-programming/library-function/string.h/strcat */
            rbpush((((ringbuffer_t *) &manip)+i), value); /* convert from str to ull -> unsigned long long (uint64_t). REF: https://cplusplus.com/reference/cstdlib/strtoull/ */
            data = strtok(NULL, ":");
            i++;
        }
        rbpush(&manip.penup, (double) atoi(data));
    }else{ /* default case */

    }
    /* wait again for incoming data */
    HAL_UART_Receive_DMA(huart, rx_data, (uint8_t) DATA_SZ); /* DATA_SZ bytes of data for each reception */
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    // TODO: Implement limit switch handling
    uint32_t now;
    now = HAL_GetTick();
    if((now - previous_trigger) > DEBOUNCE_DELAY){
        if(!triggered){
            uint8_t limit_switch = 1;
            // SECTION - DEBUG
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            // !SECTION - DEBUG
        }
        triggered = 1-triggered;
        previous_trigger = now;
    }
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
    /* given n rows and m columns, the matrix indexes i, j correspond to j+i*m array index */
    if(mA != nB){
        C = NULL;
        return 0; /* matrix multiplication cannot be done */
    }
    uint8_t i, j, k;
    for(i = 0; i < nA*mB; i++){
        C[i] = (double) 0.0;
    }

    for( i = 0; i < nA; i++){
        for( j = 0; j < mB; j++){
            for( k = 0; k < mA; k++){
                C[j+i*mB] += (double) (A[k+i*mA]*B[j+k*mB]);
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
@name: det
@brief: computes the determinant of a n x n matrix
@notes: this particular implementation of the determinant algorithm doers not use recursion: is uses the property of elementary row operations that do not change the 
determinant of the matrix to find a upper triangular matrix with the same determinant of the original matrix whose determinant is to be computed.
The determinant of a triangular matrix is the multiplication of the elements on its main diagonal: by finding a triangular matrix B with the same determinant of a given square matrix A, 
the determinant of matrix A can be found by computing the determinant of matrix B.
@inputs: 
- double *M: pointer to the determinant whose determinant is to be computed. IMPORTANT: the values in this matrix will be changed: ensure to keep the original data safe by passing a copy of the original matrix;
- uint8_t n: order of the matrix;
- double *d: pointer to the variable that will hold the resulting determinant;
@outputs: 
- void;
@#
*/
void det(double *M, uint8_t n, double *d){
    uint8_t i,j,k,found;
    int8_t det_sign;
    double temp, factor;
    if(n == 1){
        *d = M[0];
        return;
    }
    if(n == 2){
        *d = DET(M);
        return;
    }
    /*  
        Via transformations that do not change the determinant of a matrix, 
        it is possible to transform the matrix into a triangular one and find the determinant
        through the multiplication of its diagonal.
        If matrix A is transformed into matrix B via elementary row operations:
        1. row exchange: A_j exchanged with A_i-> detB = -detA;
        2. row subtraction: A_j -= k*A_i -> detB = detA;
    */
    det_sign = 1;
    for(k = 0; k < n; k++){
        found = 0; 
        for(i = k; i < n; i++){
            if(M[i*n+k] != 0){
                found = 1;
                if(i != k){
                    det_sign *=-1; /* keep track of sign change */
                    /* exchange rows */
                    for(j = k; j < n; j++){
                        temp = M[i*n+j];
                        M[i*n+j] = M[k*n+j];
                        M[k*n+j] = temp;
                    }
                }
            }
            if(!found){
                *d = 0;
                return;
            }
        }
        /* row subtraction */
        for(i = k+1; i < n; i++){
            factor = (double) (M[i*n+k]/M[k*n+k]);
            for(j = k; j < n; j++){
                M[i*n+j] -= M[k*n+j]*factor;
            }
        }
    }
    /* multiply elements on main diagonal */
    *d = 1;
    for(i = 0; i < n; i++){
        *d *= M[i*n+i];
    }
    *d *= det_sign; /* each row exchange changes the determinant sign */
}

/*
#@
@name: inv
@brief: computes the inverse of a square matrix M;
@notes: the method requires pointer to temporary variables that will hold data used for the computation;
@inputs: 
- double *M: pointer to the matrix whose inverse should be computed;
- double *adjM: pointer to the temporary variable that will hold the adjugate matrix >> should be a nxn array just like M;
- double *subM: pointer to the temporary variable that will hold the submatrices used for the computation of the adjugate matrix >> should be a (n-1)x(n-1) array;
- double *trM: pointer to the temporary variable that will hold the transposed adjugate matrix >> should be a nxn array just like M;
- uint8_t n: order of the matrices;
- double *invM: pointer to the temporary variable that will hold the inverse matrix of M;
@outputs: 
- uint8_t: it is a boolean value that shows whether the inversion is completed successfully or not.
@#
*/
uint8_t inv(double *M, double *adjM, double *subM, double *trM, uint8_t n, double *invM){
    /* cofM and trM are passed by the user so that the size of the arrays are controlled by the user */
    uint8_t i;
    double d;
    for(i = 0; i < n*n; i++){
        trM[i] = M[i]; // copy temporarily matrix M in trM
    }
    det(trM, n, &d);
    if(d == 0) return 0;
    adj(M, subM, n, adjM);
    tr(adjM, n, n, trM);
    for(i = 0; i < n*n; i++){
        invM[i] = (double) (1/d)*trM[i];
    }
    return 1;
}

/*
#@
@name: adj
@brief: computes the adjugate matrix of M
@notes: the method requires temporary variables to hold useful data for the computation;
@inputs: 
- double *M: pointer to the **square** matrix of which the adjugate should be computed;
- double *subM: pointer to the temporary variable that will hold the submatrices used for the computation >> should be a (n-1)x(n-1) array;
- uint8_t n: order of the matrix;
- double *adjM: pointer to the variable that will hold the resulting adjugate matrix;
@outputs: 
- void;
@#
*/
void adj(double *M, double *subM, uint8_t n, double *adjM){
    uint8_t i,j,w,k;
    double d;
    for( i = 0; i < n; i++){
        for(j = 0; j < n; j++){
            k=0;
            w=0;
            while(w < (n-1)*(n-1)){
                if(k%n != j && (uint8_t) (k/n) != i){
                    subM[w] = M[k];
                    w++;
                }
                k++;
            }
            det(subM, n-1, &d);
            if((i+j) % 2 != 0){
                d *= -1;
            }
            adjM[i*n+j] = d;
        }
    }
}

/*
#@
@name: tr
@brief: transposes a matrix
@inputs: 
- double *M: pointer to the matrix to transpose;
- uint8_t n: number of rows;
- uint8_t m: number of columns;
- double *trM: pointer to the variable that will hold the transposed matrix;
@outputs: 
- void;
@#
*/
void tr(double *M, uint8_t n, uint8_t m, double *trM){
    uint8_t i,j;
    for(i = 0; i < n; i++){
        for(j=0; j < m; j++){
            trM[j*n+i] = M[i*m+j];
        }
    }
}

/*
#@
@name: pseudo_inv
@brief: computes the pseudo inverse of matrix M: (M^T*M)^(-1)*M^T
@notes: the method requires temporary variables to hold useful data for the computation; 
@inputs: 
- double *M: pointer to the matrix to pseudo-invert;
- double *trM: pointer to the variable that will hold the transposed matrix used in the pseudo-inversion;
- double *tempM: pointer to the variable that will hold the temporary transposition during the inversion;
- double *adjM: pointer to the variable that will hold the adjugate matrix used during the inversion;
- double *subM: pointer to the variable that will hold the submatrix used during the computation of the adjugate;
- double *invM: pointer to the variable that will hold the inverted matrix (A^T*A)^(-1);
- double *dotM: pointer to the variable that will hold the dot product between A^T and A; 
- uint8_t n: order of the matrix to invert;
- double *psinvM: pointer to the variable that will hold the pseudo-inverse;
@outputs: 
- void;
@#
*/
void pseudo_inv(double *M, double *trM, double *tempM, double *adjM, double *subM, double *invM, double *dotM, uint8_t n, double *psinvM){
    /* (M^T*M)^(-1)*M^T */
    tr(M, n, n, trM);
    dot(trM, n, n, M, n, n, dotM);
    inv(dotM, adjM, subM, tempM, n, invM);
    dot(invM, n, n, trM, n, n, psinvM);
}

/*
#@
@name: B_calc
@brief: computes the inertia matrix of the manipulator
@notes: the inertia matrix B of the manipulator depends on its current configuration: 
it is computed analytically with the dynamic model found via the  Matlab Peter Corke toolbox 
@inputs: 
- man_t *manip: pointer to the manipulator struct that olds the reference and actual values of the position, speed and acceleration of the motors;
@outputs:
- void;
@#
*/
void B_calc(man_t *manip){
    double q1,q2;
    rblast(&manip->q0_actual, &q1);
    rblast(&manip->q1_actual, &q2);
    manip->B[0] = (double) (0.0041601*cos(q1 + 2*q2) + 0.025414*cos(q1 + q2) + 0.075401*cos(q1) + 0.01248*cos(q2) + 0.043486); // (0.024938*cos(q1 + 2*q2) + 0.12469*cos(q1 + q2) + 0.26194*cos(q1) + 0.074812*cos(q2) + 0.16349);
    manip->B[1] = (double) (0.00208*cos(q1 + 2*q2) + 0.020636*cos(q1 + q2) + 0.036429*cos(q1) + 0.0083202*cos(q2) + 0.0096571); // (0.012469*cos(q1 + 2*q2) + 0.09975*cos(q1 + q2) + 0.14962*cos(q1) + 0.049875*cos(q2) + 0.058307);
    manip->B[2] = manip->B[1]; // the matrix is symmetrical
    manip->B[3] = (double) (0.015857*cos(q1 + q2) + 0.036429*cos(q1) + 0.0041601*cos(q2) + 0.0096592); // (0.074812*cos(q1 + q2) + 0.14962*cos(q1) + 0.024938*cos(q2) + 0.058309);
    /*  manip::B is actually a vector, but it can be seen as follows: 
        [B[0], B[1]]
        [B[2], B[3]] */
        
}


/*
#@
@name: C_calc
@brief: computes the coriolis matrix of the manipulator
@notes: the coriolis matrix C of the manipulator depends on its current configuration and joint speed: 
it is computed analytically with the dynamic model found via the  Matlab Peter Corke toolbox 
@inputs: 
- man_t *manip: pointer to the manipulator struct that olds the reference and actual values of the position, speed and acceleration of the motors;
@outputs:
- void;
@#
*/
void C_calc(man_t *manip){
    double q1, q2, dq1, dq2;
    rblast(&manip->q0_actual, &q1);
    rblast(&manip->q1_actual, &q2);
    rblast(&manip->dq0_actual, &dq1);
    rblast(&manip->dq1_actual, &dq2);
    manip->C[0] = (double) ( - 0.5*dq2*(0.0041601*sin(q1 + 2*q2) + 0.009557*sin(q1 + q2) + 0.0083202*sin(q2))); // ( - 0.5*dq2*(0.024938*sin(q1 + 2*q2) + 0.049875*sin(q1 + q2) + 0.049875*sin(q2)));
    manip->C[1] = (double) ( - 0.000056218*(dq1 + dq2)*(37.0*sin(q1 + 2*q2) + 85.0*sin(q1 + q2) + 74.0*sin(q2))); // ( - 0.012469*(dq1 + dq2)*(sin(q1 + 2*q2) + 2*sin(q1 + q2) + 2*sin(q2)));
    manip->C[2] = (double) (dq1*(0.00208*sin(q1 + 2.0*q2) + 0.0047785*sin(q1 + q2) + 0.0041601*sin(q2))); // (dq1*(0.012469*sin(q1 + 2*q2) + 0.024938*sin(q1 + q2) + 0.024938*sin(q2)));
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
    double d;
    uint8_t i;

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
    B_calc(manip);
    C_calc(manip);

    diff(q, q_actual, 2, ep); /* q - q_d */
    diff(dq, dq_actual, 2, ed); /* dq - dq_d */

    ep[0] = abs(ep[0]) < THRESHOLD ? 0:ep[0];
    ep[1] = abs(ep[1]) < THRESHOLD ? 0:ep[1];

    ed[0] = abs(ed[0]) < THRESHOLD ? 0:ed[0];
    ed[1] = abs(ed[1]) < THRESHOLD ? 0:ed[1];

    dot((double *) Kp, 2, 2, ep, 2, 1, Kpep); /* Kp*ep */
    dot((double *) Kd, 2, 2, ed, 2, 1, Kded); /* Kd*ed */

    /* y = Kp*e_p + Kd*e_d + ddq */
    sum(Kpep, Kded, 2, y);
    sum(y, ddq, 2, y);

    dot(manip->B, 2, 2, y, 2, 1, By); /* B*y */
    dot(manip->C, 2, 2, dq_actual, 2, 1, Cdq); /* C*dq */
    sum(By, Cdq, 2, tau); /* tau = B*y+C*dq  */

    // TODO: TEST THIS SHIT

    d = DET(manip->C);
    if(d == 0){
        /* if C is not invertible, use the desired values as inputs */
        // TODO: Test and see if it works, otherwise use discrete integration
        *u = dq[0];
        *(u+1) = dq[1];
        return;
    }

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
void rad2stepdir(double dq, double resolution, double frequency, uint32_t *steps, int8_t *dir){
    /* 
    Given the velocity dq (discretized as delta_q/delta_t), it can be rewritten in terms of resolution and number of steps:
    dq = delta_q/delta_t = delta_q*f -> stepdir*Resolution*f
    stepdir = dq/(Resolution*f)
    dir = sign(stepdir)
    step = abs(stepdir)
    */
    int32_t stepdir = (int32_t) (dq/(resolution*frequency));
    *dir = SIGN(stepdir);
    *steps = abs(stepdir);
}

/*
#@
@name: speed_estimation
@brief: computes the speed and acceleration estimations from a fixed number of previous motor positions
@inputs: 
- ringbuffer_t *q_actual: pointer to the ringbuffer struct that holds all the data relative to the motor position;
- double *v_est: pointer to the variable that will hold the speed estimation;
- double *a_est: pointer to the variable that will hold the acceleration estimation;
@outputs: 
- void;
@#
*/
void speed_estimation(ringbuffer_t *q_actual, double *v_est, double *a_est){
    double now, esp;
    double A[ESTIMATION_STEPS*ESTIMATION_STEPS], X[ESTIMATION_STEPS], P[ESTIMATION_STEPS], invA[ESTIMATION_STEPS*ESTIMATION_STEPS];
    /* temp matrices */
    double trM[ESTIMATION_STEPS*ESTIMATION_STEPS], tempM[ESTIMATION_STEPS*ESTIMATION_STEPS];
    double adjM[ESTIMATION_STEPS*ESTIMATION_STEPS], subM[(ESTIMATION_STEPS-1)*(ESTIMATION_STEPS-1)];
    double invM[ESTIMATION_STEPS*ESTIMATION_STEPS], dotM[ESTIMATION_STEPS*ESTIMATION_STEPS];


    if(q_actual->length < 10){
        rblast(q_actual,&X[0]); /* get newest value */
        /* if not enough data is available, apply simple estimation */
        *v_est = X[0]/T_C;
        *a_est = *v_est/T_C;
        return;
    }

    now = (double) HAL_GetTick()/1000; /* time passed from when the process launch */
    uint8_t i,j;
    for(i = 0; i < ESTIMATION_STEPS; i++){
        for(j = 0; j < ESTIMATION_STEPS; j++){
            A[j+i*ESTIMATION_STEPS] = pow((double)(now - i*T_C), (double) ESTIMATION_STEPS-j-1);
        }
    }

    for(i = 0; i < ESTIMATION_STEPS; i++){
        rbget(q_actual, i, &X[i]);
    }
    /*
        x(t) = sum(p_i*t^i)
        v(t) = sum(i*p_i*t^(i-1))
        a(t) = sum(i*(i-1)*p_i*t^(i-2))
        ---
        p_i -> P[i]
        x_i=A_i*P -> X = [x_0; x_1; ...; x_n] = [A_0; A_1; ...; A_n]*P = A*P -> P = A^(-1)*X = (A^T*A)^(-1)*A^T*X
    */

    pseudo_inv(A, trM, tempM, adjM, subM, invM, dotM, ESTIMATION_STEPS, invA);
    dot(invA, ESTIMATION_STEPS, ESTIMATION_STEPS, X, ESTIMATION_STEPS, 1, P);
    *v_est = 0;
    *a_est = 0;
    for(i = 0; i < ESTIMATION_STEPS; i++){
        esp = (ESTIMATION_STEPS-i-1);
        /* the derivation of constant values is 0 -> exclude the derivative of the constant values from the computation otherwise it would be now^i with i < 0 */
        if(esp-1 >= 0){
            *v_est += esp*pow(now, esp-1)*P[i];
        }
        if(esp-2 >= 0){ 
            *a_est += esp*(esp-1)*pow(now, esp-2)*P[i];
        }
    }
}

/*
#@
@name: init_rate
@brief: initializes the rate struct
@inputs: 
- rate_t *rate: pointer to the rate struct to initialize;
- uint32_t ms: number of millisecond that define the rate;
@outputs: 
- void;
@#
*/
void init_rate(rate_t *rate, uint32_t ms){
    rate->last_time = HAL_GetTick();
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
    double now, interval;
    now = HAL_GetTick();
    interval = (uint32_t) (now - rate->last_time); /* time passed from the last rate_sleep call */
    /* wait until enough time has passed from the last rate_sleep call */
    HAL_Delay(interval);
    /* if enough time has passed, save the time stamp and go on with the process */
    rate->last_time = HAL_GetTick();
    return;
}


/*
#@
@name: read_encoders
@brief: reads data from both the encoders of the 2Dofs planar manipulator
@notes: the method uses two timers to decode the signals coming from both the encoders and memorizes the measured positions of the motors, 
taking into account the reduction ratio of each motor by means of the ARR registers of the timers (ARR=CPR*REDUCTION). 
The method also estimates the speed and accelerations by using the timestamp method.
@inputs: 
- TIM_HandleTypeDef *htim1: pointer to the timer struct that decodes the first encoder;
- TIM_HandleTypeDef *htim2: pointer to the timer struct that decodes the second encoder;
- man_t *manip: pointer to the manipulator struct that holds both the desired and actual motor positions, speeds and accelerations;
@outputs: outputs
@#
*/
void read_encoders(TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2, man_t *manip){
    /*
    CNT/ARR returns a value between 0 and 1: by multiplying it by 2*pi the resulting value shows the position of the motor
    the reduction of the motor is already taken care of in the ARR value: ARR=CPR*REDUCTION -> 4x1000xreduction
    4x is caused by the timer mode (TI1 and TI2)
    */
    uint16_t counter; 
    double displacement1, displacement2;
    double v_est, a_est; /* used to hold temporarily the estimations of speed and acceleration */
    counter = (htim1->Instance->CNT);
    if(counter >= htim1->Instance->ARR){
        counter = (htim1->Instance->ARR-1) - (counter % 1<<16); /* handle underflow */
        htim1->Instance->CNT = counter; /* correct cnt value */
    }

    displacement1 = (double) (2*M_PI*counter/(htim1->Instance->ARR));
    counter = (htim2->Instance->CNT);

    if(counter >= htim2->Instance->ARR){
        counter = (htim2->Instance->ARR-1) - (counter % 1<<16); /* handle underflow */
        htim2->Instance->CNT = counter;  /* correct cnt value */
    }
    displacement2 = (double) (2*M_PI) - (2*M_PI*counter/(htim2->Instance->ARR)); /* the motor is upside down */

    cnt = counter;
    //global_var = displacement2;

    if(displacement1 > 2*M_PI){
    	displacement1 = 2*M_PI; /* clamping */
	}
	if(displacement2 > 2*M_PI){
		displacement2 = 2*M_PI; /* clamping */
	}
    if(displacement1 > M_PI){
    	displacement1 = displacement1 - (2*M_PI); /* redefining the domain between -PI and +PI */
    }
    if(displacement2 > M_PI){
    	displacement2 = displacement2 - (2*M_PI); /* redefining the domain between -PI and +PI */
    }
    /* DIR bit: pag 287 of https://www.st.com/resource/en/reference_manual/rm0383-stm32f411xce-advanced-armbased-32bit-mcus-stmicroelectronics.pdf#page=287 */
    /* dir: 0 = counterclockwise, 1 = clockwise */
    /* the 5th bit of the CR1 register is the DIR bit */
    /*
    uint8_t dir1 = (uint8_t) (htim1->Instance->CR1 >> 4) & 1;
    uint8_t dir2 = (uint8_t) (htim2->Instance->CR1 >> 4) & 1;
    */
    rbpush(&manip->q0_actual, displacement1);
    rbpush(&manip->q1_actual, displacement2);
    /* TODO: do logging of data */

    /* speed and acceleration estimations for both motors*/
    speed_estimation(&manip->q0_actual, &v_est, &a_est);
    rbpush(&manip->dq0_actual, v_est);
    rbpush(&manip->ddq0_actual, a_est);

    speed_estimation(&manip->q1_actual, &v_est, &a_est);
    rbpush(&manip->dq1_actual, v_est);
    rbpush(&manip->ddq1_actual, a_est);

}

void apply_input(TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2, double *u){
    /* T_C = steps*clock_period */
    int8_t dir1, dir2;
    uint32_t steps, ARR, CCR;
    double clock_period;

    /*
    ARR = (uint32_t) 65000;
    CCR = (uint32_t) ARR/2;
    __HAL_TIM_SET_AUTORELOAD(htim1, ARR);
    __HAL_TIM_SET_COMPARE(htim1, TIM_CHANNEL_1, CCR);
    htim1->Instance->EGR = TIM_EGR_UG;
    */
    // rad2stepdir(u[0], RESOLUTION, (double) 1/T_C, &steps, &dir);

    dir1 = u[0] > 0 ?  GPIO_PIN_SET : GPIO_PIN_RESET;
    // dir1 = 1; // DEBUG
    HAL_GPIO_WritePin(DIR_1_GPIO_Port, DIR_1_Pin, dir1);

    dir2 = u[1] > 0 ?  GPIO_PIN_SET : GPIO_PIN_RESET;
    // dir2 = 1; // DEBUG
    HAL_GPIO_WritePin(DIR_2_GPIO_Port, DIR_2_Pin, dir2);

    dir2_global = dir2;

    ARR = (uint32_t) (HAL_RCC_GetPCLK1Freq()*2/(PWM_FREQ-1)); // FIXME - change *2 with *PRESCALER
    CCR = (uint32_t) ((abs(u[0])/MAX_SPEED)*(ARR - 1));
    CCR %= (ARR-1); /* saturate the motor, avoid too high speeds */
    __HAL_TIM_SET_AUTORELOAD(htim1, ARR);
    __HAL_TIM_SET_COMPARE(htim1, TIM_CHANNEL_1, CCR);
    htim1->Instance->EGR = TIM_EGR_UG;

    ARR = (uint32_t) (HAL_RCC_GetPCLK1Freq()*2/(PWM_FREQ-1)); // FIXME - change *2 with *PRESCALER
    CCR = (uint32_t) ((abs(u[1])/MAX_SPEED)*(ARR - 1));
    CCR %= (ARR-1); /* saturate the motor, avoid too high speeds */
    __HAL_TIM_SET_AUTORELOAD(htim2, ARR);
    __HAL_TIM_SET_COMPARE(htim2, TIM_CHANNEL_1, CCR);
    htim2->Instance->EGR = TIM_EGR_UG;
    


    // rad2stepdir(u[0], RESOLUTION, (double) 1/T_C, &steps, &dir);
    // if(steps > 0){
    //     clock_period = T_C/(steps*reduction1);
    // }else{
    //     clock_period = 0;
    // }
    // uint32_t mamt = HAL_RCC_GetPCLK1Freq()*2;
    // ARR = (uint32_t) (HAL_RCC_GetPCLK1Freq()*2*clock_period); /* read clock frequency for APB1 */
    // CCR = (uint32_t) ARR/2;
// 
    // __HAL_TIM_SET_AUTORELOAD(htim1, ARR);
    // __HAL_TIM_SET_COMPARE(htim1, TIM_CHANNEL_1, CCR);
    // htim1->Instance->EGR = TIM_EGR_UG;
// 
    // rad2stepdir(u[1], RESOLUTION, (double) 1/T_C, &steps, &dir);
    // if(steps > 0){
    //     clock_period = T_C/(steps*reduction2);
    // }else{
    //     clock_period = 0;
    // }
    // ARR = (uint32_t) (HAL_RCC_GetPCLK1Freq()*clock_period); /* read clock frequency for APB1 */
    // CCR = (uint32_t) ARR/2;
// 
    // __HAL_TIM_SET_AUTORELOAD(htim2, ARR);
    // __HAL_TIM_SET_COMPARE(htim2, TIM_CHANNEL_1, CCR);
    // htim2->Instance->EGR = TIM_EGR_UG;
}

void start_timers(TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3, TIM_HandleTypeDef *htim4){
    HAL_TIM_Base_Start_IT(htim1);
    HAL_TIM_Base_Start_IT(htim2);
    /* start motor PWM */
    HAL_TIM_Base_Start_IT(htim3);
    HAL_TIM_Base_Start_IT(htim4);
    /* start PWM */
    if(HAL_TIM_PWM_Start(htim3, TIM_CHANNEL_1) != HAL_OK){
        HardFault_Handler();
    }
    if(HAL_TIM_PWM_Start(htim4, TIM_CHANNEL_1) != HAL_OK){
        HardFault_Handler();
    }
}

void stop_timers(TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2, TIM_HandleTypeDef *htim3, TIM_HandleTypeDef *htim4){
    HAL_TIM_Base_Stop_IT(htim1);
    HAL_TIM_Base_Stop_IT(htim2);
    /* stop motor PWM */
    HAL_TIM_Base_Stop_IT(htim3);
    HAL_TIM_Base_Stop_IT(htim4);
}

void log_data(UART_HandleTypeDef *huart, man_t *manip){
    unsigned long long int encoding_q0, encoding_q1, encoding_q0_d, encoding_q1_d;
    uint32_t timestamp;
    uint8_t i;
    // SECTION DEBUG
    for(i = 0; i < sizeof tx_data; i++){
        tx_data[i] = 0;
    }
    // !SECTION DEBUG
    double q;
    rblast(&manip->q0_actual, &q);
    memcpy(&encoding_q0, &q, sizeof q);
    rblast(&manip->q1_actual, &q);
    memcpy(&encoding_q1, &q, sizeof q);
    rbpeek(&manip->q0, &q);
    memcpy(&encoding_q0_d, &q, sizeof q);
    rbpeek(&manip->q1, &q);
    memcpy(&encoding_q1_d, &q, sizeof q);
    timestamp = HAL_GetTick();
    sprintf(tx_data, "%X:%X:%X:%X:%X\n", (unsigned long long int) timestamp, encoding_q0, encoding_q1, encoding_q0_d, encoding_q1_d); /*Timestamp:q0:q1*/
    HAL_UART_Transmit_DMA(huart, (uint8_t *) tx_data, sizeof tx_data); /* send encoder data for logging purposes */
}



























