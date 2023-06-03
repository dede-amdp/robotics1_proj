#include "custom.h"

#include<stdint.h> /* used for types like uint8_t and similar */
#include<stdlib.h> /* used for string manipulation via serial */
#include <math.h> /* used for sin and cos */
#include "main.h"


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

    char *cmd, *data, *value;
    uint8_t i = 0;
    /* read the characters */
    cmd = strtok((char*) rx_data, ":");
    if(strcmp(cmd, "trj")){ /* trj case*/
        /* READ the data from the serial and populate the corresponding members of the man_t struct 
           these values will be used to set the reference value for the control loop */
        data = strtok(NULL, ":");
        while(data != NULL){
            if(i == 6) break; // reading penup
            *value = "0x"; /* will contain the value extracted from the received string */
            strcat(value, data); /* string concatenation REF: https://www.programiz.com/c-programming/library-function/string.h/strcat */
            *(((double *) &manip)+i) = strtoull(value, NULL, 16); /* convert from str to ull -> unsigned long long (uint64_t). REF: https://cplusplus.com/reference/cstdlib/strtoull/ */
            data = strtok(NULL, ":");
            i++;
        }
        manip.penup = (uint8_t) atoi(*data); // penup is just 0 or 1
    }else{ /* default case */

    }
    // update the index
    cbi++;
    // wait again for incoming data
    HAL_UART_Receive_IT(&huart, &rx_data, (uint8_t) 4); // 32 bits of data for each reception, in a buffer of BUFFER_SZ elements
}


uint8_t dot(double *A, uint8_t nA, uint8_t mA, double* B, uint8_t nB, uint8_t mB, double* C){ /* nAxmA * nBxmB dot product */
    /*  why wasn't a matrix struct created to check if the matrices can be multiplied? 
        this firmware does not make heavy use of matrices to justify creating a struct,
        this method exists just to make the code more readable and understand what each
        operation actually does instead of having meaningless calculations */
    
    /* C[i, j] = \sum_k A[i, k]*B[k, j] */
    /* given n rows and m columns, the matrix indexes i, j correspond to j+i*n array index */
    if(mA != nB){
        C = NULL;
        return 0; // matrix multiplication cannot be done
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

uint8_t inv2x2(double *M, double *invM){
    double d = DET(M);
    if(d == 0){
        invM = NULL;
        return 0;
    }
    invM[0] = M[3]/d;
    invM[3] = M[0]/d;
    invM[1] = -M[1]/d;
    invM[2] = -M[2]/d;
    return 1;
}

void sum(double *A, double *B, uint8_t n, double *C){
    uint8_t i;
    for(i = 0; i < n; i++){
        C[i] = A[i] + B[i];
    }
}

void diff(double *A, double *B, uint8_t n, double *C){
    uint8_t i;
    for(i = 0; i < n; i++){
        C[i] = A[i] - B[i];
    }
}

void B(man_t *manip){
    double q1,q2;
    q1 = manip->q_actual[0];
    q2 = manip->q_actual[1];
    manip->B[0] = (double) (0.024938*cos(q1 + 2*q2) + 0.12469*cos(q1 + q2) + 0.26194*cos(q1) + 0.074812*cos(q2) + 0.16349);
    manip->B[1] = (double) (0.012469*cos(q1 + 2*q2) + 0.09975*cos(q1 + q2) + 0.14962*cos(q1) + 0.049875*cos(q2) + 0.058307);
    manip->B[2] = manip->B[1]; // the matrix is symmetrical
    manip->B[3] = (double) (0.074812*cos(q1 + q2) + 0.14962*cos(q1) + 0.024938*cos(q2) + 0.058309);
    /*  manip::B is actually a vector, but it can be seen as follows: 
        [B[0], B[1]]
        [B[2], B[3]] */
}

void C(man_t *manip){
    double q1, q2, dq1, dq2;
    q1 = manip->q_actual[0];
    q2 = manip->q_actual[1];
    dq1 = manip->dq_actual[0]; 
    dq2 = manip->dq_actual[1];
    manip->C[0] = (double) ( - 0.5*dq2*(0.024938*sin(q1 + 2*q2) + 0.049875*sin(q1 + q2) + 0.049875*sin(q2)));
    manip->C[1] = (double) ( - 0.012469*(dq1 + dq2)*(sin(q1 + 2*q2) + 2*sin(q1 + q2) + 2*sin(q2)));
    manip->C[2] = (double) (dq1*(0.012469*sin(q1 + 2*q2) + 0.024938*sin(q1 + q2) + 0.024938*sin(q2)));
    manip->C[3] = (double) 0.0;
    /*  manip::C is actually a vector, but it can be seen as follows: 
        [C[0], C[1]]
        [C[2], C[3]] */
}

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
    double ep[2], ed[2], y[2], tau[2], Kpep[2], Kded[2], By[2], Cdq[2], tau[2];
    double Bddq[2], invC[4], result[2];
    uint8_t i;

    diff(manip->q, manip->q_actual, 2, ep); /* q - q_d */
    diff(manip->dq, manip->dq_actual, 2, ed); /* dq - dq_d */

    dot(Kp, 2, 2, ep, 2, 1, Kpep); /* Kp*ep */
    dot(Kd, 2, 2, ed, 2, 1, Kded); /* Kd*ed */

    /* y = Kp*e_p + Kd*e_d + ddq */
    sum(Kpep, Kded, 2, y);
    sum(y, manip->ddq, 2, y);

    dot(manip->B, 2, 2, y, 2, 1, By); /* B*y */
    dot(manip->C, 2, 2, manip->dq_actual, 2, 1, Cdq); /* C*dq */
    sum(By, Cdq, 2, tau); /* tau = B*y+C*dq  */

    dot(manip->B, 2, 2, manip->ddq_actual, 2, 1, Bddq); /* B*ddq */
    diff(tau, Bddq, 2, result); /* tau - B*ddq */
    inv2x2(manip->C, invC); /* inv(C) */
    dot(invC, 2, 2, result, 2, 1, u); /* u = inv(C) * (tau - B*ddq) */
}
