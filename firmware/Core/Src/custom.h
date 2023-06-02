#include<stdint.h> /* used for types like uint8_t and similar */
#include<stdlib.h> /* used for string manipulation via serial */
#include <math.h> /* used for sin and cos */
#include "main.h"

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

typedef struct manipulator {
    double q0;
    double q1;
    double dq0;
    double dq1;
    double ddq0;
    double ddq1;
    uint8_t penup;
    double q0_actual;
    double q1_actual;
    double dq0_actual;
    double dq1_actual;
    double ddq0_actual;
    double ddq1_actual;
    uint8_t penup_actual;
} man_t;

uint8_t rx_data[DATA_SZ * BUFFER_SZ]; // where the message will be saved for reception
uint8_t tx_data[DATA_SZ * BUFFER_SZ]; // where the message will be saved for transmission
man_t manip;

uint8_t cbi = 0; // circular buffer index


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
            *(((double *) &manip)+i) = strtod(value, NULL, 10); /* convert from str to ull -> unsigned long long (uint64_t). REF: https://cplusplus.com/reference/cstdlib/strtoull/ */
            data = strtok(NULL, ":");
            i++;
        }
        manip.penup = (uint8_t) atoi(*data); // penup is just 0 or 1
    }else{ /* default case */

    }
    // update the index
    cbi++;
    // wait again for incoming data
    HAL_UART_Receive_IT(&huart, (uint8_t *) *rx_data, (uint8_t) 4); // 32 bits of data for each reception, in a buffer of BUFFER_SZ elements
}

void B(q1, q2, double* matrix){
    matrix[0] = (double) (0.024938*cos(q1 + 2*q2) + 0.12469*cos(q1 + q2) + 0.26194*cos(q1) + 0.074812*cos(q2) + 0.16349);
    matrix[1] = (double) (0.012469*cos(q1 + 2*q2) + 0.09975*cos(q1 + q2) + 0.14962*cos(q1) + 0.049875*cos(q2) + 0.058307);
    matrix[2] = matrix[1]; // the matrix is symmetrical
    matrix[3] = (double) (0.074812*cos(q1 + q2) + 0.14962*cos(q1) + 0.024938*cos(q2) + 0.058309);
    /*  matrix is actually a vector, but it can be seen as follows: 
        [matrix[0], matrix[1]]
        [matrix[2], matrix[3]] */
}

void C(q1, q2, dq1, dq2, double* matrix){
    matrix[0] = (double) ( - 0.5*dq2*(0.024938*sin(q1 + 2*q2) + 0.049875*sin(q1 + q2) + 0.049875*sin(q2)));
    matrix[1] = (double) ( - 0.012469*(dq1 + dq2)*(sin(q1 + 2*q2) + 2*sin(q1 + q2) + 2*sin(q2)));
    matrix[2] = (double) (dq1*(0.012469*sin(q1 + 2*q2) + 0.024938*sin(q1 + q2) + 0.024938*sin(q2)));
    matrix[3] = (double) 0.0;
    /*  matrix is actually a vector, but it can be seen as follows: 
        [matrix[0], matrix[1]]
        [matrix[2], matrix[3]] */
}

void controller(){
    /* Access the manip variable and use its values to implement the control law */
    // TODO: implementation
}
