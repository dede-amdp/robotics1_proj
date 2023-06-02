#include<stdint.h> /* used for types like uint8_t and similar */
#include<stdlib.h> /* used for string manipulation via serial */

/*
3 for the command string (3 chars)
+8 for each ":" char (may be less)
+6*2 for each value sent via the trj command (6 values of 64 bits -> 16 bytes)
+1 for the penup value which is 1 or 0
*10 buffer size

(3+8+6*2+1)*10 = 240 (bytes)
a better value would be 32*10, so that the uint32_t type could be used instead of uint8_t
Also, using uint32_t allows for longer messages to be sent, in case it is needed

*/
#define BUFFER_SZ 10

typedef struct manipulator {
    uint64_t q0;
    uint64_t q1;
    uint64_t dq0;
    uint64_t dq1;
    uint64_t ddq0;
    uint64_t ddq1;
    uint8_t penup;
    uint64_t q0_actual;
    uint64_t q1_actual;
    uint64_t dq0_actual;
    uint64_t dq1_actual;
    uint64_t ddq0_actual;
    uint64_t ddq1_actual;
    uint8_t penup_actual;
} man_t;

uint32_t rx_data[BUFFER_SZ]; // where the message will be saved for reception
uint32_t tx_data[BUFFER_SZ]; // where the message will be saved for transmission
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
            *(((uint64_t *) &manip)+i) = strtoull(value, NULL, 16); /* convert from str to ull -> unsigned long long (uint64_t). REF: https://cplusplus.com/reference/cstdlib/strtoull/ */
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

void controller(){
    /* Access the manip variable and use its values to implement the control law */
    // TODO: implementation
}