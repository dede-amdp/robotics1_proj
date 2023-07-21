#include "custom.h"

#include<stdint.h>  /* used for types like uint8_t and similar */
#include<string.h>  /* used for string manipulation via serial */
#include<stdlib.h>  /* used for string manipulation via serial */
#include<stdio.h>
#include<math.h>    /* used for sin and cos */
#include<time.h>    /* used for fixing the while looping rate */
#include "main.h"
#include "ringbuffer.h"
#include "pid_controller.h"

uint8_t rx_data[DATA_SZ]; /* where the message will be saved for reception */
uint8_t tx_data[DATA_SZ]; /* where the message will be saved for transmission */
man_t manip;

pid_controller_t pid_pos1, pid_pos2, pid_vel1, pid_vel2;

uint32_t previous_trigger1 = 0;
uint32_t previous_trigger2 = 0;

uint8_t triggered1 = 0;
uint8_t triggered2 = 0;

/* controller parameters */
const float Kp[4] = {0.8,0,0,0.8};
const float Kd[4] = {2,0,0,2};


/* reduction values for motors */
const uint8_t reduction1 = 10;
const uint8_t reduction2 = 5;

ringbuffer_t timestamps;
float disp1, disp2;
float offset1=0.f;
float offset2=0.f;
float dq_actual0, dq_actual1;
float ddq_actual0, ddq_actual1;
uint32_t count = 0;
int limit_switch1 = 0;
int limit_switch2 = 1;
float ui[2]= {0.0 , 0.0};

float pos_prec[2]={0.f ,0.f};




void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	//__disable_irq();

	char str[DATA_SZ];

    char *cmd, *data;

    //printf(rx_data);
   // printf("\n");
    //fflush(stdout);



    double value;
    unsigned long long encoding;
    uint8_t i = 0;
    /* read the first characters */

    memcpy(str,rx_data,sizeof str);

    char  *save_ptr;

    cmd = strtok_r((char*) str, ":", &save_ptr);

   // printf("%s %d \n",cmd,!strcmp(cmd, "TRJ"));
    //fflush(stdout);

    if(!strcmp(cmd, "TRJ")){
    	   count++;

    	/* trj case*/
        /* READ the data from the serial and populate the corresponding members of the man_t struct 
           these values will be used to set the reference value for the control loop */
        //data = strtok(cmd+sizeof cmd, ":");
	   data = strtok_r(cmd+sizeof cmd, ":",  &save_ptr);
        while(data != NULL){
            if(i == 6) break; /* reading penup */
            // value = "0x"; /* will contain the value extracted from the received string */
            encoding = strtoull(data, NULL, 16);
            memcpy(&value, &encoding, sizeof value);

            // value = strcat(value, data); /* string concatenation REF: https://www.programiz.com/c-programming/library-function/string.h/strcat */
            rbpush((((ringbuffer_t *) &manip)+i), (float) value); /* convert from str to ull -> unsigned long long (uint64_t). REF: https://cplusplus.com/reference/cstdlib/strtoull/ */
            data = strtok_r(NULL, ":", &save_ptr);
            i++;
        }
        rbpush(&manip.penup, (float) atoi(data));
    }else{ /* default case */

    }
    /* wait again for incoming data */
    HAL_UART_Receive_DMA(huart, rx_data, (uint8_t) DATA_SZ); /* DATA_SZ bytes of data for each reception */

    //HAL_UART_Receive_DMA(huart, rx_data, 121);



	//printf(" count %d \n",count);



    //__enable_irq();

       return
	   count++;


				/* Debug*/

		  //HAL_UART_Receive_DMA(huart, rx_data, 121);



}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    // TODO: Implement limit switch handling
    uint32_t now;
    now = HAL_GetTick();

    if((now - previous_trigger1) > DEBOUNCE_DELAY ){
        if(!triggered1){
          limit_switch1 = 1;
          rblast(&manip.q0_actual,&offset1);
            // SECTION - DEBUG
          printf("triggered %d \n",count);
          fflush(stdout);
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            // !SECTION - DEBUG
        }
        triggered1 = 1-triggered1;
        previous_trigger1 = now;
    }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM10){
		/* check if it is the proper instance */
		read_encoders(&manip);

	}
}


#define DEMCR               *((volatile uint32_t*) 0xE000EDFCU)
#define ITM_STIMULUS_PORT0  *((volatile uint32_t*) 0xE0000000)
#define ITM_TRACE_EN        *((volatile uint32_t*) 0xE0000E00)

void ITM_Sendchar(uint8_t ch){

	// Enable TRCENA
	DEMCR |= (1<<24);

	//Enable Stimulus Port0
	ITM_TRACE_EN |= (1<<0);

	// Read FIFO Status in bit[0]:
	while(! (ITM_STIMULUS_PORT0 & 1));

	// Write to IT Stimulus Port0
	ITM_STIMULUS_PORT0 = ch;
}

int _write(int file,char *ptr, int len){

	int DataIdx;
	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		ITM_Sendchar(*ptr++);
	}
	return len;

}


/*
#@
@name: init_man
@brief: initializes the members of the man_t struct
@inputs: 
- man_t *manip: man_t obj. to initialize;
- TIM_HandleTypeDef *htim1: pointer to the timer used to decode the output of the first encoder;
- TIM_HandleTypeDef *htim2: pointer to the timer used to decode the output of the second encode;
@outputs: 
- void;
@#
*/
void init_man(man_t *manip, TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2){
    uint8_t i;
    // initialize all the ring buffers
    for(i = 0; i < 14; i++){
        rbclear((((ringbuffer_t *) manip)+i));
    }
    // initialize the inertia and coriolis matrices
    for(i = 0; i < 4; i++){
        manip->B[i] = (float) 0;
        manip->C[i] = (float) 0;
    }
    manip->htim_encoder1 = htim1;
    manip->htim_encoder2 = htim2;
}

/*
#@
@name: dot
@brief: computes the dot product between two matrices, A and B, represented in vector form;
@inputs: 
- float *A: pointer to a vector of floats of size nA*mA, which represents the first nAxmA matrix;
- uint8_t nA: number of rows of matrix A;
- uint8_t mA: number of columns of matrix A;
- float *B: pointer to a vector of floats of size nB*mB, which represents the second nBxmB matrix;
- uint8_t nB: number of rows of matrix B;
- uint8_t mB: number of columns of matrix B;
- float *C: pointer to a vector of floats of size nA*mB, which represents the resulting nAxmB matrix -> if the operation cannot be done, it will be NULL;
@outputs: 
- uint8_t: 0 or 1 that shows whether the operation completed successfully or not.
@#
*/
uint8_t dot(float *A, uint8_t nA, uint8_t mA, float* B, uint8_t nB, uint8_t mB, float* C){ /* nAxmA * nBxmB dot product */
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
        C[i] = (float) 0.0;
    }

    for( i = 0; i < nA; i++){
        for( j = 0; j < mB; j++){
            for( k = 0; k < mA; k++){
                C[j+i*mB] += (float) (A[k+i*mA]*B[j+k*mB]);
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
- float *M: pointer to the matrix to invert;
- float *invM: pointer to the inverted matrix (NULL if inversion is not possible);
@outputs: 
- uint8_t: shows whether the inversion was completed or not
@#
*/
uint8_t inv2x2(float *M, float *invM){
    float d = DET(M);
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
- float *A: pointer to the first matrix to sum;
- float *B: pointer to the second matrix to sum;
- uint8_t n: number of elements in the matrix;
- float *C: pointer to the resulting matrix;
@outputs: 
- void;
@#
*/
void sum(float *A, float *B, uint8_t n, float *C){
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
- float *A: pointer to the first matrix to subtract;
- float *B: pointer to the second matrix to subtract;
- uint8_t n: number of elements in the matrix;
- float *C: pointer to the resulting matrix;
@outputs: 
- void;
@#
*/
void diff(float *A, float *B, uint8_t n, float *C){
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
- float *M: pointer to the determinant whose determinant is to be computed. IMPORTANT: the values in this matrix will be changed: ensure to keep the original data safe by passing a copy of the original matrix;
- uint8_t n: order of the matrix;
- float *d: pointer to the variable that will hold the resulting determinant;
@outputs: 
- void;
@#
*/
void det(float *M, uint8_t n, float *d){
    uint8_t i,j,k,found;
    int8_t det_sign;
    float temp, factor;
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
            factor = (float) (M[i*n+k]/M[k*n+k]);
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
- float *M: pointer to the matrix whose inverse should be computed;
- float *adjM: pointer to the temporary variable that will hold the adjugate matrix >> should be a nxn array just like M;
- float *subM: pointer to the temporary variable that will hold the submatrices used for the computation of the adjugate matrix >> should be a (n-1)x(n-1) array;
- float *trM: pointer to the temporary variable that will hold the transposed adjugate matrix >> should be a nxn array just like M;
- uint8_t n: order of the matrices;
- float *invM: pointer to the temporary variable that will hold the inverse matrix of M;
@outputs: 
- uint8_t: it is a boolean value that shows whether the inversion is completed successfully or not.
@#
*/
uint8_t inv(float *M, float *adjM, float *subM, float *trM, uint8_t n, float *invM){
    /* cofM and trM are passed by the user so that the size of the arrays are controlled by the user */
    uint8_t i;
    float d;
    for(i = 0; i < n*n; i++){
        trM[i] = M[i]; // copy temporarily matrix M in trM
    }
    det(trM, n, &d);
    if(d == 0) return 0;
    adj(M, subM, n, adjM);
    tr(adjM, n, n, trM);
    for(i = 0; i < n*n; i++){
        invM[i] = (float) (1/d)*trM[i];
    }
    return 1;
}

/*
#@
@name: adj
@brief: computes the adjugate matrix of M
@notes: the method requires temporary variables to hold useful data for the computation;
@inputs: 
- float *M: pointer to the **square** matrix of which the adjugate should be computed;
- float *subM: pointer to the temporary variable that will hold the submatrices used for the computation >> should be a (n-1)x(n-1) array;
- uint8_t n: order of the matrix;
- float *adjM: pointer to the variable that will hold the resulting adjugate matrix;
@outputs: 
- void;
@#
*/
void adj(float *M, float *subM, uint8_t n, float *adjM){
    uint8_t i,j,w,k;
    float d;
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
- float *M: pointer to the matrix to transpose;
- uint8_t n: number of rows;
- uint8_t m: number of columns;
- float *trM: pointer to the variable that will hold the transposed matrix;
@outputs: 
- void;
@#
*/
void tr(float *M, uint8_t n, uint8_t m, float *trM){
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
- float *M: pointer to the matrix to pseudo-invert;
- float *trM: pointer to the variable that will hold the transposed matrix used in the pseudo-inversion;
- float *tempM: pointer to the variable that will hold the temporary transposition during the inversion;
- float *adjM: pointer to the variable that will hold the adjugate matrix used during the inversion;
- float *subM: pointer to the variable that will hold the submatrix used during the computation of the adjugate;
- float *invM: pointer to the variable that will hold the inverted matrix (A^T*A)^(-1);
- float *dotM: pointer to the variable that will hold the dot product between A^T and A; 
- uint8_t n: order of the matrix to invert;
- float *psinvM: pointer to the variable that will hold the pseudo-inverse;
@outputs: 
- void;
@#
*/
void pseudo_inv(float *M, float *trM, float *tempM, float *adjM, float *subM, float *invM, float *dotM, uint8_t n, float *psinvM){
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
    float q1,q2;
    rblast(&manip->q0_actual, &q1);
    rblast(&manip->q1_actual, &q2);

    manip->B[0] = (float) (0.0047413*cos(q1 + 2*q2) + 0.028554*cos(q1 + q2) + 0.078463*cos(q1) + 0.014224*cos(q2) + 0.045182); // (0.024938*cos(q1 + 2*q2) + 0.12469*cos(q1 + q2) + 0.26194*cos(q1) + 0.074812*cos(q2) + 0.16349);
    manip->B[1] = (float) (0.0023706*cos(q1 + 2*q2) + 0.023453*cos(q1 + q2) + 0.039491*cos(q1) + 0.0094825*cos(q2) + 0.01103); // (0.012469*cos(q1 + 2*q2) + 0.09975*cos(q1 + q2) + 0.14962*cos(q1) + 0.049875*cos(q2) + 0.058307);
    manip->B[2] = manip->B[1]; // the matrix is symmetrical
    manip->B[3] = (float) (0.018351*cos(q1 + q2) + 0.039491*cos(q1) + 0.0047413*cos(q2) + 0.011032); // (0.074812*cos(q1 + q2) + 0.14962*cos(q1) + 0.024938*cos(q2) + 0.058309);
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
    float q1, q2, dq1, dq2;
    rblast(&manip->q0_actual, &q1);
    rblast(&manip->q1_actual, &q2);
    rblast(&manip->dq0_actual, &dq1);
    rblast(&manip->dq1_actual, &dq2);

    // SECTION DEBUG
	// dq1 = dq_actual0;
	// dq2 = dq_actual1;
	// !SECTION DEBUG

    manip->C[0] = (float) ( - 0.5*dq2*(0.0047413*sin(q1 + 2*q2) + 0.010203*sin(q1 + q2) + 0.0094825*sin(q2))); // ( - 0.5*dq2*(0.024938*sin(q1 + 2*q2) + 0.049875*sin(q1 + q2) + 0.049875*sin(q2)));
    manip->C[1] = (float) ( - 0.000030008*(dq1 + dq2)*(79.0*sin(q1 + 2*q2) + 170*sin(q1 + q2) + 158*sin(q2))); // ( - 0.012469*(dq1 + dq2)*(sin(q1 + 2*q2) + 2*sin(q1 + q2) + 2*sin(q2)));
    manip->C[2] = (float) (   dq1*(0.0023706*sin(q1 + 2*q2) + 0.0051014*sin(q1 + q2) + 0.0047413*sin(q2))); // (dq1*(0.012469*sin(q1 + 2*q2) + 0.024938*sin(q1 + q2) + 0.024938*sin(q2)));
    manip->C[3] = (float) 0.0;
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
- float *u: float[2] vector pointer that holds the control input to apply to motors (speed control);
@outputs: 
- void;
@#
*/
void controller(man_t *manip, float *u){
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
    float q[2], dq[2], ddq[2], q_actual[2], dq_actual[2], ddq_actual[2];
    float ep[2], ed[2], y[2], tau[2], Kpep[2], Kded[2], By[2], Cdq[2];
    float Bddq[2], invC[4], result[2];
    float d;
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


    //diff(q, q_actual, 2, ep); /* q - q_d */
    //diff(dq, dq_actual, 2, ed); /* dq - dq_d */


    // debug
    //ddq_actual0=ddq_actual[0];
    //ddq_actual1=ddq_actual[1];
    //!debug

    ep[0]=q[0]-q_actual[0];
    ep[1]=q[1]-q_actual[1];

    ed[0]=dq[0]-dq_actual[0];
    ed[1]=dq[1]-dq_actual[1];

    ep[0] = abs(ep[0]) < THRESHOLD ? 0:ep[0];
    ep[1] = abs(ep[1]) < THRESHOLD ? 0:ep[1];

    ed[0] = abs(ed[0]) < THRESHOLD ? 0:ed[0];
    ed[1] = abs(ed[1]) < THRESHOLD ? 0:ed[1];

    dot((float *) Kp, 2, 2, ep, 2, 1, Kpep); /* Kp*ep */
    dot((float *) Kd, 2, 2, ed, 2, 1, Kded); /* Kd*ed */

    /* y = Kp*e_p + Kd*e_d + ddq */
    sum(Kpep, Kded, 2, y);
    sum(y, ddq, 2, y);

    ui[0]+=y[0]*T_C;
    ui[1]+=y[1]*T_C;

    *u = ui[0];
    *(u+1) = ui[1];

    return;

    dot(manip->B, 2, 2, y, 2, 1, By); /* B*y */
    dot(manip->C, 2, 2, dq_actual, 2, 1, Cdq); /* C*dq */
    sum(By, Cdq, 2, tau); /* tau = B*y+C*dq  */





    // TODO: TEST THIS SHIT

    d = DET(manip->C);
    if(ABS(d) < 1e-5){
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

    if(ABS(u[0])>1000 || ABS(u[1])>1000){

    	double a;
    	a=0;

    }

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
void rad2stepdir(float dq, float resolution, float frequency, uint32_t *steps, int8_t *dir){
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
- float *v_est: pointer to the variable that will hold the speed estimation;
- float *a_est: pointer to the variable that will hold the acceleration estimation;
@outputs: 
- void;
@#
*/
void speed_estimation(ringbuffer_t *q_actual, ringbuffer_t *dq_actual, ringbuffer_t *ddq_actual, float reduction, float *v_est, float *a_est){
    float now, esp;
    float A[ESTIMATION_STEPS*ESTIMATION_STEPS], X[ESTIMATION_STEPS], P[ESTIMATION_STEPS], invA[ESTIMATION_STEPS*ESTIMATION_STEPS];
    /* temp matrices */
    float trM[ESTIMATION_STEPS*ESTIMATION_STEPS], tempM[ESTIMATION_STEPS*ESTIMATION_STEPS];
    float adjM[ESTIMATION_STEPS*ESTIMATION_STEPS], subM[(ESTIMATION_STEPS-1)*(ESTIMATION_STEPS-1)];
    float invM[ESTIMATION_STEPS*ESTIMATION_STEPS], dotM[ESTIMATION_STEPS*ESTIMATION_STEPS];
    uint8_t i;

    /*
    float a[4];
	rbget(q_actual, 0, &a[0]);
	rbget(q_actual, 1, &a[1]);
	rbget(dq_actual, 0, &a[2]);
	rbget(dq_actual, 1, &a[3]);
	uint8_t index1 = (q_actual->head) + (q_actual->length) - 1;
	uint8_t index2 = (dq_actual->head) + (dq_actual->length) - 1;
	a[0] = q_actual->buffer[index1%RBUF_SZ];
	a[1] = q_actual->buffer[(index1-1+RBUF_SZ)%RBUF_SZ];
	a[2] = dq_actual->buffer[index2%RBUF_SZ];
	a[3] = dq_actual->buffer[(index2-1+RBUF_SZ)%RBUF_SZ];
	// if not enough data is available, apply simple estimation
	*v_est = ((a[1]-a[0])/T_S);
	*a_est = (a[3]-a[2])/T_S;
    return;
    */

    float prev, succ, vel,acc, a, b;
    succ=0;
    prev=0;
    for(i = 0; i < 5; i++){
    	rbget(q_actual, i, &a);
    	prev+=a;
    }
    for(i = 0; i < 5; i++){
    	rbget(q_actual, 5+i, &a);
    	succ+=a;
    }
    prev /=5;
    succ /=5;
    /*
    //*v_est = (succ-prev)/(T_C*5);
    rbget(q_actual,RBUF_SZ-1, &succ);
    rbget(q_actual,RBUF_SZ-2, &prev);
	*v_est=(succ-prev)/T_S;

    rbget(dq_actual, RBUF_SZ-1, &succ);
    rbget(dq_actual, RBUF_SZ-2, &prev);
    *a_est = (succ-prev)/T_S;

    /* filtering velocity with a first order filter  */




    //rbget(q_actual,RBUF_SZ-1, &pos_succ);
    //rbget(q_actual,RBUF_SZ-2, &pos_prev);
    rblast(dq_actual,&vel);


    *v_est=0.8546*vel+((1-0.8546)*(succ-prev)/(T_C*5) );


    rbget(dq_actual, RBUF_SZ-1, &succ);
    rbget(dq_actual, RBUF_SZ-2, &prev);
    //*a_est = (succ-prev)/T_C;
    rblast(ddq_actual,&acc);
    *a_est= 0.9245*acc+((1- 0.9245)*(succ-prev)/(T_C) );


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
    float now, interval;
    now = HAL_GetTick();
    interval = (uint32_t) (now - rate->last_time); /* time passed from the last rate_sleep call */
    /* wait until enough time has passed from the last rate_sleep call */
    if(interval < rate->delta_time){
        HAL_Delay(rate->delta_time-interval);
    }
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
- man_t *manip: pointer to the manipulator struct that holds both the desired and actual motor positions, speeds and accelerations;
@outputs: outputs
@#
*/
void read_encoders(man_t *manip){
    /*
    CNT/ARR returns a value between 0 and 1: by multiplying it by 2*pi the resulting value shows the position of the motor
    the reduction of the motor is already taken care of in the ARR value: ARR=CPR*REDUCTION -> 4x1000xreduction
    4x is caused by the timer mode (TI1 and TI2)
    */
    uint16_t counter; 
    float displacement1, displacement2;
    // float v_est, a_est; /* used to hold temporarily the estimations of speed and acceleration */
    TIM_HandleTypeDef *htim1, *htim2;

    htim1 = manip->htim_encoder1; /* pointer to the timer struct that decodes the first encoder output */
    htim2 = manip->htim_encoder2; /* pointer to the timer struct that decodes the first encoder output */

    /* first encoder */
    counter = (htim1->Instance->CNT);
    if(counter >= htim1->Instance->ARR){
        counter = (htim1->Instance->ARR-1) - (counter % 1<<16); /* handle underflow */
        htim1->Instance->CNT = counter; /* correct cnt value */
    }

    displacement1 = (float) (2*M_PI*counter/(htim1->Instance->ARR)-offset1);

    /* second encoder */
    counter = (htim2->Instance->CNT);
    if(counter >= htim2->Instance->ARR){
        counter = (htim2->Instance->ARR-1) - (counter % 1<<16); /* handle underflow */
        htim2->Instance->CNT = counter;  /* correct cnt value */
    }
    displacement2 = (float) (2*M_PI) - (2*M_PI*counter/(htim2->Instance->ARR)-offset2); /* the motor is upside down */

    // SECTION DEBUG
    // rbpush(&timestamps, (float) HAL_GetTick()/1000.0);
    // !SECTION DEBUG

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

    // SECTION DEBUG
    // disp1 = displacement1;
    // disp2 = displacement2;
    /* !SECTION DEBUG

    /* DIR bit: pag 287 of https://www.st.com/resource/en/reference_manual/rm0383-stm32f411xce-advanced-armbased-32bit-mcus-stmicroelectronics.pdf#page=287 */
    /* dir: 0 = counterclockwise, 1 = clockwise */
    /* the 5th bit of the CR1 register is the DIR bit */
    /*
    uint8_t dir1 = (uint8_t) (htim1->Instance->CR1 >> 4) & 1;
    uint8_t dir2 = (uint8_t) (htim2->Instance->CR1 >> 4) & 1;
    */
    // SECTION DEBUG
    rbpush(&manip->q0_actual, displacement1);
    rbpush(&manip->q1_actual, displacement2);
    /*
    float last_displacement;
    rblast(&manip->q0_actual, &last_displacement);
    if(last_displacement-displacement1 != 0){
        rbpush(&manip->q0_actual, displacement1);
    }
    rblast(&manip->q1_actual, &last_displacement);
	if(last_displacement-displacement2 != 0){
		rbpush(&manip->q1_actual, displacement2);
	}
    // !SECTION DEBUG
    */
    /*
    rbpush(&manip->q0_actual, displacement1);
    rbpush(&manip->q1_actual, displacement2);
    */
    /* TODO: do logging of data */

    /* speed and acceleration estimations for both motors*/
	/*
    speed_estimation(&manip->q0_actual, &manip->dq0_actual, &v_est, &a_est);
    rbpush(&manip->dq0_actual, v_est);
    rbpush(&manip->ddq0_actual, a_est);

    speed_estimation(&manip->q1_actual, &manip->dq1_actual, &v_est, &a_est);
    rbpush(&manip->dq1_actual, v_est);
    rbpush(&manip->ddq1_actual, a_est);
    */

    // SECTION DEBUG
	/*
	return;
    float delta_t, delta_q, delta_dq, t1, t2, q1, q2, dq1, dq2;
    rbget(&timestamps, timestamps.length-1, &t2);
    rbget(&timestamps, timestamps.length-2, &t1);
    rbget(&manip->q0_actual, manip->q0_actual.length-1, &q2);
    rbget(&manip->q0_actual, manip->q0_actual.length-2, &q1);
    rbget(&manip->dq0_actual, manip->dq0_actual.length-1, &dq2);
    rbget(&manip->dq0_actual, manip->dq0_actual.length-2, &dq1);
    delta_t = (t2-t1) == 0 ? t2-0 : t2-t1;
    delta_q = q2-q1;
    delta_dq = dq2-dq1;
    v_est = delta_q/delta_t;
    a_est = delta_dq/delta_t;

    rbpush(&manip->dq0_actual, v_est);
    rbpush(&manip->ddq0_actual, a_est);

	rbget(&manip->q1_actual, manip->q1_actual.length-1, &q2);
	rbget(&manip->q1_actual, manip->q1_actual.length-2, &q1);
	rbget(&manip->dq1_actual, manip->dq1_actual.length-1, &dq2);
	rbget(&manip->dq1_actual, manip->dq1_actual.length-2, &dq1);
	delta_q = q2-q1;
	delta_dq = dq2-dq1;
	v_est = delta_q/delta_t;
	a_est = delta_dq/delta_t;

	rbpush(&manip->dq1_actual, v_est);
	rbpush(&manip->ddq1_actual, a_est);
    // !SECTION DEBUG
	*/
}

void update_speeds(man_t *manip){
	float v_est, a_est;
	speed_estimation(&manip->q0_actual, &manip->dq0_actual,&manip->ddq0_actual, reduction1, &v_est, &a_est);
	//disp1 = v_est;
	rbpush(&manip->dq0_actual, v_est);
	rbpush(&manip->ddq0_actual, a_est);

	speed_estimation(&manip->q1_actual, &manip->dq1_actual,&manip->ddq1_actual, reduction2, &v_est, &a_est);
	//disp2 = v_est;
	rbpush(&manip->dq1_actual, v_est);
	rbpush(&manip->ddq1_actual, a_est);
}

void apply_velocity_input(TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2, float *u){
    /* T_C = steps*clock_period */
    int8_t dir1, dir2;
    uint32_t f;
    int32_t stepdir;
    uint32_t steps, ARR, CCR;
    uint32_t prescaler1, prescaler2;
    float clock_period;

   dir1 = u[0] < 0 ?  GPIO_PIN_SET : GPIO_PIN_RESET;
   // dir1 = 1; // DEBUG
   HAL_GPIO_WritePin(DIR_1_GPIO_Port, DIR_1_Pin, dir1);

   dir2 = u[1] > 0 ?  GPIO_PIN_SET : GPIO_PIN_RESET;
   // dir2 = 1; // DEBUG
   HAL_GPIO_WritePin(DIR_2_GPIO_Port, DIR_2_Pin, dir2);


	prescaler1= (uint16_t)  8400;//12000 ;//8400;
	f=HAL_RCC_GetPCLK1Freq()*2;
	ARR= ABS(u[0]) < 0.001 ? 0:(uint32_t)  (RESOLUTION*f/(ABS(u[0])*reduction1*16*prescaler1));
	CCR= ARR /2;
    __HAL_TIM_SET_PRESCALER(htim1, prescaler1);//2625
    __HAL_TIM_SET_AUTORELOAD(htim1, ARR);
   	__HAL_TIM_SET_COMPARE(htim1, TIM_CHANNEL_1, CCR);
   	htim1->Instance->EGR = TIM_EGR_UG;

   	prescaler2= (uint16_t)  8400;//12000 ;//8400;
   	f=HAL_RCC_GetPCLK1Freq()*2;
   	ARR=  ABS(u[1]) < 0.001 ? 0:(uint32_t)  (RESOLUTION*f/(ABS(u[1])*reduction2*16*prescaler2));
   	CCR= ARR /2;
   	__HAL_TIM_SET_PRESCALER(htim2, prescaler2);//2625
   	__HAL_TIM_SET_AUTORELOAD(htim2, ARR);
   	__HAL_TIM_SET_COMPARE(htim2, TIM_CHANNEL_1, CCR);
   	htim2->Instance->EGR = TIM_EGR_UG;

    return;



}


void apply_position_input(TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2, float *u , float *pos){
    /* T_C = steps*clock_period */
    int8_t dir1, dir2;
    uint32_t f;
    int32_t stepdir;
    uint32_t steps, ARR, CCR;
    uint16_t prescaler1, prescaler2;
    float duty;
    float u0,u1;
    float tc0,tc1;

    /*
    ARR = (uint32_t) 65000;
    CCR = (uint32_t) ARR/2;
    __HAL_TIM_SET_AUTORELOAD(htim1, ARR);
    __HAL_TIM_SET_COMPARE(htim1, TIM_CHANNEL_1, CCR);
    htim1->Instance->EGR = TIM_EGR_UG;
    */
    // rad2stepdir(u[0], RESOLUTION, (float) 1/T_C, &steps, &dir);

    if (ABS(u[0]-pos[0])<0.01){
    	tc0= 1000000;
    }else{
    tc0 = sqrt(2*M_PI*ABS(u[0]-pos[0])/1.05);
    }

    if (ABS(u[1]-pos[1])<0.01){
        	tc1= 1000000;
        }else{
        tc1 = sqrt(2*M_PI*ABS(u[1]-pos[1])/0.85);   //1.5 ----> come se fosse un jerk
        }


    u0=(u[0]-pos[0])/tc0;
    u1=(u[1]-pos[1])/tc1;

    //printf("%d ;%f ; %f \n",count ,u1, tc1 );
    //fflush(stdout);

        dir1 = u0 < 0 ?  GPIO_PIN_SET : GPIO_PIN_RESET;
       // dir1 = 1; // DEBUG
       HAL_GPIO_WritePin(DIR_1_GPIO_Port, DIR_1_Pin, dir1);

       dir2 = u1 > 0 ?  GPIO_PIN_SET : GPIO_PIN_RESET;
       // dir2 = 1; // DEBUG
       HAL_GPIO_WritePin(DIR_2_GPIO_Port, DIR_2_Pin, dir2);




		   prescaler1= (uint16_t) 8400;//12000 ;//8400;
		   f=HAL_RCC_GetPCLK1Freq()*2;
		   ARR= ABS(u0) < 0.01 ? 0:(uint32_t)  (RESOLUTION*f/(ABS(u0)*reduction1*16*prescaler1));
		   CCR= ARR /2;
		   __HAL_TIM_SET_PRESCALER(htim1, prescaler1);//2625
			__HAL_TIM_SET_AUTORELOAD(htim1, ARR);
		__HAL_TIM_SET_COMPARE(htim1, TIM_CHANNEL_1, CCR);
		htim1->Instance->EGR = TIM_EGR_UG;

   	prescaler2= (uint16_t) 8400;//12000 ;//8400;
   	f=HAL_RCC_GetPCLK1Freq()*2;
   	ARR=  ABS(u1) < 0.01 ? 0:(uint32_t)  (RESOLUTION*f/(ABS(u1)*reduction2*16*prescaler2));
   	CCR= ARR /2;
   	__HAL_TIM_SET_PRESCALER(htim2, prescaler2);//2625
   	__HAL_TIM_SET_AUTORELOAD(htim2, ARR);
   	__HAL_TIM_SET_COMPARE(htim2, TIM_CHANNEL_1, CCR);
   	htim2->Instance->EGR = TIM_EGR_UG;



    return

    dir1 = u[0] < 0 ?  GPIO_PIN_SET : GPIO_PIN_RESET;
    // dir1 = 1; // DEBUG
    HAL_GPIO_WritePin(DIR_1_GPIO_Port, DIR_1_Pin, dir1);

    dir2 = u[1] > 0 ?  GPIO_PIN_SET : GPIO_PIN_RESET;
    // dir2 = 1; // DEBUG
    HAL_GPIO_WritePin(DIR_2_GPIO_Port, DIR_2_Pin, dir2);


    prescaler1= (uint16_t)  8400;//12000 ;//8400;
    f=HAL_RCC_GetPCLK1Freq()*2;
    ARR= ABS(u[0]) < 0.001 ? 0:(uint32_t)  (RESOLUTION*f/(ABS(u[0])*reduction1*16*prescaler1));
    CCR= ARR /2;
    __HAL_TIM_SET_PRESCALER(htim1, prescaler1);//2625
   	__HAL_TIM_SET_AUTORELOAD(htim1, ARR);
	__HAL_TIM_SET_COMPARE(htim1, TIM_CHANNEL_1, CCR);
	htim1->Instance->EGR = TIM_EGR_UG;

	prescaler2= (uint16_t)  8400;//12000 ;//8400;
	f=HAL_RCC_GetPCLK1Freq()*2;
	ARR=  ABS(u[1]) < 0.001 ? 0:(uint32_t)  (RESOLUTION*f/(ABS(u[1])*reduction2*16*prescaler2));
	CCR= ARR /2;
	__HAL_TIM_SET_PRESCALER(htim2, prescaler2);//2625
	__HAL_TIM_SET_AUTORELOAD(htim2, ARR);
	__HAL_TIM_SET_COMPARE(htim2, TIM_CHANNEL_1, CCR);
	htim2->Instance->EGR = TIM_EGR_UG;

    return;



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
    float q;
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


void setup_encoders(TIM_HandleTypeDef *htim){
	const uint32_t clock_freq = 84000000;
	uint16_t ARR;
	ARR = (T_S*clock_freq)/PRESCALER_ENCODER;
	__HAL_TIM_SET_PRESCALER(htim, PRESCALER_ENCODER);
	__HAL_TIM_SET_AUTORELOAD(htim, ARR);
	htim->Instance->EGR = TIM_EGR_UG;
	HAL_TIM_Base_Start_IT(htim); /* start the sampling timer */
}




void PID_controller_position(man_t *manip, pid_controller_t *pid1,pid_controller_t *pid2, float *u , float setpoint){

	float set_point1,set_point2,measure1, measure2,u0,u1,tc0,tc1;

	rbpeek(&manip->q0,&set_point1);
	rbpeek(&manip->q1,&set_point2);

	//set_point1 = 0;
	//set_point2 = setpoint;

	dq_actual0=set_point1;
	//ddq_actual1=set_point2;


	rblast(&manip->q0_actual,&measure1);
	rblast(&manip->q1_actual,&measure2);

	disp1=measure1;
	disp2=measure2;




	PID_update(pid1,set_point1, measure1,T_C);
	PID_update(pid2,set_point2, measure2,T_C);

	ddq_actual0=pid1->out;
	ddq_actual1=pid2->out;


	//printf("%d ;%f ; %f ; %f \n",count ,setpoint ,measure2 ,pid1->out );

	*u=pid1->out;
	*(u+1)=pid2->out;

    if (ABS(u[0]-measure1)<0.01){
    	tc0= 1000000;
    }else{
    tc0 = sqrt(2*M_PI*ABS(u[0]-measure1)/1.05);
    }

    if (ABS(u[1]- measure2)<0.01){
        	tc1= 1000000;
        }else{
        tc1 = sqrt(2*M_PI*ABS(u[1]-measure2)/0.85);   //1.5 ----> come se fosse un jerk
        }


    u0=(u[0]-measure1)/tc0;
    u1=(u[1]-measure2)/tc1;

    *u=u0;
    *(u+1)=u1;


	//*(u)=0;


}



void PID_controller_velocity(man_t *manip, pid_controller_t *pid1,pid_controller_t *pid2, float *u , float setpoint){

	float set_point1,set_point2,measure1, measure2;

	rbpeek(&manip->dq0,&set_point1);
	rbpeek(&manip->dq1,&set_point2);

	//set_point1 = 0;
	//set_point2 = setpoint;

	dq_actual0=set_point1;
	//ddq_actual1=set_point2;



	rblast(&manip->dq0_actual,&measure1);
	rblast(&manip->dq1_actual,&measure2);

	rblast(&manip->q0_actual,&disp1);
	disp2=measure2;

	PID_update(pid1,set_point1, measure1,T_C);
	PID_update(pid2,set_point2, measure2,T_C);

	ddq_actual0=pid1->out;
	ddq_actual1=pid2->out;


	printf("%d ;%f ; %f ; %f \n",count ,setpoint ,measure2 ,pid1->out );

	*u=pid1->out;
	*(u+1)=pid2->out;
	//*(u)=0;


}



void homing(man_t *manip,TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2, pid_controller_t *pid1,pid_controller_t *pid2){

    float u[2]={0, 0};
    float pos[2]={0, 0};
    float pos_real[2]={-2.11350, 0};

    offset1=0;
    offset2=0;

	/*apply velocity input*/
	while(!limit_switch1){


	 rbpush(&manip->dq0,-0.5);
	 update_speeds(manip);
	 PID_controller_velocity( manip, pid1, pid2, u ,0);
	 apply_velocity_input(htim1, htim2, u);

	 HAL_Delay((uint32_t) (T_C*1000));



	}

	limit_switch1=0;

	u[0]=0;
	u[1]=0;
	apply_velocity_input(htim1, htim2, u);

    offset1-=pos_real[0];
    rblast(&manip->q0_actual,&pos[0]);
    rblast(&manip->q1_actual,&pos[1]);

	while(1){

	if((ABS(pos[0])> 0.01) || (ABS(pos[1])> 0.01)){

		PID_controller_position( manip, pid1, pid2, u ,0);
		apply_velocity_input(htim1, htim2,  u);
		rblast(&manip->q0_actual,&pos[0]);
		rblast(&manip->q1_actual,&pos[1]);
		HAL_Delay((uint32_t) (T_C*1000));
	 }else{
		 break;
	 }
	}

}















