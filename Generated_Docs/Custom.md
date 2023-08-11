# Custom


## init_man
> initializes the members of the man_t struct

### Inputs
- man_t \*manip: man_t obj. to initialize;
 - TIM_HandleTypeDef \*htim1: pointer to the timer used to decode the output of the first encoder;
 - TIM_HandleTypeDef \*htim2: pointer to the timer used to decode the output of the second encode;

### Outputs
- void;
 



## dot
> computes the dot product between two matrices, A and B, represented in vector form;

### Inputs
- float \*A: pointer to a vector of floats of size nA*mA, which represents the first nAxmA matrix;
 - uint8_t nA: number of rows of matrix A;
 - uint8_t mA: number of columns of matrix A;
 - float \*B: pointer to a vector of floats of size nB*mB, which represents the second nBxmB matrix;
 - uint8_t nB: number of rows of matrix B;
 - uint8_t mB: number of columns of matrix B;
 - float \*C: pointer to a vector of floats of size nA*mB, which represents the resulting nAxmB matrix -> if the operation cannot be done, it will be NULL;

### Outputs
- uint8_t: 0 or 1 that shows whether the operation completed successfully or not.
 



## inv2x2
> Inverts (if possible) a 2x2 matrix

### Inputs
- float \*M: pointer to the matrix to invert;
 - float \*invM: pointer to the inverted matrix (NULL if inversion is not possible);

### Outputs
- uint8_t: shows whether the inversion was completed or not
 



## sum
> sums two matrices (with the same size)

### Inputs
- float \*A: pointer to the first matrix to sum;
 - float \*B: pointer to the second matrix to sum;
 - uint8_t n: number of elements in the matrix;
 - float \*C: pointer to the resulting matrix;

### Outputs
- void;
 



## diff
> subtracts two matrices (with the same size)

### Inputs
- float \*A: pointer to the first matrix to subtract;
 - float \*B: pointer to the second matrix to subtract;
 - uint8_t n: number of elements in the matrix;
 - float \*C: pointer to the resulting matrix;

### Outputs
- void;
 



## det
> computes the determinant of a n x n matrix

this particular implementation of the determinant algorithm does not use recursion: is uses the property of elementary row operations that do not change the 
 determinant of the matrix to find a upper triangular matrix with the same determinant of the original matrix whose determinant is to be computed.
 The determinant of a triangular matrix is the multiplication of the elements on its main diagonal: by finding a triangular matrix B with the same determinant of a given square matrix A, 
 the determinant of matrix A can be found by computing the determinant of matrix B.
### Inputs
- float \*M: pointer to the determinant whose determinant is to be computed. IMPORTANT: the values in this matrix will be changed: ensure to keep the original data safe by passing a copy of the original matrix;
 - uint8_t n: order of the matrix;
 - float \*d: pointer to the variable that will hold the resulting determinant;

### Outputs
- void;
 



## inv
> computes the inverse of a square matrix M;

the method requires pointer to temporary variables that will hold data used for the computation;
### Inputs
- float \*M: pointer to the matrix whose inverse should be computed;
 - float \*adjM: pointer to the temporary variable that will hold the adjugate matrix >> should be a nxn array just like M;
 - float \*subM: pointer to the temporary variable that will hold the submatrices used for the computation of the adjugate matrix >> should be a (n-1)x(n-1) array;
 - float \*trM: pointer to the temporary variable that will hold the transposed adjugate matrix >> should be a nxn array just like M;
 - uint8_t n: order of the matrices;
 - float \*invM: pointer to the temporary variable that will hold the inverse matrix of M;

### Outputs
- uint8_t: it is a boolean value that shows whether the inversion is completed successfully or not.
 



## adj
> computes the adjugate matrix of M

the method requires temporary variables to hold useful data for the computation;
### Inputs
- float \*M: pointer to the **square** matrix of which the adjugate should be computed;
 - float \*subM: pointer to the temporary variable that will hold the submatrices used for the computation >> should be a (n-1)x(n-1) array;
 - uint8_t n: order of the matrix;
 - float \*adjM: pointer to the variable that will hold the resulting adjugate matrix;

### Outputs
- void;
 



## tr
> transposes a matrix

### Inputs
- float \*M: pointer to the matrix to transpose;
 - uint8_t n: number of rows;
 - uint8_t m: number of columns;
 - float \*trM: pointer to the variable that will hold the transposed matrix;

### Outputs
- void;
 



## pseudo_inv
> computes the pseudo inverse of matrix M: (M^T*M)^(-1)*M^T

the method requires temporary variables to hold useful data for the computation;
### Inputs
- float \*M: pointer to the matrix to pseudo-invert;
 - float \*trM: pointer to the variable that will hold the transposed matrix used in the pseudo-inversion;
 - float \*tempM: pointer to the variable that will hold the temporary transposition during the inversion;
 - float \*adjM: pointer to the variable that will hold the adjugate matrix used during the inversion;
 - float \*subM: pointer to the variable that will hold the submatrix used during the computation of the adjugate;
 - float \*invM: pointer to the variable that will hold the inverted matrix (A^T*A)^(-1);
 - float \*dotM: pointer to the variable that will hold the dot product between A^T and A; 
 - uint8_t n: order of the matrix to invert;
 - float \*psinvM: pointer to the variable that will hold the pseudo-inverse;

### Outputs
- void;
 



## B_calc
> computes the inertia matrix of the manipulator

the inertia matrix B of the manipulator depends on its current configuration: 
 it is computed analytically with the dynamic model found via the  Matlab Peter Corke toolbox
### Inputs
- man_t \*manip: pointer to the manipulator struct that olds the reference and actual values of the position, speed and acceleration of the motors;

### Outputs
- void;
 



## C_calc
> computes the coriolis matrix of the manipulator

the coriolis matrix C of the manipulator depends on its current configuration and joint speed: 
 it is computed analytically with the dynamic model found via the  Matlab Peter Corke toolbox
### Inputs
- man_t \*manip: pointer to the manipulator struct that olds the reference and actual values of the position, speed and acceleration of the motors;

### Outputs
- void;
 



## controller
> implements the control law

it implements the "Inverse Dynamics" control in the joint space.
### Inputs
- man_t \*manip: pointer to the manipulator struct that holds all the current motors' position, speed and acceleration and their reference values;
 - float \*u: float[2] vector pointer that holds the control input to apply to motors (speed control);

### Outputs
- void;
 



## rad2stepdir
> converts velocity (rad/s) to step and direction (step dir);

### Inputs
- float dq: velocity (rad/s);
 - float resolution: resolution of the motor (how many radians is a single step?) -> expressed in radians;
 - float frequency: reciprocal of delta_t -> time period in which the change of position happens;
 - uint8_t \*steps: pointer to the variable that will hold the number of steps;
 - int8_t \*dir: pointer to the variable that will hold the direction (+1 means counterclockwise, -1 means clockwise);

### Outputs
- void;
 



## speed_estimation
> computes the speed and acceleration estimations from a fixed number of previous motor positions

### Inputs
- ringbuffer_t \*q_actual: pointer to the ringbuffer struct that holds all the data relative to the motor position;
 - float \*v_est: pointer to the variable that will hold the speed estimation;
 - float \*a_est: pointer to the variable that will hold the acceleration estimation;

### Outputs
- void;
 



## init_rate
> initializes the rate struct

### Inputs
- rate_t \*rate: pointer to the rate struct to initialize;
 - uint32_t ms: number of millisecond that define the rate;

### Outputs
- void;
 



## rate_sleep
> stops the process to maintain a fixed framerate (useful in while loops to implement fixed time control loops)

### Inputs
- rate_t \*rate: pointer to the rate struct;

### Outputs
- void;
 



## read_encoders
> reads data from both the encoders of the 2Dofs planar manipulator

the method uses two timers to decode the signals coming from both the encoders and memorizes the measured positions of the motors, 
 taking into account the reduction ratio of each motor by means of the ARR registers of the timers (ARR=CPR*REDUCTION). 
 The method also estimates the speed and accelerations by using the timestamp method.
### Inputs
- man_t \*manip: pointer to the manipulator struct that holds both the desired and actual motor positions, speeds and accelerations;

### Outputs
outputs
 



## update_speeds
> update the estimated speed data from both the encoders of the 2Dofs planar manipulator

### Inputs
- man_t \*manip: pointer to the manipulator struct that holds both the desired and actual motor positions, speeds and accelerations;

### Outputs
outputs
 



## apply_velocity_input
> applies the velocity inputs to the motors

it uses a timer to send a pulse width modulation signal whose frequency regulates the rotation speed of the motors.

 The formulae used to set the ARR and CCR registers:
 $$ARR = \frac{Res\cdot freq_{clock}}{v\cdot reduction\cdot microstepping\cdot prescaler}$$
 $$CCR = ARR/2$$
 with Res being the motor step resolution (in radians), v is the velocity, reduction is the reduction ratio and prescaler is the prescaler value (which is fixed).
### Inputs
- TIM_HandleTypeDef \*htim1: pointer to the handler of the timer that sends the PWM signal to the first motor;
 - TIM_HandleTypeDef \*htim2: pointer to the handler of the timer that sends the PWM signal to the second motor;
 - float \*u: array of two floats containing the velocity inputs of the motors;

### Outputs
- void;
 



## apply_position_input
> applies the position inputs to the motors

it uses a timer to send a pulse width modulation signal whose frequency regulates the rotation speed of the motors.

 The position input is converted into a velocity input (using the relationship between max acceleration and time duration of a cycloidal trajectory):
 $$\Delta t = \sqrt{2\pi\cdot |u-pos|/factor}$$
 $$v = \frac{(u-pos)}{\Delta t}$$
 where the factor regulates how much the distance to move influences the time duration of the movement.
 The formulae used to set the ARR and CCR registers:
 $$ARR = \frac{Res\cdot freq_{clock}}{v\cdot reduction\cdot microstepping\cdot prescaler}$$
 $$CCR = ARR/2$$
 with Res being the motor step resolution (in radians), v is the velocity, reduction is the reduction ratio and prescaler is the prescaler value (which is fixed).
### Inputs
- TIM_HandleTypeDef \*htim1: pointer to the handler of the timer that generates the PWM signal for the first motor;
 - TIM_HandleTypeDef \*htim2: pointer to the handler of the timer that generates the PWM signal for the second motor;
 - float \*u: pointer to a float array of size two containing the position input;
 - float \*pos: pointer to a float array of size two containing the actual position of the manipulator;

### Outputs
- void;
 



## start_timers
> starts all the timers needed for the control

### Inputs
- TIM_HandleTypeDef \*htim1: pointer to the first timer handler;
 - TIM_HandleTypeDef \*htim2: pointer to the second timer handler;
 - TIM_HandleTypeDef \*htim3: pointer to the third timer handler;
 - TIM_HandleTypeDef \*htim4: pointer to the fourth timer handler;

### Outputs
- void;
 



## stop_timers
> stops all the timers needed for the control

### Inputs
- TIM_HandleTypeDef \*htim1: pointer to the first timer handler;
 - TIM_HandleTypeDef \*htim2: pointer to the second timer handler;
 - TIM_HandleTypeDef \*htim3: pointer to the third timer handler;
 - TIM_HandleTypeDef \*htim4: pointer to the fourth timer handler;

### Outputs
- void;
 



## log_data
> Send all the manipulator data via serial connection with a message

### Inputs
- UART_HandleTypeDef \*huart: pointer to the uart handler;
 - man_t \*manip: pointer to the man_t structure that holds all the manipulator data;

### Outputs
- void;
 



## setup_encoders
> performs the necessary operations for the setup of the timer that will manage the econder sampling

### Inputs
- TIM_HandleTypeDef \*htim: pointer to the sampling timer handler;

### Outputs
- void;
 



## PID_controller_velocity
> Calculate the velocity control output for the motors.

The motor driver (DRV8825) take a STEP/DIR input, that represent a velocity command.
 To accomplish the translation from position input to velocity input, first of all the current position is read, after that
 the distance between the setpoint position and the current position is calculated, then the needed time for motion is calculated using the cycloidal trajectory.
 The velocity output is given by the ratio between distance and time. To avoid vibration a threshold is set.
### Inputs
- man_t \*manip: pointer to the manipulator struct;
 - pid_controller_t \*pid1 : pointer to the PI velocity controller of the first joint;
 - pid_controller_t \*pid2 : pointer to the PI velocity controller of the second joint;
 - float \*u: pointer to the velocity output;

### Outputs
- void;
 



## PID_controller_velocity
> Calculate the velocity control  output for the links.

### Inputs
- man_t \*manip: pointer to the manipulator struct;
 - pid_controller_t \*pid1 : pointer to the PI velocity controller of the first joint
 - pid_controller_t \*pid2 : pointer to the PI velocity controller of the second joint
 - float u: velocity output

### Outputs
- void;
 



## homing
> Start the homing sequence when the HOM command is recived from the serial

In the serial callback we don't call directly the homing method because the if this happen we are unable to sense other the interrupt when the homing code is executed,
 to get around this drawback, the flag variable homing_triggered is set high and than in the main loop the homing sequence starts.
 The flag variables is_home1 and is_home2 are used to avoid changing the offset errors measured during the homing procedure.
### Inputs
- man_t \*manip: pointer to the manipulator struct;
 - TIM_HandleTypeDef \*htim1: First joint PWM out timer;
 - TIM_HandleTypeDef \*htim2: Second joint PWM out timer;
 - pid_controller_t \*pid_v1 : pointer to the PI velocity controller of the first joint;
 - pid_controller_t \*pid_v2 : pointer to the PI velocity controller of the second joint;
 - pid_controller_t \*pid_p1 : pointer to the PID  position controller of the first joint;
 - pid_controller_t \*pid_p2 : pointer to the PID  position controller of the second joint;

### Outputs
- void;
 


---

generated with [EasyGen](http://easygen.altervista.org/) - [On Github](https://github.com/dede-amdp/easygen).