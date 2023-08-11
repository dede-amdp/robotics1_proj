# Custom


## struct manipulator aka manipulator_t
> struct representation of a 2Dof planar manipulator

it holds RBUF_SZ values of the reference and actual position, speed and acceleration of each motor via multiple ring/circular buffer;
### Inputs
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
 - TIM_HandleTypeDef \*htim_encoder1: pointer to the timer used to read the first encoder;
 - TIM_HandleTypeDef \*htim_encoder2: pointer to the timer used to read the second encoder;
 - TIM_HandleTypeDef \*htim_motor1: pointer to the timer used to apply the PWM signal to the first motor;
 - TIM_HandleTypeDef \*htim_motor2: pointer to the timer used to apply the PWM signal to the second motor;

 


---

generated with [EasyGen](http://easygen.altervista.org/) - [On Github](https://github.com/dede-amdp/easygen).