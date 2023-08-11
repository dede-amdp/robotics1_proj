#ifndef SRC_PID_CONTROLLER_H_
#define SRC_PID_CONTROLLER_H_

#include<stdint.h>

/*
#@
@name: struct pid_controller_t
@brief: struct that holds all the data of a PID controller.
@inputs: 
- int type: whether the controller will be PI or PID;
- float Kp: proportional gain of the controller;
- float Ti: integral time constant of the controller;
- float Td: derivative time constant of the controller;
- float N: discrete derivative factor;
- float prev_error: previous value of the error;
- float prev_meas: previous value of the output of the system;
- float integrator: integrator value;
- float derivative: derivative value;
- float lim_out_min: minimum limit value for the output of the controller;
- float lim_out_max: maximum limit value for the output of the controller;
- float lim_integ_min: minimum limit value for the integrator value of the controller;
- float lim_integ_max: maximum limit value for the integrator value of the controller;
- float out: output value of the controller;
@#
*/
typedef struct{

		int type;

		/* GAINS */
		float Kp;
		float Ti;
		float Td;
		float N;

		/* MEMORY */
		float prev_err;
		float prev_meas;
		float integrator;
		float derivative;

		/* LIMITS */
		float lim_out_min;
		float lim_out_max;

		float lim_integ_min;
		float lim_integ_max;

		/* OUTPUT */
		float out;

} pid_controller_t; /* size: 14*32 bit = 56 B*/

int PID_init(pid_controller_t *pid, float Kp, float Ti, float Td, float N, int Controller_type);
int set_limit(pid_controller_t *pid, float lim_out_min, float lim_out_max, float lim_integ_min,float lim_integ_max );
int PID_update(pid_controller_t *pid, float set_point , float measure, float T_C);


#endif /* SRC_PID_CONTROLLER_H_ */
