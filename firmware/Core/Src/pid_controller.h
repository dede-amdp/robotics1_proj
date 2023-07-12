/*
 * pid_controller.h
 *
 *  Created on: Jul 11, 2023
 *      Author: vincenzo
 */

#ifndef SRC_PID_CONTROLLER_H_
#define SRC_PID_CONTROLLER_H_

#include<stdint.h>

typedef struct{

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

} pid_controller;

int PID_init(pid_controller *pid, float Kp, float Ti, float Td, float N);
int set_limit(pid_controller *pid, float lim_out_min, float lim_out_max, float lim_integ_min,float lim_integ_max );

int PID_update(pid_controller *pid, float set_point , float meausre, float T_C);


#endif /* SRC_PID_CONTROLLER_H_ */
