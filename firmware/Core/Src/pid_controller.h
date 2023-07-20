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

} pid_controller_t;

int PID_init(pid_controller_t *pid, float Kp, float Ti, float Td, float N, int Controller_type);
int set_limit(pid_controller_t *pid, float lim_out_min, float lim_out_max, float lim_integ_min,float lim_integ_max );

int PID_update(pid_controller_t *pid, float set_point , float measure, float T_C);


#endif /* SRC_PID_CONTROLLER_H_ */
