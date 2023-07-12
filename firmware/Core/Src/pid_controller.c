/*
 * pid_controller.c
 *
 *  Created on: Jul 11, 2023
 *      Author: vincenzo
 */

#include "pid_controller.h"

/*
#@
@name: PID_init
@brief: initialize the values of the PID controller;
@inputs:
- pid_controller *pid: struct data where all the data will be stored ;
- float Kp
- float Ti;
- float Td;
- float n

*/

int PID_init(pid_controller *pid, float KP,float TI, float TD, float N){

	pid->Kp= KP;
	pid->Ti=TI;
	pid->Td=TD;
	pid->N=N;

	pid->integrator=0.f;
	pid->derivative=0.f;
	pid->prev_err=0.f;

	pid->out=0.f;


	/*NB the limit must be setted using the proper method */
	pid->lim_out_min=0.f;
	pid->lim_out_max=0.f;

	pid->lim_integ_min=0.f;
	pid->lim_integ_max=0.f;




	return 0;
}
/*
#@
@name: set_limit
@brief: initialize the values the maximum and the minimum allowed for the output and the integrator;
@inputs:
- pid_controller *pid: struct data where all the data will be stored ;
- float lim_out_min;
- float lim_out_max;
- float lim_integ_min;
- float lim_integ_max ;

*/
int set_limit(pid_controller *pid, float lim_out_min, float lim_out_max, float lim_integ_min,float lim_integ_max ){

	pid->lim_out_min=lim_out_min;
	pid->lim_out_max=lim_out_max;

	pid->lim_integ_min=lim_integ_min;
	pid->lim_integ_max=lim_integ_max;


	return 0;
}

/*
#@
@name: PID_update
@brief: update the value of the controller calculated in relation of the error value ;
@inputs:
- pid_controller *pid: struct data where all the data will be stored ;
- float set_point;
- float measure;
- float T_C;
@outputs:
pid->out: contain the value calculated by the controller

*/

int PID_update(pid_controller *pid, float set_point , float measure, float T_C){

	float u=0.f;
	float error=0.f;
	float proportional=0.f;
	float alpha= pid->Td/T_C;


    /* calculate the error*/
	error=set_point-measure;



	/* proportional contribute*/

	proportional= pid->Kp*error;

	/*integral contribute*/

	pid->integrator=(pid->integrator+(pid->Kp/pid->Ti)*0.5f*T_C)*(error-pid->prev_err);

	/* tentative of anti wind-up*/

	if(pid->integrator > pid->lim_integ_max){
		pid->integrator=pid->lim_integ_max;
	}else if(pid->integrator < pid->lim_integ_min)
	{
		pid->integrator=pid->lim_integ_min;
	}


	/*derivative contribute*/
	pid->derivative= (2*(pid->Kp)*alpha*error - pid->derivative*(1-(2*alpha)/pid->N))/(1+(2*alpha)/pid->N);

	/* output  */

	u=proportional+pid->integrator+pid->derivative;


	if(u>pid->lim_out_max)
	{
		pid->out=pid->lim_out_max;
	}else if(u<pid->lim_out_min){

		pid->out=pid->lim_out_min;

	}else{
		pid->out=u;
	}


	pid->prev_err=error;
	pid->prev_meas=measure;

	return 0;
}









