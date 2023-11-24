#include "pid_controller.h"

/*
#@
@name: PID_init
@brief: initialize the values of the PID controller;
@inputs:
- pid_controller \*pid: struct data where all the PID data is be stored;
- float KP: proportional constant;
- float TI: integration time constant;
- float TD: derivation time constant;
- float N: discrete derivation factor;
- int controller_type: whether the controller will be a P, PI or PID controller;
@outputs:
- void;
@#
*/
int PID_init(pid_controller_t *pid, float KP,float TI, float TD, float N, int Controller_type){
	pid->type=Controller_type;
	pid->Kp= KP;
	pid->Ti=TI;
	pid->Td=TD;
	pid->N=N;

	pid->integrator=0.f;
	pid->derivative=0.f;
	pid->prev_err=0.f;

	pid->out=0.f;

	/*NB the limit must be set using the proper method */
	pid->lim_out_min=0.f;
	pid->lim_out_max=0.f;

	pid->lim_integ_min=0.f;
	pid->lim_integ_max=0.f;

	return 0;
}
/*
#@
@name: set_limit
@brief: initialize the values of the maximum and the minimum allowed for the output and for the integrator value;
@inputs:
- pid_controller \*pid: struct data where all the data will be stored ;
- float lim_out_min: minimum limit value for the output;
- float lim_out_max: maximum limit value for the output;
- float lim_integ_min: minimum limit value for the integrator value;
- float lim_integ_max: maximum limit value for the integrator value;
@outputs:
- void;
@#
*/
int set_limit(pid_controller_t *pid, float lim_out_min, float lim_out_max, float lim_integ_min,float lim_integ_max ){
	pid->lim_out_min=lim_out_min;
	pid->lim_out_max=lim_out_max;

	pid->lim_integ_min=lim_integ_min;
	pid->lim_integ_max=lim_integ_max;
	return 0;
}

/*
#@
@name: PID_update
@brief: update the value of the controller calculated in relation to the error value.
@notes: the output value of the PID controller is memorized in the `out` member of the PID struct.
@inputs:
- pid_controller \*pid: pointer to the struct where all the PID data is be stored;
- float set_point: value of the setpoint used to compute the error;
- float measure: measured value of the output of the controlled system;
- float T_C: sampling time;
@outputs:
- void;
@#
*/

int PID_update(pid_controller_t *pid, float set_point , float measure, float T_C){

	float u = 0.f;
	float error = 0.f;
	float proportional = 0.f;
	float alpha = pid->Td/T_C;


    /* calculate the error*/
	error = set_point - measure;




	/* proportional contribute*/
	proportional= pid->Kp*error;

	/*integral contribute*/
	pid->integrator+=(pid->Kp/pid->Ti)*0.5f*T_C*(error-pid->prev_err);

	/* simple anti wind-up*/
	if(pid->integrator > pid->lim_integ_max){
		pid->integrator=pid->lim_integ_max;
	}else if(pid->integrator < pid->lim_integ_min)
	{
		pid->integrator=pid->lim_integ_min;
	}

	/* output  */
	if (pid->type>0){
		u=proportional+pid->integrator;
	}else{
		/*derivative contribute*/
		pid->derivative= (2*(pid->Kp)*alpha*error - pid->derivative*(1-(2*alpha)/pid->N))/(1+(2*alpha)/pid->N);
		u=proportional+pid->integrator+0*pid->derivative;
	}

	/* output limitation */
	if(u>pid->lim_out_max){
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









