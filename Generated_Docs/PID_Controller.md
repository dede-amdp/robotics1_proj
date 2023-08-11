# Pid_controller


## PID_init
> initialize the values of the PID controller;

### Inputs
- pid_controller \*pid: struct data where all the PID data is be stored;
 - float KP: proportional constant;
 - float TI: integration time constant;
 - float TD: derivation time constant;
 - float N: discrete derivation factor;
 - int controller_type: whether the controller will be a P, PI or PID controller;

### Outputs
- void;
 



## set_limit
> initialize the values of the maximum and the minimum allowed for the output and for the integrator value;

### Inputs
- pid_controller \*pid: struct data where all the data will be stored ;
 - float lim_out_min: minimum limit value for the output;
 - float lim_out_max: maximum limit value for the output;
 - float lim_integ_min: minimum limit value for the integrator value;
 - float lim_integ_max: maximum limit value for the integrator value;

### Outputs
- void;
 



## PID_update
> update the value of the controller calculated in relation to the error value.

the output value of the PID controller is memorized in the `out` member of the PID struct.
### Inputs
- pid_controller \*pid: pointer to the struct where all the PID data is be stored;
 - float set_point: value of the setpoint used to compute the error;
 - float measure: measured value of the output of the controlled system;
 - float T_C: sampling time;

### Outputs
- void;
 


---

generated with [EasyGen](http://easygen.altervista.org/) - [On Github](https://github.com/dede-amdp/easygen).