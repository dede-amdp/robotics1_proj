# Pid_controller


## struct pid_controller_t
> struct that holds all the data of a PID controller.

### Inputs
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

 


---

generated with [EasyGen](http://easygen.altervista.org/) - [On Github](https://github.com/dede-amdp/easygen).