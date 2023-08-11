# Main


## send_data
> sends data to the micro controller

### Inputs
- str msg_type: type of message to send : "trj" is used for trajectory data;
 - any list \*\*data: any number of lists (containing the position setpoints to send in case of trj data);

### Outputs
- None;
 



## trace_trajectory
> draws the trajectories on the GUI

### Inputs
- tuple[list, list] q: a tuple containing the list of positions for each motor;

### Outputs
- None;
 



## eel.expose py_log
> simply prints a message on the python console

### Inputs
- str msg: message to be print;

### Outputs
- None;
 



## eel.expose py_get_data
> gets the trajectory data from the web GUI and converts it into a list of setpoints to be sent to the micro controller

### Inputs
- None;

### Outputs
- None;
 



## eel.expose py_homing_cmd
> sends the homing command to the micro controller

### Inputs
- None;

### Outputs
- None;
 



## eel.expose py_serial_online
> return whether the serial is online or not

### Inputs
- None;

### Outputs
- bool: bool value that shows if the serial is online or not;
 



## eel.expose py_serial_sartup
> initializes the serial communication

### Inputs
- None;

### Outputs
- None;
 


---

generated with [EasyGen](http://easygen.altervista.org/) - [On Github](https://github.com/dede-amdp/easygen).