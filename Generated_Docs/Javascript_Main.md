# Main


## (eel) js_get_data
> gives trajectory data to the python backend


### Outputs
- list: list of json objs containing trajectory data
 



## serial_online
> shows a red or green circle depending on the serial com. status

### Inputs
- bool is_online: boolean stating if the serial communication is online

 



## draw_background
> draws the background on the canvas

the background shows two circumferences with radii equal to the lengths of the two links of the manipulator, with red areas showing where the end effector should avoid going to avoid problems with cable intertwining, etc...
### Inputs
- optional string color: background color;
 - optional string line: line color;
 - optional string limit: limited zones color;

 



## draw_point
> draws a point on the canvas

### Inputs
- float x: x coordinate of the point;
 - float y: y coordinate of the point;

 



## find_circ
> find the circumference arc given 3 points

the circumference arc can be defined with 3 points:
 * the first point determines where the arc should start;
 * the second point determines the diameter of the circumference;
 * the third point determines the angle of the arc, which is the angle between the vector from the center 
 of the circumference to the first point and the vector from the center to the third point;

### Outputs
- Point center: center of the circumference;
 - Point a: starting point of the arc;
 - Point p: ending point of the arc;
 - float r: radius of the circumference;
 - theta_0: angle of the vector a-c;
 - theta_1: angle of the vector p-c;
 



## handle_input
> handles the click event on the canvas

### Inputs
- mouse event e;

 



## line_tool
> method that shows the line tool gui


 



## circle_tool
> method that shows the circle tool gui


 


---

generated with [EasyGen](http://easygen.altervista.org/) - [On Github](https://github.com/dede-amdp/easygen).