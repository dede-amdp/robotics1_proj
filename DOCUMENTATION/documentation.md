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
# Manipulator


## class Manipulator
> class that represents a 2Dof manipulator and its trajectory;

### Inputs
- list q: generalized coordinate values;

 



## Manipulator.draw_pose
> draws the two links of the manipulator

### Inputs
- canvas context 2D ctx;

 



## async Manipulator.draw_traces
> draws the "traces" of the manipulator (the trajectory that the links should have followed)

the method was made async so that it may not increase too much the page reaction time;
### Inputs
- canvas context 2D ctx;
 - optional list[string] colors: list of 2 colors to use for the links traces;

 



## Manipulator.dk
> computes the direct kinematics of the manipulator

### Inputs
- list[float] q: configuration of the robot;

### Outputs
- list[float]: list containing the positions of the ending points (list of coordinates) of the links
 



## class Point
> class modeling a point in the operational space

### Inputs
- float x: x coordinate of the point;
 - float y: y coordinate of the point;
 - dict settings: dictionary containing data for coordinate conversion (from canvas space to operational space)

 



## Point.add
> adds to point vectors together

### Inputs
- Point other: Point to add;

### Outputs
- Point: result
 



## Point.sub
> subtracts two point vectors together

### Inputs
- Point other: point to subtract;

### Outputs
- Point: result
 



## Point.mag
> computes the length of the point vector


### Outputs
- float: length of the point vector
 



## Point.scale
> scales the point vector length with the specified scalar

### Inputs
- float scalar: value used to scale the length;

### Outputs
- Point: scaled point vector
 



## Point.rot
> rotates (of the specified angle) the point vector around its origin

### Inputs
float delta: angle (in radians) used for the rotation (a positive rotation is counterclockwise);

### Outputs
- Point: rotated point vector
 



## Point.set
> sets the length of the point vector to the specified value

### Inputs
- float scalar: length of the new vector;

### Outputs
- Point: scaled point vector;
 



## Point.angle
> computes the angle of the point vector (in radians)


### Outputs
- float: angle of the point vector
 



## class Trajectory
> class that models a trajectory made fo multiple patches

the trajectory modelled by this class can be made up of two different kind of patches:
 * line: a linear movement from point A to point B;
 * circle: a curvilinear movement between two points (A and P) by following a circumference with given center and radius;

 



## Trajectory.add_line
> adds a line patch to the trajectory

### Inputs
- Point p0: starting point of the patch;
 - Point p1: ending point of the patch;
 - bool raised: boolean that states wether the pen on the end effector of the manipulator should be raised or not;

 



## Trajectory.add_circle
> adds a circular patch to the trajectory

### Inputs
- Point c: center of the circumference;
 - float r: radius of the circumference;
 - float theta_0: angle of the starting point;
 - float theta_1: angle of the ending point;
 - bool raised: states wether the pen on the end effector of the manipulator should be up or not;
 - Point a: starting point of the circumference;
 - Point p: ending point of the circumference;

 



## Trajectory.reset
> resets the trajectory data


 



## Trajectory.draw
> draws the trajectory

each patch of the trajectory will be drawn on the canvas specified by the user
### Inputs
- 2D canvas context ctx: context of the canvas that will be used to draw on;

 


---

generated with [EasyGen](http://easygen.altervista.org/) - [On Github](https://github.com/dede-amdp/easygen).