# Trajpy


## time_row
> computes and returns as a list a row of the vandermont matrix

the returned row is actually the one linked to the equation a0t^0 + a1t^1 + a2t^2 + ... + ant^n = 0 => 
 $$\sum_i^n a_it^i = 0$$
 with n specified by the user. The method can also return the *derivative* of this equation, up to the 2nd derivative
### Inputs
- float t: the value of t used to compute the list;
 - int deg: degree of the equation (n in the example above);
 - int der: order of the derivative (from 0 up to 2);

### Outputs
- list[float] : row of the vandermont matrix: [1, t, t**2, ..., t**n] (or its derivatives)
 



## spline3
> computes the coefficients of a cubic spline

the cubic spline has the following structure:\
 q = a0+a1t+a2t^2 + a3t^3\
 dq = a1+2a2t+3a3t^2\
 ddq = 2a2+6a3t\
 where q is the position spline, dq is the velocity spline and ddq is the acceleration spline
### Inputs
- list[point_time] q: it is a list of tuples of a value (float) and a time instant (float). These values and time instants will be used to write a polynomial (with variable "t") that will cross the specified values at t equal to the specified times;
 - list[point_time] dq: it is a list of a value (float) and a time instant (float). These values and time instants will be used to write a polynomial (with variable "t") which derivative will cross the specified values at t equal to the specified times;

### Outputs
- list[function] : list of functions of the variable t that represent the trajectories q(t), dq(t) and ddq(t)
 



## spline5
> computes the coefficients of a 5th order spline

the 5th order spline has the following structure:\
 q = a0+a1t+a2t^2 + a3t^3 + a4t^4 + a5t^5\
 dq = a1+2a2t+3a3t^2 + 4a4t^3 + 5a5t^4\
 ddq = 2a2+6a3t+12a4t^2 + 20a5t^3
### Inputs
- list[point_time] q: it is a list of tuples of a value (float) and a time instant (float). These values and time instants will be used to write a polynomial (with variable "t") that will cross the specified values at t equal to the specified times;
 - list[point_time] dq: it is a list of a value (float) and a time instant (float). These values and time instants will be used to write a polynomial (with variable "t") which derivative will cross the specified values at t equal to the specified times;

### Outputs
- ndarray : numpy array of the coefficients of the 5th order polynomial that crosses the specified points.
 



## rangef
> returns a list containing all the values between the initial and final values with the specified step size (that can be float)

### Inputs
- float start: the starting value of the range;
 - float step : the step size used to find the values withing the specified range;
 - float end: the final value of the range;
 - bool consider_limit: boolean that indicates whether the end value should be inserted in the returned list or not;

### Outputs
- list[float] : list of all the values found in the specified range with the specified step size.
 



## compose_spline3
> returns the trajectory that results from the composition of the cubic splines obtained for each couple of points in the specified path.

### Inputs
- list[float] q: list of points that compose the path;
 - float ddqm: maximum acceleration;
 - list[float] dts: duration of each splines;

### Outputs
- list[tuple[list[function], float]] :  list of function/spline-duration tuples.
 



## cubic_speeds
> computes the speeds of the intermediate points of a cubic spline

### Inputs
- list[float] q: list of the points of the path;
 - list[float] dts: list of the duration of each section of the path;

### Outputs
- list[float]: list of intermediate speeds.
 



## preprocess
> subdivides the ranges passed as a list into smaller ranges of size equal to the specified limit.

### Inputs
- list[float] q: list of values;
 - float limit: limit value used to subdivide the specified ranges (q);

### Outputs
- list[float]: new list of values whose ranges are smaller or equal to the specified limit;
 



## trapezoidal
> computes the trapezoidal speed profile trajectory for the specified points;

the trapezoidal trajectory is subdivided into 3 sections, 2 parabolic ones of equal duration (initial and final ones) and a linear section with constant velocity.
### Inputs
- list[float] q: list that contains the initial and final values of the trajectory;
 - float ddqm: maximum acceleration;
 - float tf: duration of the trajectory;

### Outputs
- list[tuple[ndarray, float]]: list containing the coefficients of each section of the trajectory and their durations.
 



## compose_trapezoidal
> returns the trajectory that results from the composition of the trapezoidal speed profile trajectories obtained for each couple of points of the specified path.

### Inputs
- list[float] q: list of points that compose the path;
 - float ddqm: maximum acceleration;

### Outputs
- list[tuple[ndarray, float]] :  list of coefficients/trapezoidal-duration tuples.
 



## cycloidal
> computes a cycloidal trajectory

the cycloidal trajectory is not polynomial, so it cannot be represented as a list of coefficients: for this reason a function handle is created for the position q, the speed dq and the acceleration ddq that can be used to compute the trajectory given t.
### Inputs
- list[float] q: initial and final values of the trajectory;
 - float ddqm: maximum acceleration;
 - float tf: duration of the trajectory;

### Outputs
- tuple[list[function], float] : function-handle/cycloidal-duration tuple.
 



## compose_cycloidal
> returns the trajectory resulting from the composition of the cycloidal trajectories obtained from each couple of values in the specified path.

given that the cycloidal trajectory cannot be represented with just a list of coefficients, the returned trajectory will be a list of function handles.
### Inputs
- list[float] q: list of points in the path (the timing law will be autogenerated);
 - float ddqm: maximum acceleration;

### Outputs
- list[tuple[list[function], float]]: list of trajectory/cycloidal-duration tuples.
 



## ik
> inverse kinematics of a 2Dofs planar manipulator

it can compute the joint variables values even if the orientation of the end effector is not specified.
### Inputs
- float x: x coordinate of the end effector;
 - float y: y coordinate of the end effector;
 - float theta: orientation of the end effector (angle of rotation relative to the z axis with theta=0 when the x axis of the end effector is aligned with the x axis of the base frame of reference);
 - dict[float] sizes: sizes of the two links that make up the manipulator, accessed via 'l1' and 'l2';

### Outputs
- ndarray: column numpy array containing the values of the joint coordinates.
 



## dk
> direct kinematics of a 2Dofs planar manipulator

it can compute the x, y coordinates of the end effector and its orientation theta (angle of rotation relative to the z axis with theta=0 when the x axis of the end effector is aligned with the x axis of the base frame of reference)
### Inputs
- ndarray q: colum numpy array containing the values of the joint coordinates;
 - dict[float] sizes: sizes of the two links that make up the manipulator, accessed via 'l1' and 'l2';

### Outputs
- ndarray: column numpy array containing the values of the coordinates of the end effector (x, y and the rotation angle theta).
 



## Point (class)
> Point class used to represent points in the operational space with a cartesian frame of reference

### Inputs
- float x: x coordinate of the point
 - float y: y coordinate of the point

 



## Point.mag
> computes the length of the vector <x, y>


### Outputs
- float: length of the vector
 



## Point.angle
> computes the angle of the vector <x, y>


### Outputs
- float: angle of the vector
 



## Point.rotate
> rotates the vector around its origin

### Inputs
- float phi: angle (in radians) of rotation;

### Outputs
- Point: rotated vector
 



## Point.ew
> computes the element-wise multiplication (scalar product)

given two points/vectors a and b, the method returns the value x_a*x_b+y_a*y_b (equivalent to a*b^T)
### Inputs
- Point other: the other point with which the element wise multiplication is done;

### Outputs
- float: scalar product
 



## Point.angle_between
> computes the angle between two vectors

### Inputs
- Point other: vector with which the computation will be done;

### Outputs
- float: the angle between the two vectors
 



## slice_trj
> slices the trajectory patch

depending on the type of notes (line or circle) this function slices the trajectory patch in segments depending on 
 a timing law s(t) specified by the user
### Inputs
- dict patch: trajectory patch with the following structure:
 ```python
 {
 'type': 'line' or 'circle',
 'points': [[x0, y0], [x1, y1]], # start and end points
 'data': {'center':c, 'penup':penup, ...}
 }
 ```
 - **kargs:
     * 'max_acc': maximum acceleration;
     * 'line': timing law s(t) for a linear trajectory patch;
     * 'circle': timing law s(t) for a circular trajectory patch;
     * 'sizes': sizes dict containing the sizes of the two links of the manipulator ({'l1': l1, 'l2':l2});
     * 'Tc': time step used for the timing law;

### Outputs
- list q0s: list of values for the generalized coordinate q of the first motor;
 - list q1s: list of values for the generalized coordinate q of the second motor;
 - list penups: list of values that show wether the pen should be up or down;
 - list ts: list of time instants;
 



## find_velocities
> computes the velocity of the trajectory in each time instant

### Inputs
- list[float] q: list of motor positions;
 - list[float] ts: list of time instants;

### Outputs
- list[float]: list of velocities
 



## find_accelerations
> computes the acceleration of the trajectory in each time instant

### Inputs
- list[float] dq: list of the motor velocities;
 - list[float] ts: list of time instants;

### Outputs
- list[float]: list of accelerations
 


---

generated with [EasyGen](http://easygen.altervista.org/) - [On Github](https://github.com/dede-amdp/easygen).