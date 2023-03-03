# Dynlib


## dynlib
> support class for the dynamical model of a 2Dofs planar robotic
 arm.

this class exposes all the methods used to compute the dynamical
 model of a 2Dofs planar robotic arm so that it may be used as a library.

 



## vand
> computes the vector [1 t ... t^i ... t^n]

### Inputs
- t: value used for the computations;
 - n: number of elements in the vector;

### Outputs
- M: vector of the computed values;
 



## devand
> compues the derivative of the vector computed by `vand` [0
 1 ... it^(i-1) ... nt^(n-1)]

### Inputs
- t: value used for the computations;
 - n: number of elements in the vector;

### Outputs
- M: vector of the computed values;
 



## dedevand
> computes the derivative of the vector computed by `devand` [0 0 2
 ... i(i-1)t^(i-2) ... n(n-1)t^(n-2)]

### Inputs
- t: value used for the computations;
 - n: number of elements in the vector;

### Outputs
- M: vector of the computed values;
 



## symzeros
> returns a nxm symbolic null matrix

### Inputs
- n: number of rows;
 - m: number of columns;

### Outputs
- M: symbolic null matrix;
 



## centerofmass
> computes the center of mass of a box of size (lx, ly, lz) with
 density rho

Cx = \int_V rho x dV, Cy = \int_V rho y dV, Cz = \int_V rho z dV
### Inputs
- lx: length relative to the x axis;
 - ly: length relative to the y axis;
 - lz: length relative to the z axis;
 - rho: density of the material;

### Outputs
- Cx: x coordinate of the center of mass;
 - Cy: y coordinate of the center of mass;
 - Cz: z coordinate of the center of mass;
 



## linkjacobian
> computes the i-th jacobian of the links of a 2 Dofs planar robotic
 arm with sizes (lx, ly, lz) with density rho, relative to the center of
 mass of the specified link.

J_prismatic,i = [z(i-1);0], J_rotoidal,i =
 [z(i-1)x(p-p(i-1));z(i-1)]
### Inputs
- i: index of the link relative to which the jacobian will be computed;
 - lx: array of lengths relative to the x axis of all the links of the
 robotic arm;
 - ly: array of length relative to the y axis of all the links of the
 robotic arm;
 - lz: array of length relative to the z axis of all the links of the
 robotic arm;
 - rho: density of the material of the links;

### Outputs
- J: jacobian matrix;
 



## linkinertia
> computes the intertia of the i-th link of a 2Dofs robotic arm

### Inputs
- i: index of the link relative to which the intertia will be computed;
 - lx: array of lengths relative to the x axis of the links;
 - ly: array of lengths relative to the y axis of the links;
 - lz: array of lengths relative to the z axis of the links;
 - rho: density of the material of the link;

### Outputs
- inertia: computed inertia;
 



## dh
> computes the rotation and position matrix of the end effector of a
 2Dofs planar robotic arm

computes the rotation and position matrix based on the
 Denavit-Hartenberg Convention, also returns the T matrix computed as
 follows: T = [R p;0 0 0 1]
### Inputs
- lx: array of lengths of the links relative to the x axis;

### Outputs
- T: Homogeneous Transformation Matrix;
 - R: Rotation matrix;
 - p: position vector;
 



## joint2op
> computes the transformation matrix T(phi)

### Inputs
NONE

### Outputs
- T: transformation matrix;
 



## polynomial
> computes a generic polynomial trajectory

### Inputs
- q: values that q has to take;
 - q_dot: values that the derivative of q has to take;
 - q_ddot: values that the second derivative of q has to take;
 - t: instants at which q has to take the values given as input;
 - t_dot: instants at which the derivative of q has to take the values
 given as input;
 - t_ddot: instants at which the second derivative of q has to take the
 values given as input;

### Outputs
- A: coefficients of the polynomial;
 - V: vandermont matrix;
 



## criticaltrajectory
> it computes the trajectory that is used during the dimensioning
 simulations for a 2Dofs planar robotic arm

### Inputs
- tf: duration of the trajectory;
 - amax: maximum acceleration reached at time tf/4;

### Outputs
- A: coefficients of the polynomial trajectory;
 - vandermont: vandermont matrix;
 



## motorjacobian
> computes the jacobian of the motors of a 2Dofs planar robotic arm

### Inputs
- i: index of the motor;
 - lx: array of lengths of the links relative to the x axis;

### Outputs
- J: jacobian;
 



## dyn
> computes the dynamical model of a 2Dofs planar robotic arm

the method computes the dynamical model of a 2Dofs planar robotic
 arm
 
 B(q)q:+C(q,q.)q.+g(q)+Fq.=u-J'h
 
 More in details, the function returns the matrices B(q), C(q.) and the
 value g(q) as symbolic matrices in q1, q2, dq1 and dq2 so that they can be
 adapted to any pose the robotic arm may have.
### Inputs
- lx: array of lengths of all the links relative to the x axis;
 - ly: array of lengths of all the links relative to the y axis;
 - lz: array of lengths of all the links relative to the z axis;
 - rho: density of the material;
 - mm: array of masses of the motors;
 - Im: array of inertias of the motors;
 - kr: array of reduction ratios of the motors;

 


---

generated with [EasyGen](http://easygen.altervista.org/) - [On Github](https://github.com/dede-amdp/easygen).