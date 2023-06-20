from math import sqrt,atan2,cos,sin,pi,acos, copysign
import numpy as np
from typing import Callable
#from scipy.interpolate import CubicSpline

point_time = tuple[float, float] # point_time type for type annotation
function = Callable[[float], float] # function handle type for type annotation



""" #@
@name: time_row
@brief: computes and returns as a list a row of the vandermont matrix
@notes: the returned row is actually the one linked to the equation a0t^0+a1t^1+a2t^2+...+ant^n = 0 => 
$$\sum_i^n a_it^i = 0$$
with n specified by the user. The method can also return the *derivative* of this equation, up to the 2nd derivative
@inputs: 
- float t: the value of t used to compute the list;
- int deg: degree of the equation (n in the example above);
- int der: order of the derivative (from 0 up to 2);
@outputs: 
- list[float] : row of the vandermont matrix: [1, t, t**2, ..., t**n] (or its derivatives)
@# """
def time_row(t:float, deg: int, der:int=0) -> list[float]:
    row = []
    for i in range(0, deg+1):
        match der:
            case 0: row.append(t**i)
            case 1: row.append(i*t**(i-1) if i-1 >= 0 else 0)
            case 2: row.append(i*(i-1)*t**(i-2) if i-2 >= 0 else 0)
    return row


""" #@
@name: spline3
@brief: computes the coefficients of a cubic spline
@notes: the cubic spline has the following structure:\
q = a0+a1t+a2t^2+a3t^3\
dq = a1+2a2t+3a3t^2\
ddq = 2a2+6a3t\
where q is the position spline, dq is the velocity spline and ddq is the acceleration spline
@inputs: 
- list[point_time] q: it is a list of tuples of a value (float) and a time instant (float). These values and time instants will be used to write a polynomial (with variable "t") that will cross the specified values at t equal to the specified times;
- list[point_time] dq: it is a list of a value (float) and a time instant (float). These values and time instants will be used to write a polynomial (with variable "t") which derivative will cross the specified values at t equal to the specified times;
@outputs: 
- list[function] : list of functions of the variable t that represent the trajectories q(t), dq(t) and ddq(t)
@# """
def spline3(q: list[point_time], dq: list[point_time]) -> list[function]:
    # q = a0+a1t+a2t2+a3t3
    # dq = a1+2a2t+3a3t2
    # ddq = 2a2+6a3t
    data = []
    for p in q:
        data.append(time_row(p[1], 3, 0))
    for p in dq:
        data.append(time_row(p[1], 3, 1))
    known_terms = [float(p[0]) for p in q+dq]
    A = np.array(data)
    b = np.array(known_terms).T
    a = np.dot(np.linalg.inv(A), b)
    return [
        lambda t: float(np.dot(a, np.array(time_row(t,3,0)))),
        lambda t: float(np.dot(a, np.array(time_row(t,3,1)))),
        lambda t: float(np.dot(a, np.array(time_row(t,3,2))))
    ]



""" #@
@name: spline5
@brief: computes the coefficients of a 5th order spline
@notes: the 5th order spline has the following structure:\
q = a0+a1t+a2t^2+a3t^3+a4t^4+a5t^5\
dq = a1+2a2t+3a3t^2+4a4t^3+5a5t^4\
ddq = 2a2+6a3t+12a4t^2+20a5t^3
@inputs: 
- list[point_time] q: it is a list of tuples of a value (float) and a time instant (float). These values and time instants will be used to write a polynomial (with variable "t") that will cross the specified values at t equal to the specified times;
- list[point_time] dq: it is a list of a value (float) and a time instant (float). These values and time instants will be used to write a polynomial (with variable "t") which derivative will cross the specified values at t equal to the specified times;
@outputs: 
- ndarray : numpy array of the coefficients of the 5th order polynomial that crosses the specified points.
@# """
def spline5(q: list[point_time], dq: list[point_time], ddq: list[point_time])->np.ndarray:
    '''
    q = a0+a1t+a2t2+a3t3+a4t4+a5t5
    dq = a1+2a2t+3a3t2+4a4t3+5a5t4
    ddq = 2a2+6a3t+12a4t2+20a5t3
    '''
    vandermont = []
    for point in q:
        vandermont.append(time_row(point[1],5,0))
    for point in dq:
        vandermont.append(time_row(point[1],5,1))
    for point in ddq:
        vandermont.append(time_row(point[1],5,2))
    known_terms = [[point[0]] for point in q+dq+ddq]
    A = np.array(vandermont) 
    b = np.array(known_terms)
    return np.dot(np.linalg.inv(A), b)


""" #@
@name: rangef
@brief: returns a list containing all the values between the initial and final values with the specified step size (that can be float)
@inputs: 
- float start: the starting value of the range;
- float step : the step size used to find the values withing the specified range;
- float end: the final value of the range;
- bool consider_limit: boolean that indicates whether the end value should be inserted in the returned list or not;
@outputs: 
- list[float] : list of all the values found in the specified range with the specified step size.
@# """
def rangef(start:float=0, step:float=1, end:float=0, consider_limit:bool = False) -> list[float]:
    if step == 0: return []
    if start >= end: return []
    if start < end and step < 0: return []
    if start > end and step > 0: return []
    r = []
    i = start
    if not consider_limit:
        while i < end:
            r.append(i)
            i += step
    else:
        while i <= end:
            r.append(i)
            i += step
    return r

""" #@
@name: compose_spline3
@brief: returns the trajectory that results from the composition of the cubic splines obtained for each couple of points in the specified path.
@inputs: 
- list[float] q: list of points that compose the path;
- float ddqm: maximum acceleration;
- list[float] dts: duration of each splines;
@outputs: 
- list[tuple[list[function], float]] :  list of function/spline-duration tuples.
@# """
def compose_spline3(q: list[float], ddqm: float = 1.05, dts:list[float]= None) -> list[tuple[list[function], float]]:
    trajectory = []
    if dts is None:
        # compute the duration of each polynomial
        dts = []
        for q1, q0 in zip(q[:len(q)-1], q[1:]):
            dts.append(sqrt(2*pi*abs(q1-q0)/ddqm))
        dq = cubic_speeds(q, dts) #*0 #cubic_speeds(q, dts)
        for q0, q1, dq0, dq1, dt in zip(q[:len(q)-1], q[1:], dq[:len(dq)], dq[1:], dts):
            q0_point = (q0, 0)
            q1_point = (q1, dt)
            dq0_point = (dq0, 0)
            dq1_point = (dq1, dt)
            polys = spline3([q0_point, q1_point], [dq0_point, dq1_point])
            trajectory.append((polys, dt))
    return trajectory


""" #@
@name: cubic_speeds
@brief: computes the speeds of the intermediate points of a cubic spline
@inputs: 
- list[float] q: list of the points of the path;
- list[float] dts: list of the duration of each section of the path;
@outputs: 
- list[float]: list of intermediate speeds.
@# """
def cubic_speeds(q: list[float], dts: list[float]) -> list[float]:
    if len(q) == 2 : return [0, 0]
    A = np.zeros((len(q)-2, len(q)-2))
    c = np.zeros((len(q)-2, 1))
    dqs = [(q1-q0)[0] for q0, q1 in zip(q[:len(q)-1], q[1:])]
    ck = lambda k : 3*((dts[k]**2)*dqs[k+1]+ (dts[k+1]**2)*dqs[k])/(dts[k]*dts[k+1])
    
    print(len(q), len(dts), len(dqs))
    for i in range(len(q)-2):
        A[i, i] += 2*dts[i]
        if i+1 < len(q)-2 : A[i, i+1] = dts[i]
        if i-1 >= 0 : A[i-1, i-1] += 2*dts[i]
        if i-2 >= 0 : A[i-1, i-2] = dts[i]

        if i + 1 < len(q)-1: c[i] = ck(i)

    v = np.dot(np.linalg.inv(A), c)
    return v.T.tolist()[0]


""" #@
@name: preprocess
@brief: subdivides the ranges passed as a list into smaller ranges of size equal to the specified limit.
@inputs: 
- list[float] q: list of values;
- float limit: limit value used to subdivide the specified ranges (q);
@outputs: 
- list[float]: new list of values whose ranges are smaller or equal to the specified limit;
@# """
def preprocess(q: list[float], limit:float=pi/3) -> list[float]:
    new_q = []
    for i,j in zip(range(0,len(q)-1), range(1,len(q))):
        qi = q[i]
        qj = q[j]
        if abs(qj-qi) > limit:
            if qi < qj:
                for qk in rangef(qi, limit, qj, True):
                    new_q.append(qk)
            else:
                for qk in rangef(qi, -limit, qj, True):
                    new_q.append(qk)
        else:
            new_q.append(qi)
    new_q.append(q[-1])
    return new_q


""" #@
@name: trapezoidal
@brief: computes the trapezoidal speed profile trajectory for the specified points;
@notes: the trapezoidal trajectory is subdivided into 3 sections, 2 parabolic ones of equal duration (initial and final ones) and a linear section with constant velocity.
@inputs: 
- list[float] q: list that contains the initial and final values of the trajectory;
- float ddqm: maximum acceleration;
- float tf: duration of the trajectory;
@outputs: 
- list[tuple[ndarray, float]]: list containing the coefficients of each section of the trajectory and their durations.
@# """
def trapezoidal(q:list[float], ddqm:float = 1.05, tf: float = None) -> tuple[list[function], float]: #list[tuple[np.ndarray, float]]:
    # abs(ddqm) >= 4*abs(q[1]-q[0])/tf**2
    # tf**2/(4*abs(q[1]-q[0])) >= 1/abs(ddqm)
    # tf >= +sqrt((4*abs(q[1]-q[0]))/abs(ddqm))
    tc = 0
    if tf is None:
        # if the duration time is not specified, use a bang bang profile
        tf = sqrt((4*abs(q[1]-q[0]))/abs(ddqm)) # bang-bang profile
        tc = tf/2
    else:
        if abs(ddqm) < 4*abs(q[1]-q[0])/tf**2:
            print("This trajectory is not actuable with the specified acceleration:\nchoose a bigger acceleration value")
            return None
        tc = tf/2-sqrt((ddqm*tf)**2-4*ddqm*(q[1]-q[0]))/(2*ddqm)
    qc = q[0] + 0.5*ddqm*tc**2
    qb = qc+ddqm*tc*(tf-2*tc)
    '''
    first = (np.array([[q[0]],[0],[0.5*ddqm]]).T,tc) # (coefficients, duration)
    second = (np.array([[qc], [ddqm*tc], [0]]).T, tf-2*tc)
    third = (np.array([[qb],[ddqm*tc], [-0.5*ddqm]]).T, tc)
    '''
    first = np.array([[q[0], 0, 0.5*ddqm]])
    second = np.array([[qc, ddqm*tc, 0]])
    third = np.array([[qb, ddqm*tc, -0.5*ddqm]])
    def qt(t): 
        if t <= tc : return np.dot(first[0], np.array(time_row(t, 2, 0)).T)
        if t > tc and t <= tc+tf-2*tc : return np.dot(second[0], np.array(time_row(t, 2, 0)).T)
        if t > tc+tf-2*tc and t <= tf : return np.dot(third[0], np.array(time_row(t, 2, 0)).T)
    def dqt(t): 
        if t <= tc : return np.dot(first[0], np.array(time_row(t, 2, 1)).T)
        if t > tc and t <= tc+tf-2*tc : return np.dot(second[0], np.array(time_row(t, 2, 1)).T)
        if t > tc+tf-2*tc and t <= tf : return np.dot(third[0], np.array(time_row(t, 2, 1)).T)
    def ddqt(t): 
        if t <= tc : return np.dot(first[0], np.array(time_row(t, 2, 2)).T)
        if t > tc and t <= tc+tf-2*tc : return np.dot(second[0], np.array(time_row(t, 2, 2)).T)
        if t > tc+tf-2*tc and t <= tf : return np.dot(third[0], np.array(time_row(t, 2, 2)).T)
    return ([qt, dqt, ddqt], tf)


""" #@
@name: compose_trapezoidal
@brief: returns the trajectory that results from the composition of the trapezoidal speed profile trajectories obtained for each couple of points of the specified path.
@inputs: 
- list[float] q: list of points that compose the path;
- float ddqm: maximum acceleration;
@outputs: 
- list[tuple[ndarray, float]] :  list of coefficients/trapezoidal-duration tuples.
@# """
'''
def compose_trapezoidal(q:list[float], ddqm:float = 1.05) -> list[tuple[np.ndarray, float]]:
    A = []
    for k in range(len(q)-1):
        q0 = q[k]
        q1 = q[k+1]
        qk = trapezoidal([q0, q1], ddqm)
        A += qk
    return A
'''

def compose_trapezoidal(q:list[float], ddqm:float = 1.05) -> list[tuple[list[function], float]]: #list[tuple[np.ndarray, float]]:
    A = []
    for k in range(len(q)-1):
        q0 = q[k][0]
        q1 = q[k+1][0]
        qk = trapezoidal([q0, q1], ddqm)
        A.append(qk)
    return A

""" #@
@name: cycloidal
@brief: computes a cycloidal trajectory 
@notes: the cycloidal trajectory is not polynomial, so it cannot be represented as a list of coefficients: for this reason a function handle is created for the position q, the speed dq and the acceleration ddq that can be used to compute the trajectory given t.
@inputs: 
- list[float] q: initial and final values of the trajectory;
- float ddqm: maximum acceleration;
- float tf: duration of the trajectory;
@outputs: 
- tuple[list[function], float] : function-handle/cycloidal-duration tuple. 
@# """
def cycloidal(q:list[float], ddqm:float = 1.05, tf:float=None) -> tuple[list[function], float]: # return the function handles for q, dq and ddq
    if tf is None:
        tf = sqrt(2*pi*abs(q[1]-q[0])/ddqm)
    qt = lambda t: q[0]+(q[1]-q[0])*(t/tf-sin(2*pi*t/tf)/(2*pi))
    dqt = lambda t: (q[1]-q[0])*(1-cos(2*pi*t/tf))/tf # derivative of q
    ddqt = lambda t: 2*pi*(q[1]-q[0])*sin(2*pi*t/tf)/(tf**2) # 2nd derivative of q
    return ([qt, dqt, ddqt], tf) # all of q and its derivatives are returned because they cannot be computed simply by using a different set of coefficients


""" #@
@name: compose_cycloidal
@brief: returns the trajectory resulting from the composition of the cycloidal trajectories obtained from each couple of values in the specified path.
@notes: given that the cycloidal trajectory cannot be represented with just a list of coefficients, the returned trajectory will be a list of function handles.
@inputs: 
- list[float] q: list of points in the path (the timing law will be autogenerated);
- float ddqm: maximum acceleration;
@outputs: 
- list[tuple[list[function], float]]: list of trajectory/cycloidal-duration tuples.
@# """
def compose_cycloidal(q:list[float], ddqm:float = 1.05) -> list[tuple[list[function], float]]:
    A = []
    q0 = q[:len(q)-1]
    q1 = q[1:]
    for q0t,q1t in zip(q0,q1):
        qk = cycloidal([q0t[0],q1t[0]], ddqm)
        A.append(qk)
    return A


""" #@
@name: ik
@brief: inverse kinematics of a 2Dofs planar manipulator
@notes: it can compute the joint variables values even if the orientation of the end effector is not specified.
@inputs: 
- float x: x coordinate of the end effector;
- float y: y coordinate of the end effector;
- float theta: orientation of the end effector (angle of rotation relative to the z axis with theta=0 when the x axis of the end effector is aligned with the x axis of the base frame of reference);
- dict[float] sizes: sizes of the two links that make up the manipulator, accessed via 'l1' and 'l2'; 
@outputs: 
- ndarray: column numpy array containing the values of the joint coordinates.
@# """
def ik(x:float, y:float, z:float = 0, theta:float = None, sizes:dict[float] = {'l1':0.25,'l2':0.25}) -> np.ndarray:
    if x**2+y**2 > (sizes['l1']+sizes['l2'])**2: return None
    q1 = 0
    q2 = 0
    a1 = sizes['l1']
    a2 = sizes['l2']

    if theta is not None:
        cos_q2 = (x**2+y**2-sizes['l1']**2-sizes['l2']**2)/(2*sizes['l1']*sizes['l2'])
        sin_q2 = sqrt(1-cos_q2**2)
        q2 = atan2(sin_q2, cos_q2)
        q1 = theta-q2
    else:
        q2 = acos((x**2+y**2-a1**2-a2**2)/(2*a1*a2))
        q1 = atan2(y,x)-atan2(a2*sin(q2), a1+a2*cos(q2))

    q = np.array([[q1,q2,z]]).T
    return q

""" #@
@name: dk
@brief: direct kinematics of a 2Dofs planar manipulator
@notes: it can compute the x, y coordinates of the end effector and its orientation theta (angle of rotation relative to the z axis with theta=0 when the x axis of the end effector is aligned with the x axis of the base frame of reference)
@inputs: 
- ndarray q: colum numpy array containing the values of the joint coordinates;
- dict[float] sizes: sizes of the two links that make up the manipulator, accessed via 'l1' and 'l2'; 
@outputs: 
- ndarray: column numpy array containing the values of the coordinates of the end effector (x, y and the rotation angle theta).
@# """
def dk(q:np.ndarray, sizes:dict[float] = {'l1':0.25,'l2':0.25})->np.ndarray:
    x = sizes['l1']*cos(q[0])+sizes['l2']*cos(q[0]+q[1])
    y = sizes['l1']*sin(q[0])+sizes['l2']*sin(q[0]+q[1])
    theta = q[0]+q[1]
    return np.array([[x,y,theta]]).T

"""
#@
@name: Point (class)
@brief: Point class used to represent points in the operational space with a cartesian frame of reference
@inputs: 
- float x: x coordinate of the point
- float y: y coordinate of the point
@#
"""
class Point:
    def __init__(self, x:float, y:float):
        self.x = x
        self.y = y
    
    def __add__(self, other):
        result = Point(self.x, self.y)
        result.x += other.x
        result.y += other.y
        return result

    def __sub__(self, other):
        result = Point(self.x, self.y)
        result.x -= other.x
        result.y -= other.y
        return result
    
    def __mul__(self, scalar:float):
        result = Point(self.x, self.y)
        result.x *= scalar
        result.y *= scalar
        return result

    def __rmul__(self, scalar:float):
        result = Point(self.x, self.y)
        result.x *= scalar
        result.y *= scalar
        return result

    def __div__(self, scalar:float):
        if scalar == 0: raise ZeroDivisionError()
        result = Point(self.x, self.y)
        result.x /= scalar
        result.y /= scalar
        return result

    def __rdiv__(self, scalar:float):
        if scalar == 0: raise ZeroDivisionError()
        result = Point(self.x, self.y)
        result.x /= scalar
        result.y /= scalar
        return result
    
    """
#@
@name: Point.mag
@brief: computes the length of the vector <x, y>
@outputs: 
- float: length of the vector
@#
    """
    def mag(self) -> float:
        return sqrt(self.x**2+self.y**2)

    """
#@
@name: Point.angle
@brief: computes the angle of the vector <x, y>
@outputs: 
- float: angle of the vector
@#
    """
    def angle(self) -> float:
        return atan2(self.y, self.x)
    
    """
#@
@name: Point.rotate
@brief: rotates the vector around its origin
@inputs: 
- float phi: angle (in radians) of rotation;
@outputs: 
- Point: rotated vector
@#
    """
    def rotate(self, phi):
        result = Point(self.x, self.y)
        angle = (result.angle() + phi)
        length = self.mag()
        result.x = length*cos(angle)
        result.y = length*sin(angle)
        return result

    """
#@
@name: Point.ew
@brief: computes the element-wise multiplication (scalar product)
@notes: given two points/vectors a and b, the method returns the value x_a*x_b+y_a*y_b (equivalent to a*b^T)
@inputs: 
- Point other: the other point with which the element wise multiplication is done;
@outputs: 
- float: scalar product
@#
    """
    def ew(self, other) -> float:
        # element wise multiplication
        return self.x*other.x+self.y*other.y

    """
#@
@name: Point.angle_between
@brief: computes the angle between two vectors
@inputs: 
- Point other: vector with which the computation will be done;
@outputs: 
- float: the angle between the two vectors
@#
"""
    def angle_between(self, other) -> float:
        return acos(self.ew(other)/(self.mag()*other.mag()))

    def __str__(self) -> str:
        return f'<{self.x}, {self.y}>'

    """
#@
@name: slice_trj
@brief: slices the trajectory patch 
@notes: depending on the type of notes (line or circle) this function slices the trajectory patch in segments depending on 
a timing law s(t) specified by the user
@inputs: 
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
@outputs: 
- list q0s: list of values for the generalized coordinate q of the first motor;
- list q1s: list of values for the generalized coordinate q of the second motor;
- list penups: list of values that show wether the pen should be up or down;
- list ts: list of time instants;
@#
"""
def slice_trj(patch: dict, **kargs):
    # populate arguments with default values
    if 'max_acc' not in kargs:
        kargs['max_acc'] = 1.05
    if 'line' not in kargs or 'circle' not in kargs:
        raise Exception("No line or circle timing law was specified")
    if 'Tc' not in kargs:
        kargs['Tc'] = 1e-3
    if 'sizes' not in kargs:
        print('Using default sizes')
    
    q0s = []
    q1s = []
    penups = []
    ts = []

    # patch['points'] -> [[x0, y0], [x1, y1]]
    sp = Point(*patch['points'][0]) # starting point in operational space
    ep = Point(*patch['points'][1]) # ending point in operational space
    l = (ep-sp).mag() # linear distance between the two points
    c = Point(*patch['data']['center']) if patch['type'] == 'circle' else None # center of the circle
    angle = 0
    if patch['type'] == 'circle':
        v1 = (sp-c) 
        v2 = (ep-c) 
        # find the angle of rotation between start and end point
        d_alpha = (2*pi+v2.angle())%(2*pi) - (2*pi+v1.angle())%(2*pi) # between 0 and 2pi
        angle = d_alpha if abs(d_alpha) < pi else (-(2*pi-d_alpha) if d_alpha > 0 else 2*pi+d_alpha) # angle between the two vectors starting from the center of the circumference

    length = l if patch['type'] == 'line' else patch['data']['radius']*abs(angle) # LENGTH OF THE PATH
    tf = sqrt(2*pi*length/kargs['max_acc']) # duration of the motion

    points = [] # points (in operational space)
    if patch['data']['penup']:
        # if penup -> use a point-to-point trajectory (in this case: cycloidal)
        # patch['points'] -> [[x0, y0], [x1, y1]]
        qt0 = list(ik(patch['points'][0][0], patch['points'][0][1], 1, None, kargs['sizes']).T[0])
        qt1 = list(ik(patch['points'][1][0], patch['points'][1][1], 1, None, kargs['sizes']).T[0])
        (traj0, dt0) = cycloidal([qt0[0], qt1[0]], kargs['max_acc'], tf) # first motor
        (traj1, dt1) = cycloidal([qt0[1], qt1[1]], kargs['max_acc'], tf) # second motor
        for t in rangef(0, kargs['Tc'], tf):
            if t <= dt0 : q0s.append(traj0[0](t))
            else: q0s.append(q0s[-1]) # if the trajectories don't have the same length
            if t <= dt1 : q1s.append(traj1[0](t))
            else: q1s.append(q1s[-1]) # if the trajectories don't have the same length
            ts.append(t)
        return q0s, q1s, penups, ts
    # here penup=0 surely
    if patch['type'] == 'line':
        for t in rangef(0, kargs['Tc'], tf, True):
            s = kargs['line'](t, tf) # s \in [0, 1], t \in [0, tf]
            points.append(sp + ((ep-sp)*s))
            ts.append(t)
    elif patch['type'] == 'circle':
        for t in rangef(0, kargs['Tc'], tf, True):
            s = kargs['circle'](t, tf)
            points.append(c+(sp-c).rotate(s*angle))
            ts.append(t)

    for p in points:
        qt = list(ik(p.x, p.y, 0, None, kargs['sizes']).T[0]) # points converted to joint space
        q0s.append(qt[0])
        q1s.append(qt[1])
        penups.append(0)
    
    return q0s, q1s, penups, ts


"""
#@
@name: find_velocities
@brief: computes the velocity of the trajectory in each time instant
@inputs: 
- list[float] q: list of motor positions;
- list[float] ts: list of time instants;
@outputs: 
- list[float]: list of velocities
@#
"""
def find_velocities(q: list[float], ts: list[float]) -> list[float]:
    dqs = []
    k = 0 # debug
    for q0, q1, t0, t1 in zip(q[:-1], q[1:], ts[:-1], ts[1:]):
        dq = q1-q0
        dt = t1-t0
        if dt == 0: print(k, dq) # debug
        dqs.append(dq/dt)
        k+=1 # debug
    return [0]+dqs

"""
#@
@name: find_accelerations
@brief: computes the acceleration of the trajectory in each time instant
@inputs: 
- list[float] dq: list of the motor velocities;
- list[float] ts: list of time instants;
@outputs: 
- list[float]: list of accelerations
@#
"""
def find_accelerations(dq: list[float], ts: list[float]) -> list[float]:
    ddqs = []
    for dq0, dq1, t0, t1 in zip(dq[:-1], dq[1:], ts[:-1], ts[1:]):
        ddq = dq1-dq0
        dt = t1-t0
        ddqs.append(ddq/dt)
    return [0]+ddqs
