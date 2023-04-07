from math import sqrt,atan2,cos,sin,pi,acos
import numpy as np
from typing import Callable

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
- np.ndarray : numpy array of the coefficients of the 3rd order polynomial that crosses the specified points.
@# """
def spline3(q: list[point_time], dq: list[point_time])->np.ndarray: #, ddq: list[point_time])-> list:
    '''
    q = a0+a1t+a2t2+a3t3
    dq = a1+2a2t+3a3t2
    ddq = 2a2+6a3t
    '''
    vandermont = []
    for point in q:
        vandermont.append(time_row(point[1],3,0))
    for point in dq:
        vandermont.append(time_row(point[1],3,1))
    '''for point in ddq:
        vandermont.append(time_row(point[1],3,2))'''
    known_terms = [[point[0]] for point in q+dq] #+ddq]
    A = np.array(vandermont) 
    b = np.array(known_terms) 
    return np.dot(np.linalg.inv(A), b)


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
- np.ndarray : numpy array of the coefficients of the 5th order polynomial that crosses the specified points.
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
@inputs: inputs
@outputs: outputs
@# """
def compose_spline3(q:list[float], ddqm:float = 1.05, dts:list=None)->list[tuple[np.ndarray, float]]:
    q = preprocess(q)
    A = []
    tf = 0
    if dts is None:
        dts = []
        # ?? VALORE EMPIRICO *2 -> COME LO SCEGLIAMO DT?
        tf = sqrt((4*abs(q[-1]-q[0]))/abs(ddqm)) # time of a bang bang profile from start to finish * 2
        l = 0
        for i,j in zip(q[0:len(q)-1], q[1:]): l += abs(j-i)
        for i,j in zip(q[0:len(q)-1], q[1:]): dts.append(tf*4*abs(j-i)/l)
    else:
        for t in dts: tf+=t
    dqs = cubic_speeds(q, dts)
    for i, dt in zip(range(len(q)-1), dts):
        qi = q[i]
        qj = q[i+1]
        a = spline3([(qi,0), (qj, dt)], [(dqs[i], 0), (dqs[i+1], dt)])
        A.append((a.T, dt))
    return A

def cubic_speeds(q: list[float], dts: list[float]) -> list[float]:
    if len(q) < 3 : return [0, 0]
    A = np.zeros((len(q)-2, len(q)-2))
    c = np.zeros((len(q)-2, 1))
    for i in range(len(q)-2):
        A[i,i] += 2*(dts[i]+dts[i+1])
        if i+1 < A.m:
            A[i, i+1] += dts[i] 
        if i-1 >= 0:
            A[i, i-1] += dts[i+1]
    #print(A)
    deltaq = []
    for qi,qj in zip(q[:len(q)-1], q[1:]):
        deltaq.append(qj-qi)
    for k in range(1,len(q)-2):
        c[k-1,0] = 3*(dts[k]**2*deltaq[k+1]+dts[k+1]**2+deltaq[k])/(dts[k]*dts[k+1])
    #print(c)
    # the initial and final values of the speeds are not summed because they are 0
    dq = A.inv().dot(c)
    return [0]+dq.t().data[0]+[0]

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

def trapezoidal(q:list[float], ddqm:float = 1.05, tf: float = None) -> list[tuple[np.ndarray, float]]:
    # abs(ddqm) >= 4*abs(q[1]-q[0])/tf**2
    # tf**2/(4*abs(q[1]-q[0])) >= 1/abs(ddqm)
    # tf >= +sqrt((4*abs(q[1]-q[0]))/abs(ddqm))
    tc = 0
    if tf is None:
        tf = sqrt((4*abs(q[1]-q[0]))/abs(ddqm)) # bang bang profile
        tc = tf/2
    else:
        if abs(ddqm) < 4*abs(q[1]-q[0])/tf**2:
            print("This trajectory is not actuable with the specified acceleration:\nchoose a bigger acceleration value")
            return None
        tc = tf/2-sqrt((ddqm*tf)**2-4*ddqm*(q[1]-q[0]))/(2*ddqm)
    qc = q[0] + 0.5*ddqm*tc**2
    qb = qc+ddqm*tc*(tf-2*tc)
    first = (np.array([[q[0]],[0],[0.5*ddqm]]).T,tc) # (coefficients, duration)
    second = (np.array([[qc], [ddqm*tc], [0]]).T, tf-2*tc)
    third = (np.array([[qb],[ddqm*tc], [-0.5*ddqm]]).T, tc)
    return [first, second, third]

def compose_trapezoidal(q:list[float], ddqm:float = 1.05) -> list[tuple[np.ndarray, float]]:
    A = []
    for k in range(len(q)-1):
        q0 = q[k]
        q1 = q[k+1]
        qk = trapezoidal([q0, q1], ddqm)
        A += qk
    return A

def cycloidal(q:list[float], ddqm:float = 1.05, tf:float=None) -> tuple[list[function], float]: # return the function handles for q, dq and ddq
    if tf is None:
        tf = sqrt(2*pi*abs(q[1]-q[0])/ddqm)
    qt = lambda t: q[0]+(q[1]-q[0])*(t/tf-sin(2*pi*t/tf)/(2*pi))
    dqt = lambda t: (q[1]-q[0])*(1-cos(2*pi*t/tf))/tf
    ddqt = lambda t: 2*pi*(q[1]-q[0])*sin(2*pi*t/tf)/(tf**2)
    return ([qt, dqt, ddqt], tf)

def compose_cycloidal(q:list[float], ddqm:float = 1.05) -> list[tuple[list[function], float]]:
    A = []
    for k in range(len(q)-1):
        q0 = q[k]
        q1 = q[k+1]
        qk = cycloidal([q0,q1], ddqm)
        A.append(qk)
    return A

def ik(x:float, y:float, theta:float = None, sizes:dict[float] = {'l1':0.25,'l2':0.25}) -> np.ndarray:
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
    
    q = np.array([[q1,q2]]).T
    return q

def dk(q:np.ndarray, sizes:dict[float] = {'l1':0.25,'l2':0.25})->np.ndarray:
    x = sizes['l1']*cos(q[0,0])+sizes['l2']*cos(q[0,0]+q[1,0])
    y = sizes['l1']*sin(q[0,0])+sizes['l2']*sin(q[0,0]+q[1,0])
    theta = q[0,0]+q[1,0]
    return np.array([[x,y,theta]]).T

