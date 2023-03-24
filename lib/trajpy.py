from mat import *
from math import sqrt,atan2,cos,sin,pi,acos

point_time = tuple[float, float]

def time_row(t:float, deg: int, der:int) -> list[float]:
    row = []
    for i in range(0, deg+1):
        match der:
            case 0: row.append(t**i)
            case 1: row.append(i*t**(i-1) if i-1 >= 0 else 0)
            case 2: row.append(i*(i-1)*t**(i-2) if i-2 >= 0 else 0)
    return row


def spline3(q: list[point_time], dq: list[point_time])->mat: #, ddq: list[point_time])-> list:
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
    A = mat(vandermont)
    b = mat(known_terms)
    return A.inv().dot(b)

def spline5(q: list[point_time], dq: list[point_time], ddq: list[point_time])->mat:
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
    A = mat(vandermont)
    b = mat(known_terms)
    return A.inv().dot(b)

def rangef(start:float=0, step:float=1, end:float=0) -> list:
    if step == 0: return []
    if start >= end: return []
    if start < end and step < 0: return []
    if start > end and step > 0: return []
    r = []
    i = start
    while i < end:
        r.append(i)
        i += step
    return r

def compose_tragectory(q:list[mat], method:function)->mat:
    max_speed = 1.05 # rad/s
    for i in q[:len(q)-1]:
        for j in q[1:len(q)]:
            q1 = tuple(i.t().data[0]) # initial and final positions of the first joint
            q2 = tuple(j.t().data[0]) # initial and final positions of the second joint
            pass


def ik(x:float, y:float, theta:float = None, sizes:dict[float] = {'l1':0.25,'l2':0.25}) -> mat:
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
    
    q = mat([[q1,q2]]).t()
    return q

def dk(q:mat, sizes:dict[float] = {'l1':0.25,'l2':0.25})->mat:
    x = sizes['l1']*cos(q[0,0])+sizes['l2']*cos(q[0,0]+q[1,0])
    y = sizes['l1']*sin(q[0,0])+sizes['l2']*sin(q[0,0]+q[1,0])
    theta = q[0,0]+q[1,0]
    return mat([[x,y,theta]]).t()


# TODO:
'''
Create a minimum time trajectory and use it to compose the trajectory in the method compose_trajectory:
consider the maximum speed 1.05 rad/s, because it is the speed reached by going at the maximum acceleration (1.05 rad/s^2) for exactly 1 second
'''