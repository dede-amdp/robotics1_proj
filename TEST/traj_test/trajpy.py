from mat import *
from math import sqrt,atan2,cos,sin

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


def ik(x:float, y:float, theta:float, sizes:dict[float] = {'l1':0.25,'l2':0.25}) -> mat:
    q = mat.create(2,1)
    cos_q2 = (x**2+y**2-sizes['l1']**2-sizes['l2']**2)/(2*sizes['l1']*sizes['l2'])
    sin_q2 = sqrt(1-cos_q2**2)
    q[1,0] = atan2(sin_q2, cos_q2)
    q[0,0] = theta-q.data[1][0]
    return q

def dk(q:mat, sizes:dict[float] = {'l1':0.25,'l2':0.25})->mat:
    x = sizes['l1']*cos(q.data[0,0])+sizes['l2']*cos(q.data[0,0]+q.data[1,0])
    y = sizes['l1']*sin(q.data[0,0])+sizes['l2']*sin(q.data[0,0]+q.data[1,0])
    theta = q[0,0]+q[1,0]
    return mat([[x,y,theta]]).t()