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

'''
def compose_tragectory(q:list[float], method:function)->list[mat]:
    #max_speed = 1.05 # rad/s
    A = []
    dq0 = 0
    for k in range(len(q)-1):
        q1 = tuple(q[k]) # initial position of the joint
        q2 = tuple(q[k+1]) # final position of the joint
        q0 = q[k-1] if k-1 >= 0 else q[0]
        q3 = q[k+2] if k+2 < len(q) else q[-1]
        dq1 = 0 if k == len(q)-1 else 0 ## ???
        avg_speed = (dq1-dq0)/2 # average speed
        tf = (q2-q1)/avg_speed # a strategy to compute the time is using the avg speed
        a = method([(q1,0), (q2,tf)], [(dq0,0), (dq1,tf)]).t()
        dq0 = dq1
        A.append(a)
    return mat(A)
'''

def comp_spline3(q:list[float], ddqm:float = 1.05, dt:list = None)-> list[(mat, float)]:
    # q = q0+1/2 at**2 -> t = sqrt(2(q-q0)/a)
    A = []
    dq0 = 0
    dq2 = 0
    for k in range(len(q)-1):
        q0 = q[k-1] if k-1 >=0 else q[0]
        q1 = q[k]
        q2 = q[k+1]
        q3 = q[k+2] if k+2 < len(q) else q[-1]
        dt0 = dt[k-1] if k-1 >= 0 and dt is not None else sqrt(2*(q1-q0)/ddqm)
        dt1 = dt[k] if dt is not None else sqrt(2*(q2-q1)/ddqm) # if dt is not given, compute it as if the manipulator went at a constant acceleration
        dt2 = dt[k+1] if k+1 < len(q) and dt is not None else sqrt(2*(q3-q2)/ddqm)
        dq0 = dq2 if dt0 > 0 else 0
        dq2 = (q3-q2)/dt2 if dt2 > 0 else 0
        #dq1 = (dq0+dq2)/2
        a = spline3([(q1, 0), (q2, dt1)], [(dq0,0), (dq2, dt1)]).t()#, [(0,0),(0,dt1)]).t()
        A.append((a, dt1))
    return A

def trapezoidal(q:list[float], ddqm:float = 1.05, tf: float = None) -> list[(mat, float)]:
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
    first = (mat([[q[0]],[0],[0.5*ddqm]]).t(), tc) # (coefficients, duration)
    second = (mat([[qc], [ddqm*tc], [0]]).t(), tf-2*tc)
    third =  (mat([[qb],[ddqm*tc], [-0.5*ddqm]]).t(), tc)
    return [first, second, third]

def compose_trap(q:list[float], ddqm:float = 1.05) -> list[(mat, float)]:
    A = []
    for k in range(len(q)-1):
        q0 = q[k]
        q1 = q[k+1]
        qk = trapezoidal([q0, q1], ddqm)
        A += qk
    return A

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