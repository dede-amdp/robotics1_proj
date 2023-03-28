import eel
import matplotlib.pyplot as plt
#from trajpy import *
#from mat import *
#from math import atan2, pi

import sys
sys.path.insert(1, '../../lib/')
from mat import *
from trajpy import *


web_options = {'host':'localhost', 'port':6969}

@eel.expose
def pyget_data():
    data_points = eel.jsget_points()()
    q = []
    for p in data_points:
        qt = ik(p['x'], p['y'])
        q.append(qt)
        eel.jslog(str(p))
        eel.jslog(f'q: [{qt[0,0]}, {qt[1,0]}]')
        eel.jslog(f'p: [{0.25*cos(qt[0,0])+0.25*cos(qt[0,0]+qt[1,0])},{0.25*sin(qt[0,0])+0.25*sin(qt[0,0]+qt[1,0])}]')
    eel.jsdraw_pose(q[-1][:,0])
    #print([str(r[0])+','+str(r[1]) for r in comp_spline3([qk[0,0] for qk in q])])
    A = compose_trap([qk[0,0] for qk in q])
    #print([str(r[0])+','+str(r[1]) for r in A])
    draw_traj(A,2)
    # scrivi la funzione js che prenda direttamente la dk ?
    #eel.jsdraw_pose([pi/4, pi/4])
    pass

def draw_traj(A:list[tuple], degree:int=3)-> None:
    dts = [tr[1] for tr in A]
    a = [tr[0] for tr in A]
    qtr = []
    dqtr = []
    ddqtr = []
    total_time = 0
    first = True
    for dt,coeff in zip(dts,a):
        r = []
        if first:
            r = rangef(0.01,0.01,dt)
            first = False
        else:
            r = rangef(0,0.01,dt)
        for t in r:
            time = mat([time_row(t, degree, 0)]).t()
            qtr.append(coeff.dot(time)[0,0])
            time = mat([time_row(t, degree, 1)]).t()
            dqtr.append(coeff.dot(time)[0,0])
            time = mat([time_row(t, degree, 2)]).t()
            ddqtr.append(coeff.dot(time)[0,0])
            total_time += 0.01
    plt.plot(rangef(0, 0.01, total_time), qtr)
    plt.plot(rangef(0, 0.01, total_time), dqtr)
    plt.plot(rangef(0, 0.01, total_time), ddqtr)
    plt.legend(['q','dq', 'ddq'])
    plt.grid(visible=True)
    #plt.show()
    plt.savefig("./plot.png")
    plt.close()

if __name__ == "__main__":
    eel.init("./layout")
    eel.start("./index.html", host=web_options['host'], port=web_options['port'])