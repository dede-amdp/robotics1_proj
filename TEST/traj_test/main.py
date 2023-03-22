import eel
import matplotlib.pyplot as plt
from trajpy import *
from mat import *


web_options = {'host':'localhost', 'port':6969}

@eel.expose
def pyget_data():
    data_points = eel.jsget_points()()
    q = []
    for p in data_points:
        qt = ik(p['x'], p['y'],0)
        q.append(qt)
        eel.jslog(str(p))
        eel.jslog('q:'+str(qt))
    pass



if __name__ == "__main__":
    eel.init("./layout")
    eel.start("./index.html", host=web_options['host'], port=web_options['port'])
    '''
    PI = 3.14
    a = spline5([(0,0), (PI,4)], [(0,0), (0,4)], [(0,0),(0,4)])
    q = []
    dq = []
    ddq = []
    time = []
    for it in rangef(0,0.1,2+0.1):
        t = mat([time_row(it, 5, 0)])
        dt = mat([time_row(it,5,1)])
        ddt = mat([time_row(it,5,2)])
        q.append(t.dot(a).data[0][0])
        dq.append(dt.dot(a).data[0][0])
        ddq.append(ddt.dot(a).data[0][0])
        time.append(it)

    plt.plot(time, q, time, dq, time, ddq)
    plt.grid(visible=True)
    plt.legend(['q','dq','ddq'])
    plt.show()
    '''

'''
TODO:
- check if ik is correct, it gives sus values :/
'''