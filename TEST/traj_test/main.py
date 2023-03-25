import eel
#import matplotlib.pyplot as plt
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
    print([str(r[0])+','+str(r[1]) for r in compose_trap([qk[0,0] for qk in q])])
    # scrivi la funzione js che prenda direttamente la dk ?
    #eel.jsdraw_pose([pi/4, pi/4])
    pass


if __name__ == "__main__":
    eel.init("./layout")
    eel.start("./index.html", host=web_options['host'], port=web_options['port'])