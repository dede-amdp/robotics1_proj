import eel # GUI
import matplotlib.pyplot as plt # needed for plotting
import numpy as np # arrays
from math import cos, sin, tan
from sys import stderr # standard error stream
from signal import signal, SIGINT # close the serial even if the server is forced to close

from lib import trajpy as tpy # trajectory library
from lib import serial_com as scm # serial communication library


import traceback

settings = {
    'Tc' : 1e-2, # s
    'max_acc' : 1.05, #1.05, # rad/s**2
    'ser_started': False,
    'line_tl': lambda t, tf: tpy.cycloidal([0, 1], 2, tf)[0][0](t), # timing laws for line and circle segments
    'circle_tl': lambda t, tf: tpy.cycloidal([0, 1], 2, tf)[0][0](t) # lambda t, tf: t/tf
}

sizes = {
    'l1': 0.25,
    'l2': 0.25
}

web_options = {'host':'localhost', 'port':6969} # web server setup

def print_error(*args, **kwargs):
    print(*args, file=stderr, **kwargs)

def handle_closure(sig, frame):
    print("Closing Serial...")
    if settings['ser_started']:
        scm.serial_close()
        settings['ser_started'] = False
    exit(1)

def compute_trajectory(q_list: np.ndarray, method = tpy.compose_cycloidal, ddqm=settings['max_acc']) -> list[list[tuple]]:
    q1 = method([q[0] for q in q_list], ddqm) # trajectory of joint 1
    q2 = method([q[1] for q in q_list], ddqm) # trajectory of joint 2
    q3 = [q[2] for q in q_list] # pen up or pen down ?
    return [q1, q2, q3]

def debug_plot(q, name="image"):
    #print(q)
    plt.figure()
    t = [i*settings['Tc'] for i in range(len(q))]
    plt.plot(t, q)
    plt.grid(visible=True)
    plt.savefig('images/'+name+'.png')
    plt.close()

def debug_plotXY(x, y, name="image"):
    #print(q)
    plt.figure()
    plt.plot(x, y)
    plt.grid(visible=True)
    plt.savefig('images/'+name+'.png')
    plt.close()


def send_data(msg_type: str, **data):
    match msg_type:
        case 'trj':
            if ('q' not in data) or ('dq' not in data) or ('ddq' not in data):
                print_error("Not enough data to define the trajectory")
                return # no action is done because this is a failing state
            msg_str = f"TRJ:0:{','.join(map(str, data['q'][0]))}:{','.join(map(str, data['dq'][0]))}:{','.join(map(str, data['ddq'][0]))}"+\
                        f":1:{','.join(map(str, data['q'][1]))}:{','.join(map(str, data['dq'][1]))}:{','.join(map(str, data['ddq'][1]))}\n" # TODO: ADD THE CORRECT MESSAGE FORMATTING
            #print(msg_str)
            scm.write_serial(msg_str)

def trace_trajectory(q:tuple[list,list]):
    q1 = q[0][:]
    q2 = q[1][:]
    eel.js_draw_traces([q1, q2])
    eel.js_draw_pose([q1[-1], q2[-1]])

    # DEBUG
    x = [] # [tpy.dk([q1t, q2t]) for q1t, q2t in zip(q1, q2)]
    for i in range(len(q1)):
        x.append(tpy.dk(np.array([q1[i], q2[i]]).T))
    debug_plotXY([xt[0] for xt in x], [yt[1] for yt in x], "xy")
    # END DEBUG


@eel.expose
def py_log(msg):
    print(msg)

@eel.expose
def py_get_data():
    try:
        data = eel.js_get_data()()
        if len(data) < 1: 
            raise Exception("Not Enough Points to build a Trajectory")
        # data contains the trajectory patches to stitch together
        # trajectory = {'type', 'points', 'data'}
        # example:
        # line_t = {'type':'line', 'points': [p0, p1], 'data':[penup]}
        # circle_t = {'type':'circle', 'points': [a, b], 'data':[center, radius, penup, ...]}
        q0s = []
        q1s = []
        penups = []
        ts = []
        for patch in data: 
            (q0s_p, q1s_p, penups_p, ts_p) = tpy.slice_trj( patch, 
                                                    Tc=settings['Tc'],
                                                    max_acc=settings['max_acc'],
                                                    line=settings['line_tl'],
                                                    circle=settings['circle_tl'],
                                                    sizes=sizes) # returns a tuple of points given a timing law for the line and for the circle
            q0s += q0s_p if len(q0s) == 0 else q0s_p[1:] # for each adjacent patch, the last and first values coincide, so ignore the first value of the next patch to avoid singularities
            q1s += q1s_p if len(q1s) == 0 else q1s_p[1:] # ignoring the starting and ending values of consecutive patches avoids diverging accelerations
            penups += penups_p if len(penups) == 0 else penups_p[1:]
            ts += [(t + ts[-1] if len(ts) > 0  else t) for t in (ts_p if len(ts) == 0 else ts_p[1:])] # each trajectory starts from 0: the i-th patch has to start in reality from the (i-1)-th final time instant
        
        q = (q0s, q1s, penups)
        dq = (tpy.find_velocities(q[0], ts), tpy.find_velocities(q[1], ts))
        ddq = (tpy.find_accelerations(dq[0], ts), tpy.find_accelerations(dq[1], ts))
        send_data('trj', q=q, dq=dq, ddq=ddq)
        trace_trajectory(q)
        # DEBUG
        debug_plot(q[0], 'q1')
        debug_plot(dq[0], 'dq1')
        debug_plot(ddq[0], 'ddq1')
        debug_plot(q[1], 'q2')
        debug_plot(dq[1], 'dq2')
        debug_plot(ddq[1], 'ddq2')
        # END DEBUG

    except Exception as e:
        print(e)
        print(traceback.format_exc())
        pass # do not do anything if the given points are not enough for a trajectory

@eel.expose
def py_serial_online():
    return settings['ser_started'] # return whether the serial is started or not

@eel.expose
def py_serial_startup():
    scm.ser_init()

signal(SIGINT, handle_closure) # ensures that the serial is closed 

if __name__ == "__main__":
    global ser
    settings['ser_started'] = scm.ser_init()
    if not settings['ser_started']:
        print("No serial could be found, stopping the application.")

    # GUI
    eel.init("./layout") # initialize the view
    eel.start("./index.html", host=web_options['host'], port=web_options['port']) # start the server

    scm.serial_close() # once the server stops, close the serial