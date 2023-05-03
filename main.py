import eel # GUI
import matplotlib.pyplot as plt # needed for plotting
import numpy as np # arrays
from math import cos, sin, tan
from sys import stderr # standard error stream
from signal import signal, SIGINT # close the serial even if the server is forced to close

from lib import trajpy as tpy # trajectory library
from lib import serial_com as scm # serial communication library

settings = {
    'Tc' : 1e-3, # s
    'acc_max' : 1.05, # rad/s**2
    'ser_started': False
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

def compute_trajectory(q_list: np.ndarray, ddqm=settings['acc_max']) -> tuple[list[tuple]]:
    q1 = tpy.compose_cycloidal([q[0] for q in q_list], ddqm) # trajectory of joint 1
    q2 = tpy.compose_cycloidal([q[1] for q in q_list], ddqm) # trajectory of joint 2
    return [q1, q2]

def debug_plot(q, name="image"):
    #print(q)
    plt.figure()
    plt.plot(q)
    plt.grid(visible=True)
    plt.savefig(name+'.png')


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
    # pointsq1 = []
    # pointsq2 = []
    q1 = q[0][:]
    q2 = q[1][:]
    n1 = len(q[0])
    n2 = len(q[1])
    # the trajectory may not last the same amount of time:
    # add data to the shortest trajectory to make them compatible
    if n1 < n2:
        for _ in range(n1,n2):
            q1.append(q1[-1])
    else:
        for _ in range(n2,n1):
            q2.append(q2[-1])
    '''
    for qt1, qt2 in zip(q1, q2):
        x1 = cos(qt1)*sizes['l1']
        y1 = sin(qt1)*sizes['l1']
        x2 = x1+cos(qt1+qt2)*sizes['l2']
        y2 = y1+sin(qt1+qt2)*sizes['l2']
        pointsq1.append({'x':x1, 'y':y1})
        pointsq2.append({'x':x2, 'y':y2})
    
    print(len(pointsq1)*1e-3)

    eel.js_draw_traces(pointsq1, '#0000FF')
    eel.js_draw_traces(pointsq2, '#00FF00')
    eel.js_draw_pose([q[0][-1], q[1][-1]])
    '''
    eel.js_draw_traces([q1, q2])
    eel.js_draw_pose([q[0][-1], q[1][-1]])


@eel.expose
def py_log(msg):
    print(msg)

@eel.expose
def py_get_data():
    try:
        data = eel.js_get_data()()
        if len(data) < 2: 
            raise Exception("Not Enough Points to build a Trajectory")
        q_list = []
        for point in data:
            q_list.append(tpy.ik(point['x'], point['y'], None, sizes))
        # DEBUG
        print(q_list)
        for q1,q2 in q_list:
            eel.js_draw_pose([q1[-1], q2[-1]])
        # END DEBUG
        trajectories = compute_trajectory(q_list) # get the trajectory for each motor
        q = ([], [])
        dq = ([], [])
        ddq = ([], [])
        for i in range(2):
            trajectory = trajectories[i]
            for traj,dt in trajectory: # for each trajectory section in the trajectory of the i-th motor
                for t in tpy.rangef(0, settings['Tc'], dt):
                    q[i].append(traj[0](t)[0]) # compute the position
                    dq[i].append(traj[1](t)[0]) # compute the speed
                    ddq[i].append(traj[2](t)[0]) # compute the acceleration
        send_data('trj', q=q, dq=dq, ddq=ddq)
        trace_trajectory(q)
        # DEBUG
        debug_plot(q[0], 'q1')
        debug_plot(q[1], 'q2')
        # END DEBUG
    except:
        pass # do not do anything if the given points are not enough for a trajectory

@eel.expose
def py_serial_online():
    return settings['ser_started']

@eel.expose
def py_serial_startup():
    scm.ser_init()

signal(SIGINT, handle_closure)

if __name__ == "__main__":
    global ser
    settings['ser_started'] = scm.ser_init()
    if not settings['ser_started']:
        print("No serial could be found, stopping the application.")

    # GUI
    eel.init("./layout") # initialize the view
    eel.start("./index.html", host=web_options['host'], port=web_options['port']) # start the server

    scm.serial_close() # once the server stops, close the serial