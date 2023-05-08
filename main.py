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
    plt.close()

def debug_plotXY(x, y, name="image"):
    #print(q)
    plt.figure()
    plt.plot(x, y)
    plt.grid(visible=True)
    plt.savefig(name+'.png')
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
        if len(data) < 2: 
            raise Exception("Not Enough Points to build a Trajectory")
        q_list = []
        for point in data:
            q_list.append(tpy.ik(point['x'], point['y'], None, sizes))
        # DEBUG
        # print("List of q points: ", q_list)
        # END DEBUG
        trajectories = compute_trajectory(q_list) # get the trajectory for each motor
        q = ([], [])
        dq = ([], [])
        ddq = ([], [])
        for i in range(len(trajectories[0])):
            traj1, dt1 = trajectories[0][i] # first motor trajectory
            traj2, dt2 = trajectories[1][i] # second motor trajectory
            for t in tpy.rangef(0, settings['Tc'], max(dt1, dt2)):
                if t <= dt1:
                    q[0].append(traj1[0](t))
                    dq[0].append(traj1[1](t))
                    ddq[0].append(traj1[2](t))
                else:
                    q[0].append(q[0][-1])
                    dq[0].append(dq[0][-1])
                    ddq[0].append(ddq[0][-1])
                if t <= dt2:
                    q[1].append(traj2[0](t))
                    dq[1].append(traj2[1](t))
                    ddq[1].append(traj2[2](t))
                else:
                    q[1].append(q[1][-1])
                    dq[1].append(dq[1][-1])
                    ddq[1].append(ddq[1][-1])

        send_data('trj', q=q, dq=dq, ddq=ddq)
        trace_trajectory(q)
        # DEBUG
        debug_plot(q[0], 'q1')
        debug_plot(q[1], 'q2')
        # END DEBUG
    except Exception as e:
        print(e)
        print(traceback.format_exc())
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