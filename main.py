import eel
import matplotlib.pyplot as plt
import numpy as np
from sys import stderr

from lib import trajpy as tpy
from lib import serial_com as scm

settings = {
    'Tc' : 1e-3, # s
    'acc_max' : 1.05, # rad/s**2
    'ser_started': False
}

sizes = {
    'l1': 0.25,
    'l2':0.25
}

web_options = {'host':'localhost', 'port':6969} # web server setup

def print_error(*args, **kwargs):
    print(*args, file=stderr, **kwargs)

def compute_trajectory(q_list: np.ndarray, ddqm=settings['acc_max']) -> tuple[list[tuple]]:
    q1 = tpy.compose_cycloidal([q[0] for q in q_list], ddqm) # trajectory of joint 1
    q2 = tpy.compose_cycloidal([q[1] for q in q_list], ddqm) # trajectory of joint 2
    return [q1, q2]

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

@eel.expose
def py_log(msg):
    print(msg)

@eel.expose
def py_get_data():
    data = eel.js_get_data()()
    q_list = []
    for point in data:
        q_list.append(tpy.ik(point['x'], point['y'], None, sizes))
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

@eel.expose
def py_serial_online():
    return settings['ser_started']


if __name__ == "__main__":
    settings['ser_started'] = scm.ser_init()
    if not settings['ser_started']:
        print("No serial could be found, stopping the application.")

    # GUI
    eel.init("./layout") # initialize the view
    eel.start("./index.html", host=web_options['host'], port=web_options['port']) # start the server

    if settings['ser_started']:
        ser.flush()
        ser.close()             # close port