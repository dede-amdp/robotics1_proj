import eel # GUI
import math
import matplotlib.pyplot as plt # needed for plotting
import numpy as np # arrays
from time import sleep as tsleep
from math import cos, sin, tan, pi
from struct import pack, unpack
from binascii import unhexlify

from sys import stderr # standard error stream
from signal import signal, SIGINT # close the serial even if the server is forced to close

from lib import trajpy as tpy # trajectory library
from lib import serial_com as scm # serial communication library


import traceback

settings = {
    'Tc' : 0.01, # s
    'data_rate': 1/100, # rate at which msgs are sent
    'max_acc' : 0.1,#1.05, #1.05, # rad/s**2
    'ser_started': False,
    'line_tl': lambda t, tf: tpy.cycloidal([0, 1], 2, tf)[0][0](t), # timing laws for line and circle segments
    'circle_tl': lambda t, tf: tpy.cycloidal([0, 1], 2, tf)[0][0](t) # lambda t, tf: t/tf
}

sizes = {
    'l1': 0.170,
    'l2': 0.158
}

log_data = {
    'time': [],         # time stamp
    'q0': [],           # desired q0
    'q1': [],           # desired q1
    'dq0': [],          # desired dq0
    'dq1': [],          # desired dq1
    'ddq0': [],         # desired ddq0
    'ddq1': [],         # desired ddq1
    'q0_actual': [],    # actual q0
    'q1_actual': [],    # actual q1
    'dq0_actual': [],   # actual dq0
    'dq1_actual': [],   # actual dq1
    'ddq0_actual': [],  # actual ddq0
    'ddq1_actual': [],  # actual ddq1
    'x': [],            # desired x position
    'y': [],            # desired y position
    'x_actual': [],     # actual x position
    'y_actual': []      # actual y position
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

'''
def compute_trajectory(q_list: np.ndarray, method = tpy.compose_cycloidal, ddqm=settings['max_acc']) -> list[list[tuple]]:
    q1 = method([q[0] for q in q_list], ddqm) # trajectory of joint 1
    q2 = method([q[1] for q in q_list], ddqm) # trajectory of joint 2
    q3 = [q[2] for q in q_list] # pen up or pen down ?
    return [q1, q2, q3]
'''

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


def d2h(d: float) -> str: # double to hex
    # < = little endian
    # Q = long (double)
    # d = double
    # check: https://docs.python.org/2/library/struct.html
    return hex(unpack('<Q', pack('<d', d))[0]).ljust(18,"0")

def h2d(string: str) -> float: # hex to double
    return unpack('>f', unhexlify(string))[0]


"""
#@
@name: send_data
@brief: sends data to the micro controller
@inputs: 
- str msg_type: type of message to send : "trj" is used for trajectory data;
- any list \*\*data: any number of lists (containing the position setpoints to send in case of trj data);
@outputs: 
- None;
@#
"""
def send_data(msg_type: str, **data):
    match msg_type:
        case 'trj':
            if ('q' not in data) or ('dq' not in data) or ('ddq' not in data):
                print_error("Not enough data to define the trajectory")
                return # no action is done because this is a failing state
            # msg_str = f"TRJ:0:{','.join(map(str, data['q'][0]))}:{','.join(map(str, data['dq'][0]))}:{','.join(map(str, data['ddq'][0]))}"+\
            #            f":1:{','.join(map(str, data['q'][1]))}:{','.join(map(str, data['dq'][1]))}:{','.join(map(str, data['ddq'][1]))}\n" 
            #print(msg_str)

            for i in range(len(data['q'][0])):
                # data is converted to hex so that it always uses the same number of characters
                # [2:] is used to remove "0x" from the string and save 2 chars per value
                # msg_str = f"TRJ:{d2h(data['q'][0][i])[2:]}:{d2h(data['q'][1][i])[2:]}"+\
                #             f":{d2h(data['dq'][0][i])[2:]}:{d2h(data['dq'][1][i])[2:]}"+\
                #             f":{d2h(data['ddq'][0][i])[2:]}:{d2h(data['ddq'][1][i])[2:]}"+\
                #             f":{int(data['q'][2][i])}\n"
                
                msg_str = f"TRJ:{d2h(data['q'][0][i])}:{d2h(data['q'][1][i])}"+\
                            f":{d2h(data['dq'][0][i])}:{d2h(data['dq'][1][i])}"+\
                            f":{d2h(data['ddq'][0][i])}:{d2h(data['ddq'][1][i])}"+\
                            f":{int(data['q'][2][i])}\n"

                scm.write_serial(msg_str) # send data through the serial com
                #print(msg_str)

                '''
                pos = tpy.dk([data['q'][0][i], data['q'][1][i]], sizes)
                log(
                    time=i*settings['Tc'],
                    q0=data['q'][0][i],
                    q1=data['q'][1][i],
                    dq0=data['dq'][0][i],
                    dq1=data['dq'][1][i],
                    ddq0=data['ddq'][0][i],
                    ddq1=data['ddq'][1][i],
                    x=pos[0],
                    y=pos[1]
                )'''
                # TODO: when reading the actual data from the manipulator, update the remaining data
                # it does not matter if the update is done in another moment, it still refers to the time when the reference signal is applied
                tsleep(settings['data_rate']) # regulate the speed at which data is sent
            print(f"TRJ SENT,{len(data['q'][0])}") # DEBUG
            # print(scm.read_serial().decode('utf-8')) # DEBUG

"""
#@
@name: trace_trajectory
@brief: draws the trajectories on the GUI
@inputs: 
- tuple[list, list] q: a tuple containing the list of positions for each motor;
@outputs: 
- None;
@#
"""
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


"""
#@
@name: eel.expose py_log
@brief: simply prints a message on the python console
@inputs: 
- str msg: message to be print;
@outputs: 
- None;
@#
"""
@eel.expose
def py_log(msg):
    print(msg)

"""
#@
@name: eel.expose py_get_data 
@brief: gets the trajectory data from the web GUI and converts it into a list of setpoints to be sent to the micro controller
@inputs: 
- None;
@outputs: 
- None;
@#
"""
@eel.expose
def py_get_data():

    # local method to interpret the message read on the serial com
    def read_position() -> list[float]:
        scm.ser.reset_input_buffer()
        scm.ser.reset_output_buffer()
        
        #scm.write_serial('POS:'+('0'*18+':')*6+'0\n')
        
        #string: str = str(scm.read_serial()) # the msg structure: "0x00000000:0x00000000" => "(hex) q0:(hex) q1"
        print(string)
        string=string.replace("b",'',1).replace("'",'').replace("\\n",'')
        values: list[str] = string.replace('0x', '').split(":")
        #q = [h2d(values[0]), h2d(values[1])]
        q=(0,0)
        points = tpy.dk(np.array(q), sizes)
        return [points[0,0], points[1,0]]

    try:
        data: list = eel.js_get_data()()
        # add an initial patch to move the manipulator to the correct starting position
        if len(data) < 1: 
            raise Exception("Not Enough Points to build a Trajectory")
        #current_q = [0.0,math.pi/2]  #before 0,0
        q=(0,0)
        points = tpy.dk(np.array(q), sizes)
        current_q = [points[0,0], points[1,0]]
        
        print(current_q)
        data = [{'type':'line', 'points':[current_q, data[0]['points'][0]], 'data':{'penup':True}}] + data[::]
        
        # data contains the trajectory patches to stitch together
        # trajectory = {'type', 'points', 'data'}
        # example:
        # line_t = {'type':'line', 'points': [p0, p1], 'data':[penup]}
        # circle_t = {'type':'circle', 'points': [a, b], 'data':[center, radius, penup, ...]}
        # where 'points' will be an array of arrays of size 2 (both the motors' positions)
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

def log(**data):
    global log_data
    for key in data: log_data[key].append(data[key])

@eel.expose
def py_log_data():
    content = '' # contents of the file
    for key in log_data: content+=key+',' # add the first row (the legend)
    content = content[:len(content-1)]+'\n' # remove the last comma and add '\n'
    for t in len(log_data['time']):
        row = ''
        for key in log_data:
            row += str(log_data[key]) + ','
        row = row[:len(row)-1]
        content += row + '\n'
    with open('log_data.csv', 'w') as file:
        file.write(content)
        file.close() # this is unnecessary because the with statement handles it already, but better safe than sorry

"""
#@
@name: eel.expose py_homing_cmd
@brief: sends the homing command to the micro controller
@inputs: 
- None;
@outputs: 
- None;
@#
"""
@eel.expose
def py_homing_cmd():
    # send the homing command 
    scm.write_serial('HOM:'+('0'*18+':')*6+'0\n')


"""
#@
@name: eel.expose py_serial_online
@brief: return whether the serial is online or not
@inputs: 
- None;
@outputs: 
- bool: bool value that shows if the serial is online or not;
@#
"""
@eel.expose
def py_serial_online():
    return settings['ser_started'] # return whether the serial is started or not

"""
#@
@name: eel.expose py_serial_sartup
@brief: initializes the serial communication
@inputs: 
- None;
@outputs: 
- None;
@#
"""
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