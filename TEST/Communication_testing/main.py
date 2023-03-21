import eel
import serial

ser = None

web_options = {'host':'localhost', 'port':6969}

def ser_init():
    global ser 
    found = False
    for i in range(0,200):
        try:
            ser = serial.Serial(f"/dev/ttyS{i}", 115200, timeout=10000)  # open serial port
            found = True
            break
        except:
            print(f"/dev/ttyS{i} failed")
            found = False
    if found:
        print(f"{ser.name} works")         # check which port was really used
    else:
        print("No serial Port Found")
    return found


@eel.expose
def pylog(msg:str):
    print(msg)

@eel.expose
def write_serial(msg:str):
    global ser
    if len(msg) == 0:
        msg = "EMPTY\n";
    if msg[-1] != "\n":
        msg = msg + "\n"
    ser.write(bytes(msg,'utf-8'))          # write a string

@eel.expose
def read_serial():
    global ser
    line = 'NO MSG'
    ser.flush()
    line = ser.readline()
    line = str(line) # wait until \n -> blocking call
    print(f"Read {line}")
    return line

if __name__=="__main__":
    ser_started = ser_init()
    if ser_started:
        eel.init("./layout")
        eel.start("./index.html", host=web_options['host'], port=web_options['port'])
        ser.flush()
        ser.close()             # close port
    else:
        print("No serial could be found, stopping the application.")
        eel.init("./layout")
        eel.start("./index.html", host=web_options['host'], port=web_options['port'])
    