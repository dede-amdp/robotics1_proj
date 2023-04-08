import serial

ser = None # serial object

def ser_init(serial_path:str=None) -> bool:
    global ser 
    found = False
    if serial_path is None:
        print("Starting Serial Connection:")
        for i in range(0,4):
            try:
                ser = serial.Serial(f"/dev/ttyS{i}", 115200, timeout=10000)  # open serial port
                found = True
                break
            except Exception as e:
                print(e)
                print(f"/dev/ttyS{i} failed")
                found = False
        for i in range(0,200):
            try:
                ser = serial.Serial(f"/dev/tty{i}", 115200, timeout=10000)  # open serial port
                found = True
                break
            except Exception as e:
                print(e)
                print(f"/dev/ttyS{i} failed")
                found = False
        if found:
            print(f"{ser.name} works")         # check which port was really used
        else:
            print("No serial Port Found")
    else:
        try:
            ser = serial.Serial(serial_path, 115200, timeout=10000)  # open serial port
            found = True
        except:
            print(f"{serial_path} failed")
            found = False
    return found

def write_serial(msg:str) -> bool:
    global ser
    if ser is None: return False
    if len(msg) == 0:
        msg = "EMPTY\n";
    if msg[-1] != "\n":
        msg = msg + "\n"
    ser.write(bytes(msg,'utf-8'))          # write a string
    return True

def read_serial() -> str:
    global ser
    if ser is None: return None
    line = 'NO MSG'
    ser.flush()
    line = ser.readline()
    line = str(line) # wait until \n -> blocking call
    print(f"Read {line}")
    return line