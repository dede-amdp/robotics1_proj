import serial

ser = None # serial object

def ser_init(serial_path:str=None) -> bool:
    global ser 
    print("Starting Serial Connection:\n")
    found = False
    if serial_path is None:
        ports = ["/dev/tty.usbmodem11203","/dev/tty.usbmodem1203"]+[f"/dev/ttyS{i}" for i in range(10)]+[f"/dev/tty{i}" for i in range(8,100)] + [f"COM{i}" for i in range(10)]
        for port in ports:
            try:
                ser = serial.Serial(f"{port}", 115200, timeout=10000)  # open serial port
                found = True
                break
            except Exception as e:
                print(f"{port} failed\n")
        if found:
            print(f"{ser.name} works\n")         # check which port was really used
        else:
            print("No serial Port Found\n")
    else:
        try:
            ser = serial.Serial(serial_path, 115200, timeout=10000)  # open serial port
            found = True
        except:
            print(f"{serial_path} failed\n")
            found = False
    return found

def write_serial(msg:str) -> bool:
    global ser
    if ser is None: return False
    if len(msg) == 0:
        msg = "EMPTY\n"
    #if msg[-1] != "\n":
    #    msg = msg + "\n"
    ser.write(bytes(msg,'utf-8'))          # write a string
    # print(f"Written {len(msg)} to serial")
    return True

def read_serial() -> bytes:
    global ser
    if ser is None: return None
    line = 'NO MSG'
    ser.flush()
    line = ser.readline()
    #line=ser.read(22)
    print(ser.read_all())
    # line=ser.readline()
    # line = str(line) # wait until \n -> blocking call
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    
    print(ser.read_all())

    return line

def serial_close():
    global ser
    ser.flush()
    ser.close() # close port