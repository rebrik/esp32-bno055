# from __future__ import print_function
import serial
import time
import vtpDrawScene as vds

scene = vds.vtpDrawScene()
scene.initScene()

ser = serial.Serial()
ser.port = "/dev/ttyUSB0"  

ser.baudrate = 115200
ser.bytesize = serial.EIGHTBITS
ser.parity = serial.PARITY_NONE
ser.stopbits = serial.STOPBITS_ONE
ser.timeout = .2 # Non-Block reading
ser.xonxoff = False # Disable Software Flow Control
ser.rtscts = False # Disable (RTS/CTS) flow Control
ser.dsrdtr = False # Disable (DSR/DTR) flow Control
ser.writeTimeout = 2
ser.exclusive = True

ser.open()
time.sleep(0.2)

ser.flushOutput()
ser.flushInput()

while True :
    line = ser.readline()
    try:
        linestr = line.decode('ascii')
    except UnicodeDecodeError:
        linestr = ''
        continue

    print( linestr, end='')  
    try:
        floatLst = [float(splits) for splits in linestr.split("\t") if splits != ""]
        qtr= tuple(floatLst[2:6]) # skip time
        scene.SetQuatOrientation(qtr)
    except ValueError:
        continue
    except TypeError:
        continue

ser.close()
del scene

