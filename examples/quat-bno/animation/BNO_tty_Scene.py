# from __future__ import print_function
import serial
import time
import vtpDrawScene as vds

scene = vds.vtpDrawScene()
scene.initScene()


ser = serial.Serial()
ser.port = "/dev/ttyUSB0"  # Arduino Nano
#ser.port = "/dev/ttyACM0" # Arduino Uno
# If it breaks try the below
#self.serConf() # Uncomment lines here till it works
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
ser.flushInput()
ser.flushOutput()

time.sleep(0.5)

# skip the header
while True :
    line = ser.readline()
    print line,
    if line.startswith( 'endheader' ):
        print ser.readline(),
        break

i = 1    
while True :
    line = ser.readline()
    print i, line, 
    floatLst = [float(splits) for splits in line.split("\t") if splits != ""]
    qtr= tuple(floatLst[2:6]) # skip time
    scene.SetQuatOrientation(qtr)
    i = i+1

ser.close()
del scene

