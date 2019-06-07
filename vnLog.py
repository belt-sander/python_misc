import serial
import time
import csv

ser = serial.Serial()
ser.port = "/dev/ttyUSB0"
ser.baudrate = 115200
ser.bytesize = serial.EIGHTBITS         #number of bits per bytes
ser.parity = serial.PARITY_NONE         #set parity check: no parity
ser.stopbits = serial.STOPBITS_TWO      #number of stop bits
ser.timeout = 5                         #non-block read
ser.xonxoff = False                     #disable software flow control
ser.rtscts = False                      #disable hardware (RTS/CTS) flow control
ser.dsrdtr = False                      #disable hardware (DSR/DTR) flow control

try: 
    ser.open()

except Exception, e:
    print "error open serial port: " + str(e)
    exit()

if ser.isOpen(): 
    while True:
        try:
            line = ser.readline() # unsure if needed -> .decode('utf-8')
            if line:
                if '$VNINS' in line:
                    data = line[10:-1]
                    print(data)
                    with open('vectornav_log.txt', 'a') as d:
                        write = csv.writer(d)
                        write.writerow([data])
            if line =='!':
                ser.close()
                break
        
        except:
            print('Keyboard Interrupt')
            ser.close()
            break

else:
    print("Can not open serial port")

