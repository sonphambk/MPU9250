from serial import Serial
import io
import  time

Mag = []
dt = 0
mag = open("mag_calibrated3.txt","w+")
#mag_calib = open("mag_calibrated1.txt","w+")
with Serial(port='/dev/ttyUSB0',baudrate = 115200,timeout = 100) as seri:
    while (len(Mag) < 2000):
        try:
            value = seri.readline()
            temp = (value.decode().replace('\n','').replace('\x00','')).split('  ')
            mag_x = temp[0]
            mag_y = temp[1]
            mag_z = temp[2]
            mag_x = float(mag_x)
            mag_y = float(mag_y)
            mag_z = float(mag_z)
            Mag.append(mag_x)
            mag.write(str(mag_x)+" "+str(mag_y)+" "+str(mag_z)+"\n")
            print("mag_x = %.2f mag_y = %.2f mag_z = %.2f" % (mag_x,mag_y,mag_z))
        except Exception as e:
            print(e)
             
    