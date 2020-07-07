from serial import Serial
import io

Mag = []

file_mag_x = open("Mag_x_data.txt",'w+')
file_mag_y = open("Mag_y_data.txt",'w+')
file_mag_z = open("Mag_z_data.txt","w+")
with Serial(port='COM4',baudrate = 9600,timeout = 1) as seri:
    while (len(Mag) < 700):
        try:
            value = seri.readline()
            temp = (value.decode().replace('\n','').replace('\x00','')).split('  ')
            Mag_x = temp[0]
            Mag_y = temp[1]
            Mag_z = temp[2]
            Mag_x = float(Mag_x)
            Mag_y = float(Mag_y)
            Mag_z = float(Mag_z)
            Mag.append(Mag_x)
            file_mag_x.write(str(Mag_x) +"\n")
            file_mag_y.write(str(Mag_y) +"\n")
            file_mag_z.write(str(Mag_z) +"\n")
            print("Mag_x = %.2f Mag_y = %.2f Mag_z = %.2f" % (Mag_x,Mag_y,Mag_z))
        except Exception as e:
            print(e)
    #end = time.time()
    #print("Tong thoi gian lay 100 mau",end - start)  # dt  Y 27.797778129577637 s
                                                          # X 27.862366199493408  
                                                          # Z             
    