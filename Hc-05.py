import serial
import csv
from io import IOBase

print("Start")
port = "COM8"  # This will be different for various devices and on windows it will
# probably be a COM port.
bluetooth = serial.Serial(port, 115200, timeout=0)  # Start communications with the bluetooth unit
bluetooth.write("t")
print("Connected")
bluetooth.reset_input_buffer()  # This gives the bluetooth a little kick
print("heslei")
input_data = bluetooth.read(1)  # This reads the incoming data. In this particular
with open('C:\\Users\\t8363387\\Desktop\\Projecton\\data.txt', 'wb') as csvfile:
    spamwriter = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
    i = 0
    print (bluetooth.timeout)
    while (i < 100):
        input_data = bluetooth.readline() # This reads the incoming data. In this particular
        print (input_data)
        spamwriter.writerow([input_data])
        i += 1
print ("hi")
# example it will be the "Hello from Blue" line
print(input_data.decode())  # These are bytes coming in so a decode is needed
bluetooth.close()  # Otherwise the connection will remain open until a timeout which ties up the /dev/thingamabob
print("Done")

