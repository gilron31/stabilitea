# import serial
# import csv
# from io import IOBase
#
# print("Start")
# port = "COM8"  # This will be different for various devices and on windows it will
# # probably be a COM port.
# bluetooth = serial.Serial(port, 115200, timeout=1)  # Start communications with the bluetooth unit
# bluetooth.write("t")
# print("Connected")
# bluetooth.reset_input_buffer()  # This gives the bluetooth a little kick
# with open('C:\\Users\\t8363387\\Desktop\\Projecton\\data.csv', 'wb') as csvfile:
#     spamwriter = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
#     input_data1 = bluetooth.readline()  # This reads the incoming data. In this particular
#     while (input_data1 != ""):
#         input_data1 = bluetooth.readline() # This reads the incoming data. In this particular
#         input_data2 = "1"
#         print (str(input_data1))
#         print (str(input_data2))
#         spamwriter.writerow([input_data1, input_data2])
#         #spamwriter.writerow("\n")
# # example it will be the "Hello from Blue" line
# bluetooth.close()  # Otherwise the connection will remain open until a timeout which ties up the /dev/thingamabob
# print("Done")
#
import serial
import csv
from io import IOBase

print("Start")
port = "COM8"  # This will be different for various devices and on windows it will
# probably be a COM port.
bluetooth = serial.Serial(port, 115200, timeout=1)  # Start communications with the bluetooth unit
#bluetooth.write("t") for sending
#bluetooth.close() for sending
print("Connected")
bluetooth.reset_input_buffer()  # This gives the bluetooth a little kick
with open('C:\\Users\\t8363387\\Desktop\\Projecton\\data.csv', 'wb') as csvfile:
    spamwriter = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
    input_data = "beset"
    while (("a" not in input_data) & (input_data != "")):
        input_data = bluetooth.readline()
        print("AAEea")
    while (input_data != ""):
        if("a" not in input_data):
            print (input_data)
        print(input_data)
        input_data = bluetooth.readline() # This reads the incoming data. In this particular
        input_data1 = bluetooth.readline()
        input_data2 = bluetooth.readline()
        input_data3 = bluetooth.readline()
        input_data4 = bluetooth.readline()
        input_data5 = bluetooth.readline()
        input_data_a = bluetooth.readline()
        input_data = input_data[0:len(input_data) - 2]
        input_data1 = input_data1[0:len(input_data1) - 2]
        input_data2 = input_data2[0:len(input_data2) - 2]
        input_data3 = input_data3[0:len(input_data3) - 2]
        input_data4 = input_data4[0:len(input_data4) - 2]
        input_data5 = input_data5[0:len(input_data5) - 2]
        spamwriter.writerow([input_data, input_data1, input_data2, input_data3, input_data4, input_data5])
# example it will be the "Hello from Blue" line
bluetooth.close()  # Otherwise the connection will remain open until a timeout which ties up the /dev/thingamabob
print("Done")
