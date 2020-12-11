import serial 

baud_rate = 9600
com_port1 = 'ACPI\PNP0501\0'

listener = serial.Serial(com_port1, baud_rate)

while 1:
    serial_out = listener.read(size=1)
    print(serial_out)


