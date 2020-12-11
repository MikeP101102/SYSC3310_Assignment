import serial 

serialport = serial.Serial("/dev/ttyACM1", 9600, timeout=0.5)

serialport.write(b'a')

while True: 
    command = serialport.read()
    print(command.decode('ascii'))