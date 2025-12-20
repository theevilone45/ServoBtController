import serial

ser = serial.Serial('/dev/ttyACM0', 9600)  # Update with your port and baudrate
ser.write(b'reset\n')
ser.close()