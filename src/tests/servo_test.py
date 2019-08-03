import serial
ser = serial.Serial('/dev/ttyACM0')
print(ser.name)
ser.write(bytearray([0xff, 0x05, 0x90]))
ser.close()
