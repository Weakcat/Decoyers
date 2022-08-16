
import serial
import time
import binascii

ser = serial.Serial("/dev/ttyUSB1",115200)
print(ser.portstr)
test_data_1 = b'CC 09 03 00 01 CA 7A 07 70'
test_data_2 = b'CC 09 03 00 01 E6 EF 07 C9'
test_data_3 = b'CC 09 04 00 01 E7 2D 07 0D'

aaa = bytes.fromhex("cc0903000310f10723")
aaa = bytes.fromhex("010000F00F000000")#异或校验
aaa = bytes.fromhex("0000050000000000")#异或校验
bbb = bytes.fromhex("01")#异或校验
ccc = bytes.fromhex("02")#异或校验
print(aaa.hex())

ser.write(aaa)
while True:
    # data = ser.read(16).hex()
    ser.write(bbb)
    time.sleep(0.5)    
    ser.write(ccc)
    time.sleep(1)
    # print(data)
    # if data=='dd':
    #     data = ser.read(1).hex()
    #     if data=='0a':
    #         data = ser.read(8).hex()
    #         print('dd0a'+data,time.time())
        # if data[13]=='7':
        #     print('dd'+data)
        # else: