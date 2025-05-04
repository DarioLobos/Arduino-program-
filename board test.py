######UNFINISHED WORKING ON IT DONT COMPILE#######

##Program to control arduino Nano board with the ChargersController.ino
#autor: Dario Lobos 17/mar/2025
# Theme from https://github.com/israel-dryer/ttkbootstrap?tab=readme-ov-file


import serial
import serial.tools.list_ports
import time
from time import sleep
import struct
# Define port default
arduinoPort = 'COM1' # Down in the code will have the choise for change, this is default

#_____________________________________________________________________#
#                                                                     #
#                           SERIAL PROTOCOL                           #
#_____________________________________________________________________#
#

ser =serial.Serial(port=arduinoPort,baudrate=9600, bytesize= serial.EIGHTBITS, parity= serial.PARITY_EVEN,stopbits=serial.STOPBITS_ONE, timeout=1, write_timeout=1)


SEND_STATUS=17; # THIS IS THE IDENTIFIER OR FALSE ADDREESS TO REQUEST ALL PINS STATUS
RECEIVED=23;  # THIS IS THE IDENTIFIER FOR SEND OF DATA
RESEND= 19
WAIT= 22
END = 23
CHRNULL = 0
IS_ANALOG_READ = 28
IS_PWM= 18
IS_SERVO= 20
BOARD_INFO =6; # THIS IS TO SEND DATA OBTAINED FROM BOARD IDENTIFIER LIBRARY
PC_REGISTERS_UPDATE = 14; # THIS IS TO SEND ALL THE REGISTERS AND DATA CHANGED FOR THE PC

COM_ATTEMPTS=4;  # THIS IS THE LIMIT OF LOOPS THAT PROGRAM MUST DO TO TRY TO STABLISH COMMUNICATION BEFORE RAISE AN ERROR 

TIME_UPDATE =1  # THIS ARE THE SECONDS THAT THE PROGRAM UPDATE DATA FROM ARDUINO
                # IS IMPORTANT TO SYNC ARDUINO MUST HAVE SAME VALUE IN THIS VARIABLE.

TIME_SAVE = 15 # THIS ARE THE MINUTES TO SAVE THE FILE DATA. 

AVAILABLE = False;  # THIS IS THE FLAG TO DETERMINE UNAVAILABLE STATE;

ser.close()

counterboard=0
countport=0

print("BoardInfo")
arduinoDict = dict()
portlist = serial.tools.list_ports.comports()
d = len(portlist)
p=0
while(p<d):
        starttimer = time.perf_counter() 
        if portlist[p].name!=None or portlist[p].name!="":
                stringdevice = str(portlist[p].name)
                print(f'[port= {stringdevice}')
                serialscan =serial.Serial(port= stringdevice , baudrate=9600, bytesize= serial.EIGHTBITS, parity= serial.PARITY_EVEN,stopbits=serial.STOPBITS_ONE, timeout=1, write_timeout=1)
                serialscan.flush()
                sleep(0.01)
                p= p + 1
                print(f'serialScan = {serialscan}')
                try:
                        print(f'try')
                        nexttimer = time.perf_counter()
                        elapsedtime =  nexttimer - starttimer
                        serialscan.write(struct.pack('B',BOARD_INFO))
                        sleep(0.01)
                        print(f'time={elapsedtime}')
                        reading=serialscan.read()
                        if (len(reading)>0):
                                answer_sending = struct.unpack('B',reading)[0]
                                print(f'answer_sending = {answer_sending}')
                                if (answer_sending==BOARD_INFO):
                                        print(f'if answer_sending == boardinfo {answer_sending}')
                                        AVAILABLE=True
                                        break
                        if ((int(elapsedtime*1000)) > (int(TIME_UPDATE)*1000)):  # * 1000 to improve precision
                                serialscan.close()
                                print("time out boardinfo")
                                print(" None ")
                                break;
                        answer_sending=None
                        serialscan.flush()
                except:
                                serialscan.close()
                                print("except")
                finally:
                        
                                pass

lineread = serialscan.readline()
print(lineread)
lineread = serialscan.readline()
print(lineread)
lineread = serialscan.readline()
print(lineread)
                        
        
