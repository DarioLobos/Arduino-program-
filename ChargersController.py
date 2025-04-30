######UNFINISHED WORKING ON IT DONT COMPILE#######

##Program to control arduino Nano board with the ChargersController.ino
#autor: Dario Lobos 17/mar/2025
# Theme from https://github.com/israel-dryer/ttkbootstrap?tab=readme-ov-file

import Tkinter
import PIL
from tkinter import *
from tkinter import ttk
from tkinter import messagebox
from tkinter import filedialog
from PIL import ImageTk, Image
import os
import serial
import serial.tools.list_ports
import re
import time
from time import sleep
import ttkbootstrap as ttk
from tkinterPdfViewer import tkinterPdfViewer as pdf

# Define analog ports as in pins_arduino.h 
# define PIN_A0   (14)
# define PIN_A1   (15)
# define PIN_A2   (16)
# define PIN_A3   (17)
# define PIN_A4   (18)
# define PIN_A5   (19)
# define PIN_A6   (20)
# define PIN_A7   (21)

A0= 14
A1= 15
A2= 16
A3= 17
A4= 18
A5= 19
A6= 20
A7= 21

#ARDUINO PORTS IN ARDUINO PROGRAM .INO
# const int PC_CONTROL_PIN= A6
# const int mosfet_1_pin = A6 ;
# const int mosfet_2_pin = 9 ; // digital pin can be written as analog
# const int mosfet_3_pin = 10 ;  // digital pin can be written as analog
# const int battery_voltage_pin= A3 ;
# const int device_charger_voltage_1 = A0 ;
# const int device_charger_voltage_2 = A1 ;  
# const int device_charger_voltage_3 = A2 ;  
# const int photo_resistor = A7 ;

#_______________________________________________________________________
# TAKING THE SAME NAMES                                                 #
# !!!! IMPORTANT THING !!!!!  THIS VARIABLE CONTROL PINS IN ALL CODE    #
# LATER IN A WINDOW MENU CAN BE CHANGE AS SET UP. ARDUINO .INO          #
# MUST BE CHANGED IN CODE OR LCD MENU IN CONCORDANCE WITH THIS SETUP    #
#_______________________________________________________________________#

PC_CONTROL_PIN= A6
mosfet_1_pin = 5
mosfet_2_pin = 9
mosfet_3_pin = 11
battery_voltage_pin= A3
device_charger_voltage_1 = A0
device_charger_voltage_2 = A1
device_charger_voltage_3 = A2   
photo_resistor = A7 

ROW=22 # LENGHT OF ARRAY OF DIGITAL READ

# BITWISE FUNCTIONS TO HANDLE REGISTERS
# IN ORDER TO DO SIMILAR CODES IN PHYTON AND ARDUINO I WILL SET BITS AND COUNT WITH A LOCAL FUNCTION IN BOTH THE SAME

def setBit( n, pos):
        n|=( 1<< pos )
        return n

def unsetBit( n, pos):
        n&=~( 1<< pos )
        return n

def bitON( n, pos):
        return ( n & (1<<pos)!=0)


def counterBitON(data):
  count=0;
  while(data>0):
      data &= (data-1) 
      count+=count
  return count  

# THESE ARE THE VARIABLES USED TO STORE ARDUINO STATUS RECEIVED AND ALSO
# TO MODIFIED AND SEND THEM BACK TO CHANGE ARDUINO CONFIGURATION

# PORTn define is Pins are in HIGH = 1 or LOW =0
PORTB=3 # 0 and 1 must be high are clock signals, can be used as sinchonism.
PORTC=0
PORTD=0 # 0 and 1 are serial when is serial on can be overwritten by code and use it anyway, seting PCINT16 AND PCINT17 PCMSK2 |= (1<< PCINT16) | (1<<PCINT17)

# DDRn define mode INPUT=0 OUTPUT=1 
DDRB=0
DDRC=0
DDRD=0

# ANALOG PINS ARE IN PORT C, PORT B IS 8 TO 13 FIST TWO PINS OF PORT ARE
# RESERVED TO SERIAL 0 RX 1 TX MUST NOT BE CHANGED, PORT D IS 0 TO 7

arrayRead = dict()
for i in range (8):
    for v in range (2):
        arrayRead[f'{i}{v}']=0 # ARRAY TO STORE ANALOG READ 1=HIGH 0=LOW BYTE

pwmRegisterB = 0
pwmRegisterC = 0
pwmRegisterD = 0

pwmData= dict()
for i in range (ROW):
    pwmData[f'{i}']=0  # ARRAY TO STORE PWM DATA
    
servoRegisterB = 0 # REGISTERS TO IDENTIFY EACH SERVO PIN
servoRegisterC = 0
servoRegisterD = 0

servoData = dict()
for i in range (ROW):
    servoData[f'{i}']=0 # ARRAY TO STORE SERVO DATA

# FIRST ASIGMENT  OF PORT BEFORE SERIAL CONNECTION TO ARDUINO ACCORDING ARDUINO PROGRAM
# DDRn = PIN MODE 0 INPUT / PORTn = HIGH OR LOW DDRn 1 AND PORTn 1 INPUT PULLUP RESISTOR FOR CONNECTION NOT GROUNDED 

DDRC= setBit(DDRC,PC_CONTROL_PIN-14) # 14 for arduino is 0 port C in chip
DDRD= setBit(DDRD,mosfet_1_pin) 
DDRB= setBit(DDRB,mosfet_2_pin-6) # 0,1 reserved 2 in PORTB will be 8 in arduino
DDRB= setBit(DDRB,mosfet_3_pin-6)

# FIRT ASSIGMENT OF REGISTER BEFORE CONNECTION

def registerFirstset(mode, pin):
    global pwmRegisterC, pwmRegisterB, pwmRegisterD, servoRegisterC, servoRegisterB, servoRegisterD 
    if (mode=="PWM"):
        if (pin>13):
            pwmRegisterC= setBit(pwmRegisterC, pin-14)
        elif (pin>7):
            pwmRegisterB= setBit(pwmRegisterB, pin-6)            
        elif (pin<8):
            pwmRegisterD= setBit(pwmRegisterD, pin)
    if (mode=="Servo"):
        if (pin>13):
            registerServo= setBit(servoRegisterC, pin-14)
        elif (pin>7):
            registerServo= setBit(servoRegisterB, pin-6)            
        elif (pin<8):
            registerServo= setBit(servoRegisterD, pin)

registerFirstset("PWM",mosfet_1_pin)
registerFirstset("PWM",mosfet_2_pin)
registerFirstset("PWM",mosfet_3_pin)

# TO SET DIGITAL PINS CAN BE USED DIRECT PINBx WHICH WILL BECOME OUTPUT
# INDEPENDENT OF DDR. I PREFER USE THIS TO AVOID PROBLEMS WITH ANALOG

prevpwmData= dict()
prevservoData = dict()             
for i in range (ROW):
    prevpwmData[f'{i}']=0  # ARRAY TO STORE PREVIOS PWM DATA
    prevservoData[f'{i}']=0  # ARRAY TO STORE PREVIOS SERVO DATA

# REST OF PINS ARE INPUT AND DONT NEED PULL UP SINCE HAVE CAPACITORS AND GROUND RESISTORS 

# IN CONCORDANCE WITH ARDUINO CODE MUST BE MODIFIED TOGETHER IF NECESSARY
# THIS IS THE ARDUINO CODE             
# const int PHOTO_PRESISTOR_LIMIT = 512; // VALUE MUST BE TESTED AND DETERMINE THE SOLAR PANNEL SWITCH OFF.  
# int photo_resistor_READ=0;
# const float BAT_FULL_VOLTS = 13.7;             
# float batteryState = 0;
# const float BAT_LOW_VOLTS = 12.5 ;

# const float VOLTAGE_DIVIDER_R_TOP = 5.5 ;          // THESE ARE THE VOLTAGE DIVIDER RESISTANCE TO SEND VOLTAGE BATTERY TO
# const float VOLTAGE_DIVIDER_R_GROUND = 12;          // THE ANALOG PIN DIRECTLY

# const float VOLT_FACTOR = VOLTAGE_DIVIDER_R_GROUND/ (VOLTAGE_DIVIDER_R_GROUND + VOLTAGE_DIVIDER_R_TOP);
# const float ANALOG_VOLTS = 5 / 1023 ;

# const float CHARGER_VOLTS_1 = 16;  // THIS IS THE TRANSFORMER, SOLAR PANEL, OR OTHER DEVICE VOLTAGE. I HAVE ONLY ONE BUT LEAVE FOR 3
# const float CHARGER_VOLTS_2 = 16;
# const float CHARGER_VOLTS_3 = 16;


# const float VOLTS_FACTOR_IN_OP = 1 ;  // THIS IS A FACTOR TO AMPLIFY THE SIGNAL FROM THE OP. AMPLIFIER 
#                                       // WITH IT CAN BE MODIFIED MOSFET GATE VOLTAGE TO HAVE MORE OR LESS CURRENT .
#                                       // WITHOUT CHANGE THE CIRCUIT.
# // Function to transform analog read to voltage according constant voltage divider

# SHOULD BE THE INVERSE TO THIS AND CURRENT VALUE THE SAME
             
# float analogVoltageConvertion(int read , float voltFactor)
# {
#  float voltage=0;
# voltage = read * ANALOG_VOLTS * voltFactor ;
# return voltage;

VOLTAGE_DIVIDER_R_TOP = 5.5
VOLTAGE_DIVIDER_R_GROUND = 12      
VOLT_FACTOR = VOLTAGE_DIVIDER_R_GROUND/ (VOLTAGE_DIVIDER_R_GROUND + VOLTAGE_DIVIDER_R_TOP)
VOLTS_FACTOR_IN_OP = 1    # THIS CAN BE SLIGHTLY CHANGED TO CORRECT CIRCUIT TOLERANCE ERRORS THEN APPLY IT TO THE ARDUINO PROGRAM
ANALOG_VOLTS = 5 / 1023
PWM_VOLTS = 5/255
SERVO_VOLTS = 5/180

def convertionToWritePWM(entryValue):

  float_entryValue= float(entryValue)
  writeValue= int(float_entryValue*PWM_VOLTS)
  return writeValue

def convertionToWriteServo(entryValue):

  float_entryValue= float(entryValue)
  writeValue= int(float_entryValue*SERVO_VOLTS)
  return writeValue


def convertionReadToVolts(pin):


    global arrayRead, PORTB, PORTD, PORTC, DDRB, DDRC, DDRD

    if (pin>14):
        if not bitON(DDRC, pin-14):
            if (arrayRead[f'{pin-14}0']!=0 | arrayRead[f'{pin-14}1']!=0):
                value= int(arrayRead[f'{pin-14}0']<<8 + arrayRead[f'{pin-14}1'])
                readValue= (ANALOG_VOLTS * VOLT_FACTOR)* float(value)
                return f'{readValue:.3f}'
            else:
                return f'0'
            
        elif bitON(pwmRegisterC,pin-14):
            value=int(pwmData[f'{pin-14}'])
            readValue= (PWM_VOLTS * VOLTS_FACTOR_IN_OP)* float(value)
            return f'{readValue:.3f}'
        
        elif bitON(servoRegisterC,pin-14):
            value=int(servoData[f'{pin-14}'])
            readValue= (SERVO_VOLTS * VOLTS_FACTOR_IN_OP)* float(value)
            return f'{readValue:.3f}'
        
        elif bitON(PORTC, pin-14):
            return f'1'
        else:
                return f'0'
        
    elif(pin<8):

        if not bitON(DDRD, pin):
            return f'0'
            
        elif bitON(pwmRegisterD,pin):
            value=int(pwmData[f'{pin}'])
            readValue= (PWM_VOLTS * VOLTS_FACTOR_IN_OP)* float(value)
            return f'{readValue:.3f}'
        
        elif bitON(servoRegisterD,pin):
            value=int(servoData[f'{pin}'])
            readValue= (SERVO_VOLTS * VOLTS_FACTOR_IN_OP)* float(value)
            return f'{readValue:.3f}'

        elif bitON(PORTD, pin):
            return f'1'
        else:
                return f'0'
        
    else:
        if not bitON(DDRB, pin-6):
            return f'0'
            
        elif bitON(pwmRegisterB,pin-6):
            value=int(pwmData[f'{pin-6}'])
            readValue= (PWM_VOLTS * VOLTS_FACTOR_IN_OP)* float(value)
            return f'{readValue:.3f}'
        
        elif bitON(servoRegisterB,pin-6):
            value=int(servoData[f'{pin-6}'])
            readValue= (SERVO_VOLTS * VOLTS_FACTOR_IN_OP)* float(value)
            return f'{readValue:.3f}'

        elif bitON(PORTB, pin-6):
            return f'1'
        else:
                return f'0'



# Assign The mode of PC_CONTROL_PIN for control the board with a button 

PC_CONTROL_STATE =0



# Define port type
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

AVAILABLE = True;  # THIS IS THE FLAG TO DETERMINE UNAVAILABLE STATE;


attempt_counter=0
def receiveBoardInfo(serialscan):
    print("BoardInfo")
    global attempt_counter    

    if (attempt_counter>COM_ATTEMPTS):
        AVAILABLE=False
        serialscan.close()
        attempt_counter=0
        return None

    attempt_counter= attempt_counter +1
    
    try:
        print("Try")
        serialscan.open()
        serialscan.write(BOARD_INFO)
        answer_sending = ser.read(1)
        if(answer_sending!=BOARD_INFO):
            AVAILABLE=False
            serialscan.close()
            receiveBoardInfo(serialscan)
            return None
    except:
        print("execpt")
        AVAILABLE=False
        serialscan.close()
        receiveBoardInfo(serialscan)
        return None
    else:
        AVAILABLE=True

    if (attempt_counter>COM_ATTEMPTS):
            print("exit finally")
            attempt_counter=0
            return None
    

    if (AVAILABLE):
        try:
                boardInfoType = ser.readline()
                boardInfoMake = ser.readline()
                boardInfoModel = ser.readline()
                boardInfoMCU = ser.readline()
                boardDictionary = dict([('Type', boardInfoType),('Make', boardInfoMake),('Model', boardInfoModel),('MCU', boardInfoMCU)])
                answer_sending=ser.read(1)
                if(answer_sending!=END):
                    serialscan.close()
                    sendBoardInfo()
                    return None
                else:
                    serialscan.write(RECEIVED)            
        except:
                serialscan.close()
                receiveBoardInfo(serialscan)
                return None
        else:
            serialscan.close()
            return  boardDictionary 
        
attempt_counter=0

def receiveData():
    global attempt_counter, COM_ATTEMPTS, PORTD, DDRD, PORTC, DDRC, PORTB, DDRB, arrayRead, pwmData, servoData, pwmRegisterB, pwmRegisterC, pwmRegisterD, servoRegisterB, servoRegisterC, servoRegisterD
    counter=0;
    receivedRawString= dict()
    receivedPWM = dict()
    receivedServo = dict()
    receivedArrayRead = dict() 
    for i in range (76):
        receivedRawString[f'{i}'] = 0 # ARRAY TO STORE INCOMMING CHARACTERS
    for i in range (ROW):
        receivedPWM[f'{i}'] = 0 # ARRAY TO STORE INCOMMING PWM CHARACTERS
    for i in range (ROW):
        receivedServo[f'{i}'] = 0 # ARRAY TO STORE INCOMMING PWM CHARACTERS
    for i in range (8):
        for v in range (2):
            receivedArrayRead[f'{i}{v}'] = 0  # ARRAY TO STORE INCOMMING ANALOG READ 1=HIGH 0=LOW BYTE

    counterArrayRead=0  
    counterPWM=0
    counterServo=0
    bufferPortB=0
    bufferDDRB=0
    bufferPortC=0
    bufferDDRC=0
    bufferPortD=0
    bufferDDRD=0
    bufferpwmRegisterB=0
    bufferpwmRegisterC=0
    bufferpwmRegisterD=0
    bufferservoRegisterB=0
    bufferservoRegisterC=0
    bufferservoRegisterD=0
    counterDDRDArrayRead=0
    counterRegisterPWM=0
    counterRegisterServo=0

    attempt_counter= attempt_counter +1 

    try:
        print("try")
        ser.open()
        print(f'{(attempt_counter>COM_ATTEMPTS)}')
        if (attempt_counter>COM_ATTEMPTS):
                print("exit try")
                ser.close()
                return -1
                print("after return")
        print("after open")
        ser.write(SEND_STATUS)
        print("attempt:" + f'{attempt_counter}')
        print("COM_:" + f'{COM_ATTEMPTS}')      
        print("after write")
        if((ser.read(1)!=SEND_STATUS)& (attempt_counter<=COM_ATTEMPTS)):
            AVAILABLE=False            
            ser.close()
            receiveData()
            return -1            
    except:
        print("except")
        attempt_counter= attempt_counter + 1 
        ser.close()
        print("except after close ")
        if (attempt_counter<COM_ATTEMPTS):
            AVAILABLE=False
            print("counter: " + f'{attempt_counter}')
            receiveData()
            return -1
        print("else")
        AVAILABLE=False
        return -1
        
    else:
        AVAILABLE=True        
        
    if (attempt_counter>COM_ATTEMPTS):
            print("exit finally")
            attempt_counter=0
            return -1
            
    
    if (AVAILABLE):
        while(AVAILABLE):
            try:
                receivedRawString[f'{counter}']=ser.read(1)
                if(counter > 0):
                    if(receivedRawString[f'{counter}']==END & receivedRawString[f'{counter-1}']==CHRNULL):
                          counter=0
                          break  
                counter = counter + 1
            except:
                AVAILABLE=False
                ser.close()
                receiveData()
                return -1

        if (receivedRawString[f'12']!=CHRNULL | receivedRawString[f'13']!=IS_ANALOG_READ):
            ser.flush()
            ser.write(RESEND)
            ser.close()
            receiveData()
            return -1
                
        bufferPortD= receivedRawString[f'0']
        bufferDDRD= receivedRawString[f'1']
        bufferpwmRegisterD= receivedRawString[f'2']
        bufferservoRegisterD= receivedRawString[f'3']

        bufferPortC= receivedRawString[f'4']
        bufferDDRC= receivedRawString[f'5']
        bufferpwmRegisterC= receivedRawString[f'6']
        bufferservoRegisterC= receivedRawString[f'7']

        bufferPortB= receivedRawString[f'8']
        bufferDDRB= receivedRawString[f'9']
        bufferpwmRegisterB= receivedRawString[f'10']
        bufferservoRegisterB= receivedRawString[f'11']


        counterDDRDArrayRead = 8 - counterBitON(bufferDDRD)
        couterRegisterPWM = counterBitON(bufferpwmRegisterD)+ counterBitON(bufferpwmRegisterC) + counterBitON(bufferpwmRegisterB);
        couterRegisterServo = counterBitON(bufferservoRegisterD)+ counterBitON(bufferservoRegisterC) + counterBitON(bufferservoRegisterB);

        doneReadArrayRead = False
        doneReadPWM = False
    
        for i in range(14,81):
            if (doneReadArrayRead!=True & (receivedRawString[f'{i+2}']==IS_PWM & receivedRawString[f'{i+1}']==CHRNULL)!=True):
                    receivedArrayRead[f'{counterArrayRead}'] = receivedRawString[f'{i}']
                    counterArrayRead = counterArrayRead + 1
            else:
                doneReadArrayRead = True
                if ( doneReadPWM!=True & (receivedRawString[f'{i+2}']==IS_SERVO & receivedRawString[f'{i+1}']==CHRNULL)!=True):
                    receivedPWM[f'{counterPWM}'] = receivedRawString[f'{i}']
                    counterPWM = counterPWM + 1 
                else:
                    doneReadPWM=True
                    if((receivedRawString[f'{i+2}']== END & receivedRawString[f'{i+1}']== CHRNULL)!=True):
                        receivedServo[f'{counterServo}'] = receivedRawString[f'{i}']
                        counterServo = counterServo + 1
                    else:
                        break
    
        if (counterArrayRead!=counterDDRDArrayRead | counterPWM!=counterRegisterPWM | counterServo!=counterRegisterServo):
            try:
                ser.flush()
                ser.write(RESEND)
                ser.close()
                AVAILABLE=False
                receiveData()
                return -1

            except:
                AVAILABLE=False
                ser.close
                receiveData()
                return -1

        ser.write(RECEIVED)
        ser.flush()
        ser.close()
        attempt_counter=0

        PORTD = bufferPortD
        DDRD = bufferDDRD
        PORTC = bufferPortC
        DDRC = bufferDDRC
        PORTB = bufferPortB
        DDRB = bufferDDRB
        pwmRegisterB = bufferpwmRegisterB
        pwmRegisterC = bufferpwmRegisterC
        pwmRegisterD = bufferpwmRegisterD
        servoRegisterB = bufferservoRegisterB
        servoRegisterC = bufferservoRegisterC
        servoRegisterD = bufferservoRegisterD

        

        placercounter=0

        for i in range (8):
            for v in range (2):
                if not bitON(bufferDDRD,i):
                    arrayRead[F'{i}{v}'] = receivedArrayRead[f'{placercounter}']
                    placercounter = placecounter +1
            
        placercounter=0

        for i in range(ROW):
            pwmData[f'{i}']=0
            if (i<8):
                if(bitON(bufferpwmRegisterD,i)):
                    pwmData['f{i}'] = receivedPWM[f'{placercounter}']
                    placercounter = placecounter +1
            elif (i<14):
                if(bitON(bufferpwmRegisterC,i-6)):
                    pwmData[f'{i}']= receivedPWM[f'{placercounter}']
                    placercounter = placecounter +1
            else:
                if(bitON(bufferpwmRegisterB,i-14)):
                    pwmData['F{i}'] = receivedPWM[f'{placercounter}']
                    placercounter = placecounter +1
                    
        placercounter=0
  
        for i in range(ROW):
            servoData[f'{i}']=0;
            if (i<8):
                if(bitON(bufferservoRegisterD,i)):
                    servoData[f'{i}'] = receivedServo[f'{placercounter}']
                    placercounter = placecounter +1
            elif (i<16):
                if(bitON(bufferservoRegisterC,i-6)):
                    servoData[f'{i}'] = receivedServo[f'{placercounter}']
                    placercounter = placecounter +1
            else:
                if(bitON(bufferservoRegisterB,i-14)):
                    servoData[f'{i}'] = receivedServo[f'{placercounter}']
                    placercounter = placecounter +1

        placercounter=0

            
# THIS IS THE FUNCTION TO SEND BOARD ACTIONS          

attempt_counter=0
def sendBoardUpdate():
    print("BoardUpdate")
    global attempt_counter

    if (attempt_counter>COM_ATTEMPTS):
        ser.close()
        attempt_counter=0
        AVAILABLE=False
        return -1
        print("Comattempt")

    attempt_counter = attempt_counter + 1


    try:
        ser.open()
        ser.write(PC_REGISTERS_UPDATE)
        resivedAction=ser.read(1)
        while (receivedAction==WAIT):
            pass
        while (receivedAction!=PC_REGISTERS_UPDATE):
            ser.close()
            sendBoardUpdate()
            return -1
    except:
            ser.close()
            sendBoardUpdate()
            return -1
    else:
        AVAILABLE=True

    if (attempt_counter>COM_ATTEMPTS):
        print("exit finally")
        ser.close()
        attempt_counter=0
        return -1


    if (AVAILABLE):    
        try:
            ser.write(PORTD)
            ser.write(DDRD)
            ser.write(pwmRegisterD)
            ser.write(servoRegisterD)
            ser.write(PORTC)
            ser.write(DDRC)
            ser.write(pwmRegisterC)
            ser.write(servoRegisterC)
            ser.write(PORTB)
            ser.write(DDRB)
            ser.write(pwmRegisterB)
            ser.write(servoRegisterB)      
    
    # SEND PWM INFO
            ser.write(CHRNULL)
            ser.write(IS_PWM)

            if (pwmData != prevpwmData):
                for i in range(ROW):
                    if (pwmData[f'{i}']!= 0): 
                        if (pwmData[f'{i}']!= prevpwmData[f'{i}']): 
                            ser.write(chr(pwmData[f'{i}']));

# SEND SERVO INFO
            ser.write(CHRNULL)                
            ser.write(IS_SERVO)

            if (servoData != prevservoData):
                for i in range(ROW):
                    if (servoData[f'{i}']!= 0): 
                        if (servoData[f'{i}']!= prevservoData[f'{i}']):
                            ser.write(servoData[f'{i}'])

            ser.write(CHRNULL)
            ser.write(END)

# ASK RECEIVED FROM ARDUIND
    
    
            resivedAction=ser.read(1)
            while (receivedAction==WAIT):
                pass
            if (receivedAction==RESEND):
                        ser.close()
                        sendBoardUpdate()
                        return -1
            if (receivedAction!=RECEIVED):
                resivedAction=ser.read(1)
                while (receivedAction==WAIT):
                    pass
                    if (receivedAction==RESEND):
                        ser.close()
                        sendBoardUpdate()
                        return -1
        except:
                ser.close()
                sendBoardUpdate()
                return -1
            
        else:
            AVAILABLE=True
                
        prevArrayRead = arrayRead;
        prevpwmData = pwmData;
        prevservoData = servoData; 
    return 1


#_____________________________________________________________________#
#                                                                     #
#                              WIDGET                                 #
#_____________________________________________________________________#
#

# THESE ARE TEMPORAL ARRAY TO STORE DATA TO BE SENT TO AERDUINO WHEN PROCESS BUTTON
# IS PRESSED.

entrypwmData= dict()
for i in range (ROW):
    entrypwmData[f'{i}']=0  # ARRAY TO STORE PWM DATA BEFORE SEND IT TO ARDUINO

entryservoData = dict()
for i in range (ROW):
     entryservoData[f'{i}']=0 # ARRAY TO STORE SERVO DATA BEFORE SEND IT TO AEDUINO    
 

# THESE FLAGE VARIABLE ARE TEMPORAL REGISTERS BEFORE UPDATEI USED IN THE BUTTON THAT DEFINE MODE.
# WHEN PROCESS CHANGE BUTTON IS PRESSED THESE UPDATE  DE REGISTERS AND BEFORE DEFINE THE TEMPORAL DATA STORAGE IN THE ARRAYS,  
# IN ARDUINO DIGITAL UOUTPUT ONLY NEED PORT B AND DDR
# BUT FOR CONVENIENCE OF HANDLING I ADDED THIS TO EASY
# DETERMINE IF IT IS OUTPUT DIGITAL, SERVO OR PWM
# THIS IS NECESSARY TO DETERMINE ENTRY VALIDATION
# D.O.= 0|1, PWM= 0-255, SERVO = 0-180

entrydigitalOutputB = ~(pwmRegisterB | servoRegisterB | ~DDRB)
entrydigitalOutputC = ~(pwmRegisterC | servoRegisterC | ~DDRC)   
entrydigitalOutputD = ~(pwmRegisterD | servoRegisterD | ~DDRD)  
                                                   
entrypwmRegisterB = pwmRegisterB    
entrypwmRegisterC = pwmRegisterC
entrypwmRegisterD = pwmRegisterD
 
entryservoRegisterB = servoRegisterB
entryservoRegisterC = servoRegisterC
entryservoRegisterD = servoRegisterD

# THESE ARE THE SAME TYPE OF TEMPORAL VARIABLES FOR PORTn AND DDRn
 
# PORTn define is Pins are in HIGH = 1 or LOW =0
entryPORTB=PORTB
entryPORTC=PORTC
entryPORTD=PORTD
# DDRn define mode INPUT=0 OUTPUT=1
entryDDRB=DDRB
entryDDRC=DDRC
entryDDRD=DDRD
 
 
# Define theme for environment


# Define main widget environment dimensions

root = ttk.Window(themename="vapor")
# root = Tk()

root.title("Program to remote control Battery charger and Handle data")
root.minsize(300,300)
screenWidth = root.winfo_screenwidth()
screenHeight = root.winfo_screenheight()

# check window size to make root size

rootSizerWidth= int(screenWidth*0.8)
rootSizerHeight= int(screenHeight*0.8)

topLeftPosition=(int((screenWidth- rootSizerWidth)/2),int((screenHeight- rootSizerHeight)/2))

root.geometry(f'{rootSizerWidth}x{rootSizerHeight}+{topLeftPosition[0]}+{topLeftPosition[1]}')

root.columnconfigure(0, weight=1)
root.rowconfigure(0, weight=1)

root.option_add('*tearOff', FALSE)

menubar = Menu(root)

menu_file = Menu(menubar)
menu_plot = Menu(menubar)
menu_port = Menu(menubar)
menu_help = Menu(menubar)
menubar.add_cascade(menu=menu_file, label='File')
menubar.add_cascade(menu=menu_plot, label='Plot')
menubar.add_cascade(menu=menu_port, label='Port')
menubar.add_cascade(menu=menu_help, label='Help')


# Make default filename with day and year and program identifier

secs = time.time()
timehere= time.localtime(secs)

filename= f'Charger_{timehere.tm_year}{timehere.tm_mon}{timehere.tm_mday}'
print(filename)

initial_directory = "reports"

try: 
    if not os.path.exists(initial_directory):
        os.makedirs(initial_directory)
except:
    messagebox.showwarning(title='Make the default file not allowed', message="Defaul directory for reports save is not allowed, please verify program permissions")
finally:
    pass

def openFile():
    global filename
    filename = filedialog.askopenfilename(initialdir= initial_directory, initialfile = filename , title = "Select file, for best choose default, same extension",filetypes = (("charge files","*.chtx"),("all files","*.*")))

def saveasFile():
    global filename
    filename = filedialog.asksaveasfilename(initialdir= initial_directory, initialfile = filename , title = "Select file, for best choose default, same extension",filetypes = (("charge files","*.chtx"),("all files","*.*")))

def closeFile():
    global filename
    filename.close()

def dirFile():
    global dirname
    initial_directory = filedialog.askdirectory(initialdir=initial_directory)


arduinoLabel=StringVar() # String label for board detected
countport=0
def scanPort():
    global ser, arduinoPort, arduinoLabel, countport

    ser.close()
    arduinoDict = dict()
    portlist = serial.tools.list_ports.comports()
    d = len(portlist)
    if (d >0):
        while(True):
            if (countport < d):
                if portlist[countport].name!=None or portlist[countport].name!="":
                    stringdevice = str(portlist[countport].name)
                    try: 
                        serialScan =serial.Serial(port=stringdevice,baudrate=9600, bytesize= serial.EIGHTBITS, parity= serial.PARITY_EVEN,stopbits=serial.STOPBITS_ONE, timeout=1, write_timeout=1)
                        arduinoDict = receiveBoardInfo(serialScan)
                    except:
                        countport= countport +1
                        scanPort()
                        break
                    finally:
                        if arduinoDict is not None:
                            arduinoPort = stringdevice
                            portVar.set(arduinoPort)
                            arduinoLabel.set(f'Port detected= {arduinoPort} , Boar Info: Type = {arduinoDict["Tyoe"]}, Make = {arduinoDict["Make"]}, Model = {arduinoDict["MODEL"]}, MCU = {arduinoDict["MCU"]}')
                            ser =serial.Serial(port=arduinoPort,baudrate=9600, bytesize= serial.EIGHTBITS, parity= serial.PARITY_EVEN,stopbits=serial.STOPBITS_ONE, timeout=1, write_timeout=1)
                            oountport=0
                            break
                        else:
                            countport= countport +1
                            scanPort()
                            break

            else:
                    arduinoLabel.set("BOARD NOT DETECTED, CHECK CONNECTION")
                    countport=0
                    break
    else:
                arduinoLabel.set("PORTS NOT DETECTED, CHECK PORT SETTINGS OR PERMISSIONS TO CAN SCAN")
                countport=0
                

def openHelp():

    global screenWidth, screenHeight

    parent=root
    title= 'PDF help'
    # check window size to make root size

    rootSizerWidth= int(screenWidth*0.8)
    rootSizerHeight= int(screenHeight*0.8)

    topLeftPosition=(int((screenWidth- rootSizerWidth)/2),int((screenHeight- rootSizerHeight)/2))
    top=Toplevel(parent)
    top.geometry(f'{rootSizerWidth}x{rootSizerHeight}+{topLeftPosition[0]}+{topLeftPosition[1]}')
    top.transient(parent)
    top.title(title)
    topframe=Frame(top)
    def close():
        top.destroy()
        
    button = ttk.Button(topframe, text="Close", command = close)
    button.pack()

    pdf_Charger = pdf.ShowPdf().pdf_view(topframe, pdf_location=r"ChargerHelp.pdf", width = int(screenWidth*0.8), height = int(screenHeight*0.8))
    pdf_Charger.pack()

    topframe.pack()

menu_file.add_command(label='Open', command=openFile)
menu_file.add_command(label='Save as', command=saveasFile)
menu_file.add_command(label='Close', command=closeFile)
menu_file.add_command(label='File directory', command=dirFile)
menu_port.add_command(label='Scan Port', command=scanPort)
menu_help.add_command(label='Open Help', command=openHelp)

portlist = serial.tools.list_ports.comports()
portArray = []
radioports = []
for d in portlist:
    if d.name:
        stringdevice = str(d.name)
        portArray.append(stringdevice)

menu_set = Menu(menu_port)
menu_port.add_cascade(menu=menu_set, label='Set port')
portVar= StringVar()

def stringPortFormat():
    global arduinoPort, portVar
    if(len(portVar.get())>0):
        arduinoPort= portVar.get()

for i in range(len(portArray)):
    portVar.set(arduinoPort)
    portset= portArray[i]
    menu_set.add_radiobutton(label=portset, variable=portVar, value=portset, command= stringPortFormat)


root['menu']= menubar

# To check ARDUINO STATUS in any loop slow plenty the program
# so at the end of the program have a time defined funtion 
# to receive data:
# root.after(100, eventTimeFunction) at the end


mainFrame = ttk.Frame(root, padding ="3 3 12 12")

mainFrame.grid(column=0, row=0 , sticky="nswe")

mainFrame.columnconfigure(0, weight=1)
mainFrame.rowconfigure(0, weight=1)
mainFrame.columnconfigure(1, weight=1)
mainFrame.columnconfigure(2, weight=1)
mainFrame.columnconfigure(3, weight=1)
mainFrame.columnconfigure(4, weight=1)
mainFrame.columnconfigure(5, weight=16)
mainFrame.rowconfigure(1, weight=1)             
mainFrame.rowconfigure(2, weight=1)
mainFrame.rowconfigure(3, weight=1)
mainFrame.rowconfigure(4, weight=2)
mainFrame.rowconfigure(5, weight=1)
mainFrame.rowconfigure(6, weight=2)
mainFrame.rowconfigure(7, weight=1)
mainFrame.rowconfigure(8, weight=2)
mainFrame.rowconfigure(9, weight=1)
mainFrame.rowconfigure(10, weight=2)
mainFrame.rowconfigure(11, weight=1)
mainFrame.rowconfigure(12, weight=2)
mainFrame.rowconfigure(13, weight=1)
mainFrame.rowconfigure(14, weight=2)
mainFrame.rowconfigure(15, weight=1)
mainFrame.rowconfigure(16, weight=2)
mainFrame.rowconfigure(17, weight=1)
mainFrame.rowconfigure(18, weight=2)
mainFrame.rowconfigure(19, weight=3)
mainFrame.rowconfigure(20, weight=1)


# Make an frame special for the image to can resize it well

photoFrame = ttk.Frame(mainFrame, padding ="3 3 12 12")

# variables for button text

buttonMos1Stg = StringVar()
buttonMos2Stg = StringVar()
buttonMos3Stg = StringVar()
batteryVoltStg = StringVar()
trafoVoltStg = StringVar()
solarVoltStg = StringVar()
windVoltStg = StringVar()
photoResistorStg =StringVar()



# Screen message function for button to control board
ttk.Label(mainFrame, text= "<= To control board, press button").grid(column=2,row=1, columnspan=3, sticky='w')
   
# buttons to control pin mode

textUnlock= StringVar()
textUnlock.set('DISABLED')


# booleans for check buttons are used bellow in the function and in the check button
chkMos1State= BooleanVar(value= False)
chkMos2State= BooleanVar(value= False)
chkMos3State= BooleanVar(value= False)
chkBatState= BooleanVar(value= False)
chkTrafoState= BooleanVar(value= False)
chkSolarState= BooleanVar(value= False)
chkWindState= BooleanVar(value= False)
chkPhotoState= BooleanVar(value= False)

# Text variable for entries, must be initialized first becaused are called by functions bellow

varentryMosfet1 = StringVar()
varentryMosfet2 = StringVar()
varentryMosfet3 = StringVar()
varentryBattery = StringVar()
varentryTrafo = StringVar()
varentrySolar = StringVar()
varentryWind = StringVar()
varentryPhoto = StringVar()


def controlBoardPBtn():
    global controlBoardPBtn, PORTD, PC_CONTROL_STATE, chkMosfet1, chkMosfet2, chkMosfet3, chkBatteryVolt, chkTrafoVolt, chkSolarPanelVolt, chkWindTurbineVolt, chkWindTurbineVolt, chkPhotoreResistorVolt, proceedButton, buttonMos1, buttonMos2, buttonMos3, batteryVolt, trafoVolt, solarVolt, windVolt, photoResistor, varentryMosfet1, varentryMosfet2, varentryMosfet3, varentryBattery, varentryTrafo, varentrySolar, varentryWind, varentryPhoto, entryPORTB, PORTB, entryPORTC, PORTC, entryPORTD, PORTD, entryDDRB, DDRB, entryDDRC, DDRC, entryDDRD, DDRD, chkMos1State, chkMos2State, chkMos3State, chkBatState, chkTrafoState, chkSolarState, chkWindState, chkPhotoState, entryMosfet1, entryMosfet2, entryBatteryVolt, entryTrafoVolt, entrySolarPanelVolt, entryWindTurbineVolt, entryPhotoreResistorVolt, entrydigitalOutputB, pwmRegisterB, servoRegisterB, DDRB, entrydigitalOutputC, pwmRegisterC, servoRegisterC, DDRC, entrydigitalOutputD, pwmRegisterD, servoRegister, DDRD, entrypwmRegisterB, pwmRegisterB, entrypwmRegisterC, pwmRegisterC, entrypwmRegisterD, pwmRegisterD, entryservoRegisterB, servoRegisterB, entryservoRegisterC, servoRegisterC, entryservoRegisterD, servoRegisterD, buttonMos2Stg, buttonMos3Stg, batteryVoltStg, trafoVoltStg, solarVoltStg, windVoltStg, photoResistorStg

    def chkPinInINPUT(pin):
        if (pin>13):
            if not (bitON(DDRC, pin-14)):
                return True
        elif (pin>7):
            if not (bitON(DDRB, pin-6)):
                return True
        elif (pin<8):
             if not (bitON(DDRD, pin)):
                return True
        return False

    def chkPinInPWM(pin):
        if (pin>13):
            if(bitON(pwmRegisterC, pin-14)):
                return True
        elif (pin>7):
            if(bitON(pwmRegisterB, pin-6)):
                return True
        elif (pin<8):
             if(bitON(pwmRegisterD, pin)):
                return True
        return False

    def chkPinInServo(pin):
        if (pin>13):
            if(bitON(servoRegisterC, pin-14)):
                return True
        elif (pin>7):
            if(bitON(servoRegisterB, pin-6)):
                return True
        elif (pin<8):
             if(bitON(servoRegisterD, pin)):
                return True
        return False

    def chkPinInOUTPUT(pin):                # THIS FUNCTION MUST BE CALLED AFTER RESET THE REGISTER BECAUSE DONT HAVE PERMANENT REGISTER
        if (pin>13):
            if(bitON(entrydigitalOutputC, pin-14)):
                return True
        elif (pin>7):
            if(bitON(entrydigitalOutputB, pin-6)):
                return True
        elif (pin<8):
             if(bitON(entrydigitalOutputD, pin)):
                return True
        return False
        
    
    if (bitON(PORTD,PC_CONTROL_PIN)):
        PC_CONTROL_STATE =0
        textUnlock.set('DISABLED') 
        PORTD= unsetBit(PORTD,PC_CONTROL_PIN) # THIS ONLY CHANGE THE VARIABLE AND SCREEN VIEW PROCESS CHANGE BUTTON UPDATE ARDUINO
        chkMosfet1.state(['disabled'])
        chkMosfet2.state(['disabled'])
        chkMosfet3.state(['disabled'])
        chkBatteryVolt.state(['disabled'])
        chkTrafoVolt.state(['disabled'])
        chkSolarPanelVolt.state(['disabled'])
        chkWindTurbineVolt.state(['disabled'])
        chkWindTurbineVolt.state(['disabled'])
        chkPhotoreResistorVolt.state(['disabled'])
        proceedButton.state(['disabled'])
        buttonMos1.state(['disabled'])
        buttonMos2.state(['disabled'])
        buttonMos3.state(['disabled'])
        batteryVolt.state(['disabled'])
        trafoVolt.state(['disabled'])
        solarVolt.state(['disabled'])
        windVolt.state(['disabled'])
        photoResistor.state(['disabled'])
        varentryMosfet1.set("")
        varentryMosfet2.set("")
        varentryMosfet3.set("")
        varentryBattery.set("")
        varentryTrafo.set("")
        varentrySolar.set("")
        varentryWind.set("")
        varentryPhoto.set("")
        entryPORTB=PORTB
        entryPORTC=PORTC
        entryPORTD=PORTD
        entryDDRB=DDRB
        entryDDRC=DDRC
        entryDDRD=DDRD
        chkMos1State.set(False)
        chkMos2State.set(False)
        chkMos3State.set(False)
        chkBatState.set(False)
        chkTrafoState.set(False)
        chkSolarState.set(False)
        chkWindState.set(False)
        chkPhotoState.set(False)
        entryMosfet1.state(['disabled'])
        entryMosfet2.state(['disabled'])
        entryBatteryVolt.state(['disabled'])
        entryTrafoVolt.state(['disabled'])
        entrySolarPanelVolt.state(['disabled'])
        entryWindTurbineVolt.state(['disabled'])
        entryPhotoreResistorVolt.state(['disabled'])
        entrydigitalOutputB = ~(pwmRegisterB | servoRegisterB | ~DDRB)
        entrydigitalOutputC = ~(pwmRegisterC | servoRegisterC | ~DDRC)      
        entrydigitalOutputD = ~(pwmRegisterD | servoRegisterD | ~DDRD)  
        entrypwmRegisterB = pwmRegisterB    
        entrypwmRegisterC = pwmRegisterC
        entrypwmRegisterD = pwmRegisterD
        entryservoRegisterB = servoRegisterB
        entryservoRegisterC = servoRegisterC
        entryservoRegisterD = servoRegisterD
        if(chkPinInINPUT(mosfet_1_pin)):
            buttonMos1Stg.set('INPUT')
        elif(chkPinInServo(mosfet_1_pin)):
            buttonMos1Stg.set('SERVO')
        elif(chkPinInPWM(mosfet_1_pin)):
            buttonMos1Stg.set('PWM')
        elif(chkPinInOUTPUT(mosfet_1_pin)):
            buttonMos1Stg.set('OUTPUT')
        if(chkPinInINPUT(mosfet_2_pin)):
            buttonMos2Stg.set('INPUT')
        elif(chkPinInServo(mosfet_2_pin)):
            buttonMos2Stg.set('SERVO')
        elif(chkPinInPWM(mosfet_2_pin)):
            buttonMos2Stg.set('PWM')
        elif(chkPinInOUTPUT(mosfet_2_pin)):
            buttonMos2Stg.set('OUTPUT')
        if(chkPinInINPUT(mosfet_3_pin)):
            buttonMos3Stg.set('INPUT')
        elif(chkPinInServo(mosfet_3_pin)):
            buttonMos3Stg.set('SERVO')
        elif(chkPinInPWM(mosfet_3_pin)):
            buttonMos3Stg.set('PWM')
        elif(chkPinInOUTPUT(mosfet_3_pin)):
            buttonMos3Stg.set('OUTPUT')
        if(chkPinInINPUT(battery_voltage_pin)):
            batteryVoltStg.set('INPUT')
        elif(chkPinInServo(battery_voltage_pin)):
            batteryVoltStg.set('SERVO')
        elif(chkPinInPWM(battery_voltage_pin)):
            batteryVoltStg.set('PWM')
        elif(chkPinInOUTPUT(battery_voltage_pin)):
            batteryVoltStg.set('OUTPUT')
        if(chkPinInINPUT(device_charger_voltage_1)):
            trafoVoltStg.set('INPUT')
        elif(chkPinInServo(device_charger_voltage_1)):
            trafoVoltStg.set('SERVO')
        elif(chkPinInPWM(device_charger_voltage_1)):
            trafoVoltStg.set('PWM')
        elif(chkPinInOUTPUT(device_charger_voltage_1)):
            trafoVoltStg.set('OUTPUT')
        if(chkPinInINPUT(device_charger_voltage_2)):
            solarVoltStg.set('INPUT')
        elif(chkPinInServo(device_charger_voltage_2)):
            solarVoltStg.set('SERVO')
        elif(chkPinInPWM(device_charger_voltage_2)):
            solarVoltStg.set('PWM')
        elif(chkPinInOUTPUT(device_charger_voltage_2)):
            solarVoltStg.set('OUTPUT')
        if(chkPinInINPUT(device_charger_voltage_2)):
            solarVoltStg.set('INPUT')
        elif(chkPinInServo(device_charger_voltage_2)):
            solarVoltStg.set('SERVO')
        elif(chkPinInPWM(device_charger_voltage_2)):
            solarVoltStg.set('PWM')
        elif(chkPinInOUTPUT(device_charger_voltage_2)):
            solarVoltStg.set('OUTPUT')
        if(chkPinInINPUT(device_charger_voltage_3)):
            windVoltStg.set('INPUT')
        elif(chkPinInServo(device_charger_voltage_3)):
            windVoltStg.set('SERVO')
        elif(chkPinInPWM(device_charger_voltage_3)):
            windVoltStg.set('PWM')
        elif(chkPinInOUTPUT(device_charger_voltage_3)):
            windVoltStg.set('OUTPUT')
        if(chkPinInINPUT(photo_resistor)):
            photoResistorStg.set('INPUT')
        elif(chkPinInServo(photo_resistor)):
            photoResistorStg.set('SERVO')
        elif(chkPinInPWM(photo_resistor)):
            photoResistorStg.set('PWM')
        elif(chkPinInOUTPUT(photo_resistor)):
            photoResistorStg.set('OUTPUT')
        
    else:
        PC_CONTROL_STATE =1
        textUnlock.set("ENABLED")
        PORTD = setBit(PORTD,PC_CONTROL_PIN) # THIS ONLY CHANGE THE VARIABLE AND SCREEN VIEW PROCESS CHANGE BUTTON UPDATE ARDUINO
        if not (chkPinInINPUT(mosfet_1_pin)):
            chkMosfet1.state(['!disabled'])
        if not (chkPinInINPUT(mosfet_2_pin)):
            chkMosfet2.state(['!disabled'])
        if not (chkPinInINPUT(mosfet_3_pin)):
            chkMosfet3.state(['!disabled'])  
        if not (chkPinInINPUT(battery_voltage_pin)):            
            chkBatteryVolt.state(['!disabled'])
        if not (chkPinInINPUT(device_charger_voltage_1)): 
            chkTrafoVolt.state(['!disabled'])
        if not (chkPinInINPUT(device_charger_voltage_2)): 
            chkSolarPanelVolt.state(['!disabled'])
        if not (chkPinInINPUT(device_charger_voltage_3)): 
            chkWindTurbineVolt.state(['!disabled'])
        if not (chkPinInINPUT(photo_resistor)): 
            chkPhotoreResistorVolt.state(['!disabled'])
        proceedButton.state(['!disabled'])
        buttonMos1.state(['!disabled'])
        buttonMos2.state(['!disabled'])
        buttonMos3.state(['!disabled'])
        batteryVolt.state(['!disabled'])
        trafoVolt.state(['!disabled'])
        solarVolt.state(['!disabled'])
        windVolt.state(['!disabled'])
        photoResistor.state(['!disabled'])
            
controlBoardPBtn=ttk.Button(mainFrame, textvariable = textUnlock, command = controlBoardPBtn, state='enabled').grid(column=1,row=1)

# Labels for columns
ttk.Label(mainFrame, text= "Pin nbr.").grid(column=0,row=2)
ttk.Label(mainFrame, text= "Mode of Pins").grid(column=1,row=2)
ttk.Label(mainFrame, text= "Current value").grid(column=2,row=2)
ttk.Label(mainFrame, text= "Value to update").grid(column=3,row=2)
ttk.Label(mainFrame, text= "<= To process changes and send them to the ARDUINO board  press the button").grid(column=5, row=1, sticky ="w")
ttk.Label(mainFrame, text= "Check to update").grid(column=4, row=2)

# label for connectio status

connectionText = StringVar()
connectionText.set("")
conectionStatus = ttk.Label(mainFrame, textvariable = connectionText, foreground='firebrick1') 
conectionStatus.grid(column=5, row=2)

#function to resize image when window size change PENDING USE FRAME INSTEAD OF LABEL
# RESIZE WORK BUT IS TO SLOW AN MY COMPUTER STAY SOME TIME IN NOT RESPOND UNTIL REDRAW
# MAYBE I DISABLE IT TO HAVE FASTER ACTIONS.
def boardResize(event):
    _imageWidth = int(root.winfo_width()* 0.4)
    _imageHeight = int(root.winfo_height() * 0.4)
    global sizeChangedBoardImg, sizeChangedBoardPho
    sizeChangedBoardImg= dynamicChangeBoardImg.resize((_imageWidth,_imageHeight))
    sizeChangedBoardPho= ImageTk.PhotoImage(sizeChangedBoardImg)
    boardlabel.config(image=sizeChangedBoardPho)
    # avoid garbage collector
    boardlabel.image = sizeChangedBoardPho
    boardlabel.pack(fill=BOTH, expand=True, anchor='center')

# Label for image

boardImage = Image.open('chargerController_bb.png')
imageWidth = int(rootSizerWidth * 0.4)
imageHeight = int(rootSizerHeight * 0.4)
resizedboardImage=boardImage.copy().resize((imageWidth,imageHeight))
dynamicChangeBoardImg=boardImage.copy()
photoBoard=ImageTk.PhotoImage(resizedboardImage)
boardlabel= ttk.Label(photoFrame,image=photoBoard)
boardlabel.pack(fill=BOTH, expand=True, anchor='center')
photoFrame.grid(column=5,row=3, rowspan=18)

# FUNCTIONS TO HANDLE PIN MODE CHANGES, FUNTIONS MODIFY A TEMPORAL VARIABLE
# THAT CHANGE LAST VERIFIED STATUS WHEN UPDATE WAS SENT AND RECEIVE CONFIRMATION
# FROM ARDUINO BOARD, A LAVEL SHOW "PENDING UPDATE DATA BEFORE THIS HAPPENS" 

def textToButton(text,pin):
  global buttonMos1Stg, buttonMos2Stg, buttonMos3Stg, batteryVoltStg, trafoVoltStg, solarVoltStg, windVoltStg, photoResistorStg

  if pin==mosfet_1_pin:
      buttonMos1Stg.set(text)
  elif pin==mosfet_2_pin:
      buttonMos2Stg.set(text)
  elif pin==mosfet_3_pin:
      buttonMos3Stg.set(text)
  elif pin==battery_voltage_pin:
      batteryVoltStg.set(text)
  elif pin==device_charger_voltage_1:
      trafoVoltStg.set(text)
  elif pin==device_charger_voltage_2:
      solarVoltStg.set(text)
  elif pin==device_charger_voltage_3:
      windVoltStg.set(text)
  else: 
      photoResistorStg.set(text)

def setOutput(pin):
    global top, entryDDRB, entryDDRC, entryDDRD, entrydigitalOutputD, entrydigitalOutputB,  entrydigitalOutputC, entryservoRegisterD, entrypwmRegisterD, entryservoRegisterB, entrypwmRegisterB, entryservoRegisterC, entrypwmRegisterC,  , varentryMosfet1, varentryMosfet2, varentryMosfet3, varentryBattery, varentryTrafo, varentrySolar, varentryWind, varentryPhoto

    pin = int(pin)

    if (pin<8):
        entryDDRD= setBit(entryDDRD,pin)
        entrydigitalOutputD = setBit(entrydigitalOutputD,pin)
        entryservoRegisterD = unsetBit(entryservoRegisterD,pin)
        entrypwmRegisterD = unsetBit(entrypwmRegisterD,pin)

    elif (pin<14):
        entryDDRB= setBit(entryDDRB,pin-6)
        entrydigitalOutputB = setBit(entrydigitalOutputB,pin-6)
        entryservoRegisterB = unsetBit(entryservoRegisterB,pin-6)
        entrypwmRegisterB = unsetBit(entrypwmRegisterB,pin-6)

    else:
        print("setting to output")
        entryDDRC= setBit(entryDDRC,pin-14)
        entrydigitalOutputC = setBit(entrydigitalOutputC,pin-14)
        entryservoRegisterC = unsetBit(entryservoRegisterC,pin-14)
        entrypwmRegisterC = unsetBit(entrypwmRegisterC,pin-14)
        
    top.destroy()
    textToButton("OUTPUT",pin)
    if (pin== mosfet_1_pin):
        varentryMosfet1.set("")
        entryMosfet1.state(['disabled'])
        chkMos1State.set(False)
        chkMosfet1.state(['!disabled'])

    elif(pin== mosfet_2_pin):
        varentryMosfet2.set("")
        entryMosfet2.state(['disabled'])
        chkMos2State.set(False)
        chkMosfet2.state(['!disabled'])
            
    elif(pin== mosfet_3_pin):
        varentryMosfet3.set("")
        entryMosfet3.state(['disabled'])
        chkMos3State.set(False)
        chkMosfet3.state(['!disabled'])
            
    elif(pin== battery_voltage_pin):
        varentryBattery.set("")
        entryBatteryVolt.state(['disabled'])
        chkBatState.set(False)
        chkBatteryVolt.state(['!disabled'])
            
    elif(pin== device_charger_voltage_1):
            varentryTrafo.set("")
            entryTrafoVolt.state(['disabled'])
            chkTrafoState.set(False)
            chkTrafoVolt.state(['!disabled'])
    elif(pin== device_charger_voltage_2):
            varentrySolar.set("")
            entrySolarPanelVolt.state(['disabled'])
            chkSolarState.set(False)
            chkSolarPanelVolt.state(['!disabled'])
            
    elif(pin== device_charger_voltage_3):
            varentryWind.set("")
            entryWindTurbineVolt.state(['disabled'])
            chkWindState.set(False)
            chkWindTurbineVolt.state(['!disabled'])
    elif(pin== photo_resistor): 
            varentryPhoto.set("")
            entryPhotoreResistorVolt.state(['disabled'])
            chkPhotoState.set(False)
            chkPhotoreResistorVolt.state(['!disabled'])

setOutput_wrapper = root.register(setOutput)

def setPWM(pin):
    global top, entryDDRB, entryDDRC, entryDDRD, entrypwmRegisterD, entrypwmRegisterB,  entrypwmRegisterC, entryservoRegisterD, entrydigitalOutputD, entryservoRegisterB, entrydigitalOutputB, entryservoRegisterC, entrydigitalOutputC, varentryMosfet1, varentryMosfet2, varentryMosfet3, varentryBattery, varentryTrafo, varentrySolar, varentryWind, varentryPhoto

    pin = int(pin)

    if (pin<8):
        entryDDRD= setBit(entryDDRD,pin)
        entrypwmRegisterD = setBit(entrypwmRegisterD,pin)
        entryservoRegisterD = unsetBit(entryservoRegisterD,pin)
        entrydigitalOutputD = unsetBit(entrydigitalOutputD,pin)        

    elif (pin<14):
        entryDDRB= setBit(entryDDRB,pin-6)
        entrypwmRegisterB = setBit(entrypwmRegisterB,pin-6)
        entryservoRegisterB = unsetBit(entryservoRegisterB,pin-6)
        entrydigitalOutputB = unsetBit(entrydigitalOutputB,pin-6)        
        
    else:
        entryDDRC= setBit(entryDDRC,pin-14)
        entrypwmRegisterC = setBit(entrypwmRegisterC,pin-14)
        entryservoRegisterC = unsetBit(entryservoRegisterC,pin-14)
        entrydigitalOutputC = unsetBit(entrydigitalOutputC,pin-14)          


    top.destroy()
    textToButton("PWM",pin)
    if (pin== mosfet_1_pin):
        varentryMosfet1.set("")
        entryMosfet1.state(['disabled'])
        chkMos1State.set(False)
        chkMosfet1.state(['!disabled'])

    elif(pin== mosfet_2_pin):
        varentryMosfet2.set("")
        entryMosfet2.state(['disabled'])
        chkMos2State.set(False)
        chkMosfet2.state(['!disabled'])
            
    elif(pin== mosfet_3_pin):
        varentryMosfet3.set("")
        entryMosfet3.state(['disabled'])
        chkMos3State.set(False)
        chkMosfet3.state(['!disabled'])
            
    elif(pin== battery_voltage_pin):
        varentryBattery.set("")
        entryBatteryVolt.state(['disabled'])
        chkBatState.set(False)
        chkBatteryVolt.state(['!disabled'])
            
    elif(pin== device_charger_voltage_1):
            varentryTrafo.set("")
            entryTrafoVolt.state(['disabled'])
            chkTrafoState.set(False)
            chkTrafoVolt.state(['disabled'])
    elif(pin== device_charger_voltage_2):
            varentrySolar.set("")
            entrySolarPanelVolt.state(['disabled'])
            chkSolarState.set(False)
            chkSolarPanelVolt.state(['!disabled'])
            
    elif(pin== device_charger_voltage_3):
            varentryWind.set("")
            entryWindTurbineVolt.state(['disabled'])
            chkWindState.set(False)
            chkWindTurbineVolt.state(['!disabled'])
    elif(pin== photo_resistor): 
            varentryPhoto.set("")
            entryPhotoreResistorVolt.state(['disabled'])
            chkPhotoState.set(False)
            chkPhotoreResistorVolt.state(['!disabled'])

setPWM_wrapper = root.register(setPWM)

def setServo(pin):
    global top,entryDDRB, entryDDRC, entryDDRD, entryservoRegisterD, entryservoRegisterB,  entryservoRegisterC, entrypwmRegisterD, entrydigitalOutputD , entrypwmRegisterB, entrydigitalOutputB, entrypwmRegisterC, entrydigitalOutputC, varentryMosfet1, varentryMosfet2, varentryMosfet3, varentryBattery, varentryTrafo, varentrySolar, varentryWind, varentryPhoto

    pin = int(pin)
    if (pin<8):
        entryDDRD= setBit(entryDDRD,pin)
        entryservoRegisterD = setBit(entryservoRegisterD,pin)
        entrypwmRegisterD = unsetBit(entrypwmRegisterD,pin)
        entrydigitalOutputD = unsetBit(entrydigitalOutputD,pin)        


    elif (pin<14):
        entryDDRB= setBit(entryDDRB,pin-6)
        entryservoRegisterB = setBit(entryservoRegisterB,pin-6)
        entrypwmRegisterB = unsetBit(entrypwmRegisterB,pin-6)
        entrydigitalOutputB = unsetBit(entrydigitalOutputB,pin-6)        

    else:
        entryDDRC= setBit(entryDDRC,pin-14)
        entryservoRegisterC = setBit(entryservoRegisterC,pin-14)
        entrypwmRegisterC = unsetBit(entrypwmRegisterC,pin)
        entrydigitalOutputC = unsetBit(entrydigitalOutputC,pin-14)          

    top.destroy()
    textToButton("SERVO",pin)
    if (pin== mosfet_1_pin):
        varentryMosfet1.set("")
        entryMosfet1.state(['disabled'])
        chkMos1State.set(False)
        chkMosfet1.state(['!disabled'])

    elif(pin== mosfet_2_pin):
        varentryMosfet2.set("")
        entryMosfet2.state(['disabled'])
        chkMos2State.set(False)
        chkMosfet2.state(['!disabled'])
            
    elif(pin== mosfet_3_pin):
        varentryMosfet3.set("")
        entryMosfet3.state(['disabled'])
        chkMos3State.set(False)
        chkMosfet3.state(['!disabled'])
            
    elif(pin== battery_voltage_pin):
        varentryBattery= None
        entryBatteryVolt.state(['disabled'])
        chkBatState.set(False)
        chkBatteryVolt.state(['!disabled'])
            
    elif(pin== device_charger_voltage_1):
            varentryTrafo.set("")
            entryTrafoVolt.state(['disabled'])
            chkTrafoState.set(False)
            chkTrafoVolt.state(['!disabled'])
    elif(pin== device_charger_voltage_2):
            varentrySolar.set("")
            entrySolarPanelVolt.state(['disabled'])
            chkSolarState.set(False)
            chkSolarPanelVolt.state(['!disabled'])
            
    elif(pin== device_charger_voltage_3):
            varentryWind.set("")
            entryWindTurbineVolt.state(['disabled'])
            chkWindState.set(False)
            chkWindTurbineVolt.state(['!disabled'])
    elif(pin== photo_resistor): 
            varentryPhoto.set("")
            entryPhotoreResistorVolt.state(['disabled'])
            chkPhotoState.set(False)
            chkPhotoreResistorVolt.state(['!disabled'])

setServo_wrapper = root.register(setServo)

def setInput(pin):
    global top, entryDDRB, entryDDRC, entryDDRD, varentryMosfet1, chkMos1State, chkMosfet1, varentryMosfet2, chkMos2State, chkMosfet2, varentryMosfet3, chkMos3State, chkMosfet3, varentryBattery, chkBatState, chkBatteryVolt, varentryTrafo, chkTrafoState, chkTrafoVolt, varentrySolar, chkSolarState, chkSolarPanelVolt, varentryWind, chkWindState, chkWindTurbineVolt, varentryPhoto, chkPhotoState, chkPhotoreResistorVolt, entryMosfet1, entryMosfet2, entryMosfet3, entryBatteryVolt, entryTrafoVolt, entrySolarPanelVolt, entryWindTurbineVolt, entryPhotoreResistorVolt, entryservoRegisterD, entrypwmRegisterD, entrydigitalOutputD, entryservoRegisterB, entrypwmRegisterB, entrydigitalOutputB, entrypwmRegisterC, entrydigitalOutputC

    pin = int(pin)
    if (pin<8):
        entryDDRD= unsetBit(entryDDRD,pin)
        entryservoRegisterD = unsetBit(entryservoRegisterD,pin)
        entrypwmRegisterD = unsetBit(entrypwmRegisterD,pin)
        entrydigitalOutputD = unsetBit(entrydigitalOutputD,pin)        
    elif (pin<14):
        entryDDRB= unsetBit(entryDDRB,pin-6)
        entryservoRegisterB = unsetBit(entryservoRegisterB,pin-6)
        entrypwmRegisterB = unsetBit(entrypwmRegisterB,pin-6)
        entrydigitalOutputB = unsetBit(entrydigitalOutputB,pin-6)        
    else:
        toUpdateDDRC= unsetBit(entryDDRC,pin-14)
        entryservoRegisterC = unsetBit(entryservoRegisterC,pin-14)
        entrypwmRegisterC = unsetBit(entrypwmRegisterC,pin-14)
        entrydigitalOutputC = unsetBit(entrydigitalOutputC,pin-14)          

    top.destroy()
    textToButton("INPUT",pin)
    if (pin== mosfet_1_pin):
        varentryMosfet1.set("")
        entryMosfet1.state(['disabled'])
        chkMos1State.set(False)
        chkMosfet1.state(['disabled'])

    elif(pin== mosfet_2_pin):
        varentryMosfet2.set("")
        entryMosfet2.state(['disabled'])
        chkMos2State.set(False)
        chkMosfet2.state(['disabled'])
            
    elif(pin== mosfet_3_pin):
        varentryMosfet3.set("")
        entryMosfet3.state(['disabled'])
        chkMos3State.set(False)
        chkMosfet3.state(['disabled'])
            
    elif(pin== battery_voltage_pin):
        varentryBattery.set("")
        entryBatteryVolt.state(['disabled'])
        chkBatState.set(False)
        chkBatteryVolt.state(['disabled'])
            
    elif(pin== device_charger_voltage_1):
        varentryTrafo.set("")
        entryTrafoVolt.state(['disabled'])
        chkTrafoState.set(False)
        chkTrafoVolt.state(['disabled'])
    elif(pin== device_charger_voltage_2):
        varentrySolar.set("")
        entrySolarPanelVolt.state(['disabled'])
        chkSolarState.set(False)
        chkSolarPanelVolt.state(['disabled'])
            
    elif(pin== device_charger_voltage_3):
        varentryWind.set("")
        entryWindTurbineVolt.state(['disabled'])
        chkWindState.set(False)
        chkWindTurbineVolt.state(['disabled'])
    elif(pin== photo_resistor): 
            varentryPhoto.set("")
            entryPhotoreResistorVolt.state(['disabled'])
            chkPhotoState.set(False)
            chkPhotoreResistorVolt.state(['disabled'])

setInput_wrapper = root.register(setInput)

def setARead(pin):

    global top, entryDDRB, entryDDRC, entryDDRD, varentryMosfet1, chkMos1State, chkMosfet1, varentryMosfet2, chkMos2State, chkMosfet2, varentryMosfet3, chkMos3State, chkMosfet3, varentryBattery, chkBatState, chkBatteryVolt, varentryTrafo, chkTrafoState, chkTrafoVolt, varentrySolar, chkSolarState, chkSolarPanelVolt, varentryWind, chkWindState, chkWindTurbineVolt, varentryPhoto, chkPhotoState, chkPhotoreResistorVolt, entryMosfet1, entryMosfet2, entryMosfet3, entryBatteryVolt, entryTrafoVolt, entrySolarPanelVolt, entryWindTurbineVolt, entryPhotoreResistorVolt, entryservoRegisterD, entrypwmRegisterD, entrydigitalOutputD, entryservoRegisterB, entrypwmRegisterB, entrydigitalOutputB, entrypwmRegisterC, entrydigitalOutputC
    pin = int(pin)
    
    if (pin<14):
         messagebox.showwarning(title='Invalid imput', message="Analog Read only is possible in Analog Pins A0 to A7 (14-21)")
    else:
        toUpdateDDRC= unsetBit(entryDDRC,pin-14)
        entryservoRegisterC = unsetBit(entryservoRegisterC,pin-14)
        entrypwmRegisterC = unsetBit(entrypwmRegisterC,pin-14)
        entrydigitalOutputC = setBit(entrydigitalOutputC,pin-14)          

        textToButton("INPUT",pin)
        if (pin== mosfet_1_pin):
            varentryMosfet1.set("")
            entryMosfet1.state(['disabled'])
            chkMos1State.set(False)
            chkMosfet1.state(['disabled'])

        elif(pin== mosfet_2_pin):
            varentryMosfet2.set("")
            entryMosfet2.state(['disabled'])
            chkMos2State.set(False)
            chkMosfet2.state(['disabled'])
            
        elif(pin== mosfet_3_pin):
            varentryMosfet3.set("")
            entryMosfet3.state(['disabled'])
            chkMos3State.set(False)
            chkMosfet3.state(['disabled'])
            
        elif(pin== battery_voltage_pin):
            varentryBattery.set("")
            entryBatteryVolt.state(['disabled'])
            chkBatState.set(False)
            chkBatteryVolt.state(['disabled'])
            
        elif(pin== device_charger_voltage_1):
            varentryTrafo.set("")
            entryTrafoVolt.state(['disabled'])
            chkTrafoState.set(False)
            chkTrafoVolt.state(['disabled'])
        elif(pin== device_charger_voltage_2):
            varentrySolar.set("")
            entrySolarPanelVolt.state(['disabled'])
            chkSolarState.set(False)
            chkSolarPanelVolt.state(['disabled'])
            
        elif(pin== device_charger_voltage_3):
            varentryWind= ""
            entryWindTurbineVolt.state(['disabled'])
            chkWindState.set(False)
            chkWindTurbineVolt.state(['disabled'])
        elif(pin== photo_resistor): 
            varentryPhoto.set("")
            entryPhotoreResistorVolt.state(['disabled'])
            chkPhotoState.set(False)
            chkPhotoreResistorVolt.state(['disabled'])
    
setARead_wrapper = root.register(setARead)

        
# class to define custom boxes with plenty buttons to change pin status
# each button name is defined in *button as key and the function wrapped with register
# like this example:
# modeMessageBox= CustomMessage(pin, root, buttonOUTPUT = {"text": "OUTPUT", "command": (setOutput_wrapper, pin)}, buttonINPUT = {"text": "INPUT", "command": (setInput_wrapper, pin)}, buttonPWM = {"text": "PWM", "command": (setPWM_wrapper, pin)}, buttonServo = {"text": "SERVO", "command": (setServo_wrapper, pin)})  


class CustomMessage(object):

        def __init__(self,pin,parent,**button):

            global top, rootSizerWidth, screenWidth, screenHeight
            
            self.pin= pin
            self.parent=parent
            title= 'Pin Mode change selector'
            question = 'Please, choose the mode that you want to apply'
            
            # check window size to make root size

            rootSizerWidth= int(screenWidth*0.3)
            rootSizerHeight= int(screenHeight*0.25)

            topLeftPosition=(int((screenWidth- rootSizerWidth)/2),int((screenHeight- rootSizerHeight)/2))
            top=Toplevel(self.parent)
            top.geometry(f'{rootSizerWidth}x{rootSizerHeight}+{topLeftPosition[0]}+{topLeftPosition[1]}')
            top.transient(self.parent)
            top.title(title)
            top.frame=Frame(top)
            top.label=ttk.Label(top.frame, text=question)
            top.grab_set()

#            def setOutputforPin():              # command call back function with pin must be defined into the class
#                setOutput(self.pin)             # general action outside THIS IS TO DONT USE REGISTER, OTHER WAY
                
#            def setPWMforPin():
#                setPWM(self.pin)
                
#            def setServoforPin():
#                setServo(self.pin)
                
#            def setInputforPin():
#                setInput(self.pin)
                    
            i=1
            for key, value in button.items():
                i=i+1
                buttondata= dict()
                buttondata = value 
                key = ttk.Button(top.frame, text= buttondata["text"], command = buttondata["command"])
                key.grid(column =i, row =3, sticky=(W,E,N,S))
                top.frame.columnconfigure(i, weight=1)
                
            top.label.grid(column =1,columnspan=i, row =2)
            top.frame.columnconfigure(0, weight=1)
            top.frame.rowconfigure(0, weight=1)
            top.frame.rowconfigure(1, weight=1)
            top.frame.rowconfigure(2, weight=1)
            top.frame.rowconfigure(3, weight=1)
            top.frame.rowconfigure(4, weight=1)

            for child in top.frame.winfo_children():
                child.grid_configure(padx=int(root.winfo_width()*0.0075), pady=int(root.winfo_width()*0.0075))

            

            top.frame.grid(column =0, row =0, sticky=(W,E,N,S), pady=(screenWidth/600))


# Functions and alert Window for Buttons and Lanels for PIN MODE 

def onPressOk(pin):
    global alertWindow

    alertWindow.destroy()
    modeMessageBox= CustomMessage(pin, root, buttonOUTPUT = {"text": "OUTPUT", "command": (setOutput_wrapper, pin)}, buttonINPUT = {"text": "INPUT", "command": (setInput_wrapper, pin)}, buttonARead = {"text": "A. Read", "command": (setARead_wrapper, pin)}, buttonPWM = {"text": "PWM", "command": (setPWM_wrapper, pin)}, buttonServo = {"text": "SERVO", "command": (setServo_wrapper, pin)})  

    
def onPressCancell():
    global alertWindow
    alertWindow.destroy()

def onPressMode(pin):
    global alertWindow, screenWidth, screenWidth
    thispin = pin
    rootSizerWidth= int(screenWidth*0.3)
    rootSizerHeight= int(screenHeight*0.3)
    topLeftPosition=(int((screenWidth- rootSizerWidth)/2),int((screenHeight- rootSizerHeight)/2))
    alertWindow = Toplevel(root)
    alertWindow.geometry(f'{rootSizerWidth}x{rootSizerHeight}+{topLeftPosition[0]}+{topLeftPosition[1]}')
    alertWindow.transient(root)
    alertWindow.title("Warning, risk of damage")
    alertWindow.minsize(200,200)
    alertFrame = ttk.Frame(alertWindow, padding ="3 3 12 12")
    alertFrame.grid(column=0, row=0)
    ttk.Label(alertFrame, text= "Board mode change read or send voltage signal").grid(column=1, columnspan=4, row=1, sticky="nswe")
    ttk.Label(alertFrame, text= "THIS IS ONLY TO TEST BOARD CONNECTIONS AND MCU SIGNALS").grid(column=1, columnspan=4, row=2, sticky="nswe")
    
    def onPressOKforPin():
        nonlocal thispin
        onPressOk(thispin)
        
    ttk.Button(alertFrame, text="Proceed", command= onPressOKforPin).grid(column =2, row =4)
    ttk.Button(alertFrame, text="Cancell", command=onPressCancell).grid(column =3, row =4)
    ttk.Label(alertFrame, text= "       ").grid(column=4, row=4)
        

    for child in alertWindow.winfo_children():
        child.grid_configure(padx=int(root.winfo_width()/150), pady=int(root.winfo_width()/150))

onPressMode_wrapper= root.register(onPressMode)    

#def onPresMode_mosfet_1():             THIS IS TO DONT USE REGISTER, OTHER WAY
#    onPressMode(mosfet_1_pin)
        
#def onPresMode_mosfet_2():
#    onPressMode(mosfet_2_pin)

#def onPresMode_mosfet_3():
#    onPressMode(mosfet_3_pin)

#def onPresMode_mosfet_3():
#    onPressMode(mosfet_3_pin)

#def onPresMode_battery_voltage():
#    onPressMode(battery_voltage_pin)

#def onPresMode_device_voltage_1():
#    onPressMode(device_charger_voltage_1)

#def onPresMode_device_voltage_2():
#    onPressMode(device_charger_voltage_2)

#def onPresMode_device_voltage_3():
#    onPressMode(device_charger_voltage_3)

#def onPresMode_photo_resistor():
#    onPressMode(photo_resistor)

# Buttons and Labels for PIN MODE 

buttonMos1Stg= StringVar()
buttonMos1Stg.set('PWM')

ttk.Label(mainFrame, text= f'{mosfet_1_pin}').grid(column=0,row=3, sticky='n')
ttk.Label(mainFrame, text= "Transformer AC/DC Mosfet").grid(column=1,row=3, sticky='n')
buttonMos1= ttk.Button(mainFrame, state='disabled', textvariable=buttonMos1Stg, command = (onPressMode_wrapper, mosfet_1_pin))
buttonMos1.grid(column =1, row =4, sticky='s')

buttonMos2Stg= StringVar()
buttonMos2Stg.set('PWM')

ttk.Label(mainFrame, text= f'{mosfet_2_pin}').grid(column=0,row=5, sticky='n')
ttk.Label(mainFrame, text= "Solar Panel Mosfet").grid(column=1,row=5, sticky='n')
buttonMos2 = ttk.Button(mainFrame, state='disabled', textvariable=buttonMos2Stg, command = (onPressMode_wrapper, mosfet_2_pin))
buttonMos2.grid(column =1, row =6, sticky='s')

buttonMos3Stg= StringVar()
buttonMos3Stg.set('PWM')

ttk.Label(mainFrame, text= f'{mosfet_3_pin}').grid(column=0,row=7, sticky='n')
ttk.Label(mainFrame, text= "Wind generator Mosfet").grid(column=1,row=7, sticky='n')
buttonMos3 = ttk.Button(mainFrame, state='disabled', textvariable=buttonMos3Stg, command = (onPressMode_wrapper, mosfet_3_pin))
buttonMos3.grid(column =1, row =8, sticky='s')


batteryVoltStg= StringVar()
batteryVoltStg.set('INPUT')

ttk.Label(mainFrame, text= f'{battery_voltage_pin}').grid(column=0,row=9, sticky='n')
ttk.Label(mainFrame, text= "Battery Voltage").grid(column=1,row=9, sticky='n')
batteryVolt = ttk.Button(mainFrame, state='disabled', textvariable=batteryVoltStg, command = (onPressMode_wrapper, battery_voltage_pin))
batteryVolt.grid(column =1, row =10, sticky='s')

trafoVoltStg= StringVar()
trafoVoltStg.set('INPUT')

ttk.Label(mainFrame, text= f'{device_charger_voltage_1}').grid(column=0,row=11, sticky='n')
ttk.Label(mainFrame, text= "Transformer Voltage").grid(column=1,row=11, sticky='n')
trafoVolt = ttk.Button(mainFrame, state='disabled', textvariable=trafoVoltStg, command = (onPressMode_wrapper, device_charger_voltage_1))
trafoVolt.grid(column =1, row =12, sticky='s')

solarVoltStg= StringVar()
solarVoltStg.set('INPUT')

ttk.Label(mainFrame, text= f'{device_charger_voltage_2}').grid(column=0,row=13, sticky='n')
ttk.Label(mainFrame, text= "Solar panel Voltage").grid(column=1,row=13, sticky='n')
solarVolt = ttk.Button(mainFrame, state='disabled', textvariable=solarVoltStg, command = (onPressMode_wrapper, device_charger_voltage_2))
solarVolt.grid(column =1, row =14, sticky='s')

windVoltStg= StringVar()
windVoltStg.set('INPUT')

ttk.Label(mainFrame, text= f'{device_charger_voltage_3}').grid(column=0,row=15, sticky='n')
ttk.Label(mainFrame, text= "Wind gen. Voltage").grid(column=1,row=15, sticky='n')
windVolt = ttk.Button(mainFrame, state='disabled', textvariable=windVoltStg, command = (onPressMode_wrapper, device_charger_voltage_3))
windVolt.grid(column =1, row =16, sticky='s')

pin= photo_resistor

photoResistorStg= StringVar()
photoResistorStg.set('INPUT')

ttk.Label(mainFrame, text= f'{photo_resistor}').grid(column=0,row=17, sticky='n')
ttk.Label(mainFrame, text= "Photo resistor pin").grid(column=1,row=17, sticky='n')
photoResistor = ttk.Button(mainFrame, state='disabled', textvariable=photoResistorStg, command = (onPressMode_wrapper, photo_resistor))
photoResistor.grid(column =1, row =18, sticky='s')
 
    
  
# Labels for current values of PINS

stringMos1= convertionReadToVolts(mosfet_1_pin)  
textMos1 = StringVar()
textMos1.set(stringMos1) 
valMos1 = ttk.Label(mainFrame,textvariable=textMos1).grid(column=2,row=4)

stringMos2= convertionReadToVolts(mosfet_2_pin)
textMos2 = StringVar()
textMos2.set(stringMos2)

valMos2 = ttk.Label(mainFrame,textvariable=textMos2).grid(column=2, row=6)

stringMos2 = convertionReadToVolts(mosfet_3_pin) 
textMos3 = StringVar()
textMos3.set(stringMos2)
valMos3 = ttk.Label(mainFrame,textvariable=textMos3).grid(column=2, row=8)

StringBattery = convertionReadToVolts(battery_voltage_pin)
textBattery = StringVar()
textBattery.set(StringBattery)
valBattery = ttk.Label(mainFrame,textvariable=textBattery).grid(column=2, row=10,)

stringTrafo = convertionReadToVolts(device_charger_voltage_1) 
textTrafo = StringVar()
textTrafo.set(stringTrafo)
valTrafo = ttk.Label(mainFrame,textvariable=textTrafo).grid(column=2, row=12)

stringSolar = convertionReadToVolts(device_charger_voltage_2)
textSolar = StringVar()
textSolar.set(stringSolar)
valSolar = ttk.Label(mainFrame,textvariable=textSolar).grid(column=2, row=14)

stringWing = convertionReadToVolts(device_charger_voltage_3)
textWind = StringVar()
textWind.set(stringWing)
valWind = ttk.Label(mainFrame,textvariable=textWind).grid(column=2,row =16)

strongPhoto = convertionReadToVolts(photo_resistor) 
textPhotoR = StringVar()
textPhotoR.set(strongPhoto)
valPhotoR = ttk.Label(mainFrame,textvariable=textPhotoR).grid(column =2, row =18)

# functions to enable entry and disable with a .bind

def enablerMos1():
    global PC_CONTROL_STATE, entryMosfet1, chkMos1State
    boolchk = chkMos1State.get()
    
    if ((PC_CONTROL_STATE ==1) & boolchk ):
          entryMosfet1.state(['!disabled'])
    else:
          entryMosfet1.state(['disabled'])
          chkMos1State.set(False)
          
def enablerMos2():
    global PC_CONTROL_STATE, entryMosfet2, chkMos2State
 
    boolchk= chkMos2State.get()
    
    if ((PC_CONTROL_STATE ==1) & boolchk ):
          entryMosfet2.state(['!disabled'])      
    else:
          entryMosfet2.state(['disabled'])
          chkMos2State.set(False)

def enablerMos3():
    global PC_CONTROL_STATE, entryMosfet3,  chkMos3State

    boolchk = chkMos3State.get()
    
    if ((PC_CONTROL_STATE ==1) & boolchk ):
          entryMosfet3.state(['!disabled'])      
    else:
          entryMosfet3.state(['disabled'])
          chkMos3State.set(False)

def enablerBattery():
    global PC_CONTROL_STATE, entryBatteryVolt, chkBatState

    boolchk= chkBatState.get()
    
    if ((PC_CONTROL_STATE ==1) & boolchk ):
          entryBatteryVolt.state(['!disabled'])      
    else:
          entryBatteryVolt.state(['disabled'])
          chkBatState.set(False)

def enablerTrafoVolt():
    global PC_CONTROL_STATE, entryTrafoVolt, chkTrafoState

    boolchk= chkTrafoState.get()
    
    if ((PC_CONTROL_STATE ==1) & boolchk ):
          entryTrafoVolt.state(['!disabled'])      
    else:
          entryTrafoVolt.state(['disabled'])
          chkTrafoState.set(False)

def enablerSolar():
    global PC_CONTROL_STATE, entrySolarPanelVolt, chkSolarState    

    boolchk= chkSolarState.get()
    
    if ((PC_CONTROL_STATE ==1) & boolchk ):
          entrySolarPanelVolt.state(['!disabled'])      
    else:
          entrySolarPanelVolt.state(['disabled'])          
          chkSolarState.set(False)

def enablerWind():
    global PC_CONTROL_STATE, entryWindTurbineVolt, chkWindState

    boolchk= chkWindState.get()
    
    if ((PC_CONTROL_STATE ==1) & boolchk ):
          entryWindTurbineVolt.state(['!disabled'])      
    else:
          entryWindTurbineVolt.state(['disabled'])          
          chkWindState.set(False)

def enablerPhoto():
    global PC_CONTROL_STATE, entryPhotoreResistorVolt, chkPhotoState

    boolchk= chkPhotoState.get()
    
    if ((PC_CONTROL_STATE ==1) & boolchk ):
          entryPhotoreResistorVolt.state(['!disabled'])      
    else:
          entryPhotoreResistorVolt.state(['disabled'])          
          chkPhotoState= BooleanVar(value= False)
    # Check buttons to change value of a ping
s =ttk.Style()
s.configure('Brigther.TCheckbutton', background="greenYellow", foreground ="midnightblue") 
s.map('Brigther.TCheckbutton', foreground =[ ('disabled', 'seashell3'), ('pressed','seashell1'),('active','seashell2')], background =[ ('disabled', 'palegreen'), ('pressed','greenYellow'),('active','Springgreen2')])

chkMosfet1= ttk.Checkbutton(mainFrame, command = enablerMos1, state= 'disabled', variable=chkMos1State, onvalue=True, offvalue=False, style = 'Brigther.TCheckbutton')
chkMosfet1.grid(column =4, row =4)

chkMosfet2= ttk.Checkbutton(mainFrame, command= enablerMos2, state= 'disabled', variable=chkMos2State, onvalue=True, offvalue=False,  style = 'Brigther.TCheckbutton')
chkMosfet2.grid(column =4, row =6)

chkMosfet3= ttk.Checkbutton(mainFrame, command= enablerMos3, state= 'disabled', variable=chkMos3State, onvalue=True, offvalue=False,style = 'Brigther.TCheckbutton')
chkMosfet3.grid(column =4, row =8)

chkBatteryVolt= ttk.Checkbutton(mainFrame, command= enablerBattery, state= 'disabled', variable=chkBatState, onvalue=True, offvalue=False,style = 'Brigther.TCheckbutton')
chkBatteryVolt.grid(column =4, row =10)

chkTrafoVolt= ttk.Checkbutton(mainFrame,command= enablerTrafoVolt, state= 'disabled', variable=chkTrafoState,onvalue=True, offvalue=False,style = 'Brigther.TCheckbutton')
chkTrafoVolt.grid(column =4, row =12)

chkSolarPanelVolt= ttk.Checkbutton(mainFrame, command=enablerSolar, state= 'disabled', variable=chkSolarState, onvalue=True, offvalue=False,style = 'Brigther.TCheckbutton')
chkSolarPanelVolt.grid(column =4, row =14)

chkWindTurbineVolt= ttk.Checkbutton(mainFrame, command=enablerWind, state= 'disabled', variable=chkWindState, onvalue=True, offvalue=False,style = 'Brigther.TCheckbutton')
chkWindTurbineVolt.grid(column =4, row =16)

chkPhotoreResistorVolt= ttk.Checkbutton(mainFrame, command= enablerPhoto, state= 'disabled', variable=chkPhotoState,onvalue=True, offvalue=False,style = 'Brigther.TCheckbutton')
chkPhotoreResistorVolt.grid(column =4, row =18)


#Validation the entry with re numbers 1 digit . 0 to 3 digits 
# validation= re.compile(r'[0-5]?(.{1}\d{0,3})?$') is necesary erase dot in focusout

# Temporal pin check state according to user selection previos update. Similar to current status check.

def chkTempPinInINPUT(pin):
    if (pin>13):
        if not(bitON(entryDDRC, pin-14)):
            return True
    elif (pin>7):
        if not (bitON(entryDDRB, pin-6)):
            return True
    elif (pin<8):
        if not (bitON(entryDDRD, pin)):
            return True
    return False

def chkTempPinInPWM(pin):
    if (pin>13):
        if(bitON(entrypwmRegisterC, pin-14)):
            print ("check registerC")
            return True
    elif (pin>7):
        if(bitON(entrypwmRegisterB, pin-6)):
            print ("check registerB")
            return True
    elif (pin<8):
        if(bitON(entrypwmRegisterD, pin)):
            print ("check registerD")
            return True
    return False

def chkTempPinInServo(pin):
    if (pin>13):
        if(bitON(entryservoRegisterC, pin-14)):
            return True
    elif (pin>7):
        if(bitON(entryservoRegisterB, pin-6)):
            return True
    elif (pin<8):
        if(bitON(entryservoRegisterD, pin)):
            return True
    return False

def chkTempPinInOUTPUT(pin):                
    if (pin>13):
        if(bitON(entrydigitalOutputC, pin-14)):
            print("output register C")
            return True
    elif (pin>7):
        if(bitON(entrydigitalOutputB, pin-6)):
            print("output register B")
            return True
    elif (pin<8):
        if(bitON(entrydigitalOutputD, pin)):
            print("output register D")
            return True
    return False

prevInput=False
prevPWM=False
prevServo=False
prevOutput=False

prevpinandPinMOde= dict()


# value = value of the insertion to validate ( %P sustitution in wrapper)
# key= type of action key, focus in, focus out, etc ( %V sustitution in wrapper)
# char = last character entry ( %S sustitution in wrapper NOT USED IN THIS CASE)
# del_inst = 0 for deletion , 1 for insertion (%d sustitution in wrapper used only in OUTPUT)

def setVariableText(val, _pin):
    global mosfet_1_pin, mosfet_2_pin, mosfet_3_pin, battery_voltage_pin, device_charger_voltage_1, device_charger_voltage_2, device_charger_voltage_3, photo_resistor, varentryMosfet1, varentryMosfet2, varentryMosfet3, varentryBattery, varentryTrafo, varentrySolar, varentryWind, varentryPhoto
    if(_pin==mosfet_1_pin):
        varentryMosfet1.set(val)
    elif(_pin==mosfet_2_pin):
        varentryMosfet2.set(val)
    elif(_pin==mosfet_3_pin):
        varentryMosfet3.set(val)
    elif(_pin==battery_voltage_pin):
        varentryBattery.set(val)
    elif(_pin==device_charger_voltage_1):
        varentryTrafo.set(val)
    elif(_pin==device_charger_voltage_2):
        varentrySolar.set(val)
    elif(_pin==device_charger_voltage_3):
        varentryWind.set(val)
    elif(_pin==photo_resistor):
        varentryPhoto.set(val)
        
def getVariableText(_pin):
    global mosfet_1_pin, mosfet_2_pin, mosfet_3_pin, battery_voltage_pin, device_charger_voltage_1, device_charger_voltage_2, device_charger_voltage_3, photo_resistor, varentryMosfet1, varentryMosfet2, varentryMosfet3, varentryBattery, varentryTrafo, varentrySolar, varentryWind, varentryPhoto
    if(_pin==mosfet_1_pin):
        return varentryMosfet1.get()
    elif(_pin==mosfet_2_pin):
        return varentryMosfet2.get()
    elif(_pin==mosfet_3_pin):
        return varentryMosfet3.get()
    elif(_pin==battery_voltage_pin):
        return varentryBattery.get()
    elif(_pin==device_charger_voltage_1):
        return varentryTrafo.get()
    elif(_pin==device_charger_voltage_2):
        return varentrySolar.get()
    elif(_pin==device_charger_voltage_3):
        return varentryWind.get()
    elif(_pin==photo_resistor):
        return varentryPhoto.get()


def validationFunction(value, key,char, del_inst, pin):
    global prevInput, prevPWM, prevServo, prevOutput, prevpinandPinMOde
    pin = int(pin)
    
    pinandPinMOde= dict()
    
    if (chkTempPinInINPUT(pin)):
        pinandPinMOde[pin] = "IsInput"
        
    if (chkTempPinInPWM(pin)):
        pinandPinMOde[pin] = "IsPWM"

    if (chkTempPinInServo(pin)):
        pinandPinMOde[pin] = "IsServo"

    if (chkTempPinInOUTPUT(pin)):
        pinandPinMOde[pin] = "IsOuput"
        
    if (pinandPinMOde.get(pin)== "IsInput"):
        print("input")
        if(pinandPinMOde.get(pin) != prevpinandPinMOde.get(pin)):
            value=""
            setVariableText("", pin)
            prevpinandPinMOde[pin] = pinandPinMOde.get(pin)
            
        messagebox.showwarning(title="Invalid imput", message="Pin mode is in INPUT can't be modified") 
        setVariableText(value, pin)
        return False

    if (pinandPinMOde.get(pin)=="IsOuput"):
        print("output")
        print("isoutput =" + f'{pinandPinMOde.get(pin)}')
        print("prev comp = " + f'{(pinandPinMOde.get(pin) != prevpinandPinMOde.get(pin))}')
        print("Prev output out if =" + f'{prevpinandPinMOde.get(pin)}')

        if(pinandPinMOde.get(pin) != prevpinandPinMOde.get(pin)):
            value=""
            setVariableText("", pin)
            prevpinandPinMOde[pin] = pinandPinMOde.get(pin)
        print("Prev output after in if =" + f'{prevpinandPinMOde.get(pin)}')

        if (int(del_inst)==0):
            value=""        
            setVariableText("", pin)
            print("del_inst =" + f'{del_inst} Ok')
            
        print("del_inst =" + f'{del_inst}')

        valueck= re.match('[0-1]',value) is not None and len(value)<=1
        
        if key=='key':
            if len(value)>0:
                if not (valueck):
                    messagebox.showwarning(title="Invalid imput", message="Pin mode OUTPUT must be 1 (HIGH) or 0 (LOW) ")             
                    value=""
                    setVariableText("", pin)
        elif key=='focusout':
            if len(value)>0:
                if not valueck:
                    messagebox.showwarning(title='Invalid imput', message="Format should be from 0 to 180 for Servo")
                    value=""
                    setVariableText("", pin)
                else:
                    setVariableText(value, pin)
            
        return (valueck )

    if (pinandPinMOde.get(pin)== "IsPWM"):
        print("pwm")
        print("isPWM =" + f'{pinandPinMOde.get(pin)}')
        print("prev comp = " + f'{(pinandPinMOde.get(pin) != prevpinandPinMOde.get(pin))}')
        print("Prev PWM out if =" + f'{prevpinandPinMOde.get(pin)}')

        if(pinandPinMOde.get(pin) != prevpinandPinMOde.get(pin)):
            value=""
            setVariableText("", pin)
            prevpinandPinMOde[pin] = pinandPinMOde.get(pin)

            print("Prev PWM in if =" + f'{prevpinandPinMOde.get(pin)}')
        print("Prev PWM after in if =" + f'{prevpinandPinMOde.get(pin)}')
        print("len:" + f'{len(value)}')
        valuematch= re.match(r'[0-4]{0,1}((\.{1}\d{0,3}){0,1})$', value)
        print("value: " + value)
        print("Key: " + key)
        print("char: " + char)
        print("valuematch: " + f'{valuematch}')
        valueck= valuematch is not None and len(value) <= 5
        if key=='key':
            if len(value)>0:
                if not valueck:
                    messagebox.showwarning(title='Invalid imput', message="Format should be from 0.000 to 4.999 for PWM")
                    setVariableText("", pin)
                    value=""
        elif key=='focusout':
            if len(value)>0:
                if not valueck:
                    messagebox.showwarning(title='Invalid imput', message="Format should be from 0.000 to 4.999 for PWM")
                    setVariableText("", pin)
                    value=""
            else:
                print(f'{value.endswith('.')}')
                if value.endswith('.'):
                    value= value.removesuffix('.')
                    setVariableText(value, pin)
                       
    return valueck

    if (pinandPinMOde.get(pin)== "IsServo"):
        print("servo")
        
        if(pinandPinMOde.get(pin) != prevpinandPinMOde.get(pin)):
            value=""
            setVariableText(value, pin)
            prevpinandPinMOde[pin] = pinandPinMOde.get(pin)
        
        valueck= re.match(r'[0-1]{0,1}\d{0,2}$', value) is not None and len(value) <= 3 and int(value) <= 180
        if key=='key':
            if len(value)>0:
                if not valueck:
                    messagebox.showwarning(title='Invalid imput', message="Format should be from 0 to 180 for Servo")
                    setVariableText("", pin)
                    value=""
        elif key=='focusout':
            if not valueck:
                messagebox.showwarning(title='Invalid imput', message="Format should be from 0 to 180 for Servo")
                setVariableText("", pin)
                value=""
            else:
                setVariableText(value, pin)
                
    return valueck
        

check_num_wrapper = root.register(validationFunction)


    # Entries for change values


entryMosfet1= ttk.Entry(mainFrame,state= 'disabled', textvariable= varentryMosfet1,  validate='all', validatecommand=(check_num_wrapper, '%P','%V','%S','%d',mosfet_1_pin))

entryMosfet2= ttk.Entry(mainFrame,state= 'disabled', textvariable= varentryMosfet2, validate='all', validatecommand=(check_num_wrapper, '%P','%V','%S','%d',mosfet_2_pin))

entryMosfet3= ttk.Entry(mainFrame,state= 'disabled', textvariable= varentryMosfet3, validate='all', validatecommand=(check_num_wrapper, '%P','%V','%S','%d',mosfet_3_pin))

entryBatteryVolt= ttk.Entry(mainFrame,state= 'disabled', textvariable= varentryBattery, validate='all', validatecommand=(check_num_wrapper, '%P','%V','%S','%d',battery_voltage_pin))

entryTrafoVolt= ttk.Entry(mainFrame,state= 'disabled', textvariable= varentryTrafo, validate='all', validatecommand=(check_num_wrapper, '%P','%V','%S','%d',device_charger_voltage_1))

entrySolarPanelVolt= ttk.Entry(mainFrame,state= 'disabled', textvariable= varentrySolar, validate='all', validatecommand=(check_num_wrapper, '%P','%V','%S','%d',device_charger_voltage_2))

entryWindTurbineVolt= ttk.Entry(mainFrame,state= 'disabled', textvariable= varentryWind, validate='all', validatecommand=(check_num_wrapper, '%P','%V','%S','%d',device_charger_voltage_3))

entryPhotoreResistorVolt= ttk.Entry(mainFrame,state= 'disabled', textvariable= varentryPhoto, validate='all', validatecommand=(check_num_wrapper, '%P','%V','%S','%d',device_charger_voltage_3))

entryMosfet1.grid(column =3, row =4)
entryMosfet2.grid(column =3, row =6)
entryMosfet3.grid(column =3, row =8)
entryBatteryVolt.grid(column =3, row =10)
entryTrafoVolt.grid(column =3, row =12)
entrySolarPanelVolt.grid(column =3, row =14)
entryWindTurbineVolt.grid(column =3, row =16)
entryPhotoreResistorVolt.grid(column =3, row =18)

updatelabel = StringVar()
def onPressProceed():
    global PORTB, PORTC, PORTD, entryservoData, entrypwmData, entryPORTB, entryPORTC, entryPORTD, servoData, pwmData, updatelabel, mosfet_1_pin, mosfet_2_pin, mosfet_3_pin, battery_voltage_pin, device_charger_voltage_1, device_charger_voltage_2, device_charger_voltage_3, photo_resistor 
    print("process")
    sleep(20)
    tempPORTB = PORTB
    tempPORTC = PORTC
    tempPORTD = PORTD
    tempServoData = servoData
    temppwmData = pwmData

    totalPins=[mosfet_1_pin, mosfet_2_pin, mosfet_3_pin, battery_voltage_pin, device_charger_voltage_1, device_charger_voltage_2, device_charger_voltage_3, photo_resistor]

    for pin in totalPins:
        if len(getVariableText(pin))>0:
            intValue=int(getVariableText(pin))
            if (chkTempPinInServo(pin)):
                servoValue = convertionToWriteServo(intValue)
                entryservoData[f'{pin}'] = servoValue
            if (chkTempPinInOUTPUT(pin)):
                if (pin>14):
                    if(intValue>0):
                        entryPORTC=setBit(entryPORTC, pin-14)
                    else:
                        entryPORTC=unsetBit(entryPORTC, pin-14)
                elif(pin<8):
                    if(intValue>0):
                        entryPORTD=setBit(entryPORTD, pin)
                    else:
                        entryPORTD=unsetBit(entryPORTD, pin)
                else:
                    if(intValue>0):
                        entryPORTB=setBit(entryPORTB, pin-6)
                    else:
                        entryPORTB=unsetBit(entryPORTB, pin-6)
            if (chkTempPinInPWM(pin)):
                pwmValue = convertionToWritePWM(intValue)
                entrypwmData[f'{pin}'] = pwmVale
    			
    PORTC = entryPORTC
    PORTB = entryPORTB
    PORTD = entryPORTD
    pwmData = entrypwmData
    servoData = entryservoData

    if (sendBoardUpdate()==-1):
        print("board update-1")
        sleep(20)
        PORTB = tempPORTB
        PORTC = tempPORTC
        PORTD = tempPORTD
        servoData = tempServoData
        pwmData = temppwmData
        updatelabel.set("LAST INFORMATION WAS NOT SENT,  CONNECTION  NOT AVAILABLE")
    else:
        secs = time.time()
        timehere= time.localtime(secs)
        message=f'Last INFORMATION  WAS SENT AT= {timehere.tm_hour}:{timehere.tm_min}:{timehere.tm_seg}'
        conectionStatus.configure(foreground = 'DeepSkyBlue2')
        conectionText.set(message)

proceedButton=ttk.Button(mainFrame,state='disabled', text="SEND UPDATE", command=onPressProceed)
proceedButton.grid(column =4, row =1) 


# label and buttona to plot and print historic file
ttk.Label(mainFrame, text="Historical data reporting:").grid(column =1, row =19, sticky='sw')                            
ttk.Button(mainFrame, text="Plot").grid(column =2, row =20)
ttk.Button(mainFrame, text="Print").grid(column =3, row =20)

# Label for detected Board

boardDetected = ttk.Label(mainFrame, textvariable = arduinoLabel,foreground='firebrick1')
boardDetected.grid(column =5, row =20, sticky='w')  

# Label for last update

lastUpdateLabel = ttk.Label(mainFrame, textvariable = updatelabel,foreground='firebrick1')
lastUpdateLabel.grid(column =5, row =21, sticky='w') 

# pending sample data appand on file and plot and print and eventually erase file
def onPressPlot():
  pass
  
def onPressPrint():
  pass
ttk.Label(mainFrame, text="Change of mode is attampted to be for test only f.e. wires or check analog output").grid(column =1,columnspan=5, row =21,  sticky='w')                            
ttk.Label(mainFrame, text="Arduino only have three timers in complementary pins pairs so PWM must use only ONE of EACH PAIR FOR A SINGLE NON COMPLEMENTARY OUTPUT.").grid(column =1,columnspan=5, row =22, sticky='w')                            
ttk.Label(mainFrame, text="History data save a day file with each charger voltage and light each 15 minutes sample ").grid(column =1,columnspan=5, row =23, sticky='w')                            
ttk.Label(mainFrame, text="This data can be plotted for day, week, month or year").grid(column =1,columnspan=5, row =24, sticky='w')                            




                          
for child in mainFrame.winfo_children():
  child.grid_configure(padx=int(root.winfo_width()*0.007), pady=int(root.winfo_width()*0.007))

# CUSTOM EVENT TO HANDLE DATA WITH ARDUINO WITH TIME (REQUEST DATA EACH 5 MINUTES, SAVE EACH 15 MINUTES) AND IDLE OPTION (AFTER ALL WIDGET WORKS ARE FINISHED CAN BE CHAGED FOR LESS TIME 

starttimerData = time.perf_counter()  # this will make a counter with next call in seconds
starttimerSave = time.perf_counter()


def eventTimeFunction():
    global starttimerData, starttimerSave, arduinoPort, conectionStatus, connectionText
    print("event called")
    print(starttimerData)

    def ifreceiveData():
            global conectionStatus, connectionText
            
            if (receiveData()==-1):
                connectionText.set("SERIAL CONNECTION TO ARDUINO NOT AVAILABLE, CHANGE PORT OR SCAN TO VERIFY ")
            else:
                secs = time.time()
                timehere= time.localtime(secs)
                message=f'Last RECEIVED Update: {timehere.tm_hour}:{timehere.tm_min}:{timehere.tm_seg}'
                conectionStatus.configure(foreground = 'DeepSkyBlue2')
                conectionText.set(message)

    print("selectedport = " + arduinoPort)
    nexttimer = time.perf_counter()
    elapsedtimeData =  nexttimer - starttimerData
    elapsedtimeSave =   nexttimer - starttimerSave
    print(elapsedtimeData)
    if ((int(elapsedtimeData*1000)) > (int(TIME_UPDATE)*1000)):  # * 1000 to improve precision
        print("ready if 1")
        #root.after_idle(ifreceiveData)
        ifreceiveData()
        starttimerData = time.perf_counter()
        if (int(elapsedtimeSave*1000) > ((TIME_SAVE*60)*1000)):
            secs = time.time()
            timehere= time.localtime(secs)
            timestamp= f'{timehere.tm_year}{timehere.tm_mon}{timehere.tm_mday}{timehere.tm_hour}{timehere.tm_min}'
            print(timestamp)
            pass  # here will be file data WRITE FILE append first time  stamp next line each 3 voltage readings if not open open, 'a' etc 
            starttimerSave = time.perf_counter()
    root.after((200), eventTimeFunction)
            
root.after(100, eventTimeFunction)

root.bind('<Configure>',boardResize)

root.mainloop()                                 

# end of file
