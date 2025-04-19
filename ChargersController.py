######UNFINISHED WORKING ON IT DONT COMPILE#######

##Program to control arduino Nano board with the ChargersController.ino
#autor: Dario Lobos 17/mar/2025

import Tkinter
import PIL
from tkinter import *
from tkinter import ttk
from tkinter import messagebox
from PIL import ImageTk, Image
import os
import serial
import re
from time import sleep

# Define port type
arduinoPort = 'COM1' # Down in the code will have the choise for change, this is default

#_____________________________________________________________________#
#                                                                     #
#                           SERIAL PROTOCOL                           #
#_____________________________________________________________________#
#
ser =serial.Serial(port=arduinoPort,baudrate=9600, bytesize= serial.EIGHTBITS, parity= serial.PARITY_EVEN,stopbits=serial.STOPBITS_ONE, timeout=2, write_timeout=2)

# THESE ARE THE VARIABLES USED TO STORE ARDUINO STATUS RECEIVED AND ALSO
# TO MODIFIED AND SEND THEM BACK TO CHANGE ARDUINO CONFIGURATION

# PORTn define is Pins are in HIGH = 1 or LOW =0
PORTB=0
PORTC=0
PORTD=0
# DDRn define mode INPUT=0 OUTPUT=1
DDRB=0
DDRC=0
DDRD=0



SEND_STATUS=17; # THIS IS THE IDENTIFIER OR FALSE ADDREESS TO REQUEST ALL PINS STATUS
RECEIVED=23;  # THIS IS THE IDENTIFIER FOR SEND OF DATA

IS_ANALOG_READ = 28

ROW=22

arrayRead = dict()
for i in range (8):
    for v in range (2):
        arrayRead[f'{i}{v}']=0 # ARRAY TO STORE ANALOG READ 1=HIGH 0=LOW BYTE
IS_PWM= 18
pwmRegisterB = 0
pwmRegisterC = 0
pwmRegisterD = 0

pwmData= dict()
for i in range (8):
    pwmData[f'{i}']=0  # ARRAY TO STORE PWM DATA
    
IS_SERVO= 20

servoRegisterB = 0 # REGISTERS TO IDENTIFY EACH SERVO PIN
servoRegisterC = 0
servoRegisterD = 0

servoData = dict()
for i in range (8):
    servoData[f'{i}']=0 # ARRAY TO STORE SERVO DATA

RESEND= 19
WAIT= 22
END = 23
CHRNULL = 0

BOARD_INFO =6; # THIS IS TO SEND DATA OBTAINED FROM BOARD IDENTIFIER LIBRARY
PC_REGISTERS_UPDATE = 14; # THIS IS TO SEND ALL THE REGISTERS AND DATA CHANGED FOR THE PC

COM_ATTEMPTS=20;  # THIS IS THE LIMIT OF LOOPS THAT PROGRAM MUST DO TO TRY TO STABLISH COMMUNICATION BEFORE RAISE AN ERROR 

AVAILABLE = True;  # THIS IS THE FLAG TO DETERMINE UNAVAILABLE STATE;





# ALL THESE STORE LAST SENT TO AVOID RESEND THE SAME
 
prevPortB = 0
prevDDRB = 0
prevPortC = 0
prevDDRC = 0
prevPortD = 0
prevDDRD = 0

# TO SET DIGITAL PINS CAN BE USED DIRECT PINBx WHICH WILL BECOME OUTPUT
# INDEPENDENT OF DDR. I PREFER USE THIS TO AVOID PROBLEMS WITH ANALOG

prevpwmData= dict()
prevservoData = dict()             
for i in range (8):
    prevpwmData[f'{i}']=0  # ARRAY TO STORE PREVIOS PWM DATA
    prevservoData[f'{i}']=0  # ARRAY TO STORE PREVIOS SERVO DATA

attempt_counter=0
def receiveBoardInfo():
    global attempt_counter    
    if (attempt_counter>COM_ATTEMPTS):
        AVAILABLE=False
        ser.close()
        attempt_counter=0
        return None
    
    try:
        ser.open()
        ser.write(chr(BOARD_INFO))
        answer_sending = ser.read(1)
        while(answer_sending!=chr(BOARD_INFO)):
            ser.flush()
            AVAILABLE=False
            ++attempt_counter
            ser.close()
            receiveBoardInfo()
            
    except:
        AVAILABLE=False
        ++attempt_counter
        ser.flush()
        ser.close()
        receiveBoardInfo()
        
    else:
        AVAILABLE=True


    if (AVAILABLE):
        try:
                boardInfoType = ser.readline()
                boardInfoMake = ser.readline()
                boardInfoModel = ser.readline()
                boardInfoMCU = ser.readline()
                boardDictionary = dict([('Type', boardInfoType),('Make', boardInfoMake),('Model', boardInfoModel),('MCU', boardInfoMCU)])
                answer_sending=ser.read(1)
                if(answer_sending!=chr(END)):
                    ser.flush()
                    ser.close()
                    ++attempt_counter
                    sendBoardInfo()
                else:
                    ser.write(chr(RECEIVED))            
        except:
                ser.flush()
                ser.close()
                ++attempt_counter
                sendBoardInfo()
        else:
            ser.flush()
            ser.close()
            return  boardDictionary 
        

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

attempt_counter=0

def receiveData():
    global attempt_counter
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
    bufferRegisterPWMC=0
    bufferRegisterPWMD=0
    bufferRegisterServoB=0
    bufferRegisterServoC=0
    bufferRegisterServoD=0
    counterDDRDArrayRead=0
    counterRegisterPWM=0
    counterRegisterServo=0

    try:
        ser.open()
        ser.write(chr(SEND_STATUS))
        while(ser.read(1)!=chr(SEND_STATUS)):
            AVAILABLE=False            
            ++attempt_counter
            ser.flush()
            ser.close()
            receiveData()
            if (attempt_counter>COM_ATTEMPTS):
                ser.close()
                attempt_counter=0
                return None
    except:
        if (attempt_counter<COM_ATTEMPTS):
            AVAILABLE=False
            ++attempt_counter
            ser.flush()
            ser.close()
            receiveData()
        else:
            ser.close()
            attempt_counter=0
            AVAILABLE=False
            return None
    else:
        AVAILABLE=True

    if (AVAILABLE):
        while(AVAILABLE):
            try:
                receivedRawString[f'{counter}']=ser.read(1)
                if(counter > 0):
                    if(receivedRawString[f'{counter}']==chr(END) & receivedRawString[f'{counter-1}']==chr(CHRNULL)):
                          counter=0
                          break  
                ++counter
            except:
                AVAILABLE=False
                ser.close()
                receiveData()

        if (receivedRawString[f'12']!=chr(IS_ANALOG_READ)):
            ser.flush()
            ser.write(chr(RESEND))
            ser.close()
            ++attempt_counter
            receiveData()
                
        bufferPortD= receivedRawString[f'0']
        bufferDDRD= receivedRawString[f'1']
        bufferRegisterPWMD= receivedRawString[f'2']
        bufferRegisterServoD= receivedRawString[f'3']

        bufferPortC= receivedRawString[f'4']
        bufferDDRC= receivedRawString[f'5']
        bufferRegisterPWMC= receivedRawString[f'6']
        bufferRegisterServoC= receivedRawString[f'7']

        bufferPortB= receivedRawString[f'8']
        bufferDDRB= receivedRawString[f'9']
        bufferRegisterPWMB= receivedRawString[f'10']
        bufferRegisterServoB= receivedRawString[f'11']


        counterDDRDArrayRead = 8 - counterBitON(bufferDDRD)
        couterRegisterPWM = counterBitON(bufferRegisterPWMD)+ counterBitON(bufferRegisterPWMC) + counterBitON(bufferRegisterPWMB);
        couterRegisterServo = counterBitON(bufferRegisterServoD)+ counterBitON(bufferRegisterServoC) + counterBitON(bufferRegisterServoB);

        doneReadArrayRead = False
        doneReadPWM = False
    
        for i in range(12,76):
            if (doneReadArrayRead!=True & (receivedRawString[f'{i+2}']==chr(IS_PWM) & receivedRawString[f'{i+1}']==chr(CHRNULL))!=True):
                    receivedArrayRead[f'{counterArrayRead}'] = receivedRawString[f'{i}']
                    ++counterArrayRead
            else:
                doneReadArrayRead = True
                if ( doneReadPWM!=True & (receivedRawString[f'{i+2}']==chr(IS_SERVO) & receivedRawString[f'{i+1}']==chr(CHRNULL))!=True):
                    receivedPWM[f'{counterPWM}'] = receivedRawString[f'{i}']
                    ++counterPWM
                else:
                    doneReadPWM=True
                    if((receivedRawString[f'{i+2}']==chr(END) & receivedRawString[f'{i+1}']==chr(CHRNULL))!=True):
                        receivedServo[f'{counterServo}'] = receivedRawString[f'{i}']
                        ++counterServo
                    else:
                        break
    
        if (counterArrayRead!=counterDDRDArrayRead | counterPWM!=counterRegisterPWM | counterServo!=counterRegisterServo):
            try:
                ser.flush()
                ser.write(chr(RESEND))
                ser.close()
                AVAILABLE=False
                ++attempt_counter
                receiveData()

            except:
                AVAILABLE=False
                ++attempt_counter
                ser.close
                receiveData()

        sendChar(RECEIVED);
        flush();
        ser.close()
        attempt_counter=0

        PORTD = bufferPortD
        DDRD = bufferDDRD
        PORTC = bufferPortC
        DDRC = bufferDDRC
        PORTB = bufferPortB
        DDRB = bufferDDRB

        placercounter=0

        for i in range (8):
            for v in range (2):
                if not bitON(bufferDDRD,i):
                    arrayRead[F'{i}{v}'] = receivedArrayRead[f'{placercounter}']
                    ++placercounter
            
        placercounter=0

        for i in range(f'{ROW}'):
            pwmData[f'{i}']=0
            if (i<8):
                if(bitON(bufferRegisterPWMB,i)):
                    pwmData['f{i}'] = receivedPWM[f'{placercounter}']
                    ++placercounter
            elif (i<16):
                if(bitON(bufferRegisterPWMC,i-8)):
                    pwmData[f'{i}']= receivedPWM[f'{placercounter}']
                    ++placercounter
            else:
                if(bitON(bufferRegisterPWMD,i-16)):
                    pwmData['F{i}'] = receivedPWM[f'{placercounter}']
                    ++placercounter

        placercounter=0
  
        for i in range(f'{ROW}'):
            servoData[f'{i}']=0;
            if (i<8):
                if(bitON(bufferRegisterServoB,i)):
                    servoData[f'{i}'] = receivedServo[f'{placercounter}']
                    ++placercounter
            elif (i<16):
                if(bitON(bufferRegisterServoC,i-8)):
                    servoData[f'{i}'] = receivedServo[f'{placercounter}']
                    ++placercounter
            else:
                if(bitON(bufferRegisterServoD,i-16)):
                    servoData[f'{i}'] = receivedServo[f'{placercounter}']
                    ++placercounter

        placercounter=0

            
# THIS IS THE FUNCTION TO SEND BOARD ACTIONS          

attempt_counter=0
def sendBoardUpdate():
    global attempt_counter
    if (attempt_counter>COM_ATTEMPTS):
        ser.close()
        attempt_counter=0
        AVAILABLE=False
        return None

    try:
        ser.open()
        ser.write(chr(PC_REGISTERS_UPDATE))
        resivedAction=ser.read(1)
        while (receivedAction==chr(WAIT)):
            pass
        while (receivedAction!=chr(PC_REGISTERS_UPDATE)):
            ser.flush()
            ++attempt_counter
            sendBoardUpdate()
            
    except:
            ser.flush()
            ++attempt_counter
            sendBoardUpdate()            
    else:
        AVAILABLE=True

    if (AVAILABLE):    
        try:
            resivedAction=ser.read(1)
        
            if( PORTD != prevPortD):

                ser.write(chr(PORTD))
                ser.write(chr(DDRD))
                ser.write(chr(pwmRegisterD))
                ser.write(chr(servoRegisterD))
    
            if( PORTC != prevPortC):
    
                ser.write(chr(PORTC))
                ser.write(chr(DDRC))
                ser.write(chr(pwmRegisterC))
                ser.write(chr(servoRegisterC))
    

            if( PORTB != prevPortB):

                ser.write(chr(PORTB))
                ser.write(chr(DDRB))
                ser.write(chr(pwmRegisterB))
                ser.write(chr(servoRegisterB))      
    
# SEND PWM INFO
            ser.write(chr(CHRNULL))
            ser.write(chr(IS_PWM))

            if (pwmData != prevpwmData):
                for i in range(ROW):
                    if (pwmData[f'{i}']!= 0): 
                        if (pwmData[f'{i}']!= prevpwmData[f'{i}']): 
                            ser.write(chr(pwmData[f'{i}']));

# SEND SERVO INFO
            ser.write(chr(CHRNULL))                
            sendChar(IS_SERVO);

            if (servoData != prevservoData):
                for i in range(8):
                    if (servoData[f'{i}']!= 0): 
                        if (servoData[f'{i}']!= prevservoData[f'{i}']):
                            ser.write(chr(servoData[f'{i}']))

            ser.write(chr(CHRNULL))
            ser.write(chr(END))

# ASK RECEIVED FROM ARDUIND
    
    
            resivedAction=ser.read(1)
            while (receivedAction==WAIT):
                pass
            if (receivedAction==RESEND):
                        ser,flush()
                        ++attempt_counter
                        sendBoardUpdate()
            while (receivedAction!=RECEIVED):
                resivedAction=ser.read(1)
                while (receivedAction==WAIT):
                    pass
                    if (receivedAction==RESEND):
                        ser,flush()
                        ++attempt_counter
                        sendBoardUpdate()
        except:
                ser.flush()
                ++attempt_counter
                sendBoardUpdate()
            
        else:
            AVAILABLE=True
                
        prevPortB = PORTB;
        prevDDRB = DDRB;
        prevPortC = PORTC;
        prevDDRB = DDRB;
        prevPortD = PORTD;
        prevDDRB = DDRB;
        prevArrayRead = arrayRead;
        prevpwmData = pwmData;
        prevservoData = servoData; 

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
# const int PC_CONTROL_PIN= 6
# const int mosfet_1_pin = A6 ;
# const int mosfet_2_pin = 9 ; // digital pin can be written as analog
# const int mosfet_3_pin = 10 ;  // digital pin can be written as analog
# const int battery_voltage_pin= A3 ;
# const int device_charger_voltage_1 = A0 ;
# const int device_charger_voltage_2 = A1 ;  
# const int device_charger_voltage_3 = A2 ;  
# const int photo_resistor = A7 ;

# TAKING THE SAME NAMES
PC_CONTROL_PIN= 6
mosfet_1_pin = A6
mosfet_2_pin = 9
mosfet_3_pin = 10
battery_voltage_pin= A3
device_charger_voltage_1 = A0
device_charger_voltage_2 = A1
device_charger_voltage_3 = A2   
photo_resistor = A7 

# ANALOG PINS ARE IN PORT C, PORT B IS 8 TO 13 FIST TWO PINS OF PORT ARE
# RESERVED TO SERIAL 0 RX 1 TX MUST NOT BE CHANGED, PORT D IS 0 TO 7

# FIRST ASIGMENT  OF PORT BEFORE SERIAL CONNECTION TO ARDUINO ACCORDING ARDUINO PROGRAM
# DDRn = PIN MODE 0 INPUT / PORTn = HIGH OR LOW DDRn 1 AND PORTn 1 INPUT PULLUP RESISTOR FOR CONNECTION NOT GROUNDED 

DDRD= setBit(DDRD,PC_CONTROL_PIN)
DDRC= setBit(DDRC,mosfet_1_pin-14) # 14 for arduino is 0 port C in chip
DDRB= setBit(DDRB,mosfet_2_pin-6) # 0,1 reserved 2 in PORTB will be 8 in arduino
DDRB= setBit(DDRB,mosfet_3_pin-6)

# REST OF PINS ARE INPUT AND DONT NEED PULL UP SINCE HAVE CAPACITORS AND GROUND RESISTORS 


# Assign The mode of PC_CONTROL_PIN for control the board with a button 

PC_CONTROL_STATE =0

#_____________________________________________________________________#
#                                                                     #
#                              WIDGET                                 #
#_____________________________________________________________________#
#

# THESE ARE TEMPORAL ARRAY TO STORE DATA TO BE SENT TO AERDUINO WHEN PROCESS BUTTON
# IS PRESSED.

entrypwmData= dict()
for i in range (8):
    entrypwmData[f'{i}']=0  # ARRAY TO STORE PWM DATA BEFORE SEND IT TO ARDUINO

entryservoData = dict()
for i in range (8):
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
entryservoRegisterD = servoRegisterB

# THESE ARE THE SAME TYPE OF TEMPORAL VARIABLES FOR PORTn AND DDRn
 
# PORTn define is Pins are in HIGH = 1 or LOW =0
entryPORTB=0
entryPORTC=0
entryPORTD=0
# DDRn define mode INPUT=0 OUTPUT=1
entryDDRB=0
entryDDRC=0
entryDDRD=0
 
 
# Define theme for environment


# Define main widget environment dimensions

root = Tk()
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

# Binds to check ARDUINO STATUS in any loop
# at the end with a flag because slow plenty the program the check
# the counter is to make check after some updates
# root.bind('<Activate>',receiveData())
# root.bind('<Deactivate>',receiveData())
# root.bind('<Visibility>',receiveData())
# HAVE THE THREE BECAUSE PROGRAM WILL APPEND PIN STATUS EACH 30 MIN TO A FILE
# AND PLOT IT CAN SET TIME STEP AND WILL PLOT CHARGE STATUS FOR EACH DEVICE
# LIGHT STATUS AND CAN BE DETERMINED CALCULATED CURRENT WITH MOSFETS VOLTS AND
# TEMPERATURE IN FUTURE UPGRADES

counter_availability=0
COUNTER_LIMIT=50   # This can be changed according usage

mainFrame = ttk.Frame(root, padding ="3 3 12 12")
mainFrame.grid(column=0, row=0)

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

def convertionToWrite(entryValue):

  float_entryValue= float(entryValue)
  writeValue= int(float_entryValue*5/255)
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
            
        elif bitON(pwmRegisterD,pin-14):
            value=int(pwmData[f'{pin}'])
            readValue= (PWM_VOLTS * VOLTS_FACTOR_IN_OP)* float(value)
            return f'{readValue:.3f}'
        
        elif bitON(servoRegisterD,pin-14):
            value=int(servoData[f'{pin}'])
            readValue= (SERVO_VOLTS * VOLTS_FACTOR_IN_OP)* float(value)
            return f'{readValue:.3f}'
        
        elif bitON(PORTC, pin-14):
            return f'1'
        else:
                return f'0'
        
    elif(pin<8):

        if not bitON(DDRD, pin):
            return 0
            
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
            value=int(pwmData[f'{pin}'])
            readValue= (PWM_VOLTS * VOLTS_FACTOR_IN_OP)* float(value)
            return f'{readValue:.3f}'
        
        elif bitON(servoRegisterB,pin-6):
            value=int(servoData[f'{pin}'])
            readValue= (SERVO_VOLTS * VOLTS_FACTOR_IN_OP)* float(value)
            return f'{readValue:.3f}'

        elif bitON(PORTB, pin-6):
            return f'1'
        else:
                return f'0'


# Screen message function for button to control board
ttk.Label(mainFrame, text= "<= To control the board press the button").grid(column=2,row=1, columnspan=5, sticky='w')
   
# buttons to control pin mode

textUnlock= StringVar()
textUnlock.set('DISABLED')

def controlBoardPBtn():
    global controlBoardPBtn, PORTD, PC_CONTROL_STATE, chkMosfet1, chkMosfet2, chkMosfet3, chkBatteryVolt, chkTrafoVolt, chkSolarPanelVolt, chkWindTurbineVolt, chkWindTurbineVolt, chkPhotoreResistorVolt, proceedButton, buttonMos1, buttonMos2, buttonMos3, batteryVolt, trafoVolt, solarVolt, windVolt, photoResistor

    
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
        
    else:
        PC_CONTROL_STATE =1
        textUnlock.set("ENABLED")
        PORTD = setBit(PORTD,PC_CONTROL_PIN) # THIS ONLY CHANGE THE VARIABLE AND SCREEN VIEW PROCESS CHANGE BUTTON UPDATE ARDUINO
        chkMosfet1.state(['!disabled'])
        chkMosfet2.state(['!disabled'])
        chkMosfet3.state(['!disabled'])
        chkBatteryVolt.state(['!disabled'])
        chkTrafoVolt.state(['!disabled'])
        chkSolarPanelVolt.state(['!disabled'])
        chkWindTurbineVolt.state(['!disabled'])
        chkWindTurbineVolt.state(['!disabled'])
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
ttk.Label(mainFrame, text= "Check to update").grid(column=4, row=2, sticky ="w")

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
    global top, entryDDRB, entryDDRC, entryDDRD, entrydigitalOutputD, entrydigitalOutputB,  entrydigitalOutputC
    pin = int(pin)
    if (pin<8):
        entryDDRD= setBit(entryDDRD,pin)
        entrydigitalOutputD = setBit(entrydigitalOutputD,pin)
    elif (pin<14):
        entryDDRB= setBit(entryDDRB,pin)
        entrydigitalOutputB = setBit(entrydigitalOutputB,pin)
    else:
        entryDDRC= setBit(entryDDRC,pin)
        entrydigitalOutputC = setBit(entrydigitalOutputC,pin)
    top.destroy()
    textToButton("OUTPUT",pin)
    if (pin== mosfet_1_pin):
        varentryMosfet1= None
        entryMosfet1.state(['!disabled'])
        chkMos1State= False
        chkMosfet1.state(['!disabled'])

    elif(pin== mosfet_2_pin):
        varentryMosfet2= None
        entryMosfet2.state(['!disabled'])
        chkMos2State= False
        chkMosfet2.state(['!disabled'])
            
    elif(pin== mosfet_3_pin):
        varentryMosfet3= None
        entryMosfet3.state(['!disabled'])
        chkMos3State= False
        chkMosfet3.state(['!disabled'])
            
    elif(pin== battery_voltage_pin):
        varentryBattery= None
        entryBatteryVolt.state(['!disabled'])
        chkBatState= False
        chkBatteryVolt.state(['!disabled'])
            
    elif(pin== device_charger_voltage_1):
            varentryTrafo= None
            entryTrafoVolt.state(['!disabled'])
            chkTrafoState= False
            chkTrafoVolt.state(['!disabled'])
    elif(pin== device_charger_voltage_2):
            varentrySolar= None
            entrySolarPanelVolt.state(['!disabled'])
            chkSolarState= False
            chkSolarPanelVolt.state(['!disabled'])
            
    elif(pin== device_charger_voltage_3):
            varentryWind= None
            entryWindTurbineVolt.state(['disabled'])
            chkWindState= False
            chkWindTurbineVolt.state(['disabled'])
    elif(pin== photo_resistor): 
            varentryPhoto= None
            entryPhotoreResistorVolt.state(['disabled'])
            chkPhotoState= False
            chkPhotoreResistorVolt.state(['disabled'])

setOutput_wrapper = root.register(setOutput)

def setPWM(pin):
    global top, entryDDRB, entryDDRC, entryDDRD, entrypwmRegisterD, entrypwmRegisterB,  entrypwmRegisterC
    pin = int(pin)
    if (pin<8):
        entryDDRD= setBit(entryDDRD,pin)
        entrypwmRegisterD = setBit(entrypwmRegisterD,pin)
    elif (pin<14):
        entryDDRB= setBit(entryDDRB,pin)
        entrypwmRegisterB = setBit(entrypwmRegisterB,pin)
    else:
        entryDDRC= setBit(entryDDRC,pin)
        entrypwmRegisterC = setBit(entrypwmRegisterC,pin)
    top.destroy()
    textToButton("PWM",pin)
    if (pin== mosfet_1_pin):
        varentryMosfet1= None
        entryMosfet1.state(['!disabled'])
        chkMos1State= False
        chkMosfet1.state(['!disabled'])

    elif(pin== mosfet_2_pin):
        varentryMosfet2= None
        entryMosfet2.state(['!disabled'])
        chkMos2State= False
        chkMosfet2.state(['!disabled'])
            
    elif(pin== mosfet_3_pin):
        varentryMosfet3= None
        entryMosfet3.state(['!disabled'])
        chkMos3State= False
        chkMosfet3.state(['!disabled'])
            
    elif(pin== battery_voltage_pin):
        varentryBattery= None
        entryBatteryVolt.state(['!disabled'])
        chkBatState= False
        chkBatteryVolt.state(['!disabled'])
            
    elif(pin== device_charger_voltage_1):
            varentryTrafo= None
            entryTrafoVolt.state(['!disabled'])
            chkTrafoState= False
            chkTrafoVolt.state(['!disabled'])
    elif(pin== device_charger_voltage_2):
            varentrySolar= None
            entrySolarPanelVolt.state(['!disabled'])
            chkSolarState= False
            chkSolarPanelVolt.state(['!disabled'])
            
    elif(pin== device_charger_voltage_3):
            varentryWind= None
            entryWindTurbineVolt.state(['disabled'])
            chkWindState= False
            chkWindTurbineVolt.state(['disabled'])
    elif(pin== photo_resistor): 
            varentryPhoto= None
            entryPhotoreResistorVolt.state(['disabled'])
            chkPhotoState= False
            chkPhotoreResistorVolt.state(['disabled'])

setPWM_wrapper = root.register(setPWM)

def setServo(pin):
    global top,entryDDRB, entryDDRC, entryDDRD, entryservoRegisterD, entryservoRegisterB,  entryservoRegisterC
    pin = int(pin)
    if (pin<8):
        entryDDRD= setBit(entryDDRD,pin)
        entryservoRegisterD = setBit(entryservoRegisterD,pin)
    elif (pin<14):
        entryDDRB= setBit(entryDDRB,pin)
        entryservoRegisterB = setBit(entryservoRegisterB,pin)
    else:
        entryDDRC= setBit(entryDDRC,pin)
        entryservoRegisterC = setBit(entryservoRegisterC,pin)
    top.destroy()
    textToButton("SERVO",pin)
    if (pin== mosfet_1_pin):
        varentryMosfet1= None
        entryMosfet1.state(['!disabled'])
        chkMos1State= False
        chkMosfet1.state(['!disabled'])

    elif(pin== mosfet_2_pin):
        varentryMosfet2= None
        entryMosfet2.state(['!disabled'])
        chkMos2State= False
        chkMosfet2.state(['!disabled'])
            
    elif(pin== mosfet_3_pin):
        varentryMosfet3= None
        entryMosfet3.state(['!disabled'])
        chkMos3State= False
        chkMosfet3.state(['!disabled'])
            
    elif(pin== battery_voltage_pin):
        varentryBattery= None
        entryBatteryVolt.state(['!disabled'])
        chkBatState= False
        chkBatteryVolt.state(['!disabled'])
            
    elif(pin== device_charger_voltage_1):
            varentryTrafo= None
            entryTrafoVolt.state(['!disabled'])
            chkTrafoState= False
            chkTrafoVolt.state(['!disabled'])
    elif(pin== device_charger_voltage_2):
            varentrySolar= None
            entrySolarPanelVolt.state(['!disabled'])
            chkSolarState= False
            chkSolarPanelVolt.state(['!disabled'])
            
    elif(pin== device_charger_voltage_3):
            varentryWind= None
            entryWindTurbineVolt.state(['disabled'])
            chkWindState= False
            chkWindTurbineVolt.state(['disabled'])
    elif(pin== photo_resistor): 
            varentryPhoto= None
            entryPhotoreResistorVolt.state(['disabled'])
            chkPhotoState= False
            chkPhotoreResistorVolt.state(['disabled'])

setServo_wrapper = root.register(setServo)

def setInput(pin):
    global top, entryDDRB, entryDDRC, entryDDRD, varentryMosfet1, chkMos1State, chkMosfet1, varentryMosfet2, chkMos2State, chkMosfet2, varentryMosfet3, chkMos3State, chkMosfet3, varentryBattery, chkBatState, chkBatteryVolt, varentryTrafo, chkTrafoState, chkTrafoVolt, varentrySolar, chkSolarState, chkSolarPanelVolt, varentryWind, chkWindState, chkWindTurbineVolt, varentryPhoto, chkPhotoState, chkPhotoreResistorVolt, entryMosfet1, entryMosfet2, entryMosfet3, entryBatteryVolt, entryTrafoVolt, entrySolarPanelVolt, entryWindTurbineVolt, entryPhotoreResistorVolt

    pin = int(pin)
    if (pin<8):
        entryDDRD= unsetBit(entryDDRD,pin)        
    elif (pin<14):
        entryDDRB= unsetBit(entryDDRB,pin) 
    else:
        toUpdateDDRC= unsetBit(entryDDRC,pin)
    top.destroy()
    textToButton("INPUT",pin)
    if (pin== mosfet_1_pin):
        varentryMosfet1= None
        entryMosfet1.state(['disabled'])
        chkMos1State= False
        chkMosfet1.state(['disabled'])

    elif(pin== mosfet_2_pin):
        varentryMosfet2= None
        entryMosfet2.state(['disabled'])
        chkMos2State= False
        chkMosfet2.state(['disabled'])
            
    elif(pin== mosfet_3_pin):
        varentryMosfet3= None
        entryMosfet3.state(['disabled'])
        chkMos3State= False
        chkMosfet3.state(['disabled'])
            
    elif(pin== battery_voltage_pin):
        varentryBattery= None
        entryBatteryVolt.state(['disabled'])
        chkBatState= False
        chkBatteryVolt.state(['disabled'])
            
    elif(pin== device_charger_voltage_1):
            varentryTrafo= None
            entryTrafoVolt.state(['disabled'])
            chkTrafoState= False
            chkTrafoVolt.state(['disabled'])
    elif(pin== device_charger_voltage_2):
            varentrySolar= None
            entrySolarPanelVolt.state(['disabled'])
            chkSolarState= False
            chkSolarPanelVolt.state(['disabled'])
            
    elif(pin== device_charger_voltage_3):
            varentryWind= None
            entryWindTurbineVolt.state(['disabled'])
            chkWindState= False
            chkWindTurbineVolt.state(['disabled'])
    elif(pin== photo_resistor): 
            varentryPhoto= None
            entryPhotoreResistorVolt.state(['disabled'])
            chkPhotoState= False
            chkPhotoreResistorVolt.state(['disabled'])

setInput_wrapper = root.register(setInput)

        
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
                child.grid_configure(padx=int(root.winfo_width()/150), pady=int(root.winfo_width()/150))

            

            top.frame.grid(column =0, row =0, sticky=(W,E,N,S), pady=(screenWidth/600))


# Functions and alert Window for Buttons and Lanels for PIN MODE 

def onPressOk(pin):
    global alertWindow

    alertWindow.destroy()
    modeMessageBox= CustomMessage(pin, root, buttonOUTPUT = {"text": "OUTPUT", "command": (setOutput_wrapper, pin)}, buttonINPUT = {"text": "INPUT", "command": (setInput_wrapper, pin)}, buttonPWM = {"text": "PWM", "command": (setPWM_wrapper, pin)}, buttonServo = {"text": "SERVO", "command": (setServo_wrapper, pin)})  

    
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

#    entryMosfet1 = event.widget
    global PC_CONTROL_STATE, entryMosfet1
    boolchk = chkMos1State.get()
    
    if ((PC_CONTROL_STATE ==1) & boolchk ):
          entryMosfet1.state(['!disabled'])
    else:
          entryMosfet1.state(['disabled'])
          
def enablerMos2():
    global PC_CONTROL_STATE, entryMosfet2
 
    boolchk= chkMos2State.get()
    
    if ((PC_CONTROL_STATE ==1) & boolchk ):
          entryMosfet2.state(['!disabled'])      
    else:
          entryMosfet2.state(['disabled'])

def enablerMos3():
    global PC_CONTROL_STATE, entryMosfet3

    boolchk = chkMos3State.get()
    
    if ((PC_CONTROL_STATE ==1) & boolchk ):
          entryMosfet3.state(['!disabled'])      
    else:
          entryMosfet3.state(['disabled'])

def enablerBattery():
    global PC_CONTROL_STATE, entryBatteryVolt

    boolchk= chkBatState.get()
    
    if ((PC_CONTROL_STATE ==1) & boolchk ):
          entryBatteryVolt.state(['!disabled'])      
    else:
          entryBatteryVolt.state(['disabled'])

def enablerTrafoVolt():
    global PC_CONTROL_STATE, entryTrafoVolt

    boolchk= chkTrafoState.get()
    
    if ((PC_CONTROL_STATE ==1) & boolchk ):
          entryTrafoVolt.state(['!disabled'])      
    else:
          entryTrafoVolt.state(['disabled'])

def enablerSolar():
    global PC_CONTROL_STATE, entrySolarPanelVolt    

    boolchk= chkSolarState.get()
    
    if ((PC_CONTROL_STATE ==1) & boolchk ):
          entrySolarPanelVolt.state(['!disabled'])      
    else:
          entrySolarPanelVolt.state(['disabled'])          

def enablerWind():
    global PC_CONTROL_STATE, entryWindTurbineVolt

    boolchk= chkWindState.get()
    
    if ((PC_CONTROL_STATE ==1) & boolchk ):
          entryWindTurbineVolt.state(['!disabled'])      
    else:
          entryWindTurbineVolt.state(['disabled'])          

def enablerPhoto():
    global PC_CONTROL_STATE, entryPhotoreResistorVolt

    boolchk= chkPhotoState.get()
    
    if ((PC_CONTROL_STATE ==1) & boolchk ):
          entryPhotoreResistorVolt.state(['!disabled'])      
    else:
          entryPhotoreResistorVolt.state(['disabled'])          

    # Check buttons to change value of a ping

chkMos1State= BooleanVar()
chkMosfet1= ttk.Checkbutton(mainFrame, command = enablerMos1, state= 'disabled', variable=chkMos1State, onvalue=True, offvalue=False)
chkMosfet1.grid(column =4, row =4)

chkMos2State= BooleanVar()
chkMosfet2= ttk.Checkbutton(mainFrame, command= enablerMos2, state= 'disabled', variable=chkMos2State, onvalue=True, offvalue=False)
chkMosfet2.grid(column =4, row =6)

chkMos3State= BooleanVar()
chkMosfet3= ttk.Checkbutton(mainFrame, command= enablerMos3, state= 'disabled', variable=chkMos3State, onvalue=True, offvalue=False)
chkMosfet3.grid(column =4, row =8)

chkBatState= BooleanVar()
chkBatteryVolt= ttk.Checkbutton(mainFrame, command= enablerBattery, state= 'disabled', variable=chkBatState, onvalue=True, offvalue=False)
chkBatteryVolt.grid(column =4, row =10)

chkTrafoState= BooleanVar()
chkTrafoVolt= ttk.Checkbutton(mainFrame,command= enablerTrafoVolt, state= 'disabled', variable=chkTrafoState,onvalue=True, offvalue=False)
chkTrafoVolt.grid(column =4, row =12)

chkSolarState= BooleanVar()
chkSolarPanelVolt= ttk.Checkbutton(mainFrame, command=enablerSolar, state= 'disabled', variable=chkSolarState, onvalue=True, offvalue=False)
chkSolarPanelVolt.grid(column =4, row =14)

chkWindState= BooleanVar()
chkWindTurbineVolt= ttk.Checkbutton(mainFrame, command=enablerWind, state= 'disabled', variable=chkWindState, onvalue=True, offvalue=False)
chkWindTurbineVolt.grid(column =4, row =16)

chkPhotoState= BooleanVar()
chkPhotoreResistorVolt= ttk.Checkbutton(mainFrame, command= enablerPhoto, state= 'disabled', variable=chkPhotoState,onvalue=True, offvalue=False)
chkPhotoreResistorVolt.grid(column =4, row =18)

#Validation the entry with re numbers 1 digit . 0 to 3 digits 
# validation= re.compile(r'[0-5]?(.{1}\d{1,3})?$')

def validationFunction(value, key):
    
    key_checkfinal= re.match(r'[0-4]{0,1}((\.{1}\d{1,3}){0,1})$', value) is not None
    if key=='key':
        if len(value)==1:
            key_check = re.match(r'[0-4]{1}',value) is not None
            if not key_check:
                key_check = re.match(r'\.{1}',value) is not None
                if not key_check:
                    messagebox.showwarning(title='Invalid imput', message='Format should be 0 or 1 for digital and 0.000 0(.000) for analog less than 5') 
        elif len(value)==2:
            key_check = re.match(r'[0-4]{1}\.{1}$',value) is not None
            if not key_check:
                key_check = re.match(r'\.{1}\d{1}$',value) is not None
                if not key_check:
                    messagebox.showwarning(title='Invalid imput', message='Format should be 0 or 1 for digital and 0.000 0(.000) for analog less than 5') 
        else:                
            key_check = re.match(r'[0-4]{0,1}((\.{1}\d{1,3}){0,1})$', value) is not None and len(value) <= 5
        if not key_check:
            messagebox.showwarning(title="Invalid imput", message="Format should be 0 or 1 for digital and 0.000 0(.000) for analog less than 5") 
    elif key=='focusout':
        if not key_check:
            messagebox.showwarning(title="Invalid imput", message='Format should be 0 or 1 for digital and 0.000 0(.000) for analog less than 5') 
    return key_checkfinal


check_num_wrapper = (root.register(validationFunction), '%P','%V' )


    # Entries for change values

varentryMosfet1 = StringVar()
entryMosfet1= ttk.Entry(mainFrame,state= 'disabled', textvariable= varentryMosfet1, show= varentryMosfet1,  validate='all', validatecommand=check_num_wrapper)

varentryMosfet2 = StringVar()
entryMosfet2= ttk.Entry(mainFrame,state= 'disabled', textvariable= varentryMosfet2, validate='all', validatecommand=check_num_wrapper)

varentryMosfet3 = StringVar()
entryMosfet3= ttk.Entry(mainFrame,state= 'disabled', textvariable= varentryMosfet3, validate='all', validatecommand=check_num_wrapper)

varentryBattery = StringVar()
entryBatteryVolt= ttk.Entry(mainFrame,state= 'disabled', textvariable= varentryBattery, validate='all', validatecommand=check_num_wrapper)

varentryTrafo = StringVar()
entryTrafoVolt= ttk.Entry(mainFrame,state= 'disabled', textvariable= varentryTrafo, validate='all', validatecommand=check_num_wrapper)

varentrySolar = StringVar()
entrySolarPanelVolt= ttk.Entry(mainFrame,state= 'disabled', textvariable= varentrySolar, validate='all', validatecommand=check_num_wrapper)

varentryWind = StringVar()
entryWindTurbineVolt= ttk.Entry(mainFrame,state= 'disabled', textvariable= varentryWind, validate='all', validatecommand=check_num_wrapper)

varentryPhoto = StringVar()
entryPhotoreResistorVolt= ttk.Entry(mainFrame,state= 'disabled', textvariable= varentryPhoto, validate='all', validatecommand=check_num_wrapper)

entryMosfet1.grid(column =3, row =4)
entryMosfet2.grid(column =3, row =6)
entryMosfet3.grid(column =3, row =8)
entryBatteryVolt.grid(column =3, row =10)
entryTrafoVolt.grid(column =3, row =12)
entrySolarPanelVolt.grid(column =3, row =14)
entryWindTurbineVolt.grid(column =3, row =16)
entryPhotoreResistorVolt.grid(column =3, row =18)


def onPressProceed():
    pass      


proceedButton=ttk.Button(mainFrame,state='disabled', text="SEND UPDATE", command=onPressProceed)
proceedButton.grid(column =4, row =1) 


# label and buttona to plot and print historic file
ttk.Label(mainFrame, text="Historical data reporting:").grid(column =1, row =19, sticky='sw')                            
ttk.Button(mainFrame, text="Plot").grid(column =2, row =20)
ttk.Button(mainFrame, text="Print").grid(column =3, row =20)

# pending sample data appand on file and plot and print and eventually erase file
def onPressPlot():
  pass
  
def onPressPrint():
  pass
ttk.Label(mainFrame, text="Change of mode is attampted to be for test only f.e. wires or check correct analog output").grid(column =1,columnspan=5, row =21,  sticky='w')                            
ttk.Label(mainFrame, text="Arduino only have three timers in complementary pins pairs so PWM must use only ONE of EACH PAIR FOR A SINGLE NON COMPLEMENTARY OUTPUT.").grid(column =1,columnspan=5, row =22, sticky='w')                            
ttk.Label(mainFrame, text="History data save a day file with each charger voltage and light each 15 minutes sample ").grid(column =1,columnspan=5, row =23, sticky='w')                            
ttk.Label(mainFrame, text="This data can be plotted for day, week, month or year").grid(column =1,columnspan=5, row =24, sticky='w')                            


                          
for child in mainFrame.winfo_children():
  child.grid_configure(padx=int(root.winfo_width()/150), pady=int(root.winfo_width()/150))
               

# root.bind('<Activate>',receiveData)
# root.bind('<Deactivate>',receiveData)
# root.bind('<Visibility>',receiveData)


root.bind('<Configure>',boardResize)

root.mainloop()                                 

# end of file
KeyboardInterrupt
