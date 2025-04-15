######UNFINISHED WORKING ON IT DONT COMPILE#######

##Program to control arduino Nano board with the ChargersController.ino
#autor: Dario Lobos 17/mar/2025

import Tkinter
# import PIL
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
# pending check parity
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
    servoData[f'{i}']=0 # ARRAY TO STORE PWM DATA

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
            sleep(5)
            ser.flush()
            AVAILABLE=False
            ++attempt_counter
            ser.close()
            receiveBoardInfo()
            
    except:
        sleep(5)
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
                    sleep(5)
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

def unsetBit( n, pos):
        n&=~( 1<< pos )

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
            sleep(5)
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
            sleep(5)
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
            sleep(5)
            ser.write(chr(RESEND))
            sleep(5)
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
                sleep(5)
                ser.write(chr(RESEND))
                sleep(5)
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
                if(bitON(bufferDDRD,i)):
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
        sleep(5)
        resivedAction=ser.read(1)
        while (receivedAction==chr(WAIT)):
            pass
        while (receivedAction!=chr(PC_REGISTERS_UPDATE)):
            ser.flush()
            ++attempt_counter
            sendBoardUpdate()
            
    except:
            ser.flush()
            sleep(5)
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
                        aleep(5)
                        ++attempt_counter
                        sendBoardUpdate()
            while (receivedAction!=RECEIVED):
                resivedAction=ser.read(1)
                while (receivedAction==WAIT):
                    pass
                    if (receivedAction==RESEND):
                        ser,flush()
                        sleep(5)
                        ++attempt_counter
                        sendBoardUpdate()
        except:
                ser.flush()
                sleep(5)
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

setBit(DDRD,PC_CONTROL_PIN)
setBit(DDRC,mosfet_1_pin-14) # 14 for arduino is 0 port C in chip
setBit(DDRB,mosfet_2_pin-6) # 0,1 reserved 2 in PORTB will be 8 in arduino
setBit(DDRB,mosfet_3_pin-6)

# REST OF PINS ARE INPUT AND DONT NEED PULL UP SINCE HAVE CAPACITORS AND GROUND RESISTORS 


# Assign The mode of PC_CONTROL_PIN for control the board with a button 

PC_CONTROL_STATE =0
    
# Define main widget environment dimensions
root = Tk()
root.title("Program to remote control Battery charger and Handle data")
root.minsize(300,300)
ScreenWidth = root.winfo_screenwidth()
ScreenHeight = root.winfo_screenheight()

# check window size to make root size

rootSizerWidth= int(ScreenWidth*0.8)
rootSizerHeight= int(ScreenHeight*0.8)

topLeftPosition=(int((ScreenWidth- rootSizerWidth)/2),int((ScreenHeight- rootSizerHeight)/2))

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
mainFrame.rowconfigure(1, weight=2)             
mainFrame.rowconfigure(2, weight=1)
mainFrame.rowconfigure(3, weight=1)
mainFrame.rowconfigure(4, weight=1)
mainFrame.rowconfigure(5, weight=1)
mainFrame.rowconfigure(6, weight=1)
mainFrame.rowconfigure(7, weight=1)
mainFrame.rowconfigure(8, weight=1)
mainFrame.rowconfigure(9, weight=1)
mainFrame.rowconfigure(10, weight=1)
mainFrame.rowconfigure(11, weight=1)
mainFrame.rowconfigure(12, weight=2)


# Make an frame special for the image to can resize it well

photoFrame = ttk.Frame(mainFrame, padding ="3 3 12 12")

textUnlock= 'DISABLED'

def onPressBoardUnlock():
    controlBoardPBtn.state('disabled')

    if (bitON(PORTD,PC_CONTROL_PIN)):
        PC_CONTROL_STATE =0
        textUnlock='DISABLED'
        controlBoardPBtn.state('!disabled')
        unsetBit(PORTD,PC_CONTROL_PIN) # THIS ONLY CHANGE THE VARIABLE AND SCREEN VIEW PROCESS CHANGE BUTTON UPDATE ARDUINO
    else:
        PC_CONTROL_STATE =1
        textUnlock="ENABLED"
        controlBoardPBtn.state('!disabled')
        setBit(PORTD,PC_CONTROL_PIN) # THIS ONLY CHANGE THE VARIABLE AND SCREEN VIEW PROCESS CHANGE BUTTON UPDATE ARDUINO

# variables for button text

buttonMos1Stg = StringVar()
buttonMos2Stg = StringVar()
buttonMos3Stg = StringVar()
batteryVoltStg = StringVar()
trafoVoltStg = StringVar()
solarVoltStg = StringVar()
windVoltStg = StringVar()
photoResistorStg =StringVar()

def pinCurrentValue(pin):
    if (pin>14):
        return arrayRead[f'{pin-14}0']<<8 + arrayRead[f'{pin-14}1']
    elif(pin<8):
        if(bitON(PORTD, pin)):
            return 1
        else:
            return 0
        
    else:
        if(bitON(PORTB, (pin - 6))):
            return 1
        else:
            return 0
            
    
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
#                                       // WITHOUT CHANGE THR CIRCUIT.
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
VOLTS_FACTOR_IN_OP = 1
ANALOG_VOLTS = 5 / 1023

def convertionToWrite(entryValue):

  float_entryValue= float(entryValue)
  writeValue= int(float_entryValue*5/255)
  return writeValue

def convertionReadToVolts(entryValue, pin):
    
    if pin!=mosfet_1_pin & pin!=mosfet_2_pin & pin!=mosfet_3_pin: 
        readValue= (ANALOG_VOLTS * VOLT_FACTOR)* float(entryValue)
    else:
        readValue= (ANALOG_VOLTS * VOLTS_FACTOR_IN_OP)* float(entryValue)
    returnString= f'{readValue:.3f}'
    return returnString
                     

# Screen message function for button to control board
ttk.Label(mainFrame, text= "To control the board press the button").grid(column=2,row=1, columnspan=5, sticky='w')
   
# buttons to control pin mode

controlBoardPBtn=ttk.Button(mainFrame, textvariable = textUnlock, command=onPressBoardUnlock).grid(column=1,row=1)


# Labels for columns

ttk.Label(mainFrame, text= "Mode of Pins").grid(column=1,row=2)
ttk.Label(mainFrame, text= "Current value").grid(column=2,row=2)
ttk.Label(mainFrame, text= "Value to update").grid(column=3,row=2)
ttk.Label(mainFrame, text= "Process change").grid(column=4,row=2)

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
photoFrame.grid(column=5,row=3, rowspan=10)



# Buttons and labels to control board

# Buttons to PIN MODE 

pin=mosfet_1_pin
buttonMos1Stg='PWM'

ttk.Label(mainFrame, text= "Transformer AC/DC Mosfet").grid(column=1,row=3, sticky='nwe')
buttonMos1= ttk.Button(mainFrame, textvariable=buttonMos1Stg)
buttonMos1.grid(column =1, row =3, sticky='swe')

pin=mosfet_2_pin
buttonMos2Stg='PWM'

ttk.Label(mainFrame, text= "Solar Panel Mosfet").grid(column=1,row=4, sticky='nwe')
buttonMos2 = ttk.Button(mainFrame, textvariable=buttonMos2Stg)
buttonMos2.grid(column =1, row =4, sticky='swe')

pin=mosfet_3_pin
buttonMos3Stg='PWM'

ttk.Label(mainFrame, text= "Wind generator Mosfet").grid(column=1,row=5, sticky='nwe')
buttonMos3 = ttk.Button(mainFrame, textvariable=buttonMos2Stg)
buttonMos3.grid(column =1, row =5, sticky='swe')


pin=battery_voltage_pin
batteryVoltStg='INPUT'

ttk.Label(mainFrame, text= "Battery Voltage").grid(column=1,row=6, sticky='nwe')
batteryVolt = ttk.Button(mainFrame, textvariable=batteryVoltStg)
batteryVolt.grid(column =1, row =6, sticky='swe')

pin=device_charger_voltage_1
trafoVoltStg='INPUT'

ttk.Label(mainFrame, text= "Transformer Voltage").grid(column=1,row=7, sticky='nwe')
trafoVolt = ttk.Button(mainFrame, textvariable=trafoVoltStg)
trafoVolt.grid(column =1, row =7, sticky='swe')

pin=device_charger_voltage_2
solarVoltStg='INPUT'

ttk.Label(mainFrame, text= "Solar panel Voltage").grid(column=1,row=8, sticky='nwe')
solarVolt = ttk.Button(mainFrame, textvariable=solarVoltStg)
solarVolt.grid(column =1, row =8, sticky='swe')

pin=device_charger_voltage_3
windVoltStg='INPUT'

ttk.Label(mainFrame, text= "Wind gen. Voltage").grid(column=1,row=9, sticky='nwe')
windVolt = ttk.Button(mainFrame, textvariable=windVoltStg)
windVolt.grid(column =1, row =9, sticky='swe')

pin= photo_resistor
photoResistorStg='INPUT'

ttk.Label(mainFrame, text= "Photo resistor pin").grid(column=1,row=10, sticky='nwe')
photoResistor = ttk.Button(mainFrame, textvariable=photoResistorStg)
photoResistor.grid(column =1, row =10, sticky='swe')
 

def buttonDisabler(pin):

  global buttonMos1, buttonMos2, buttonMos3, batteryVolt, trafoVolt, solarVolt, photoResistor
# can be used frame parent instead of global   
  if pin==mosfet_1_pin:
      buttonMos1.state(['disabled'])
  elif pin==mosfet_2_pin:
      buttonMos2.state(['disabled'])
  elif pin==mosfet_3_pin:
      buttonMos3.state(['disabled'])
  elif pin==battery_voltage_pin:
      batteryVolt.state(['disabled'])
  elif pin==device_charger_voltage_1:
      trafoVolt.state(['disabled'])
  elif pin==device_charger_voltage_2:
      solarVolt.state(['disabled'])
  elif pin==device_charger_voltage_3:
      windVolt.state(['disabled'])
  else: 
      photoResistor.state(['disabled'])

def buttonEnabler(pin):
  global buttonMos1, buttonMos2, buttonMos3, batteryVolt, trafoVolt, solarVolt, photoResistor
  if pin==mosfet_1_pin:
      buttonMos1.state(['!disabled'])
  elif pin==mosfet_2_pin:
      buttonMos2.state(['!disabled'])
  elif pin==mosfet_3_pin:
      buttonMos3.state(['!disabled'])
  elif pin==battery_voltage_pin:
      batteryVolt.state(['!disabled'])
  elif pin==device_charger_voltage_1:
      trafoVolt.state(['!disabled'])
  elif pin==device_charger_voltage_2:
      solarVolt.state(['!disabled'])
  elif pin==device_charger_voltage_3:
      windVolt.state(['!disabled'])
  else: 
      photoResistor.state('enabled')

def textToButton(text,pin):
  global buttonMos1Stg, buttonMos2Stg, buttonMos3Stg, batteryVoltStg, trafoVoltStg, solarVoltStg, windVoltStg, photoResistorStg
  if pin==mosfet_1_pin:
      buttonMos1Stg = text
  elif pin==mosfet_2_pin:
      buttonMos2Stg = text
  elif pin==mosfet_3_pin:
      buttonMos3Stg = text
  elif pin==battery_voltage_pin:
      batteryVoltStg = text
  elif pin==device_charger_voltage_1:
      trafoVoltStg = text
  elif pin==device_charger_voltage_2:
      solarVoltStg = text
  elif pin==device_charger_voltage_3:
      windVoltStg = text
  else: 
      photoResistorStg = text

def onPressMode(pin):
  buttonDisabler(pin)
  alertWindow = Tk(root)
  alertWindow.title("Warning, risk of damage")
  alertWindow.minsize(200,200)
  alertFrame = ttk.Frame(alertWindow, padding ="3 3 12 12")
  alertFrame.grid(column=0, row=0)
  tk.Label(alertFrame, text= "Board mode change read or send voltage signal").grid(column=1,row=1)
  tk.Label(alertFrame, text= "THIS IS ONLY TO TEST BOARD CONNECTIONS AND MCU SIGNALS").grid(column=1,row=2)
  tk.Label(alertFrame, text= "Do you want to proceed ?").grid(column=1,row=3)
  ttk.Button(alertFrame, text="Proceed", command=onPressOk(pin)).grid(column =1, row =4)
  ttk.Button(alertFrame, text="Cancell", command=onPressCancell(pin)).grid(column =2, row =4)

def setOutput(pin):
    if (pin<8):
        setbit(toUpdateDDRD,pin)        
    elif (pin<14):
        setbit(toUpdateDDRB,pin) 
    else:
        setbit(toUpdateDDRC,pin)
    textToButton('OUTPUT',pin)

def setPWM(pin):
    if (pin<8):
        setbit(toUpdateDDRD,pin)        
    elif (pin<14):
        setbit(toUpdateDDRB,pin) 
    else:
        setbit(toUpdateDDRC,pin)
    textToButton('PWM',pin)

def setServo(pin):
    if (pin<8):
        setbit(toUpdateDDRD,pin)        
    elif (pin<14):
        setbit(toUpdateDDRB,pin) 
    else:
        setbit(toUpdateDDRC,pin) 
    textToButton('SERVO',pin)

def setInput(pin):
    if (pin<8):
        unsetbit(toUpdateDDRD,pin)        
    elif (pin<14):
        unsetbit(toUpdateDDRB,pin) 
    else:
        unsetbit(toUpdateDDRC,pin) 
    textToButton('INPUT',pin)

# class to define custom boxes with plenty buttons to change pin status
# each button name is defined in *button as key and configure is value as text
# like this example:
# modeMessageBox= CustomMessage(mosfet_1_pin, root, buttonOUTPUT = "frame, text='OUTPUT', command = setOutput(pin)", buttonINPUT = "frame, text='INPUT', command = setInput(pin)", buttonPWM = "frame, text='PWM', command = setPWM(pin)", buttonServo = "frame, text='SERVO', command = setServo(pin)"  


class CustomMessage(object):

        def __init__(self,pin,parent,**button):

            self.parent=parent
            ScreenWidth = self.parent.winfo_screenwidth()
            ScreenHeight = self.parent.winfo_screenheight()
            self.pin = pin
            title= 'Pin Mode change selector'
            question = 'Please, choose the mode that you want to apply'
            
            # check window size to make root size

            rootSizerWidth= int(ScreenWidth*0.25)
            rootSizerHeight= int(ScreenHeight*0.25)

            topLeftPosition=(int((ScreenWidth- rootSizerWidth)/2),int((ScreenHeight- rootSizerHeight)/2))
            top=Toplevel(self.parent)
            top.geometry(f'{rootSizerWidth}x{rootSizerHeight}+{topLeftPosition[0]}+{topLeftPosition[1]}')
            top.transient(self.parent)
            top.tittle(title)
            top.frame=Frame(top)
            top.label=tk.Label(frame, text=question).grid(column =1,columnspan=1, row =2)
            top.grab_set()
            i=0
            for key, value in button1config.items():        
                ++i        
                key = ttk.Button(f'{value}')
                key.grid(column =i, row =3, sticky=(W,E,N,S))
                frame.columnconfigure(i, weight=1)
                top.label.grid(column =1,columnspan=i, row =2, sticky=( W,E,N,S))
        
            top.frame.columnconfigure(0, weight=1)
            top.frame.rowconfigure(0, weight=1)
            top.frame.rowconfigure(1, weight=1)
            top.frame.rowconfigure(2, weight=1)
            top.frame.rowconfigure(3, weight=1)
            top.frame.rowconfigure(4, weight=1)

            top.frame.grid(column =0, row =0, sticky=(W,E,N,S), pady=(SreemWidth/200))

# this are temp variables to store mode changes until user press process input

toUpdateDDRB=DDRB
toUpdateDDRC=DDRC
toUpdateDDRD=DDRD

    
def onPressOk(pin):
    
    modeMessageBox= CustomMessage(pin, root, buttonOUTPUT = "frame, text='OUTPUT', command = setOutput(pin)", buttonINPUT = "frame, text='INPUT', command = setInput(pin)", buttonPWM = "frame, text='PWM', command = setPWM(pin)", buttonServo = "frame, text='SERVO', command = setServo(pin)")



    
def onPressCancell(button):
    sleep(5)
    alertWindow.quit()
    button.state('enabled')

  
    # Entries dissabled for current values
textMos1 = f'{convertionReadToVolts(pinCurrentValue(mosfet_1_pin),mosfet_1_pin)}'    
valMos1 = ttk.Entry(mainFrame,textvariable=textMos1, state= 'disabled').grid(column=2,row=3)
#valMos1.state(['disabled'])

textMos2 = f'{convertionReadToVolts(pinCurrentValue(mosfet_2_pin),mosfet_2_pin)}'
valMos2 = ttk.Entry(mainFrame,textvariable=textMos2, state= 'disabled').grid(column=2, row=4)
#valMos2.state(['disabled'])

textMos3 = f'{convertionReadToVolts(pinCurrentValue(mosfet_3_pin),mosfet_3_pin)}'
valMos3 = ttk.Entry(mainFrame,textvariable=textMos3, state= 'disabled').grid(column=2, row=5)
#valMos3.state(['disabled'])

textBattery = f'{convertionReadToVolts(pinCurrentValue(battery_voltage_pin),battery_voltage_pin)}'
valBattery = ttk.Entry(mainFrame,textvariable=textBattery, state= 'disabled').grid(column=2, row=6)
#valBattery.state(['disabled'])

textTrafo = f'{convertionReadToVolts(pinCurrentValue(device_charger_voltage_1),device_charger_voltage_1)}'
valTrafo = ttk.Entry(mainFrame,textvariable=textTrafo, state= 'disabled' ).grid(column=2, row=7)
#valTrafo.state(['disabled'])

textSolar = f'{convertionReadToVolts(pinCurrentValue(device_charger_voltage_2),device_charger_voltage_2)}'
valSolar = ttk.Entry(mainFrame,textvariable=textSolar, state= 'disabled' ).grid(column=2, row=8)
#valSolar.state(['disabled'])

textWind = f'{convertionReadToVolts(pinCurrentValue(device_charger_voltage_3),device_charger_voltage_3)}'
valWind = ttk.Entry(mainFrame,textvariable=textWind, state= 'disabled' ).grid(column=2,row =9)
#valWind.state(['disabled'])

textPhotoR = f'{convertionReadToVolts(pinCurrentValue(photo_resistor),photo_resistor)}'
valPhotoR = ttk.Entry(mainFrame,textvariable=textPhotoR, state= 'disabled').grid(column =2, row =10)
#valPhotoR.state(['disabled'])

    # Entries for change values

entryMosfet1= ttk.Entry(mainFrame,show= pinCurrentValue(mosfet_1_pin))
entryMosfet2= ttk.Entry(mainFrame,show= pinCurrentValue(mosfet_2_pin))
entryMosfet3= ttk.Entry(mainFrame,show= pinCurrentValue(mosfet_3_pin))
entryBatteryVolt= ttk.Entry(mainFrame,show= pinCurrentValue(battery_voltage_pin))
entryTrafoVolt= ttk.Entry(mainFrame,show= pinCurrentValue(device_charger_voltage_1))
entrySolarPanelVolt= ttk.Entry(mainFrame,show= pinCurrentValue(device_charger_voltage_2))
entryWindTurbineVolt= ttk.Entry(mainFrame,show= pinCurrentValue(device_charger_voltage_3))
entryPhotoreResistorVolt= ttk.Entry(mainFrame,show= pinCurrentValue(photo_resistor))

if (PC_CONTROL_STATE ==1):
  entryMosfet1.state(['!disabled'])
  entryMosfet2.state(['!disabled'])
  entryMosfet3.state(['!disabled'])
  entryBatteryVolt.state(['!disabled'])
  entryTrafoVolt.state(['!disabled'])
  entrySolarPanelVolt.state(['!disabled'])
  entryWindTurbineVolt.state(['!disabled'])
  entryPhotoreResistorVolt.state(['!disabled'])
else:
  entryMosfet1.state(['disabled'])
  entryMosfet2.state(['disabled'])
  entryMosfet3.state(['disabled'])
  entryBatteryVolt.state(['disabled'])
  entryTrafoVolt.state(['disabled'])
  entrySolarPanelVolt.state(['disabled'])
  entryWindTurbineVolt.state(['disabled'])
  entryPhotoreResistorVolt.state(['disabled'])

entryMosfet1.grid(column =3, row =3)
entryMosfet2.grid(column =3, row =4)
entryMosfet3.grid(column =3, row =5)
entryBatteryVolt.grid(column =3, row =6)
entryTrafoVolt.grid(column =3, row =7)
entrySolarPanelVolt.grid(column =3, row =8)
entryWindTurbineVolt.grid(column =3, row =9)
entryPhotoreResistorVolt.grid(column =3, row =10)









# buttons and function to proceed changes

#Validation the entry with re numbers 1 digit . 0 to 3 digits and also with Ioerror
validation= re.compile(r'\d[.\d{1,3}]{0,1}')

proceedButton=ttk.Button(mainFrame, text="Proceed Change")


def onPressProceed():
    pass      

proceedButton.configure(command=onPressProceed())

if (PC_CONTROL_STATE ==1):
  proceedButton.state(['!disabled'])

else:
  proceedButton.state(['disabled'])

                           
proceedButton.grid(column =4, row =12)


# label and buttona to plot and print historic file
ttk.Label(mainFrame, text="Historical data reporting:").grid(column =1, row =11, sticky='w')                            
ttk.Button(mainFrame, text="Plot").grid(column =2, row =12)
ttk.Button(mainFrame, text="Print").grid(column =3, row =12)

# pending sample data appand on file and plot and print and eventually erase file
def onPressPlot():
  pass
  
def onPressPrint():
  pass

                          
for child in mainFrame.winfo_children():
  child.grid_configure(padx=int(root.winfo_width()/150), pady=int(root.winfo_width()/150))
               
sleep(5)




# root.bind('<Activate>',receiveData())
# root.bind('<Deactivate>',receiveData())
# root.bind('<Visibility>',receiveData())


root.bind('<Configure>',boardResize)

root.mainloop()                                 
