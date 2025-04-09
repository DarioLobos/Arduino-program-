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

# Define analog ports as in pins_arduino.h ARE NOT DEFINED IN Pyfirmata and easyarduino
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


# Define port type
arduinoPort = 'COM1' # Down in the code will have the choise for change, this is default

#_____________________________________________________________________#
#                                                                     #
#                           SERIAL PROTOCOL                           #
#_____________________________________________________________________#

ser =serial.Serial(arduinoPort)

# THESE ARE THE VARIABLES USED TO STORE ARDUINO STATUS RECEIVED AND ALSO
# TO MODIFIED AND SEND THEM BACK TO CHANGE ARDUINO CONFIGURATION

PORTB=0
PORTC=0
PORTD=0
DDRB=0
DDRC=0
DDRD=0

SEND_STATUS=17; # THIS IS THE IDENTIFIER OR FALSE ADDREESS TO REQUEST ALL PINS STATUS
RECEIVED=23;  # THIS IS THE IDENTIFIER FOR SEND OF DATA

IS_ANALOG_READ = 28

ROW=22

for i in range (8):
    for v in range (2):
        arrayRead = dict([(f'{i}{v}',0)]) # ARRAY TO STORE ANALOG READ 1=HIGH 0=LOW BYTE
IS_PWM= 18
pwmRegisterB = 0
pwmRegisterC = 0
pwmRegisterD = 0

for i in range (8):
    pwmData = dict([(f'{i}',0)])  # ARRAY TO STORE PWM DATA
    
IS_SERVO= 20

servoRegisterB = 0 # REGISTERS TO IDENTIFY EACH SERVO PIN
servoRegisterC = 0
servoRegisterD = 0

for i in range (8):
    servoData = dict([(f'{i}',0)]) # ARRAY TO STORE PWM DATA

RESEND= 19
WAIT= 22
END = 23
BOARD_INFO =6; # THIS IS TO SEND DATA OBTAINED FROM BOARD IDENTIFIER LIBRARY
PC_REGISTERS_UPDATE = 14; # THIS IS TO SEND ALL THE REGISTERS AND DATA CHANGED FOR THE PC

# ALL THESE STORE LAST SENT TO AVOID RESEND THE SAME
 
prevPortB = 0
prevDDRB = 0
prevPortC = 0
prevDDRC = 0
prevPortD = 0
prevDDRD = 0

# TO SET DIGITAL PINS CAN BE USED DIRECT PINBx WHICH WILL BECOME OUTPUT
# INDEPENDENT OF DDR. I PREFER USE THIS TO AVOID PROBLEMS WITH ANALOG

for i in range (8):
    prevpwmData = dict([(f'{i}',0)])  # ARRAY TO STORE PREVIOS PWM DATA
    prevservoData = dict([(f'{i}',0)])  # ARRAY TO STORE PREVIOS SERVO DATA


def receiveBoardInfo():

    ser.open()
    ser.write(chr(BOARD_INFO))
    answer_sending=ser.read(1)
    if ( answer_sending == chr(BOARD_INFO)):
        boardInfoType = ser.readline()
        boardInfoMake = ser.readline()
        boardInfoModel = ser.readline()
        boardInfoMCU = ser.readline()
        boardDictionary = dict([('Type', boardInfoType),('Make', boardInfoMake),('Model', boardInfoModel),('MCU', boardInfoMCU)])           answer_sending=ser.read(1)
        answer_sending=ser.read(1)
        if(answer_sending!=chr(END)):
            ser.flush()
            sendBoardInfo()
        else:
            ser.write(chr(RECEIVED))            
    else:
        ser.flush()
        sendBoardInfo()
        
    return  boardDictionary

# IN ORDER TO DO SIMILAR CODES IN PHYTON AND ARDUINO I WILL SET BITS AND COUNT WITH A LOCAL FUNCTION IN BOTH THE SAME

def setBit( n, pos):
        n|=( 1<< pos )


def bitON( n, pos):
        return ( n & (1<<pos)!=0)


def counterBitON(uint8_t data){
  int count=0;
  while(data>0){
      data &= (data-1) 
      count+=count
  return count  

def receiveData():
  
    int counter=0;
    for i in range (76):
        receivedRawString = dict([(f'{i}',0)]) # ARRAY TO STORE INCOMMING CHARACTERS
    for i in range (f'{ROW}'):
        receivedPWM = dict([(f'{i}',0)]) # ARRAY TO STORE INCOMMING PWM CHARACTERS
    for i in range (f'{ROW}'):
        receivedServo = dict([(f'{i}',0)]) # ARRAY TO STORE INCOMMING PWM CHARACTERS
    for i in range (8):
        for v in range (2):
            receivedArrayRead = dict([(f'{i}{v}',0)]) # ARRAY TO STORE INCOMMING ANALOG READ 1=HIGH 0=LOW BYTE

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
    counterArrayRead=0  
    counterRegisterPWM=0
    counterRegisterServo=0
    ser.write(chr(SEND_STATUS)) 
    while(ser.read(1)!=chr(SEND_STATUS):
        ser.flush()
        ser.write(chr(SEND_STATUS))
    while(true):
        receivedRawString[f'{counter}']=ser.read(1)
        if(receivedRawString[f'{counter}']==END){
          counter=0
          break  
        counter+=counter
    bufferPortD= receivedRawString[f'0'])
    bufferDDRD= receivedRawString[f'1'])
    bufferRegisterPWMD= receivedRawString[f'2'])
    bufferRegisterServoD= receivedRawString[f'3'])

    bufferPortC= receivedRawString[f'4']
    bufferDDRC= receivedRawString[f'5']
    bufferRegisterPWMC= receivedRawString[f'6']
    bufferRegisterServoC= receivedRawString[f'7']

    bufferPortB= receivedRawString[f'8']);
    bufferDDRB= receivedRawString[f'9']);
    bufferRegisterPWMB= receivedRawString[f'10']);
    bufferRegisterServoB= receivedRawString[f'11']);;

    couterRegisterPWM = counterBitON(bufferRegisterPWMD)+ counterBitON(bufferRegisterPWMC) + counterBitON(bufferRegisterPWMB);

    couterRegisterServo = counterBitON(bufferRegisterServoD)+ counterBitON(bufferRegisterServoC) + counterBitON(bufferRegisterServoB);

    doneReadArrayRead = False
    doneReadPWM = False
    
    for in range(12,76):
    
        if (receivedRawString[f'{i+1']!=chr(IS_PWM) | !dondeReadArrayRead):
                receivedArrayRead[f'{counterArrayRead}'] = receivedRawString[f'{i}']
                counterArrayRead+= counterArrayRead
        else:
            doneReadArrayRead = True
            if (receivedRawString[f'{i+1}']!=chr(IS_SERVO) | doneReadArrayRead | !doneReadPWM):
                receivedPWM[f'{counterPWM}'] = receivedRawString[f'{i}']
                counterPWM+= counterPWM
            else:
                doneReadPWM=True
                if(receivedRawString[f'{i+1}']!=chr(END)| doneReadArrayRead | doneReadPWM):
                    receivedServo[f'{counterServo}'] = receivedRawString[f'{i}']
                    counterServo+=counterServo
                else:
                    break
    
    if (counterPWM!=counterRegisterPWM | counterServo!=counterRegisterServo):
        ser.flush()  
        ser.write(chr(RESEND))
        receiveData()
    
    sendChar(RECEIVED);
    flush();

    PORTD = bufferPortD
    DDRD = bufferDDRD
    PORTC = bufferPortC
    DDRC = bufferDDRC
    PORTB = bufferPortB
    DDRB = bufferDDRB

    placercounter=0

    for i in range (8):
        for v in range (2):
            arrayRead[F'{i}{v}'] = receivedArrayRead[F'{i}{v}']

    placercounter=0

    for i in range(f'{ROW}'):
        pwmData[f'{i}']=0
        if (i<8):
            if(bitON(bufferRegisterPWMB,i)):
                pwmData['f{i}'] = receivedPWM[f'{placercounter}']
                ++placercounter
        else if (i<16):
            if(bitON(bufferRegisterPWMC,i-8)){
                pwmData[f'{i}']= receivedPWM[f'{placercounter}']
                ++placercounter
        else:
            if(bitON(bufferRegisterPWMD,i-16)){
                pwmData['F{i}'] = receivedPWM[f'{placercounter}']
                ++placercounter

    placercounter=0
  
    for i in range(f'{ROW}'):
        servoData[f'{i}']=0;
        if (i<8):
            if(bitON(bufferRegisterServoB,i)):
                servoData[f'{i}'] = receivedServo[f'{placercounter}']
                ++placercounter
        else if (i<16){
            if(bitON(bufferRegisterServoC,i-8)):
                servoData[f'{i}'] = receivedServo[f'{placercounter}']
                ++placercounter
        else{
            if(bitON(bufferRegisterServoD,i-16)):
            servoData[f'{i}'] = receivedServo[f'{placercounter}']
            ++placercounter
          

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



# Assign The mode of PC_CONTROL_PIN for control the board with a button 

PC_CONTROL_STATE =0

ANALOG_PINS={"Pin":"Pin Number"}

if controller!=None:
# Initialization of pin modes can be done reading status from the board
# so when connection start all  ports must be assigned as working default
# when can be changed by taking control of board. Arduino program call
# back that I did will not take effect of this and only will setup variables
# as working default can be used controller.board.digital[pin].mode or function
# to check status then (don't have function} is reading controller.board.digital[pin].mode
# analog read pin don't have a stored variable so I will do a dictionary  


    controller.set_pin_mode(pin=PC_CONTROL_PIN, mode='OUTPUT')
    controller.set_pin_mode(pin=PC_CONTROL_PIN, mode='OUTPUT')
    controller.set_pin_mode(pin=mosfet_1_pin, mode='PWM')
    controller.set_pin_mode(pin=mosfet_2_pin, mode='PWM')
    controller.set_pin_mode(pin=mosfet_3_pin, mode='PWM')

    # Dictionary to validate ANALOG ports (USER CAN CHANGE IT SO MUST REMOVE, ADD OR REPLACE}
    ANALOG_PINS['battery_voltage_pin']=A3
    ANALOG_PINS['device_charger_voltage_1']=A0
    ANALOG_PINS['device_charger_voltage_2']=A1
    ANALOG_PINS['device_charger_voltage_3']=A2
    ANALOG_PINS['photo_resistor']=A7
    
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

# Binds to check availability in any loop
# at the end with a flag because slow plenty the program the check
# the counter is to make check after some updates
# root.bind('<Activate>',controllerAvalable)
# root.bind('<Deactivate>',controllerAvalable)
# root.bind('<Visibility>',controllerAvalable)
# HAVE THE THREE BECAUSE PROGRAM WILL APPEND PIN STATUS EACH 30 MIN TO A FILE
# AND PLOT IT CAN SET TIME STEP AND WILL PLOT CHARGE STATUS FOR EACH DEVICE
# LIGHT STATUS AND CAN BE DETERMINED CALCULATED CURRENT WITH MOSFETS VOLTS AND
# TEMPERATURE IN FUTURE UPGRADES

counter_availability=0
COUNTER_LIMIT=50   # This can be changed according usage

def controllerAvalable(event):
    global controller, counter_availability
    counter_availability = counter_availability+1
    if counter_availability >=COUNTER_LIMIT:
        try:
            controller = ArduinoController(port= ArduinoPort)
        except:
            controller= None
        else:
            if PC_CONTROL_STATE ==0:
                controller.set_pin_mode(pin=PC_CONTROL_PIN, mode='OUTPUT')
                controller.set_pin_mode(pin=PC_CONTROL_PIN, mode='OUTPUT')
                controller.set_pin_mode(pin=mosfet_1_pin, mode='PWM')
                controller.set_pin_mode(pin=mosfet_2_pin, mode='PWM')
                controller.set_pin_mode(pin=mosfet_3_pin, mode='PWM')

                # Dictionary to validate ANALOG ports (USER CAN CHANGE IT SO MUST REMOVE, ADD OR REPLACE}
                if 'battery_voltage_pin' not in ANALOG_PINS:
                    ANALOG_PINS['battery_voltage_pin']=A3
                if 'device_charger_voltage_1' not in ANALOG_PINS:    
                    ANALOG_PINS['device_charger_voltage_1']=A0
                if 'device_charger_voltage_2' not in ANALOG_PINS:    
                    ANALOG_PINS['device_charger_voltage_2']=A1
                if 'device_charger_voltage_3' not in ANALOG_PINS:    
                    ANALOG_PINS['device_charger_voltage_3']=A2
                if 'photo_resistor' not in ANALOG_PINS:    
                    ANALOG_PINS['photo_resistor']=A7
            counter_availability =0

mainFrame = ttk.Frame(root, padding ="3 3 12 12")

# Bind to check availability in any loop


# Make an frame special for the image to can resize it well

photoFrame = ttk.Frame(mainFrame, padding ="3 3 12 12")

def onPressBoardUnlock():
    mainFrame.startButton.state('disabled')
    if controller!=None: 
        if PC_CONTROL_STATE ==1:
            PC_CONTROL_STATE =O
            mainframe.startButton.config(text="Disabled")
            controller.digital_write(pin=PC_CONTROL_PIN, state=False)
            sleep(5)
            startButton.state('enabled')

        else:
            PC_CONTROL_STATE =1
            mainFrame.startButton.config(text="Enabled")
            controller.digital_write(pin=PC_CONTROL_PIN, state=True)
            sleep(5)
            mainFrame.startButton.state('enabled')

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
  if controller!= None:
    if isPinMode==INPUT :
        pinHandling = controller.digital_read(pin)
        return pinHandling
    else:
        pinHandling = controller.analog_read(pin)
        return pinHandling
  else:
    return "0.000"

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
ttk.Label(mainFrame, text= "To control the board press the button").grid(column=1,row=1,columnspan=5, sticky=(W,N,S))
   
# buttons to control pin mode

controlBoardPBtn=ttk.Button(mainFrame, text="Control Board", command=onPressBoardUnlock).grid(column=1,row=2,columnspan=5, sticky=(W,N,S))

if controller!=None:
    controlBoardPBtn.state(['!disabled'])
else:
    controlBoardPBtn.state(['disabled'])
    controlBoardPBtn.configure(text="Control Board N/A")

controlBoardPBtn.grid(column =1, row =2, sticky=(W,N,S))

# Labels for columns

ttk.Label(mainFrame, text= "Mode of Pins").grid(column=1,row=2, sticky=(W,E,N,S))
ttk.Label(mainFrame, text= "Current value").grid(column=2,row=2, sticky=(W,E,N,S))
ttk.Label(mainFrame, text= "Value to update").grid(column=3,row=2, sticky=(W,E,N,S))
ttk.Label(mainFrame, text= "Process change").grid(column=4,row=2, sticky=(W,E,N,S))

#function to resize image when window size change PENDING USE FRAME INSTEAD OF LABEL
def boardResize(event):
#  boardlabel = event.widget
   _imageWidth = int(root.winfo_width()/2.5)
   _imageHeight = int(root.winfo_height()/2.5)
   global sizeChangedBoardImg, sizeChangedBoardPho
   sizeChangedBoardImg= dynamicChangeBoardImg.resize((_imageWidth,_imageHeight))
   sizeChangedBoardPho= ImageTk.PhotoImage(sizeChangedBoardImg)
   boardlabel.config(image=sizeChangedBoardPho)
   # avoid garbage collector
   boardlabel.image = sizeChangedBoardPho
#   boardlabel.pack(fill=BOTH, expand=True, anchor='center')

# Label for image
boardImage = Image.open('chargerController_bb.png')

imageWidth = int(rootSizerWidth * 0.4)
imageHeight = int(rootSizerHeight * 0.4)
resizedboardImage=boardImage.copy().resize((imageWidth,imageHeight))
dynamicChangeBoardImg=boardImage.copy()
photoBoard=ImageTk.PhotoImage(resizedboardImage)
boardlabel= ttk.Label(photoFrame,image=photoBoard)
boardlabel.pack(fill=BOTH, expand=True, anchor='center')
photoFrame.grid(column=5,row=3, rowspan=10, sticky=(W,E,N,S))



# Buttons and labels to control board

# Buttons to PIN MODE 

isPinMode= None

pin=mosfet_1_pin

try:
    isPinMode=controller.board.digital[pin].mode
except: 
    isPinMode= None
    
if controller!=None:
  if isPinMode!=INPUT and isPinMode!=None :
    if isPinMode!=OUTPUT and isPinMode!=None :
        buttonMos1Stg = "PWM"
    else:
        if isPinMode!=None :
            buttonMos1Stg = "OUTPUT"
        else:
            buttonMos1Stg = "N/A"  

  else:
        if isPinMode!=None :
            buttonMos1Stg = "INPUT"
        else:
            buttonMos1Stg = "N/A"  

else:
  buttonMos1Stg = "N/A"  
ttk.Label(mainFrame, text= "Transformer AC/DC Mosfet").grid(column=1,row=3, sticky=(W,E,N))
buttonMos1= ttk.Button(mainFrame, textvariable=buttonMos1Stg)
buttonMos1.grid(column =1, row =3, sticky=(W,E,S))

pin=mosfet_2_pin

try:
    isPinMode=controller.board.digital[pin].mode
except: 
    isPinMode= None

if controller!=None:
  if isPinMode!=INPUT and isPinMode!=None :
    if isPinMode!=OUTPUT and isPinMode!=None :
        buttonMos1Stg = "PWM"
    else:
        if isPinMode!=None :
            buttonMos1Stg = "OUTPUT"
        else:
            buttonMos1Stg = "N/A"  

  else:
        if isPinMode!=None :
            buttonMos1Stg = "INPUT"
        else:
            buttonMos1Stg = "N/A"  

else:
  buttonMos1Stg = "N/A"  
ttk.Label(mainFrame, text= "Solar Panel Mosfet").grid(column=1,row=4, sticky=(W,E,N))
buttonMos2 = ttk.Button(mainFrame, textvariable=buttonMos2Stg)
buttonMos2.grid(column =1, row =4, sticky=(W,E,S))

pin=mosfet_3_pin

try:
    isPinMode=controller.board.digital[pin].mode
except: 
    isPinMode= None

if controller!=None:
  if isPinMode!=INPUT and isPinMode!=None :
    if isPinMode!=OUTPUT and isPinMode!=None :
        buttonMos1Stg = "PWM"
    else:
        if isPinMode!=None :
            buttonMos1Stg = "OUTPUT"
        else:
            buttonMos1Stg = "N/A"  

  else:
        if isPinMode!=None :
            buttonMos1Stg = "INPUT"
        else:
            buttonMos1Stg = "N/A"  

else:
  buttonMos1Stg = "N/A"  
  ttk.Label(mainFrame, text= "Wind generator Mosfet").grid(column=1,row=5, sticky=(W,E,N))
buttonMos3 = ttk.Button(mainFrame, textvariable=buttonMos2Stg)
buttonMos3.grid(column =1, row =5, sticky=(W,E,S))


pin=battery_voltage_pin

try:
    isPinMode=controller.board.digital[pin].mode
except: 
    isPinMode= None

if controller!=None:
  if isPinMode!=INPUT and isPinMode!=None :
    if isPinMode!=OUTPUT and isPinMode!=None :
        buttonMos1Stg = "PWM"
    else:
        if isPinMode!=None :
            buttonMos1Stg = "OUTPUT"
        else:
            buttonMos1Stg = "N/A"  

  else:
        if isPinMode!=None :
            buttonMos1Stg = "INPUT"
        else:
            buttonMos1Stg = "N/A"  

else:
  buttonMos1Stg = "N/A"  
ttk.Label(mainFrame, text= "Battery Voltage").grid(column=1,row=6, sticky=(W,E,N))
batteryVolt = ttk.Button(mainFrame, textvariable=batteryVoltStg)
batteryVolt.grid(column =1, row =6, sticky=(W,E,S))

pin=device_charger_voltage_1

try:
    isPinMode=controller.board.digital[pin].mode
except: 
    isPinMode= None


if controller!=None:
  if isPinMode!=INPUT and isPinMode!=None :
    if isPinMode!=OUTPUT and isPinMode!=None :
        buttonMos1Stg = "PWM"
    else:
        if isPinMode!=None :
            buttonMos1Stg = "OUTPUT"
        else:
            buttonMos1Stg = "N/A"  

  else:
        if isPinMode!=None :
            buttonMos1Stg = "INPUT"
        else:
            buttonMos1Stg = "N/A"  

else:
  buttonMos1Stg = "N/A"  
ttk.Label(mainFrame, text= "Transformer Voltage").grid(column=1,row=7, sticky=(W,E,N))
trafoVolt = ttk.Button(mainFrame, textvariable=trafoVoltStg)
trafoVolt.grid(column =1, row =7, sticky=(W,E,S))

pin=device_charger_voltage_2

try:
    isPinMode = controller.board.digital[pin].mode
except: 
    isPinMode= None

if controller!=None:
  if isPinMode!=INPUT and isPinMode!=None :
    if isPinMode!=OUTPUT and isPinMode!=None :
        buttonMos1Stg = "PWM"
    else:
        if isPinMode!=None :
            buttonMos1Stg = "OUTPUT"
        else:
            buttonMos1Stg = "N/A"  

  else:
        if isPinMode!=None :
            buttonMos1Stg = "INPUT"
        else:
            buttonMos1Stg = "N/A"  

else:
  buttonMos1Stg = "N/A"  
ttk.Label(mainFrame, text= "Solar panel Voltage").grid(column=1,row=8, sticky=(W,E,N))
solarVolt = ttk.Button(mainFrame, textvariable=solarVoltStg)
solarVolt.grid(column =1, row =8, sticky=(W,E,S))

pin=device_charger_voltage_3

try:
    isPinMode=controller.board.digital[pin].mode
except: 
    isPinMode= None


if controller!=None:
  if isPinMode!=INPUT and isPinMode!=None :
    if isPinMode!=OUTPUT and isPinMode!=None :
        buttonMos1Stg = "PWM"
    else:
        if isPinMode!=None :
            buttonMos1Stg = "OUTPUT"
        else:
            buttonMos1Stg = "N/A"  

  else:
        if isPinMode!=None :
            buttonMos1Stg = "INPUT"
        else:
            buttonMos1Stg = "N/A"  

else:
  buttonMos1Stg = "N/A"  
ttk.Label(mainFrame, text= "Wind gen. Voltage").grid(column=1,row=9, sticky=(W,E,N))
windVolt = ttk.Button(mainFrame, textvariable=windVoltStg)
windVolt.grid(column =1, row =9, sticky=(W,E,S))

pin= photo_resistor

try:
    isPinMode=controller.board.digital[pin].mode
except: 
    isPinMode= None

if controller!=None:
  if isPinMode!=INPUT and isPinMode!=None :
    if isPinMode!=OUTPUT and isPinMode!=None :
        photoResistorStg = "PWM"
    else:
        if isPinMode!=None :
            photoResistorStg = "OUTPUT"
        else:
            photoResistorStg = "N/A"  

  else:
        if isPinMode!=None :
            photoResistorStg = "INPUT"
        else:
            photoResistorStg = "N/A"  

else:
  buttonMos1Stg = "N/A"  
ttk.Label(mainFrame, text= "Photo resistor pin").grid(column=1,row=10, sticky=(W,E,N))
photoResistor = ttk.Button(mainFrame, textvariable=photoResistorStg)
photoResistor.grid(column =1, row =10, sticky=(W,E,S))
 

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
  alertFrame.grid(column=0, row=0, sticky=(N,W,E,S))
  tk.Label(alertFrame, text= "Board mode change read or send voltage signal").grid(column=1,row=1, sticky=(W,E,N,S))
  tk.Label(alertFrame, text= "depending on connection can damage the board").grid(column=1,row=2, sticky=(W,E,N,S))
  tk.Label(alertFrame, text= "Do you want to proceed ?").grid(column=1,row=3, sticky=(W,E,N,S))
  ttk.Button(alertFrame, text="Proceed", command=onPressOk(pin)).grid(column =1, row =4, sticky=(W,E,N,S))
  ttk.Button(alertFrame, text="Cancell", command=onPressCancell(pin)).grid(column =2, row =4, sticky=(W,E,N,S))

class CustomMessage(object):

        def __init__(self, parent, *titlesAndQuestion, **buttonText):

            self.parent=parent
            ScreenWidth = self.parent.winfo_screenwidth()
            ScreenHeight = self.parent.winfo_screenheight()

            # check window size to make root size

            rootSizerWidth= int(ScreenWidth*0.25)
            rootSizerHeight= int(ScreenHeight*0.25)

            topLeftPosition=(int((ScreenWidth- rootSizerWidth)/2),int((ScreenHeight- rootSizerHeight)/2))
            top=Toplevel(self.parent)
            top.geometry(f'{rootSizerWidth}x{rootSizerHeight}+{topLeftPosition[0]}+{topLeftPosition[1]}')
            top.transient(self.parent)
            top.tittle(titlesAndQuestion[0])
            top.frame=Frame(top)
            top.label=Label(frame, text=titlesAndQuestion[1]).pack(pady=(SreemWidth/200))
            frame.pack(pady=(SreemWidth/200))
            top.grab_set()
            top.frameButton=Frame(top)
            for key, value in buttonText.items():
                text=f" ,{"{}='{}'".format(key,value)}"        
                key=Button(f'{top.frameButton} {text:}')
                key.pack(side=LEFT)
            top.frameButton.pack()
        
# check window size to make root size

rootSizerWidth= int(ScreenWidth*0.8)
rootSizerHeight= int(ScreenHeight*0.8)

topLeftPosition=(int((ScreenWidth- rootSizerWidth)/2),int((ScreenHeight- rootSizerHeight)/2))

root.geometry(f'{rootSizerWidth}x{rootSizerHeight}+{topLeftPosition[0]}+{topLeftPosition[1]}')

  
def onPressOk(pin):
    global controller
    if isPinMode!=INPUT and isPinMode!=None :
        controller.modeset_pin_mode(pin=pin, mode='INPUT')
        textToButton("INPUT",pin)
        sleep(5)
        alertWindow.quit()
        buttonEnabler(pin)
    elif isPinMode!=OUTPUT and isPinMode!=None :
        inputoroutput = messagebox.askyesno(message='Do you want INPUT (yes) OR OUTPUT (no) mode?', icon='question', tittle='INPUT OR OUTPUT instead of PWM')
        if inputoroutput==True:
            controller.set_pin_mode(pin=pinfunc, mode='INPUT')
            textToButton("INPUT",pin)
            sleep(5)
            alertWindow.quit()
            buttonEnabler(pin)
        else:
            controller.set_pin_mode(pin=pin, mode='OUTPUT')
            textToButton("OUTPUT",pin)
            sleep(5)
            alertWindow.quit()
            buttonEnabler(pin)
    elif isPinMode==INPUT :
        outputorpwm = messagebox.askyesno(message='Do you want OUTPUT (yes) OR PWM (no) mode?', icon='question', tittle='INPUT OR PWM instead of INPUT')
        if outputorpwm==True:
            controller.set_pin_mode(pin=pin, mode='OUTPUT')
            textToButton("OUTOUT",pin)
            sleep(5)
            alertWindow.quit()
            buttonEnabler(pin)
        else:
            controller.set_pin_mode(pin=pin, mode='PWM')
            textToButton("PWM",pin)
            sleep(5)
            alertWindow.quit()
            buttonEnabler(pin)
    else:
        outputorpwm = messagebox.askyesno(message='Do you want INPUT (yes) OR PWM (no) mode?', icon='question', tittle='INPUT OR PWM instead of OUTPUT')
        if outputorpwm==True:
            controller.set_pin_mode(pin=pinfunc, mode='INPUT')
            textToButton("INPUT",pin)
            sleep(5)
            alertWindow.quit()
            buttonEnabler(pin)
        else:
            controller.set_pin_mode(pin=pin, mode='PWM')
            textToButton("PWM",pin)
            sleep(5)
            alertWindow.quit()
            alertWindow.quit()
            buttonEnabler(pin)
    
def onPressCancell(button):
    sleep(5)
    alertWindow.quit()
    button.state('enabled')

if controller!= None and PC_CONTROL_STATE ==1 :
  buttonMos1.state(['!disabled'])
  buttonMos1.configure(command='onPressMode(mosfet_1_pin)')
  buttonMos2.state(['!disabled'])
  buttonMos2.configure(command='onPressMode(mosfet_2_pin)')
  buttonMos3.state(['!disabled'])
  buttonMos3.configure(command='onPressMode(mosfet_3_pin)')
  batteryVolt.state(['!disabled'])
  batteryVolt.configure(command='onPressMode(battery_voltage_pin)')
  trafoVolt.state(['!disabled'])
  trafoVolt.configure(command='onPressMode(device_charger_voltage_1)')
  solarVolt.state(['!disabled'])
  solarVolt.configure(command='onPressMode(device_charger_voltage_2)')
  windVolt.state(['!disabled'])
  windVolt.configure(command='onPressMode(device_charger_voltage_2)')
  photoResistor.state(['!disabled'])
  photoResistor.configure(command='onPressMode(photo_resistor)')
else:
  buttonMos1.state(['disabled'])
  buttonMos2.state(['disabled'])
  buttonMos3.state(['disabled'])
  batteryVolt.state(['disabled'])
  trafoVolt.state(['disabled'])
  solarVolt.state(['disabled'])
  windVolt.state(['disabled'])
  photoResistor.state(['disabled'])
  
    # Entries dissabled for current values
textMos1 = f'{convertionReadToVolts(pinCurrentValue(mosfet_1_pin),mosfet_1_pin)}'    
valMos1 = ttk.Entry(mainFrame,textvariable=textMos1).grid(column=2,row=3, sticky=(W,E,N,S))
#valMos1.state(['disabled'])

textMos2 = f'{convertionReadToVolts(pinCurrentValue(mosfet_2_pin),mosfet_2_pin)}'
valMos2 = ttk.Entry(mainFrame,textvariable=textMos2).grid(column=2, row=4, sticky=(W,E,N,S))
#valMos2.state(['disabled'])

textMos3 = f'{convertionReadToVolts(pinCurrentValue(mosfet_3_pin),mosfet_3_pin)}'
valMos3 = ttk.Entry(mainFrame,textvariable=textMos3).grid(column=2, row=5, sticky=(W,E,N,S))
#valMos3.state(['disabled'])

textBattery = f'{convertionReadToVolts(pinCurrentValue(battery_voltage_pin),battery_voltage_pin)}'
valBattery = ttk.Entry(mainFrame,textvariable=textBattery).grid(column=2, row=6, sticky=(W,E,N,S))
#valBattery.state(['disabled'])

textTrafo = f'{convertionReadToVolts(pinCurrentValue(device_charger_voltage_1),device_charger_voltage_1)}'
valTrafo = ttk.Entry(mainFrame,textvariable=textTrafo ).grid(column=2, row=7, sticky=(W,E,N,S))
#valTrafo.state(['disabled'])

textSolar = f'{convertionReadToVolts(pinCurrentValue(device_charger_voltage_2),device_charger_voltage_2)}'
valSolar = ttk.Entry(mainFrame,textvariable=textSolar ).grid(column=2, row=8, sticky=(W,E,N,S))
#valSolar.state(['disabled'])

textWind = f'{convertionReadToVolts(pinCurrentValue(device_charger_voltage_3),device_charger_voltage_3)}'
valWind = ttk.Entry(mainFrame,textvariable=textWind ).grid(column=2,row =9, sticky=(W,E,N,S))
#valWind.state(['disabled'])

textPhotoR = f'{convertionReadToVolts(pinCurrentValue(photo_resistor),photo_resistor)}'
valPhotoR = ttk.Entry(mainFrame,textvariable=textPhotoR).grid(column =2, row =10, sticky=(W,E,N,S))
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

if controller!= None and PC_CONTROL_STATE ==1:
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

entryMosfet1.grid(column =3, row =3, sticky=(W,E,N,S))
entryMosfet2.grid(column =3, row =4, sticky=(W,E,N,S))
entryMosfet3.grid(column =3, row =5, sticky=(W,E,N,S))
entryBatteryVolt.grid(column =3, row =6, sticky=(W,E,N,S))
entryTrafoVolt.grid(column =3, row =7, sticky=(W,E,N,S))
entrySolarPanelVolt.grid(column =3, row =8, sticky=(W,E,N,S))
entryWindTurbineVolt.grid(column =3, row =9, sticky=(W,E,N,S))
entryPhotoreResistorVolt.grid(column =3, row =10, sticky=(W,E,N,S))









# buttons and function to proceed changes

#Validation the entry with re numbers 1 digit . 0 to 3 digits and also with Ioerror
validation= re.compile(r'\d[.\d{1,3}]{0,1}')

proceedButton1=ttk.Button(mainFrame, text="Proceed Change")
proceedButton2=ttk.Button(mainFrame, text="Proceed Change")
proceedButton3=ttk.Button(mainFrame, text="Proceed Change")
proceedButton4=ttk.Button(mainFrame, text="Proceed Change")
proceedButton5=ttk.Button(mainFrame, text="Proceed Change")
proceedButton6=ttk.Button(mainFrame, text="Proceed Change")
proceedButton7=ttk.Button(mainFrame, text="Proceed Change")
proceedButton8=ttk.Button(mainFrame, text="Proceed Change")



def onPressProceed(entryValue,pin):
      
    match pin:
      case int(mosfet_1_pin):
        proceedButton1.state(['disabled'])
      case int(mosfet_2_pin):
        proceedButton2.state(['disabled'])
      case int(mosfet_3_pin):
        proceedButton3.state(['disabled'])
      case int(battery_voltage_pin):
        proceedButton4.state(['disabled'])
      case int(device_charger_voltage_1):
        proceedButton5.state(['disabled'])
      case int(device_charger_voltage_2):
        proceedButton6.state(['disabled'])
      case int(device_charger_voltage_3):
        proceedButton7.state(['disabled'])
      case int(photo_resistor):
        proceedButton8.state(['disabled'])
  
    matchValid= validation.match(f'{entryValue}')
    if matchValid!=None:
        float_entryvalue= float(entryValue)
        if float_entryvalue<=5:
            try:
                controller.analog_write(pin,convertionToWrite(entryValue)) 
                sleep(5)
            except :
                if float_entryvalue==1 or float_entryvalue==0:
                    try:
                        controller.digital_write(pin,convertionToWrite(entryValue)) 
                        sleep(5)
                    except :
                        messagebox.showinfo(title="Process Error", icon='warning',message="Pin mode is in Input")
        else :
            messagebox.showinfo(title="Process Error", icon='warning',message="Value must be =<5 tree digit maximun 0.000 format for Analog 0.000, 1 or 0 for Digital")
    else :
        messagebox.showinfo(title="Process Error", icon='warning',message="Entry data must be a number in range format for Analog 0.000, 1 or 0 for Digital")

    match pin:
      case int(mosfet_1_pin):
        proceedButton1.state(['!disabled'])
      case int(mosfet_2_pin):
        proceedButton2.state(['!disabled'])
      case int(mosfet_3_pin):
        proceedButton3.state(['!disabled'])
      case int(battery_voltage_pin):
        proceedButton4.state(['!disabled'])
      case int(device_charger_voltage_1):
        proceedButton5.state(['!disabled'])
      case int(device_charger_voltage_2):
        proceedButton6.state(['!disabled'])
      case int(device_charger_voltage_3):
        proceedButton7.state(['!disabled'])
      case int(photo_resistor):
        proceedButton8.state(['!disabled'])
  

proceedButton1.configure(command=onPressProceed(entryMosfet1,mosfet_1_pin))
proceedButton2.configure(command=onPressProceed(entryMosfet2,mosfet_2_pin))
proceedButton3.configure(command=onPressProceed(entryMosfet3,mosfet_3_pin))
proceedButton4.configure(command=onPressProceed(entryBatteryVolt,battery_voltage_pin))
proceedButton5.configure(command=onPressProceed(entryTrafoVolt,device_charger_voltage_1))
proceedButton6.configure(command=onPressProceed(entrySolarPanelVolt,device_charger_voltage_2))
proceedButton7.configure(command=onPressProceed(entryWindTurbineVolt,device_charger_voltage_3))
proceedButton8.configure(command=onPressProceed(entryPhotoreResistorVolt,photo_resistor))

if controller!= None and PC_CONTROL_STATE ==1:
  proceedButton1.state(['!disabled'])
  proceedButton2.state(['!disabled'])
  proceedButton3.state(['!disabled'])
  proceedButton4.state(['!disabled'])
  proceedButton5.state(['!disabled'])
  proceedButton6.state(['!disabled'])
  proceedButton7.state(['!disabled'])
  proceedButton8.state(['!disabled'])

else:
  proceedButton1.state(['disabled'])
  proceedButton2.state(['disabled'])
  proceedButton3.state(['disabled'])
  proceedButton4.state(['disabled'])
  proceedButton5.state(['disabled'])
  proceedButton6.state(['disabled'])
  proceedButton7.state(['disabled'])
  proceedButton8.state(['disabled'])

                           
proceedButton1.grid(column =4, row =3, sticky=(W,E,N,S))
proceedButton2.grid(column =4, row =4, sticky=(W,E,N,S))
proceedButton3.grid(column =4, row =5, sticky=(W,E,N,S))
proceedButton4.grid(column =4, row =6, sticky=(W,E,N,S))
proceedButton5.grid(column =4, row =7, sticky=(W,E,N,S))
proceedButton6.grid(column =4, row =8, sticky=(W,E,N,S))
proceedButton7.grid(column =4, row =9, sticky=(W,E,N,S))
proceedButton8.grid(column =4, row =10, sticky=(W,E,N,S))


# label and buttona to plot and print historic file
ttk.Label(mainFrame, text="Historical data reporting:").grid(column =1, row =11, sticky=(W,N,S))                            
ttk.Button(mainFrame, text="Plot").grid(column =2, row =12, sticky=(W,E,N,S))
ttk.Button(mainFrame, text="Print").grid(column =4, row =12, sticky=(W,E,N,S))

# pending sample data appand on file and plot and print and eventually erase file
def onPressPlot():
  print('pending')
  
def onPressPrint():
  print('pending')

                          
for child in mainFrame.winfo_children():
  child.grid_configure(padx=int(root.winfo_width()/150), pady=int(root.winfo_width()/150))
               
sleep(5)

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
mainFrame.grid(column=0, row=0, sticky=(N,W,E,S))

root.bind('<Activate>',controllerAvalable)
root.bind('<Deactivate>',controllerAvalable)
root.bind('<Visibility>',controllerAvalable)

root.columnconfigure(0, weight=1)
root.rowconfigure(0, weight=1)
root.bind('<Configure>',boardResize)

root.mainloop()                                 
