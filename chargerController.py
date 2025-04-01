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
# import pyfirmata2
# import pymata
# in the easyarduino library I had to modidify the import to firmata2
# \Lib\site-packages\easyarduino\arduino_controller.py
# because firmata through an error if the arduino is not connected
# Also I had to define the modes there because was not defined there (other user need to do that changes)
# INPUT = 0          # as defined in wiring.h
# OUTPUT = 1         # as defined in wiring.h
# ANALOG = 2         # analog pin in analogInput mode
# PWM = 3            # digital pin in PWM output mode
# SERVO = 4          # digital pin in SERVO mode
# INPUT_PULLUP = 11  # Same as INPUT, but with the pin's internal pull-up resistor enabled
#         self.board.digital[pin].mode = self.board.INPUT  // throw error
#         self.board.digital[pin].mode = OUTPUT  MUST BE THIS  
#         elif mode == "OUTPUT":
#         self.board.digital[pin].mode = self.board.OUTPUT  // throw error
#         self.board.digital[pin].mode = OUTPUT  MUST BE THIS
#         elif mode == "PWM":
#         self.board.digital[pin].mode = self.board.PWM  // throw error
#         self.board.digital[pin].mode = PWM  MUST BE THIS  
#         elif mode == "SERVO":
#         self.board.digital[pin].mode = self.board.SERVO // throw error
#         self.board.digital[pin].mode = SERVO  MUST BE THIS

import easyarduino
from time import sleep
#from PyFirmata2 import Arduino, util
#from PyFirmata2 import INPUT, OUTPUT, PWM
from easyarduino import ArduinoController,get_arduino_ports
import re

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
ArduinoPort = 'COM1' # Down in the code will have the choise for change, this is default

# Define board using Pyfirmata easy airduino do the same inside of class
# board = Arduino(ArduinoPort)
# it = util.Iterator(board)
# it.start()

# Define board usinf easy arduino library Iterator and board is in class and use controller
controller= ArduinoController(port= ArduinoPort)

#ARDUINO PORTS IN ARDUINO PROGRAM .INO
# const int PC_CONTROL_PIN= 6
# const int MOSFET_1_PIN = A6 ;
# const int MOSFET_2_PIN = 9 ; // digital pin can be written as analog
# const int MOSFET_3_PIN = 10 ;  // digital pin can be written as analog
# const int BATTERY_VOLTAGE_PIN= A3 ;
# const int DEVICE_CHARGER_VOLTAGE_1 = A0 ;
# const int DEVICE_CHARGER_VOLTAGE_2 = A1 ;  
# const int DEVICE_CHARGER_VOLTAGE_3 = A2 ;  
# const int PHOTO_RESISTOR = A7 ;

# TAKING THE SAME NAMES
PC_CONTROL_PIN= 6
MOSFET_1_PIN = A6
MOSFET_2_PIN = 9
MOSFET_3_PIN = 10
BATTERY_VOLTAGE_PIN= A3
DEVICE_CHARGER_VOLTAGE_1 = A0
DEVICE_CHARGER_VOLTAGE_2 = A1
DEVICE_CHARGER_VOLTAGE_3 = A2   
PHOTO_RESISTOR = A7 

# Assign The mode of PC_CONTROL_PIN for control the board with a button 
PC_CONTROL_STATE =0
controller.set_pin_mode(pin=PC_CONTROL_PIN, mode='OUTPUT')

# Define main widget environment dimensions
root = Tk()
root.title("Program to remote control Battery charger and Handle data")
root.minsize(600,600)
ScreenWidth = root.winfo_screenwidth()
ScreenHeight = root.winfo_screenheight()

rootSizerWidth= int(ScreenWidth*0.8)
rootSizerHeight= int(ScreenHeight*0.8)

topLeftPosition=(int((ScreenWidth- rootSizerWidth)/2),int((ScreenHeight- rootSizerHeight)/2))

root.geometry(f'{rootSizerWidth}x{rootSizerHeight}+{topLeftPosition[0]}+{topLeftPosition[1]}')

# check window size to make image size

mainFrame = ttk.Frame(root, padding ="3 3 12 12")
mainFrame.grid(column=0, row=0, sticky=(N,W,E,S))     

root.columnconfigure(0, weight=1)
root.rowconfigure(0, weight=1)

def onPressBoardUnlock():
  startButton.state('disabled')
  if PC_CONTROL_STATE ==1:
     PC_CONTROL_STATE =O
     startButton.config(text="Disabbled")
     controller.digital_write(pin=PC_CONTROL_PIN, state=False)
     sleep(5)
     startButton.state('enabled')

  else:
    PC_CONTROL_STATE =1
    startButton.config(text="Enabled")
    controller.digital_write(pin=PC_CONTROL_PIN, state=True)
    sleep(5)
    startButton.state('enabled')
# variables for button text

buttonMos1Stg = StringVar()
buttonMos2Stg = StringVar()
buttonMos3Stg = StringVar()
batteryVoltStg = StringVar()
trafoVoltStg = StringVar()
solarVoltStg = StringVar()
windVoltStg = StringVar()
photoResistorSrg =StringVar()


def textToButton(text,pin): 
  if pin==MOSFET_1_PIN:
      buttonMos1Stg = text
  elif pin==MOSFET_2_PIN:
      buttonMos2Stg = text
  elif pin==MOSFET_3_PIN:
      buttonMos3Stg = text
  elif pin==BATTERY_VOLTAGE_PIN:
      batteryVoltStg = text
  elif pin==DEVICE_CHARGER_VOLTAGE_1:
      trafoVoltStg = text
  elif pin==DEVICE_CHARGER_VOLTAGE_2:
      solarVoltStg = text
  elif pin==DEVICE_CHARGER_VOLTAGE_3:
      windVoltStg = text
  else: 
      photoResistorStg = text
            
    


def pinCurrentValue(pin):
  if controller.board.digital[pin].mode==INPUT :
    pinHandling = controller.digital_read(pin)
  else:
    controller.analog_read(pin)
  return pinHandling

# IN CONCORDANCE WITH ARDUINO CODE MUST BE MODIFIED TOGETHER IF NECESSARY
# THIS IS THE ARDUINO CODE             
# const int PHOTO_PRESISTOR_LIMIT = 512; // VALUE MUST BE TESTED AND DETERMINE THE SOLAR PANNEL SWITCH OFF.  
# int PHOTO_RESISTOR_READ=0;
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

def convertionReadToVolts(entryValue,pin):
  if pin!=MOSFET_1_PIN & pin!=MOSFET_2_PIN & pin!=MOSFET_3_PIN: 
    readValue= String(float(entryValue * ANALOG_VOLTS * VOLT_FACTOR,2))
  else:
    readValue= String(float(entryValue * ANALOG_VOLTS * VOLTS_FACTOR_IN_OP,2))
  return readValue
                     

# Screen message function for button to control board
ttk.Label(mainFrame, text= "To control the board press the button").grid(column=1,row=1,columnspan=5, sticky=(W,N,S))
   
# buttons to control pin mode

ttk.Button(mainFrame, text="Control Board", command=onPressBoardUnlock).grid(column =1, row =2, sticky=(W,N,S))

# Labels for columns

ttk.Label(mainFrame, text= "Mode of Pins").grid(column=1,row=2, sticky=(W,E,N,S))
ttk.Label(mainFrame, text= "Current value").grid(column=2,row=2, sticky=(W,E,N,S))
ttk.Label(mainFrame, text= "Value to update").grid(column=3,row=2, sticky=(W,E,N,S))
ttk.Label(mainFrame, text= "Process change").grid(column=4,row=2, sticky=(W,E,N,S))

#function to resize image when window size change
def boardResize(event):
   boardlabel=event.widget
   _imageWidth = int(root.width()/12)
   _imageHeight = int(root.height()/12)
   global sizeChangedBoardImg, sizeChangedBoardPho
   sizeChangedBoardImg= dynamicChangeBoardImg.resize((_imageWidth, _imageHeight))
   sizeChangedBoardPho= ImageTk.PhotoImage(sizeChangedBoardImg)
   boardlabel.config(image=sizeChangedBoardPho)
#   avoid garbage collector
   boardlabel.image = sizeChangedBoardPho

# Label for image
boardImage = Image.open('chargerController_bb.png')
resizedboardImage=boardImage.copy().resize((int(rootSizerWidth/2),int(rootSizerHeight/2)))
dynamicChangeBoardImg=boardImage.copy()
photoBoard=ImageTk.PhotoImage(resizedboardImage)
boardlabel= ttk.Label(mainFrame,image=photoBoard).grid(column=5,row=3, rowspan=10, sticky=(W,E,N,S))
boardlabel.pack()
boardlabel.bind('<Configure>',boardResize)
boardlabel.pack(fill=BOTH, expand=YES)

# Buttons and labels to control board

# Buttons to PIN MODE 

ttk.Label(mainFrame, text= "Transformer AC/DC Mosfet").grid(column=1,row=3, sticky=(W,E,N))
buttonMos1Stg = "PWM"
buttonMos1= ttk.Button(mainFrame, textvariable=buttonMos1Stg)
buttonMos1.grid(column =1, row =3, sticky=(W,E,N,S))

buttonMos2Stg = "PWM"
ttk.Label(mainFrame, text= "Solar Panel Mosfet").grid(column=1,row=4, sticky=(W,E,N))
buttonMos2 = ttk.Button(mainFrame, textvariable=buttonMos2Stg)
buttonMos2.grid(column =1, row =4, sticky=(W,E,N,S))

buttonMos2Stg = "PWM"
ttk.Label(mainFrame, text= "Wind generator Mosfet").grid(column=1,row=5, sticky=(W,E,N))
buttonMos3 = ttk.Button(mainFrame, textvariable=buttonMos2Stg)
buttonMos3.grid(column =1, row =5, sticky=(W,E,N,S))

batteryVoltStg="INPUT"
ttk.Label(mainFrame, text= "Battery Voltage").grid(column=1,row=6, sticky=(W,E,N))
batteryVolt = ttk.Button(mainFrame, textvariable=batteryVoltStg)
batteryVolt.grid(column =1, row =6, sticky=(W,E,N,S))

trafoVoltStg="INPUT"
ttk.Label(mainFrame, text= "Transformer Voltage").grid(column=1,row=7, sticky=(W,E,N))
trafoVolt = ttk.Button(mainFrame, textvariable=trafoVoltStg)
trafoVolt.grid(column =1, row =7, sticky=(W,E,N,S))

solarVoltStg="INPUT"
ttk.Label(mainFrame, text= "Solar panel Voltage").grid(column=1,row=8, sticky=(W,E,N))
solarVolt = ttk.Button(mainFrame, textvariable=solarVoltStg)
solarVolt.grid(column =1, row =8, sticky=(W,E,N,S))

windVoltStg="ÏNPUT"
ttk.Label(mainFrame, text= "Wind gen. Voltage").grid(column=1,row=9, sticky=(W,E,N))
windVolt = ttk.Button(mainFrame, textvariable=windVoltStg)
windVolt.grid(column =1, row =9, sticky=(W,E,N,S))

photoResistorStg="INPUT"
ttk.Label(mainFrame, text= "Photo resistor pin").grid(column=1,row=10, sticky=(W,E,N))
photoResistor = ttk.Button(mainFrame, textvariable=photoResistorStg)
photoResistor.grid(column =1, row =10, sticky=(W,E,N,S))
 

def buttonDisabler(pin):
  if pin==MOSFET_1_PIN:
      buttonMos1.state(['disabled'])
  elif pin==MOSFET_2_PIN:
      buttonMos2.state(['disabled'])
  elif pin==MOSFET_3_PIN:
      buttonMos3.state(['disabled'])
  elif pin==BATTERY_VOLTAGE_PIN:
      batteryVolt.state(['disabled'])
  elif pin==DEVICE_CHARGER_VOLTAGE_1:
      trafoVolt.state(['disabled'])
  elif pin==DEVICE_CHARGER_VOLTAGE_2:
      solarVolt.state(['disabled'])
  elif pin==DEVICE_CHARGER_VOLTAGE_3:
      windVolt.state(['disabled'])
  else: 
      photoResistor.state(['disabled'])

def buttonEnabler(pin):
  if pin==MOSFET_1_PIN:
      buttonMos1.state(['enabled'])
  elif pin==MOSFET_2_PIN:
      buttonMos2.state(['enabled'])
  elif pin==MOSFET_3_PIN:
      buttonMos3.state(['enabled'])
  elif pin==BATTERY_VOLTAGE_PIN:
      batteryVolt.state(['enabled'])
  elif pin==DEVICE_CHARGER_VOLTAGE_1:
      trafoVolt.state(['enabled'])
  elif pin==DEVICE_CHARGER_VOLTAGE_2:
      solarVolt.state(['enabled'])
  elif pin==DEVICE_CHARGER_VOLTAGE_3:
      windVolt.state(['enabled'])
  else: 
      photoResistor.state('enabled')

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
  

def onPressOk(pin):
  if controller.board.digital[pin].mode!=INPUT :
    controller.modeset_pin_mode(pin=pin, mode='INPUT')
    textToButton("INPUT",pin)
    sleep(5)
    alertWindow.quit()
    buttonEnabler(pin)
  elif controller.board.digital[pin].mode!=OUTPUT :
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
  elif controller.board.digital[pin].mode==INPUT :
    outputorpwm = messagebox.askyesno(message='Do you want OUTPUT (yes) OR PWM (no) mode?', icon='question', tittle='INPUT OR PWM instead of INPUT')
    if outputorpwm==True:
      controller.set_pin_mode(pin=pinfunc, mode='OUTPUT')
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

buttonMos1.configure(command='onPressMode(MOSFET_1_PIN)')
buttonMos2.configure(command='onPressMode(MOSFET_2_PIN)')
buttonMos3.configure(command='onPressMode(MOSFET_3_PIN)')
batteryVolt.configure(command='onPressMode(BATTERY_VOLTAGE_PIN)')
trafoVolt.configure(command='onPressMode(DEVICE_CHARGER_VOLTAGE_1)')
solarVolt.configure(command='onPressMode(DEVICE_CHARGER_VOLTAGE_2)')
windVolt.configure(command='onPressMode(DEVICE_CHARGER_VOLTAGE_2)')
photoResistor.configure(command='onPressMode(PHOTO_RESISTOR)')

    # Entries dissabled for current values

ttk.Entry(mainFrame,show= convertionReadToVolts(pinCurrentValue(MOSFET_1_PIN),MOSFET_1_PIN)).grid(column =2, row =3, sticky=(W,E,N,S)).state(dissable)
ttk.Entry(mainFrame,show= convertionReadToVolts(pinCurrentValue(MOSFET_2_PIN),MOSFET_2_PIN)).grid(column =2, row =4, sticky=(W,E,N,S)).state(dissable)
ttk.Entry(mainFrame,show= convertionReadToVolts(pinCurrentValue(MOSFET_3_PIN),MOSFET_3_PIN)).grid(column =2, row =5, sticky=(W,E,N,S)).state(dissable)
ttk.Entry(mainFrame,show= convertionReadToVolts(pinCurrentValue(BATTERY_VOLTAGE_PIN),BATTERY_VOLTAGE_PIN)).grid(column =2, row =6, sticky=(W,E,N,S)).state(dissable)
ttk.Entry(mainFrame,show= convertionReadToVolts(pinCurrentValue(DEVICE_CHARGER_VOLTAGE_1),DEVICE_CHARGER_VOLTAGE_1)).grid(column =2, row =7, sticky=(W,E,N,S)).state(dissable)
ttk.Entry(mainFrame,show= convertionReadToVolts(pinCurrentValue(DEVICE_CHARGER_VOLTAGE_2),DEVICE_CHARGER_VOLTAGE_2)).grid(column =2, row =8, sticky=(W,E,N,S)).state(dissable)
ttk.Entry(mainFrame,show= convertionReadToVolts(pinCurrentValue(DEVICE_CHARGER_VOLTAGE_3),DEVICE_CHARGER_VOLTAGE_3)).grid(column =2, row =9, sticky=(W,E,N,S)).state(dissable)
ttk.Entry(mainFrame,show= convertionReadToVolts(pinCurrentValue(PHOTO_RESISTOR))).grid(column =2, row =10, sticky=(W,E,N,S)).state(dissable)

    # Entries for change values

entryMosfet1= ttk.Entry(mainFrame,show= pinCurrentValue(MOSFET_1_PIN)).grid(column =3, row =3, sticky=(W,E,N,S))
entryMosfet2= ttk.Entry(mainFrame,show= pinCurrentValue(MOSFET_2_PIN)).grid(column =3, row =4, sticky=(W,E,N,S))
entryMosfet3= ttk.Entry(mainFrame,show= pinCurrentValue(MOSFET_3_PIN)).grid(column =3, row =5, sticky=(W,E,N,S))
entryBatteryVolt= ttk.Entry(mainFrame,show= pinCurrentValue(BATTERY_VOLTAGE_PIN)).grid(column =3, row =6, sticky=(W,E,N,S))
entryTrafoVolt= ttk.Entry(mainFrame,show= pinCurrentValue(DEVICE_CHARGER_VOLTAGE_1)).grid(column =3, row =7, sticky=(W,E,N,S))
entrySolarPanleVolt= ttk.Entry(mainFrame,show= pinCurrentValue(DEVICE_CHARGER_VOLTAGE_2)).grid(column =3, row =8, sticky=(W,E,N,S))
entryWindTurbineVolt= ttk.Entry(mainFrame,show= pinCurrentValue(DEVICE_CHARGER_VOLTAGE_3)).grid(column =3, row =9, sticky=(W,E,N,S))
entryPhotoreResistorVolt= ttk.Entry(mainFrame,show= pinCurrentValue(PHOTO_RESISTOR)).grid(column =3, row =10, sticky=(W,E,N,S))

  # button to proceed changes
  
proceedButton(entryMosfet1,MOSFET_1_PIN,3)
proceedButton(entryMosfet2,MOSFET_2_PIN,4)
proceedButton(entryMosfet3,MOSFET_3_PIN,5)
proceedButton(entryBatteryVolt,BATTERY_VOLTAGE_PIN,6)
proceedButton(entryTrafoVolt,DEVICE_CHARGER_VOLTAGE_1,7)
proceedButton(entrySolarPanleVolt,DEVICE_CHARGER_VOLTAGE_2,8)
proceedButton(entryWindTurbineVolt,DEVICE_CHARGER_VOLTAGE_3,9)
proceedButton(entryPhotoreResistorVolt,PHOTO_RESISTOR,10)

#Validation the entry with re numbers 1 digit . 0 to 3 digits and also with Ioerror
validation= re.compile(r'\d{1,1}.\d{0,3}')

def proceedButton(entryValue,pin,rowPlaced):
  
  proceedButton=ttk.Button(mainFrame, text="Proceed Change", command=onPressProceed(entryValue,pin)).grid(column =4, row =rowPlaced, sticky=(W,E,N,S))

def onPressProceed(entryValue,pin):
  proceedButton.state('disabled')
  matchValid= validation.match(entryvalue)
  if matchValid!=None:
    float_entryvalue= float(entryvalue)
    if float_entryvalue<=5:
      if controller.analog_write(pin,convertionToWrite(entryValue))!=IoError: 
        controller.analog_write(pin,convertionToWrite(entryValue))
        sleep(5)
        proceedButton.state('enabled')
      else :
        messagebox.showinfo(tittle='Process Error', icon='alert',message='Pin mode is in Input')
    else :
      messagebox.showinfo(tittle='Process Error', icon='alert',message='Value must be =<5 tree digit maximun 0.000')

  else :
    messagebox.showinfo(tittle='Process Error', icon='alert',message='Entry data must be a number in range 0.000')

                           
# label and buttona to plot and print historic file
ttk.Label(mainFrame, text="Historical data reporting:").grid(column =1, row =11, sticky=(W,N,S))                            
ttk.Button(mainFrame, text="Plot", command=onPressPlot()).grid(column =2, row =12, sticky=(W,E,N,S))
ttk.Button(mainFrame, text="Print", command=onPressPrint()).grid(column =2, row =12, sticky=(W,E,N,S))

# pending sample data appand on file and plot and print and eventually erase file
def onPressPlot():
  print('pending')
  
def onPressPrint():
  print('pending')

                           
mainFrame.columnconfigure(1, weight=1)
mainFrame.columnconfigure(2, weight=1)
mainFrame.columnconfigure(3, weight=1)
mainFrame.columnconfigure(4, weight=1)
mainFrame.columnconfigure(5, weight=4)
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

for child in mainframe.winfo_children():
  child.grid_configure(padx=5, pady=5)
               
sleep(5)

root.mainloop()                                 
