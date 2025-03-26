######UNFINISHED WORKING ON IT DONT COMPILE#######

##Program to control arduino Nano board with the ChargersController.ino
#autor: Dario Lobos 17/mar/2025

import Tkinter
from tkinter import *
from tkinter import ttk

import pyfirmata2
import pymata
from time import sleep
from PyFirmata2 import Arduino, util
from PyFirmata2 import INPUT, OUTPUT, PWM

it = util.Iterator(board)
it.start()

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
DEVICE_CHARGER_VOLTAGE_2 = A0
DEVICE_CHARGER_VOLTAGE_3 = A1
DEVICE_CHARGER_VOLTAGE_3 = A2   
PHOTO_RESISTOR = A7 

# Assign The mode of PC_CONTROL_PIN for control the board with a button 
PC_CONTROL_STATE =0
PC_CONTROL = board.get_pin('d : PC_CONTROL_PIN :o')
PC_CONTROL.write(0)


def onPressBoardUnlock():
  startButton.state('disabled')
  if PC_CONTROL_STATE ==1:
     PC_CONTROL_STATE =O
     startButton.config(text="Disabbled")
     PC_CONTROL.write(0)
     sleep(5)
     startButton.state('enabled')

  else:
    PC_CONTROL_STATE =1
    startButton.config(text="Enabled")
    PC_CONTROL.write(1)
    sleep(5)
    startButton.state('enabled')
    
def onPressMode(int pin,Ttk button):
  button.state('disabled')
  alertWindow = Tk(root)
  alertWindow.title("Warning, risk of damage")
  alertWindow.minsize(200,200)
  alertFrame = ttk.Frame(alertWindow, padding ="3 3 12 12")
  alertFrame.grid(column=0, row=0, sticky=(N,W,E,S))
  tk.Label(alertFrame, text= "Board mode change read or send voltage signal").grid(column=1,row=1, sticky=W,E,N,S)
  tk.Label(alertFrame, text= "depending on connection can damage the board").grid(column=1,row=2, sticky=W,E,N,S)
  tk.Label(alertFrame, text= "Do you want to proceed ?").grid(column=1,row=3, sticky=W,E,N,S)
  ttk.Button(alertFrame, text="Proceed", command=onPressOk(button)).grid(column =1, row =4, sticky=W,E,N,S)
  ttk.Button(alertFrame, text="Cancell", command=onPressCancell(button)).grid(column =2, row =4, sticky=W,E,N,S)
  
def onPressOk(Ttk button)
  pinHandling = board.get_pin('d : pin :o')
  pinStatus = board.read()
  if pinStatus == 1 :
    pinHandling.write(0)
    button.config(Text"INPUT")
    sleep(5)
    alertWindow.quit()
    button.state('enabled')
  else:
    pinHandling.write(1)
    button.config(Text"OUTPUT")
    sleep(5)
    alertWindow.quit()
    button.state('enabled')
    
def onPressCancell(Ttk button)
    sleep(5)
    alertWindow.quit()
    button.state('enabled')

def pinCurrentValue(int pin):
  pinHandling = board.read(pin)
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

def convertionToWrite(entryValue,pin)

  if pin!=MOSFET_1_PIN && pin!=MOSFET_2_PIN && pin!=MOSFET_3_PIN && pin!=PHOTO_RESISTOR: 
    writeValue= entryValue / (ANALOG_VOLTS * VOLT_FACTOR)
  elif: pin=PHOTO_RESISTOR:
    writeValue=entryValue
  else:  
    writeValue= entryValue / (ANALOG_VOLTS * VOLTS_FACTOR_IN_OP)
  return: writeValue

def convertionReadToVolts(entryValue,pin)
  if pin!=MOSFET_1_PIN && pin!=MOSFET_2_PIN && pin!=MOSFET_3_PIN: 
    readValue= entryValue * ANALOG_VOLTS * VOLT_FACTOR
  else:
    readValue= entryValue * (NALOG_VOLTS * VOLTS_FACTOR_IN_OP
  return: readValue

# Define port type
port = 'COM1' # Down in the code will have the choise for change, this is default

# Define board
board = Arduino(port)
                   
# Define main widget environment dimensions
root = Tk()
root.title("Program to remote control Battery charger and Handle data")
root.minsize(600,600)
mainFrame = ttk.Frame(root, padding ="3 3 12 12")
mainFrame.grid(column=0, row=0, sticky=(N,W,E,S))
root.columnconfigure(0, weight=1)
root.rowfigure(0, weight=1)

# Screen message function for button to control board
ttk.Label(mainFrame, text= "To control the board press the button").grid(column=1,row=1,columnspan=5, sticky=W,N,S)
   
# buttons to control pin mode

ttk.Button(mainFrame, text="Control Board", command=onPressBoardUnlock).grid(column =1, row =2, sticky=W,N,S)

# Labels for columns

ttk.Label(mainFrame, text= "Mode of Pins").grid(column=1,row=2, sticky=(W,E,N,S))
ttk.Label(mainFrame, text= "Current value").grid(column=2,row=2, sticky=(W,E,N,S))
ttk.Label(mainFrame, text= "Value to update").grid(column=3,row=2, sticky=(W,E,N,S))
ttk.Label(mainFrame, text= "Process change").grid(column=4,row=2, sticky=(W,E,N,S))

# Label for image
boardImage = PhotoImage(file='./img/chargerController_bb.png')
label['image'] = image

ttk.Label(mainFrame,image='boardImage').grid(column=5,row=3, rowspan=10, sticky=(W,E,N,S))

# Buttons and labels to control board

    # Buttons to PIN MODE 

ttk.Label(mainFrame, text= "Transformer AC/DC Mosfet").grid(column=1,row=3, sticky=W,E,N)
buttonMos1 = ttk.Button(mainFrame, text="OUTPUT", command=onPressMode(MOSFET_1_PIN,buttonMos1)).grid(column =1, row =3, sticky=(W,E,N,S))

ttk.Label(mainFrame, text= "Solar Panel Mosfet").grid(column=1,row=4, sticky=(W,E,N))
buttonMos2= ttk.Button(mainFrame, text="OUTPUT", command=onPressMode(MOSFET_2_PIN,buttonMos2)).grid(column =1, row =4, sticky=(W,E,N,S))

ttk.Label(mainFrame, text= "Wind generator Mosfet").grid(column=1,row=5, sticky=(W,E,N))
buttonMos3= ttk.Button(mainFrame, text="OUTPUT", command=onPressMode(MOSFET_3_PIN, buttonMos3)).grid(column =1, row =5, sticky=(W,E,N,S))

ttk.Label(mainFrame, text= "Battery Voltage").grid(column=1,row=6, sticky=W,E,N)
batteryVolt= ttk.Button(mainFrame, text="INPUT", command=onPressMode(BATTERY_VOLTAGE_PIN, batteryVolt)).grid(column =1, row =6, sticky=(W,E,N,S))

ttk.Label(mainFrame, text= "Transformer Voltage").grid(column=1,row=7, sticky=W,E,N)
trafoVolt= ttk.Button(mainFrame, text="INPUT", command=onPressMode(DEVICE_CHARGER_VOLTAGE_1, trafoVolt)).grid(column =1, row =7, sticky=(W,E,N,S))

ttk.Label(mainFrame, text= "Solar panel Voltage").grid(column=1,row=8, sticky=W,E,N)
solarVolt= ttk.Button(mainFrame, text="INPUT", command=onPressMode(DEVICE_CHARGER_VOLTAGE_2, solarVolt)).grid(column =1, row =8, sticky=(W,E,N,S))

ttk.Label(mainFrame, text= "Wind gen. Voltage").grid(column=1,row=9, sticky=W,E,N)
windVolt= ttk.Button(mainFrame, text="INPUT", command=onPressMode(DEVICE_CHARGER_VOLTAGE_3, winVolt)).grid(column =1, row =9, sticky=(W,E,N,S))

ttk.Label(mainFrame, text= "Wind gen. Voltage").grid(column=1,row=10, sticky=W,E,N)
windVolt= ttk.Button(mainFrame, text="INPUT", command=onPressMode(PHOTO_RESISTOR, winVolt)).grid(column =1, row =10, sticky=(W,E,N,S))

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

entryMosfet1= ttk.Entry(mainFrame,,show= pinCurrentValue(MOSFET_1_PIN)).grid(column =3, row =3, sticky=(W,E,N,S))
entryMosfet2= ttk.Entry(mainFrame,,show= pinCurrentValue(MOSFET_2_PIN)).grid(column =3, row =4, sticky=(W,E,N,S))
entryMosfet3= ttk.Entry(mainFrame,,show= pinCurrentValue(MOSFET_3_PIN)).grid(column =3, row =5, sticky=(W,E,N,S))
entryBatteryVolt= ttk.Entry(mainFrame,,show= pinCurrentValue(BATTERY_VOLTAGE_PIN)).grid(column =3, row =6, sticky=(W,E,N,S))
entryTrafoVolt= ttk.Entry(mainFrame,,show= pinCurrentValue(DEVICE_CHARGER_VOLTAGE_1)).grid(column =3, row =7, sticky=(W,E,N,S))
entrySolarPanleVolt= ttk.Entry(mainFrame,,show= pinCurrentValue(DEVICE_CHARGER_VOLTAGE_2)).grid(column =3, row =8, sticky=(W,E,N,S))
entryWindTurbineVolt= ttk.Entry(mainFrame,,show= pinCurrentValue(DEVICE_CHARGER_VOLTAGE_3)).grid(column =3, row =9, sticky=(W,E,N,S))
entryPhotoreResistorVolt= ttk.Entry(mainFrame,,show= pinCurrentValue(PHOTO_RESISTOR)).grid(column =3, row =10, sticky=(W,E,N,S))

  # button to proceed changes
  
proceedButton(entryMosfet1,MOSFET_1_PIN,3)
proceedButton(entryMosfet2,MOSFET_2_PIN,4)
proceedButton(entryMosfet3,MOSFET_3_PIN,5)
proceedButton(entryBatteryVolt,BATTERY_VOLTAGE_PIN,6)
proceedButton(entryTrafoVolt,DEVICE_CHARGER_VOLTAGE_1,7)
proceedButton(entrySolarPanleVolt,DEVICE_CHARGER_VOLTAGE_2,8)
proceedButton(entryWindTurbineVolt,DEVICE_CHARGER_VOLTAGE_3,9)
proceedButton(entryPhotoreResistorVolt,PHOTO_RESISTOR,10)

def proceedButton(Ttk entryValue,pin,rowPlaced)
  proceedButton=ttk.Button(mainFrame, text="Proceed Change", command=onPressProceed(entryValue,pin).grid(column =4, row =rowPlaced, sticky=(W,E,N,S))

def onPressProceed(Ttk entryValue,pin)
  proceedButton.state('disabled')           
  pinHandling = board.isOuput(pin)  
  if pinHandling : ???????
    analogWrite(pin,convertionTowrite(entryValue,pin))
    sleep(5)
    proceedButton.state('enabled')
                           
# label and buttona to plot and print historic file
ttk.Label(mainFrame, text="Historical data reporting:").grid(column =1, row =11, sticky=W,N,S)                            
ttk.Button(mainFrame, text="Plot", command=onPressPlot()).grid(column =2, row =12, sticky=W,E,N,S)
ttk.Button(mainFrame, text="Print", command=onPressPrint()).grid(column =2, row =12, sticky=W,E,N,S)

# pending sample data appand on file and plot and print and eventually erase file
def onPressPlot():
  #pending
def onPressPrint():
  #pending

                           
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
