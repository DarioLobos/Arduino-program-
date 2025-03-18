#Program to control arduino Nano board with the ChargersController.ino
#autor: Dario Lobos 17/mar/2025

import Tkinter
from tkinter import *
from tkinter import ttk

import pyfirmata
import pymata
from time import sleep
from PyFirmata import Arduino, util
from PyFirmata import INPUT, OUTPUT, PWM

# Define port type
port = 'COM1'

# Define board
board = pyfirmata.ArduinoMega(port)
nano= {
  'digital' : tuple (x for x in range(14)),
  'analog' : tuple (x for x in range(8)),
  'pwm' : (3,5,6,9,10,11),
  'use_ports' : True,
  'disabled': (0,1)
  }

board.setup_layout(nano)
                   
# Define main widget environment dimensions
root = Tkinter.Tk()
root.title("Program to remote control Battery charger and Handle data")
root.minsize(600,600)
mainFrame = ttk.Frame(root, padding ="3 3 12 12")
mainFrame.grid(column=2, row=9, sticky=(N,W,E,S))
root.columnconfigure(1, weight=1)
root.columnconfigure(2, weight=1)
root.columnconfigure(3, weight=0.5)
root.rowfigure(1, weight=1)
root.rowfigure(1, weight=0.6)
root.rowfigure(2, weight=0.6)
root.rowfigure(3, weight=0.6)
root.rowfigure(4, weight=0.6)
root.rowfigure(5, weight=0.6)
root.rowfigure(6, weight=0.6)
root.rowfigure(7, weight=0.6)
root.rowfigure(8, weight=0.6)
root.rowfigure(9, weight=0.6)
for child in mainframe.winfo_children():
  child.grid_configure(padx=5, pady=5)
               
sleep(5)
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


# Screen message for button
controlLabel = Tkinter.Label(top,Text = "To control the board press the button")
controlLabel.pack()

# Button to control board
boardControlButton = Tkinter.Button(top, text="Control Board", command=onPressBoardUnlock).grid(column =1, row =1, sticky=E)
boardControlButton.pack()
                                 
def onPressBoardUnlock():
  if PC_CONTROL_STATE ==1:
     PC_CONTROL_STATE =O
     startButton.config(text="Disabbled")
     PC_CONTROL.write(0)
     sleep(5)
  else:
    PC_CONTROL_STATE =1
    startButton.config(text="Enabled")
    PC_CONTROL.write(1)
    sleep(5)

