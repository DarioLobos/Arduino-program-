
/* THIS DEFINE SHOW IN THE WARNING BOARD DETECTED I INCLUDE THE LIBRARY
TO SEND INFO TO COMPUTER AND SET UP MINS WITH A TUPLE I WILL USE SERIAL
*/

#define BOARD_IDENTIFY_WARNING
#include <Board_Identify.h>

/* I am not using servo but I leave it for other usages to complete data transfer 
 * servo should be more than one when are used and new servo shold be modified 
 * this is because needs one attach per servo, Can be modified as PWM analogWrite 
 * and calculation for the write = angle x 255/ 180 
 * the newServo or as well using direct manipulation and then calculation
  */
  
#include <Servo.h>

Servo myServo;

/*ALL THE PROGRAM IS DONE BY DARIO LOBOS, I TOOK SOME PARTS FROM EXAMPLES, 1O/MAR/2025
 * THE PROGRAM IS TO CONTROL 3 DEVICES TO CHARGE A BATTERY.
 * 
 * BOARD USED IS ARDUINO NANO 
 * 
 * DEVICE 1: IS A TRANSFORMER WITH A RECTIFIER TO HAVE DC CURRENT AND ELECTRICITY COMES FROM 
 * THE POWER SUPPLY NET. THIS CAN CHARGE AFTER A TIME SET BY THE USER. TO CHARGE BATTERY WHEN 
 * THE OTHER DEVICES DON'T REACH THE NEEDED FOR ALL NIGHT. CAN DE SET WITHOUT TIME AND BATTERY 
 * WILL BE CHARGED ALL THE DAY. BY SETTING 23:59 WILL NEVER CHARGE (OR JUST A MINUTE). I WILL 
 * ADD THE OPTION TO DISABLE BUT FOR DISABLES CAN BE DISCONNECTED,
 * 
 * 
 * DEVICE 2: IS A SOLAR PANEL THIS CHARGE WHEN BATTERY DON'T REACH THE MAXIMUN CHARGE AND WHEN
 * A PHOTO RESISTO DETERMINE THAT LIGHT IS ENOUGH TO PRODUCE CURRENT, IF NOT IS DISCONNECTED. 
 * ALSO IF VOLTAGE PRODUCED IS LEES THAN BATTERY IS DISCONNECTED, 
 * PHOTO RESISTOR VALUE CAN BE SET UP BY CONSTANT "PHOTO_PRESISTOR_LIMIT" IN THE CODE.
 * 
 * DEVICE 3: IS A WIND TURBINE AND THE ONLY RESTRICTION ARE BATTERY VOLTAGE AND DEVICE VOLTAGE.
 * 
 * I DID A MOSFET CIRCUIT FOR THIS SYSTEM AND IS HERE(Revised, added FILTERS )
 * 
 * https://drive.google.com/file/d/1AJ6Pxyg2a4SylqBItQMUnToGb4nMWeYA/view?usp=drivesdk
 *
 *
 * https://drive.google.com/file/d/1AI77g_Waz6oLzmIdAXDt1WZzXRl4m_--/view?usp=drivesdk
 * 
 * MUST BE ADJUSTED ACCORDING DEVICE CURRENT AND MOSFET USED, THERE RESISTOR CAN MORE AND LESS FIT  
 * AN IRF520.
 * 
 * CIRCUITS NEED CAPACITORS AND GROUND RESISTORS TO MAKE LESS THAN 60 HRZ LOW PASS FILTER, OTHER RESISTOR TO 
 * GROUND BIG TO DISCHARGE THE SIGNAL. 
 * 
 * FOR EXAMPLE ONE RESISTOR 12K  TO SIGNAL IN (BATTERY AND DEVICES VOLTAGE) AND 47 uf capacitor and 
 * a 5.5K (with and adjust potenciometer small) to do a voltage divider 16/5. Capacitor is to ground 
 * in parallelto R2 of voltage divider. 
 * THIS IS THE VOLTAGE DIVIDER CIRCUIT WITH A ZENER 5V AND A FUSE
 * 
 * https://drive.google.com/file/d/1ANG8-GtRzkY1PIZ6kYH60ZUBgf1YUTnj/view?usp=drivesdk
 *
 *
 * ARDUINO IS CHARGING FROM THE BATTERY  USING AN LM 7805 OR 340 WHICH IS THE SIMPLEST OPTION.
 *
 *  OPERATIONAL AMPLIFIERS NEEDS LM315 TO SET IT TO 10 VOLTS OR 12 VOLTS.
 " 
 * THE MICROCONTROLLER SEND ALL THE DATA USING FIRMATA AND WITH PHYTON IS
 " HANDLED TO MAKE HISTORIAL OF INFO AND GRAPHICS WiTH TkINTER AND PYFIRMATA.
 */
 //-----------------SERIAL TRANSFER PART -----------------

/* THIS CODE IS FROM THE CHIP ATMEGA DATASHEET
 ^ UBRROH, UBRR0H ARE REGISTERS FOR BAUTRATE
 * UCSR0B IS REGISTER FOR TX AND RX WIRES USED 
 * IN RS232 PROTOCOL RXEN0 
 * UCSR0C, UCSZ00,UCSZ01 ARE TO PROTOCOL
 * TO USE 8N1 
 * BAUD AND BAUD_PRESCALLER ARE 
*/

#include <avr/io.h>
#include <avr/interrupt.h>
 
// SET CLOCK TO 16 MHZ

#define F_CPU 16000000UL

// SET TIME OUT TO 1 SEG
//setTimeout( 1000L);

#define BAUDRATE 9600
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) -1)   

void initSerial(void){

  UBRR0H = (uint8_t)(BAUD_PRESCALLER>>8);
  UBRR0L = (uint8_t)(BAUD_PRESCALLER);
/* IN OUR CASE RXEND IS BIT 0 AND 1 CAN BE USED 01 AND 10
 * BOTH POSITIONS ARE SER TO 1 IN THE OR STATEMENT 
 */
  UCSR0B = (1<<RXEN0) | (1<<RXEN0);

  UCSR0C = (1<<UCSZ00) | (1<<UCSZ00);
  } 

// IS CONVENIENT USE THEM WITH TRY TO WHEN ARE USED AND IF FAIL HAPPENS WILL TRY AGAIN IN NEXT LOOP WITHOUT EXCEPTION

unsigned char receiveChar(){
  while(!(UCSR0A & (1<<RXC0))){
    
  if ( UCSR0A & (1<<4)|(1<<3)|(1<<2) ){   // 4= FRAME ERROR 3= OVERRUN ERROR 2= PARITY ERROR
return -1;
}
  return UDR0;  // THIS IS THE BUFFER 3 BYTES REGISTER TO SEND/RECEIVE DATA 
}
}


void sendChar(unsigned char data){
  while(!(UCSR0A & (1<<UDRE0))){
  UDR0= data;  // THIS IS THE BUFFER 3 BYTES REGISTER TO SEND/RECEIVE DATA 
}
}


void sendString(char *StringPtr){
  while (*StringPtr != 0x00){ // HERE THE TRANSMISSION FINISHES IN A NULL CHARACTER CAN BE CHANGED
    sendChar(*StringPtr);
  }
}

void flush(){
unsigned char dummy;
while ( UCSR0A & (1<<RXC0) ) dummy = UDR0;  
}

const char SEND_STATUS=17; // THIS IS THE IDENTIFIER OR FALSE ADDREESS TO REQUEST ALL PINS STATUS


const char RECEIVED=23; // THIS IS THE IDENTIFIER FOR SEND OF DATA


/* PORT B AND PORT C ARE DIGITAL, PORT D IS ANALOG REGISTERS ARE DDRB, DDRC, DDRD, OUTPUT IN 1 
 *  THIS IS THE IDENTIFIER FOR TELL THAT NEXT BYTES WILL BE PWM FIRST CONFIGURATION THEN DATA 
 ^ PARITY IN THE RECEIVER MUST BE THAT PWN ON MUST BE EQUAL TO ARRAY RECEIVED LENGHT OR RESEND
 * AND THE DDRX IN HIGH IF ONE PIN IS NOT AS BOARD CAPAVILITY ALSO MUST REQUEST RESEND
 * SEQUENCE ES 2 CHAR FOR PINS SELECTION AND THEN AND THEN 2CHAR FOR EACH PWM WHICH DATA IS IN  
 * THE PWM ARRAY AND ORDER IS PIN NUMBER 
 * SAME IS POR SERVO
 */

const int IS_ANALOG_READ = 28; // IDENTIFIER FOR ANALOG READ PENDING FIND REGISTER

const int ROW=24;

unsigned char arrayRead [7][1]; // ARRAY TO STORE ANALOG READ DATA PENDING FIND REGISTER
                          // ARE CERO NO CHAR 0
// AVOIDING NULL ERROR

for (int i=0; i<8 ;i++){
  arrayRead[i][0]=0;
  arrayRead[i][1]=0;
}

const char IS_PWM= 18;  // IDENTIFIER FOR PWM

uint8_t pwmRegisterB; // REGISTERS TO IDENTIFY EACH PWM PIN
uint8_t pwmRegisterC;
uint8_t pwmRegisterD;


unsigned char pwmData[ROW];  // ARRAY TO STORE PWM DATA PENDING FIND REGISTER

for (int i=0; i<ROW ;i++){
  pwmData[i]=0;
  }


const char IS_SERVO= 20; // IDENTIFIER FOR SERVO

uint8_t servoRegisterB; // REGISTERS TO IDENTIFY EACH SERVO PIN
uint8_t servoRegisterC;
uint8_t servoRegisterD;

unsigned char servoData[ROW];  // ARRAY TO STORE SERVO DATA PENDING FIND REGISTER

for (int i=0; i<ROW ;i++){
  servoData[i]=0;
  }

const char RESEND= 19;

const char WAIT= 22;

const char END = 23;

const char CHRNULL = 0;

const char BOARD_INFO =6; // THIS IS TO SEND DATA OBTAINED FROM BOARD IDENTIFIER LIBRARY

const int PC_REGISTERS_UPDATE = 14; // THIS IS TO SEND ALL THE REGISTERS AND DATA CHANGED FOR THE PC

// ALL THESE STORE LAST SENT TO AVOID RESEND THE SAME
 
uint8_t prevPortB;
uint8_t prevDDRB;
uint8_t prevPortC;
uint8_t prevDDRC;
uint8_t prevPortD;
uint8_t prevDDRD;

unsigned char prevArrayRead [7][1];
unsigned char prevpwmData [ROW];
unsigned char prevservoData [ROW];

for (int i=0; i<ROW ;i++){
  prevpwmData[i]=0;
  prevservoData[i]=0;
  }
unsigned char receivedAction=0;


// THIS IS TO SEND THE DATA WHEN RECEIVE THE CHAR SEND_STATUS

void boardInfo();
void receibeData();

 
void listen_PC_Start(){

/* exception handling is dissabled in the compiler pending implement 
 *  try catch for the case of wire get disconnected in the middle of
 *  a transmission to don't let program crack for an exception 
 */ 


receivedAction= receiveChar();

while(receiveAction==-1){
  flush();
  receivedAction= receiveChar();
}

// THIS IS NOT USED IS FOR THREAD AND TIMING OF THREADS

  while (receivedAction==WAIT){
   }

   if (receivedAction==BOARD_INFO) {
    boardInfo();
   }

   if (receivedAction==PC_REGISTERS_UPDATE) {
    receiveData();
   }

  if (receivedAction==SEND_STATUS) {

    // SEND PORTS STATUS

    if( PORTD != prevPortD){

      sendChar(unsigned char (PORTD));
      sendChar(unsigned char (DDRD));
      sendChar(unsigned char (pwmRegisterD));
      sendChar(unsigned char(servoRegisterD));
          }
    
    if( PORTC != prevPortC){
    
      sendChar(unsigned char (PORTC));
      sendChar(unsigned char (DDRC));
      sendChar(unsigned char (pwmRegisterC));
      sendChar(unsigned char (servoRegisterC));
    }

    if( PORTB != prevPortB){

      sendChar(unsigned char (PORTB));
      sendChar(unsigned char (DDRB));
      sendChar(unsigned char (pwmRegisterB));
      sendChar(unsigned char(servoRegisterB));      
    }

// SEND ANALOG READ DATA
    
    sendChar(CHRNULL);           //USING TWO BYTES AS IDENTIFIER 
    sendChar(IS_ANALOG_READ);

    if (arrayRead != prevArrayRead){

      unsigned char byteLow;
      unsigned char byteHigh;
        
      for (int i=0; i<8 ;i++){
        if (arrayRead[i][0]!= 0 & arrayRead[i][1]!= 0){ 
           if(arrayRead[i][0]!= prevArrayRead[i][0] | arrayRead[i][1]!= prevArrayRead[i][1]) {
            byteLow= arrayRead[i][0];
            byteHigh=arrayRead[i][1];
            sendChar(byteHigh);
            sendChar(byteLow);
            }
        }
     }
    } 
     
//SEND PWM INFO

    sendChar(CHRNULL);
    sendChar(IS_PWM);

    if (pwmData != prevpwmData){

      for (int i=0; i<ROW ;i++){
        if (pwmData[i]!= 0){ 
          if (pwmData[i]!= prevpwmData[i]){ 
            sendChar(pwmData[i]);
          }
        }
      }
    }
    
//SEND DERVO INFO
   sendChar(IS_SERVO);
  if (servoData != prevservoData){
    
    for (int i=0; i<ROW ;i++){
      if (servoData[i]!= 0){ 
        if (servoData[i]!= prevservoData[i]){ 
          sendChar(servoData[i]);
        }
      }
    }
  }
  sendChar(CHRNULL);  
  sendChar(END);
  
  // ASK DIRECTIONS TO COMPUTER RECEIVED?
  // CONPUTER CAN SEND WAIT AND RECEIVED

      do {
        receivedAction= receiveChar();
        while (receivedAction==WAIT){
        }
        if (receivedAction==RESEND) {
          flush();
          listen_PC_Start();
        }
      }
       while (receivedAction!=RECEIVED);
    
  prevPortB = PORTB;
  prevDDRB = DDRB;
  prevPortC = PORTC;
  prevDDRB = DDRB;
  prevPortD = PORTD;
  prevDDRB = DDRB;
  prevArrayRead = arrayRead;
  prevpwmData = pwmData;
  prevservoData = servoData; 
  }
}

// CHECK WHICH PINS ADMITS PWM USAGE
int pinForPWM[]={3,5,6,9,10,11,14,15,16,17,18,19,20,21};

// IN ORDER TO DO SIMILAR CODES IN PHYTON AND ARDUINO I WILL SET BITS AND COUNT WITH A LOCAL FUNCTION IN BOTH THE SAME
// int& this is a reference of memory position of number and this change bit direct in the addrees that is the data

void setBit( int& n,int pos){
  n|=( 1<< pos );
}

boolean bitON( int n,int pos){
return ( n & (1<<pos)!=0)
}

unsigned int counterBitON(uint8_t data){
  int count=0;
  while(data){
      data &= (data-1) 
      count++;
    }
  return count  
  }
    
void newAnalogWrite(int pin, int value){  
  for (int i=0; i<sizeof(pinForPWM); i++){
    if (pinForPWM[i]==pin){

    unsigned char lowbyte = unsigned char(value);
    
    analogWrite(pin, value);

    pwmData[pin]= lowbyte; 
    
    int n=0
    
    if(pinForPWM[i]<8){
      
      bitON(pwmRegisterB,i);
   
    else if(pinForPWM[i]<16){
      bitON(pwmRegisterC,i-8);    
    }
    else{
      bitON(pwmRegisterD,i-16);
    }
  }  
}
}

void newServo(int pin, int value){  
  for (int i=0; i<sizeof(pinForPWM); i++){
    if (pinForPWM[i]==pin){

    unsigned char lowbyte = unsigned char(value);

    myservo.attach(pin);
    myServo.write(value);

    servoData[pin]= lowbyte; 
    if(pinForPWM[i]<8){
      bitON(servoRegisterB,i);
       
    }
    else if(pinForPWM[i]<16){
      bitON(servoRegisterC,i-8);
       
    }
    else{
      bitON(servoRegisterD,i-16);
    }
  }  
}
}

int pinForAnalog[]={A0,A1,A2,A3,A4,A5,A6,A7};

int newAnalogRead(int pin){
  int value=analogRead(pin);
  for (int i=0; i<sizeof(pinForAnalog); i++){
    if (pinForAnalog[i]==pin){
        uint8_t Highbyte =  value >>8;
        uint8_t Lowbyte = value;
        arrayRead[pin][0] = unsigned char(Highbyte);
        arrayRead[pin][1] = unsigned char(Lowbyte);
    }
  }
  return value;
}

void boardInfo(){
while(true){
sendChar(BOARD_INFO)
sendString(BoardIdentify::type);
sendChar('/n');
sendString(BoardIdentify::make);
sendChar('/n');
sendString(BoardIdentify::model);
sendChar('/n');
sendString(BoardIdentify::mcu);
sendChar('/n');
sendChar(END);
receivedAction= receiveChar();
if(receivedAction==RECEIVED){
 break; 
}
else{
  flush();
}
}
}

      
void receiveData(){
  
int counter=0;
unsigned char receivedRawString[59];
unsigned char receivedPWM[ROW];
unsigned char receivedServo[ROW];
int counterPWM=0;
int counterServo=0;
uint8_t bufferPortB;
uint8_t bufferDDRB;
uint8_t bufferPortC;
uint8_t bufferDDRC;
uint8_t bufferPortD;
uint8_t bufferDDRD;
uint8_t bufferRegisterPWMC;
uint8_t bufferRegisterPWMD;
uint8_t bufferRegisterServoB;
uint8_t bufferRegisterServoC;
uint8_t bufferRegisterServoD;
int counterRegisterPWM=0;
int counterRegisterServo=0;

while(true){
  receivedRawString[counter]=receiveChar();
  if(countew <0){
    if(receivedRawString[counter]==END & receivedRawString[counter-1]==CHRNULL){
      counter=0;
      break;  
  }
  }
  ++counter;
}

bufferPortD= uint8_t(receivedRawString[0]);
bufferDDRD= uint8_t(receivedRawString[1]);
bufferRegisterPWMD= uint8_t(receivedRawString[2]);
bufferRegisterServoD= uint8_t(receivedRawString[3]);

bufferPortC= uint8_t(receivedRawString[4]);
bufferDDRC= uint8_t(receivedRawString[5]);
bufferRegisterPWMC= uint8_t(receivedRawString[6]);
bufferRegisterServoC= uint8_t(receivedRawString[7]);

bufferPortB= uint8_t(receivedRawString[8]);
bufferDDRB= uint8_t(receivedRawString[9]);
bufferRegisterPWMB= uint8_t(receivedRawString[10]);
bufferRegisterServoB= uint8_t(receivedRawString[11]);;

couterRegisterPWM = counterBitON(bufferRegisterPWMD)+ counterBitON(bufferRegisterPWMC) + counterBitON(bufferRegisterPWMB);

couterRegisterServo = counterBitON(bufferRegisterServoD)+ counterBitON(bufferRegisterServoC) + counterBitON(bufferRegisterServoB);

boolean doneReadPWM = False

for(int i=12;i<59;i++){
    if(!doneReadPWM & !(receivedRawString[i+2]==IS_SERVO & receivedRawString[i+1]==CHRNULL) ){
    receivedPWM[counterPWM] = receivedRawString[i];
    ++counterPWM;
    }
    else{
      doneReadPWM=True
      if(!(receivedRawString[i+2]==END & receivedRawString[i+1]==CHRNULL) ){
        receivedServo[counterServo] = receivedRawString[i];
        ++counterServo; 
      }
      else{
        break;
      }
    }
    }
  

  if (counterPWM!=counterRegisterPWM | counterServo!=counterRegisterServo){
  flush()  
  sendChar(RESEND);
  receiveData();
  }
  
  sendChar(RECEIVED);
  flush();

  PORTD = bufferPortD;
  DDRD = bufferDDRD;
  PORTC = bufferPortC;
  DDRC = bufferDDRC;
  PORTB = bufferPortB;
  DDRB = bufferDDRB;

  int placercounter=0;

  for (int i=0; i<ROW; i++){
    pwmData[i]=0;
    if (i<8){
      if(bitON(bufferRegisterPWMB,i)){
        pwmData[i] = receivedPWM[placercounter];
        analogWrite(i,int(pwmData[i]));
        ++placercounter;
        }
    }
    else if (i<16){
          if(bitON(bufferRegisterPWMC,i-8)){
            pwmData[i]= receivedPWM[placercounter];
            analogWrite(i,int(pwmData[i]));
            ++placercounter;
          }
    }
    else{
         if(bitON(bufferRegisterPWMD,i-16)){
            pwmData[i] = receivedPWM[placercounter];
            analogWrite(i,int(pwmData[i]));
            ++placercounter;
          }
    }
  }

  placercounter=0;
  
  for (int i=0; i<ROW; i++){
    servoData[i]=0;
    if (i<8){
      if(bitON(bufferRegisterServoB,i)){
        servoData[i] = receivedServo[placercounter];
        myServo.attach(i);
        myServo.write(int(servoData[i]));
        ++placercounter;
      }
    }
    else if (i<16){
          if(bitON(bufferRegisterServoC,i-8)){
            servoData[i] = receivedServo[placercounter];
            myServo.attach(i); 
            myServo.write(int(servoData[i]));
            ++placercounter;
          }
    }
    else{
         if(bitON(bufferRegisterServoD,i-16)){
            servoData[i] = receivedServo[placercounter];
            myServo.attach(i);
            myServo.write(int(servoData[i]));

            ++placercounter;
          }
    }
  }
}


   
// ASSIGNMENT OF ONE PICK LOCKED FOR REMOTE CONTROL 

int PC_CONTROL_STATE= LOW;

const int PC_CONTROL_PIN= 6;

const int PC_CONTROL_MODE= OUTPUT;



byte pinPc = byte (PC_CONTROL_PIN);

byte currentPinValue;
byte previousPinValue;


byte analogPin = 0;


 
/* THIS IS FOR THE PIN EXTENDER USING THE CHIP MCP23X17 
 *  EXPANDER PIN CONNECTION:
 *  MCP         ARDUINO
 *  PIN 12 TO   A5 (I2C CLOCK)
 *  PIN 13 TO   A4 (I2C DATA)
 *  PIN 15:     GROUND (ONLY ONE EXPANDER CONNECTED, THIS IS TO ADDRESS MORE THAN ONE)
 *  PIN 16:     GROUND
 *  PIN 17:     GROUND
 *  PIN 9:      5V
 *  PIN 10:     GROUND (OF CHIP)
 *  PIN 18:     10 KOHM TO 5V  (CHIP RESET ACTIVE LOW)
 *  
 *  FROM LIBRARY README.MD
 *  When using single pin operations such as _pinMode(pinId, dir)_ or _digitalRead(pinId)_  or _digitalWrite(pinId, val)_ then the pins are addressed using the ID's below. 
 *  For example, for set the mode of _GPB0_ then use _pinMode(8, ...)_. **NOTE** The MCP23008 and MCP23S08 only have _GPAx_ pins.

| MCP23x08 Pin # | MCP23x17 Pin # | Pin Name | Pin ID |
| :------------: | :------------: | :------: | :----: |
|       10       |       21       |   GPA0   |   0    |
|       11       |       22       |   GPA1   |   1    |
|       12       |       23       |   GPA2   |   2    |
|       13       |       24       |   GPA3   |   3    |
|       14       |       25       |   GPA4   |   4    |
|       15       |       26       |   GPA5   |   5    |
|       16       |       27       |   GPA6   |   6    |
|       17       |       28       |   GPA7   |   7    |
|       --       |       1        |   GPB0   |   8    |
|       --       |       2        |   GPB1   |   9    |
|       --       |       3        |   GPB2   |   10   |
|       --       |       4        |   GPB3   |   11   |
|       --       |       5        |   GPB4   |   12   |
|       --       |       6        |   GPB5   |   13   |
|       --       |       7        |   GPB6   |   14   |
|       --       |       8        |   GPB7   |   15   |
for serial transmission is connected to pin 0 and 1 (RX,TX)
*/

#include <Adafruit_MCP23X17.h>
#include <Wire.h>

# define MCP_PIN0 0  
# define MCP_PIN1 1  
# define MCP_PIN2 2
# define MCP_PIN3 3
# define MCP_PIN4 4
# define MCP_PIN5 5
# define MCP_PIN6 6
# define MCP_PIN7 7
# define MCP_PIN8 8
# define MCP_PIN9 9
# define MCP_PIN10 10
# define MCP_PIN11 11
# define MCP_PIN12 12
# define MCP_PIN13 13
# define MCP_PIN14 14
# define MCP_PIN15 15

 Adafruit_MCP23X17 mcp;

// DEFINE RESET PIN



// DS1302:  CE pin    -> Arduino Digital 2
//          I/O pin   -> Arduino Digital 3
//          SCLK pin  -> Arduino Digital 4

#include <DS1302.h>
DS1302 rtc(2, 3, 4);


/* LCD CONTROL PINS THIS GOES HERE IN THE ARDUINO, keyPad WILL BE IN THE EXTENDER. 
 *  keyPad WILL HAVE LESS USAGE THAN LCD ALWAYS WORKING
----------------arduino control part------------------

 * The circuit:
 * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin 8
 * LCD D5 pin to digital pin 7
 * LCD D6 pin to digital pin 6
 * LCD D7 pin to digital pin 5
 * LCD R/W pin to ground
 * LCD VSS pin to ground
 * LCD VCC pin to 5V
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)
 Mellis
 library modified 5 Jul 2009
 by Limor Foldied (http://www.ladyada.net)
 example added 9 Jul 2009
 by Tom Igoe
 modified 22 Nov 2010
 by Tom Igoe
 modified 7 Nov 2016
 by Arturo Guadalupi

 This example code is in the public domain.

 http://www.arduino.cc/en/Tutorial/LiquidCrystalHelloWorld

 I took written parts from this example to for time reduction

*/



#include <LiquidCrystal.h>

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 8, d5 = 7, d6 = 6, d7 = 5;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

const int numRows = 2;
const int numCols = 16;

// LCD MESSAGES
const String LCD_CHARGE_MESSAGE = "BATTERY CHARGING";
const int CHARGE_LENGHT = sizeof(LCD_CHARGE_MESSAGE);
const String LCD_FULL_BATTERY = "BATTERY FULL CHARGED";
const int FULL_LENGHT = sizeof(LCD_FULL_BATTERY);
const String LCD_BATTERY_NO_CHARGE = "POWER SUPPLY DISCONECTED";
const int NO_CHARGE_LENGHT = sizeof(LCD_BATTERY_NO_CHARGE);
const String set_clock_time1 ="Set clk times?";
const String set_clock_time2 ="A=Y B=N C=Clear";
const String clock_or_time1 ="Set clk or time?";
const String clock_or_time2 ="A=Clk B=Tm *=Esc";
const String time_explain1 = "Time for start"; // devices can be two for autogeneration
const String time_explain2 = "charge from net"; // and one from public electricity company
const String time_explain3 = "public network"; // days with no sun or wind to charge.
const String time_explain4 = "for no sun& wind";
const String entry_time1 = "Insert hour 0-24";
const String entry_time2 = "Insert min. 0-60";                               
const String lcdEsc ="           *=Esc";
const String lcdMenu ="#=Menu   ";
const String ok_setup1 = "  Time stored  ";
const String ok_setup2 = "  succefully.  ";
const String ok_clear1 = "The Time stored";
const String ok_clear2 = " was erased.  ";

// BATTERY CHARACTERISTIC
// MEASSURING RANGE 3 T0 5 
// VOLTS 14 TO 12

const int PHOTO_PRESISTOR_LIMIT = 512; // VALUE MUST BE TESTED AND DETERMINE THE SOLAR PANNEL SWITCH OFF.  
int PHOTO_RESISTOR_READ=0;
const float BAT_FULL_VOLTS = 13.7;             
float batteryState = 0;
const float BAT_LOW_VOLTS = 12.5 ;

const float VOLTAGE_DIVIDER_R_TOP = 5.5 ;          // THESE ARE THE VOLTAGE DIVIDER RESISTANCE TO SEND VOLTAGE BATTERY TO
const float VOLTAGE_DIVIDER_R_GROUND = 12;          // THE ANALOG PIN DIRECTLY

const float VOLT_FACTOR = VOLTAGE_DIVIDER_R_GROUND/ (VOLTAGE_DIVIDER_R_GROUND + VOLTAGE_DIVIDER_R_TOP);
const float ANALOG_VOLTS = 5 / 1023 ;

const float CHARGER_VOLTS_1 = 16;  // THIS IS THE TRANSFORMER, SOLAR PANEL, OR OTHER DEVICE VOLTAGE. I HAVE ONLY ONE BUT LEAVE FOR 3
const float CHARGER_VOLTS_2 = 16;
const float CHARGER_VOLTS_3 = 16;


const float VOLTS_FACTOR_IN_OP = 1 ;  // THIS IS A FACTOR TO AMPLIFY THE SIGNAL FROM THE OP. AMPLIFIER 
                                      // WITH IT CAN BE MODIFIED MOSFET GATE VOLTAGE TO HAVE MORE OR LESS CURRENT .
                                      // WITHOUT CHANGE THR CIRCUIT.
                                                      
// Function to transform analog read to voltage according constant voltage divider

float analogVoltageConvertion(int read , float voltFactor){
  float voltage=0;
  voltage = read * ANALOG_VOLTS * voltFactor ;
  return voltage;
}

// convertion from analog read to analog write

int analogConvertionToWrite(int inputread,float voltMax){
    
float inputout = float(inputread) * 255 * (voltMax-BAT_LOW_VOLTS)/1023;

if (inputout > 0){

return int(inputout);
  
}
}

// Function to display voltage and message

void lcdMessage (String message,int lenght, float newAnalogReadVolts ){
   
   lcd.clear();
   float voltTemp=analogVoltageConvertion(newAnalogReadVolts,VOLT_FACTOR);
   String castVolt= String(voltTemp,2);
  char castChar[sizeof(castVolt)];
   for (int i=0; i< sizeof(castVolt);i++){
   castChar[i]= castVolt.charAt(i) ; 
   }
    int thisCol=0;
    int thisRow=0;
    lcd.setCursor(thisCol, thisRow);
    lcd.print(message);
    lcd.display();
    delay(1000);
    
  // scroll positions (string length) to the left
  // to move it offscreen left:
  for (int positionCounter = 0; positionCounter < lenght ; positionCounter++) {
  // scroll one position left:
   
    lcd.scrollDisplayLeft();
    // wait a bit:
    delay(150);    
  }
  thisCol=0;
  thisRow=0;   
  lcd.clear();
  lcd.setCursor(thisCol, thisRow);
  String timeString = rtc.getTimeStr();
  String concatString = lcdMenu + timeString;
  lcd.print(concatString);
  lcd.display();
  thisCol=5;
  thisRow=1;
  lcd.setCursor(thisCol, thisRow);
   for (int i=0; i< sizeof(castVolt);i++){
    
    lcd.setCursor(thisCol+i, thisRow);
   lcd.write(castChar[i]); 
   }
}


void lcdSetupTimes (String message1, String message2){

  lcd.clear();
  int thisCol=0;
    int thisRow=0;
    lcd.setCursor(thisCol, thisRow);
    lcd.print(message1);
    lcd.display();
    thisRow=1;    
    lcd.setCursor(thisCol, thisRow);
    lcd.print(message1);
    lcd.display();
}

//
//    FILE: I2CkeyPad_keymap.ino
//  AUTHOR: Rob Tillaart
// PURPOSE: demo key mapping
//     URL: https://github.com/RobTillaart/I2CkeyPad
//
//  PCF8574 I changed the address for MCP23017 d. lobos
//    pin p0-p3 rows
//    pin p4-p7 columns
//  4x4 or smaller keyPad.


#include <I2CKeyPad.h>

/* KEY PAD PINS 
 *  
 *  PIN NRO.          PIN CONNECTION
 *  MCP_PIN0 0        21
 *  MCP_PIN1 1        22          
 *  MCP_PIN2 2        23
 *  MCP_PIN3 3        24
 *  MCP_PIN4 4        25
 *  MCP_PIN5 5        26
 *  MCP_PIN6 6        27
 *  MCP_PIN7 7        28
 */

const uint8_t keyPad_ADDRESS = 0x20; // address of MCP23017 chip on I2C bus PORT A. 
                                     // ARDUINO HAVE ONLY ONE IC2 PORT SO A0,A1 AND A2 IN EXTENDER GOES TO GROUND 

I2CKeyPad keyPad(keyPad_ADDRESS);

char keymap[19] = "123A456B789C*0#DNF";  // N = NoKey, F = Fail


void Setup_menu_call(){

char key = keyPad.getChar();

void lcdTimeMainMenu (String message1, String message2);

if (key == '#') {
          lcdTimeMainMenu(set_clock_time1,set_clock_time2);
          }

}

 //STORED TIME AND HOUR TO START CHARGE FROM PUBLIC SUPPLY NETWORK
int HOUR_FOR_CHARGE= -1;
int MINUTES_FOR_CHARGE= -1; 


void lcdTimeMainMenu (String message1, String message2) {
int timeAppend=0;
char key;

void lcdMenuA(void);

lcdSetupTimes(message1, message2);
    
while (true){
  
while (!key){
key= keyPad.getChar();
}
switch (key){
  case 'A':
  lcdMenuA();
  break;
 case 'B':
  break;
  case 'C':
  HOUR_FOR_CHARGE= -1;
  MINUTES_FOR_CHARGE= -1;
 lcdSetupTimes(ok_clear1,ok_clear2);
  break;
  case '*':
  break;
}
}
}

void TimeSetAppend(void);
void ClockAppend(void);

void lcdMenuA(void){

char key;  
lcdSetupTimes(clock_or_time1, clock_or_time1);

while (true){
  
while (!key){
key = keyPad.getChar();
}
switch (key){
  case 'A':
  ClockAppend();
  break;
 case 'B':
  TimeSetAppend();
  break;
  
  case '*':
  break;
}
}  
}


void ClockAppend(void){
int hours;
int minutes;  
char key;

lcdSetupTimes(entry_time1, lcdEsc);
   lcd.setCursor(10, 1);
   lcd.write(':'); 

for(int i=0;i<4;i++){

while (true){ 
key = keyPad.getChar();   
while (!key){
  key = keyPad.getChar();
}
if (key=='*'){
  break;
}
if(i==0){
  lcd.setCursor(12, 1); 
if (key>='0' && key<='2'){ 
hours = int(key)*10;
lcd.write(key);
}
}
if(i==1){
  lcd.setCursor(11, 1);

if (key>='0' && key<='9' && hours!=20 && hours!=0){
hours += int(key);
lcd.write(key);

}
if (key>='0' && key<='4' && hours==20){
hours += int(key);
lcd.write(key);
}
if (key>='1' && key<='9' && hours==0){
hours += int(key);
lcd.write(key);
}
}

if(i==2){
  lcd.setCursor(0, 0);
  lcd.print(entry_time2);
  lcd.display();
  lcd.setCursor(9, 1);

if (key>='0' && key<='6'){
minutes=int(key)*10;
lcd.write(key);
}
}

if(i==3){
  lcd.setCursor(8, 1);

if (key>='0' && key<='9'){
minutes+=int(key);
lcd.write(key);
rtc.writeProtect(false);
rtc.setTime(hours,minutes,0);
rtc.writeProtect(true);
lcdSetupTimes(ok_setup1 ,ok_setup2 );
delay(1000);

break;
}
}

}  
}
}


void TimeSetAppend(void){

int hours;
int minutes;  
char key;

lcdSetupTimes(time_explain1 ,time_explain2 );
delay(1000);
lcdSetupTimes(time_explain3 ,time_explain4 );
delay(1000);

lcdSetupTimes(entry_time1, lcdEsc);
   lcd.setCursor(10, 1);
   lcd.write(':'); 

for(int i=0;i<4;i++){

while (true){ 
key = keyPad.getChar();   
while (!key){
  key = keyPad.getChar();
}
if (key=='*'){
  break;
}
if(i==0){
  lcd.setCursor(12, 1); 
if (key>='0' && key<='2'){ 
hours = int(key)*10;
lcd.write(key);
}
}
if(i==1){
  lcd.setCursor(11, 1);

if (key>='0' && key<='9' && hours!=20 && hours!=0){
hours += int(key);
lcd.write(key);

}
if (key>='0' && key<='4' && hours==20){
hours += int(key);
lcd.write(key);
}
if (key>='1' && key<='9' && hours==0){
hours += int(key);
lcd.write(key);
}
}

if(i==2){
  
  lcd.setCursor(0, 0);
  lcd.print(entry_time2);
  lcd.display();
  lcd.setCursor(9, 1);

if (key>='0' && key<='6'){
minutes=int(key)*10;
lcd.write(key);
}
}

if(i==3){
  lcd.setCursor(8, 1);

if (key>='0' && key<='9'){
minutes+=int(key);
lcd.write(key);
HOUR_FOR_CHARGE= hours;
MINUTES_FOR_CHARGE= minutes;
lcdSetupTimes(ok_setup1 ,ok_setup2 );
delay(1000);
break;
}
}

}  
}
}


// initialize constant for pins to read voltages and mosfets
// A4 and A5 ARE RESERVED FOR PIN EXTENDER BOARD, SERIAL PINS

const int MOSFET_1_PIN = A6 ;
int mosfet1Signal=0; // signal to send to the mosfet; 
const int MOSFET_2_PIN = 9 ; // digital pin can be written as analog
int mosfet2Signal=0;
const int MOSFET_3_PIN = 10 ;  // digital pin can be written as analog
int mosfet3Signal=0;

const int BATTERY_VOLTAGE_PIN= A3 ;

const int DEVICE_CHARGER_VOLTAGE_1 = A0 ;
float device1ChargerRead;
const int DEVICE_CHARGER_VOLTAGE_2 = A1 ;  
float device2ChargerRead;
const int DEVICE_CHARGER_VOLTAGE_3 = A2 ;  
float device3ChargerRead;

const int PHOTO_RESISTOR = A7 ; 


void setup()
{  

// initialize the PC control pin as an output:
  pinMode(PC_CONTROL_PIN, PC_CONTROL_MODE);

 // Set the clock to run-mode, and disable the write protection
  rtc.halt(false);
  rtc.writeProtect(true);
  
// SerialBegin for MCP extension board chip 
 Serial.begin(115200);

while( !Serial ){/*wait*/}   //for USB serial switching boards
  Wire.begin( );
  Wire.setClock(400000);
   if (keyPad.begin() == false)
  {
    Serial.println("\nERROR: cannot communicate to keyPad.\nPlease reboot.\n");
    while (1);
  }             
  keyPad.loadKeyMap(keymap);
 
// LCD display 16 x 2 configuration 
  lcd.begin(numCols, numRows);
  
// BEGIN AND PIN ASSIGMENT IN MCP23017 

mcp.begin_I2C();
mcp.pinMode(3, OUTPUT);


// pin assigment modes
  pinMode( MOSFET_1_PIN, OUTPUT);
  pinMode( MOSFET_2_PIN, OUTPUT);
  pinMode( MOSFET_3_PIN, OUTPUT);
  pinMode( DEVICE_CHARGER_VOLTAGE_1, INPUT);
  pinMode( DEVICE_CHARGER_VOLTAGE_2, INPUT);
  pinMode( DEVICE_CHARGER_VOLTAGE_3, INPUT);
  pinMode( PHOTO_RESISTOR, INPUT);
  pinMode( BATTERY_VOLTAGE_PIN, INPUT);

// keyPadaddEventListener(keyPadEvemt); // Listener to detect pressed key l.istener don't work in extender MCP


}

void loop(){
  
  while (PC_CONTROL_STATE == LOW) {
    
if (PC_CONTROL_STATE == HIGH){
break;
}
//-----------------------------------
//Arduino program control
//-----------------------------------

Setup_menu_call();

batteryState = analogVoltageConvertion(newAnalogRead(BATTERY_VOLTAGE_PIN),VOLT_FACTOR);
device1ChargerRead= analogVoltageConvertion(newAnalogRead(DEVICE_CHARGER_VOLTAGE_1),VOLTS_FACTOR_IN_OP);
device2ChargerRead= analogVoltageConvertion(newAnalogRead(DEVICE_CHARGER_VOLTAGE_2),VOLTS_FACTOR_IN_OP);
device3ChargerRead= analogVoltageConvertion (newAnalogRead(DEVICE_CHARGER_VOLTAGE_3),VOLTS_FACTOR_IN_OP);
PHOTO_RESISTOR_READ = newAnalogRead(PHOTO_RESISTOR);
  
while (batteryState <  BAT_FULL_VOLTS) {




mosfet1Signal = analogConvertionToWrite(device1ChargerRead,CHARGER_VOLTS_1); // I ONLY HAVE ONE DEVICE UP TO NOW
mosfet2Signal = analogConvertionToWrite(device2ChargerRead,CHARGER_VOLTS_2);
mosfet3Signal = analogConvertionToWrite(device3ChargerRead,CHARGER_VOLTS_3);

if (mosfet1Signal ==0 &&  mosfet2Signal ==0 && mosfet3Signal ==0){
lcdMessage (LCD_BATTERY_NO_CHARGE,NO_CHARGE_LENGHT, batteryState );
}
else{
  lcdMessage (LCD_CHARGE_MESSAGE,CHARGE_LENGHT, batteryState );

}


String timeclk = rtc.getTimeStr(FORMAT_SHORT);
char hourclk[]= {timeclk.charAt(0),timeclk.charAt(1),timeclk.charAt(3), timeclk.charAt(4)};
int timeint=-1;
for (int i=0;i<4;i++){
timeint += hourclk[i]*10^(3-i);
}
if(timeint>=0){
  timeint+=1;
}

int storedTime= HOUR_FOR_CHARGE^100+ MINUTES_FOR_CHARGE;

if (storedTime <= timeint){
newAnalogWrite(MOSFET_1_PIN,mosfet1Signal); // SIGNAL TO TRANSFORMER MOSFET
}

if ( PHOTO_RESISTOR_READ > PHOTO_PRESISTOR_LIMIT){
newAnalogWrite(MOSFET_2_PIN,mosfet2Signal); // SIGNAL TO SOLAR PANNEL
}

newAnalogWrite(MOSFET_3_PIN,mosfet3Signal); // SIGNAL FOR WIND GENERATOR


   
// Last sentences to update communication with PC

  byte i;



 PC_CONTROL_STATE = digitalRead(PC_CONTROL_PIN);
}
lcdMessage (LCD_FULL_BATTERY,FULL_LENGHT, batteryState );
}

  
  
while (PC_CONTROL_STATE== HIGH) {
  
//
// pc program that will be only call back process and read check.

PC_CONTROL_STATE = digitalRead(PC_CONTROL_PIN);


// reset is necessary because the computer
// can change pins or ports and modes of operation  so is necessary hardware with a jumper and reset pin.
// create a standard reset function
// THE RESET PIN MUST BE CONNECTED TO THE MCP23X17 PHYSICAL PIN NR. 8 (pin ID 15) 

mcp.digitalWrite(MCP_PIN15, HIGH);


}  
}
// END OF FILE
