#include <avr/io.h>
#include <avr/interrupt.h>

// THIS IS LCD HANDLING USING DIRECT MANIPULATION 
// AND AN LCD 1602 WITH ST7070 CONTROLLER OR SIMILAR WITH
// NO EXTERNAL RESISTOR FOR BIAS VOLTAGES SEEN IN THE BOARD
// DARIO LOBOS 13/MAY/2025


unsigned long s= micros();
  
// THESE ARE BITWISE FUNCTION NEEDED TO HANDLE DATA 

void setBit( int& n,int pos){
  n|=( 1<< pos );
}

boolean bitON( int n,int pos){
return ( n & (1<<pos)!=0);
}

unsigned int counterBitON(uint8_t data){
  int count=0;
  while(data){
      data &= (data-1); 
      count++;
    }
  return count;  
  }
  
void delayMicros(int micro){
  unsigned long start= micros();
  unsigned long now = micros();
int elapsed=  int (now - start);

while(elapsed < micro ){
// stop delayed micros
now = micros();
elapsed= int (now - start);
if (elapsed < 0) { // overflow control 

elapsed = int (now + 4294967295 - start);

}

}

}


/*
 * The circuit:
 * LCD RS pin to digital pin 7
 * LCD Enable pin to digital pin 13
 * LCD D4 pin to digital pin 2
 * LCD D5 pin to digital pin 3
 * LCD D6 pin to digital pin 4
 * LCD D7 pin to digital pin 5
 * LCD R/W pin 12
 * LCD VSS pin to ground
 * LCD VCC pin to 5V
* ends to +5V and ground
 * VO is contrast to a potenciomenter 100 ohm + 220 ohm to ground  2 parallel 1 kohm to Vcc 220 Vo ≈ 1 volt (can be higher resistors divider )
*/

// PINS 2,3,4,5,7 ARE PORTD AND DDRD 
// PINS 12, 13 ARE PORT B DDRB

// ACCORDING DATASHEET HITACHI HD44780U 
// MODE OF OPERATION FOUR BITS SO A BYTE 8 BITS IS SENT IN TWO SETS FIRST 4 HIGH THEN 4 LOW
// THIS USE D4,D5,D6,D7 (0,1,2,3 ARE DISCONNECTED)
// DATA IS ACCORDING ASCII CODE
// ENABLE MUST PUT IT ON TO READ OR WRITE DATA
// READ IS TO READ FLAG IN D7 WHICH IS BUSY FLAG WHILE IS HIGH MUST NOT BE SENT DATA TO THE LCD ONLY IS AVAILABLE WHEN IS 0
// TO SET DDRAM  ADDRESS D7 MUST BE HIGH THEN THE 7 BITS FOR THE ADDRESS 
// IN 16 X 2 ADDRESS OF FIRST LINE ADDRESS  START IN 00 AND FINISH IN 0F (OR 15 IN DECIMAL)
// IN 16 X 2 ADDRESS OF SECOND LINE ADDRESS  START IN 40 (OR 64 IN DECIMAL) AND FINISH IN 4F (OR 79 IN DECIMAL OR  b1001111)
// SO TO WRITE 40(SECOND LINE START IN FOUR BITS 1100 0000 THE SECOND SET OF FOUR BITS
// FUNCTIONS OR COMMANDS ARE IN REGISTERS ARE:
// CLEAR DISPLAT 0000 0001
// RETURN HOME 0000 001x   x can be 0 or 1
// ENTRY MODE SET 0000 0100 DECREMENT NEXT REGISTER MEMORY POSITION 
//                0000 0101 DECREMENT NEXT REGISTER POSITION AND DISPLAY SHIFT (THIS IS SHIFT TO THE LEFT)
//                0000 0110 INCREMENT NEXT REGOSTER MEMORY POSITION
//                0000 0111 INCREMENT NEXT REGISTER POSITION AND DISPLAY SHIFT (SHIFT TO THE RIGHT)
// DISPLAY ON/OFF 0000 1000 SET ENTIRE DISPLAY OFF, CURSOR OFF 
//                0000 110p SET ENTIRE DISPLAY ON, CURSOR OFF
//                0000 111p SET ENTIRE DISPLAY ON, CURSOR ON
//                0000 111p SET ENTIRE DISPLAY ON
//                p= font type page 1 = 0 font type page 2 = 1
// CURSOR OR 
// DISPLAY SHIFT  0001 00xx MOVE CURSOR TO THE LEFT WITHOUT CHANGE MEMORY DATA
//                0001 10xx MOVE DISPLAY TO THE LEFT WITOUT CHANGE MEMORY DATA
//                0001 01xx MOVE CURSOR TO THE RIGHT WITHOUT CHANGE MEMORY DATA
//                0001 11xx MOVE DISPLAY TO THE RIGHT WITHOUT CHANGE MEMORY DATA
// FUNCTION SET   0010 00xx MODE 4 BITS DATA BUS, ONE LINE, PERFORM MEMORY ACCCESS ADDRESS OPERATIONS
//                0011 00xx MODE 8 BITS DATA BUS, ONE LINE, PERFORM MEMORY ACCCESS ADDRESS OPERATIONS
//                0010 O1xx MODE 4 BITS DATA BUS, ONE LINE, PERFORM BIAS RESISTORS PULL UP OPERATIONS
//                0011 01xx MODE 8 BITS DATA BUS, ONE LINE, PERFORM BIAS RESISTORS PULL UP OPERATIONS
//                0010 10xx MODE 4 BITS DATA BUS, TWO LINES, PERFORM MEMORY ACCCESS ADDRESS OPERATIONS
//                0011 10xx MODE 8 BITS DATA BUS, TWO LINES, PERFORM MEMORY ACCCESS ADDRESS OPERATIONS   
//                0010 11xx MODE 4 BITS DATA BUS, TWO LINES, PERFORM BIAS RESISTORS PULL UP OPERATIONS
//                0011 11xx MODE 8 BITS DATA BUS, TWO LINES, PERFORM BIAS RESISTORS PULL UP OPERATIONS
// BIAS RESISTOR  0000 0100 EXTERNAL BIAS RESISTOR
//                0000 0101 2.2 KOHM 
//                0000 0110 6.8 KOHM
//                0000 0111 9 KOHM
// SET CGRAM 
// ADDRESS        01AA AAAA A= A IS THE ADDRESS OF CUSTOM CHARACTER
// SET DDRAM 
// ADDRESS        1AAA AAAA A= A IS THE ADDRESS OF THE POSITION OR SHIFTED POSITION ( SEE DATASHEET FOR MORE DETAILS)
// BUSY FLAG R/W = 1 (READ) AND D7=1
// WRITE DATA E=1, RS=1 R/W=0
// READ DATA E=1 RS=1 R/W=1
 
// I WILL DO JUST EASY SIMPLE FUNCTIONS FAST STRING SHOULD BE >16

// OPERATION MODE IS INIIALIZING BY INSTRUCTION

void operationMode(){

 DDRD = (DDRD | B10111100); // RS = OUTPUT, PIN 7  DATABITS OUTPUT, PINS 2,3,4,5
 DDRB = (DDRB | B00110000); // ENABLE = OUTPUT, PIN 13 AND R/W=OUTPUT, PIN 12 

  // FIRST FUNCTION SET
 PORTB = (PORTB | B00100000) & B11101111 ; // ENABLE=1 R/W=0
 PORTD = (PORTD | B00001100) & B01111111 ; // RS=0 D5=1 D4=1
 Serial.println(PORTB);
 Serial.println(PORTD);
  
 delayMicros(120);
  
 PORTB = PORTB & B11011111 ; // ENABLE=0 R/W=0
  delay(5); // minimun 4,1
 
 
 PORTB = PORTB | B00100000 ; // ENABLE=1 R/W=0 Second set funcion same as first.
Serial.println("First");
 Serial.println(PORTB);
 Serial.println(PORTD);
 
 delayMicros(120); 

PORTB = PORTB & B11011111 ; // ENABLE=0 R/W=0
Serial.println("Second");
Serial.println(PORTB);
 Serial.println(PORTD);
 
delayMicros(120); 

 PORTB = PORTB | B00100000 ; // ENABLE=1 R/W=0 third set funcion same as first.
Serial.println("Third");
Serial.println(PORTB);
 Serial.println(PORTD);
 
 delayMicros(120); 

PORTB = PORTB & B11011111 ; // ENABLE=0 R/W=0
delayMicros(120); 

PORTB = PORTB | B00100000 ; // ENABLE=1 R/W=0 
PORTD = (PORTD | B00001000) & B01001011 ; // DEFINE 4 BITS OPERATION
delayMicros(120);
PORTB = PORTB & B11011111 ; // ENABLE=0 R/W=0
Serial.println("4 bit operation");
Serial.println(PORTB);
 Serial.println(PORTD);
 
delayMicros(120);

PORTB = PORTB | B00100000 ; // ENABLE=1 R/W=0 
// DEFINE 4 BITS OPERATION SAME FIRST BITS
delayMicros(120);  
PORTB = PORTB & B11011111 ; // ENABLE=0 R/W=0
Serial.println("4 bit operation 2");
Serial.println(PORTB);
 Serial.println(PORTD);

delayMicros(120);

PORTB = PORTB | B00100000 ; // ENABLE=1 R/W=0 
PORTD = (PORTD | B00100000) & B11100011 ; // NEXT LOW BITS TO DEFINE OPERATION D7= 1 TWO LINES 
 delayMicros(120);
PORTB = PORTB & B11011111 ; // ENABLE=0 R/W=0
Serial.println("4 bit operation LOw");
Serial.println(PORTB);
 Serial.println(PORTD);

delayMicros(120);


PORTB = PORTB | B00100000 ; // ENABLE=1 R/W=0 
PORTD = (PORTD & B01000011); // DISPLAY ON CURSOR OFF HIGH  BITS 0 (PIN 6 IS RESERVED FOR PWM, PIN 0 AND 1 ARE Serial)
delayMicros(120);
PORTB = PORTB & B11011111 ; // ENABLE=0 R/W=0
Serial.println("Display on cursor off");
Serial.println(PORTB);
 Serial.println(PORTD);

delayMicros(120);

PORTB = PORTB | B00100000 ; // ENABLE=1 R/W=0 
PORTD = (PORTD | B00110000); // DISPLAY ON CURSOR OFF LOW BITS D3=1 D3=2 DISPLAY ON D1=1 CURSOR OFF D0= FONT 1 
delayMicros(120);
PORTB = PORTB & B11011111 ; // ENABLE=0 R/W=0
Serial.println("Display on cursor off LOW");
Serial.println(PORTB);
 Serial.println(PORTD);

delayMicros(120);

PORTB = PORTB | B00100000 ; // ENABLE=1 R/W=0 

PORTD = PORTD& B11000011 ; // DISPLAY CLEAR HIGH  ALL CERO
delayMicros(120);
PORTB = PORTB & B11011111 ; // ENABLE=0 R/W=0
Serial.println("Display claer");
Serial.println(PORTB);
 Serial.println(PORTD);
delayMicros(120);

PORTB = PORTB | B00100000 ; // ENABLE=1 R/W=0 
PORTD = (PORTD | B00000100); //  DISPLAY CLEAR LOW  BITS D6=1 D5=1
delayMicros(120);
PORTB = PORTB & B11011111 ; // ENABLE=0 R/W=0
Serial.println("Display clear LOW");
Serial.println(PORTB);
 Serial.println(PORTD);

delayMicros(120);

// THIS IS TO CHECK BUSY FLAG BEFORE CONTINUE

 PORTB = (PORTB | B00110000); // ENABLE=1 R/W=1
 PORTD = (PORTD & B01000011); 
 DDRD = (DDRD & B11000011); // RS = OUTPUT, PIN 7  DATABITS INPUT, PINS 2,3,4,5
 delayMicros(120);

 while (bitON(PORTD,5)){
 // WAIT UNTIL BUSY FLAG IS 0
 PORTB = PORTB & B11011111 ; // ENABLE=0 R/W=1
delayMicros(120);
PORTB = PORTB | B00100000 ; // ENABLE=1 R/W=1
delayMicros(120);
 Serial.println("busy flag");
Serial.println(PORTB);
 Serial.println(PORTD);

} 

PORTB = (PORTB & B1110111); // ENABLE=1 R/W=0
DDRD = (DDRD | B00111100); // RS = OUTPUT, PIN 7  DATABITS OUTPUT, PINS 2,3,4,5

PORTD = PORTD & B01000011 ; // HIGH BITS OF ENTRY MODE ALL CERO
delayMicros(120);
PORTB = PORTB & B11011111 ; // ENABLE=0 R/W=0
Serial.println("Entry mode");
Serial.println(PORTB);
 Serial.println(PORTD);

delayMicros(120);

PORTB = PORTB | B00100000 ; // ENABLE=1 R/W=0 
PORTD = PORTD | B01011011 ; // LOW BITS OF ENTRY MODE CURSOR MOVES TO RIGHT AND ADDRESS INCREMENT BY 1 EACH STEP
delayMicros(120);
PORTB = (PORTB & B11011111) | B00010000; // ENABLE=0 R/W=1
Serial.println("Entry mode LOW");
Serial.println(PORTB);
 Serial.println(PORTD);

delayMicros(120);
     
PORTD = PORTD & B01000011 ; // PUT DATA BUS IN CERO
delayMicros(50);
PORTD = (PORTD | B00111100);   // PULL UP RESISTOR INPUT MODE 
DDRD = (DDRD & B11000011); // RS = OUTPUT, PIN 7  DATABITS INPUT, PINS 2,3,4,5


}

void displayFirtLine (String text){

    char * msgchar = & text[0];

    uint8_t charBits [sizeof(text)-1][1];

    uint8_t center = (16 - (sizeof(text)-1))/2-1; // IN FIRST LINE DDRAM ADDRESS START IN CERO SO THIS WILL BE FIRS MEMORY POSITION

    // THIS IS TO CHECK BUSY FLAG BEFORE CONTINUE

PORTB = (PORTB | B00110000); // ENABLE=1 R/W=1
PORTD = (PORTD & B01000011); 
 
while (bitON(PORTD,5)){
// WAIT UNTIL BUSY FLAG IS 0
PORTB = PORTB & B11011111 ; // ENABLE=0 R/W=0
delayMicros(120);
PORTB = PORTB | B00100000 ; // ENABLE=1 R/W=0 
delayMicros(120);
 
} 

PORTB = (PORTB & B11101111); // ENABLE=1 R/W=0
DDRD = (DDRD | B00111100); // RS = OUTPUT, PIN 7  DATABITS OUTPUT, PINS 2,3,4,5

PORTD = ((PORTD | B00100000) & B01100011)| (center>>4)<<2 ; // RS= 0 D7=1 WRITE ADDRESSS OF START MEMORY HIGH BITS;
delayMicros(120);
PORTB = PORTB & B11011111 ; // ENABLE=0 R/W=0
delayMicros(120);


PORTB = PORTB | B00100000 ; // ENABLE=1 R/W=0       
PORTD = (PORTD & B01000011)| ((center<<4)>>4)<<2 ; // RS= 0 WRITE ADDRESSS OF START MEMORY LOW BITS;
PORTB = PORTB & B11011111 ; // ENABLE=0 R/W=0
Serial.println("Memory address");
Serial.println(PORTB);
 Serial.println(PORTD);

delayMicros(120);


for (int i=0; i<sizeof(text)-1; i++){
msgchar+=i;
charBits [i][0] = uint8_t(msgchar) >>4;
charBits [i][1] = (uint8_t(msgchar) <<4) >> 4;       

 // THIS IS TO CHECK BUSY FLAG BEFORE CONTINUE

 PORTB = (PORTB | B00110000); // ENABLE=1 R/W=1
 PORTD = (PORTD & B01000011); 
 DDRD = (DDRD & B11000011); // RS = OUTPUT, PIN 7  DATABITS INPUT, PINS 2,3,4,5
 
 while (bitON(PORTD,5)){
// WAIT UNTIL BUSY FLAG IS 0
PORTB = PORTB & B11011111 ; // ENABLE=0 R/W=1
delayMicros(120);
PORTB = PORTB | B00100000 ; // ENABLE=1 R/W=1 
delayMicros(120); 
} 

PORTB = (PORTB & B11101111); // ENABLE=1 R/W=0
DDRD = (DDRD | B00111100); // RS = OUTPUT, PIN 7  DATABITS OUTPUT, PINS 2,3,4,5

PORTD = (PORTD | B10000000) | charBits [i][0]<<2 ; // RS= 1 WRITE DATA HIGH BITS;
delayMicros(120);
PORTB = PORTB & B11011111 ; // ENABLE=0 R/W=0
delayMicros(120);

PORTB = PORTB | B00100000 ; // ENABLE=1 R/W=0 
PORTD = ((PORTD | B10000000) & B11000011)| charBits [i][1]<<2 ; // RS= 1 D7=1 WRITE DATA LOW BITS;
delayMicros(120);
PORTB = PORTB & B11011111 ; // ENABLE=0 R/W=0
Serial.println("Chart ");
Serial.println(PORTB);
 Serial.println(PORTD);

delayMicros(120);

      }

PORTB = (PORTB & B11011111) | B00010000; // ENABLE=0 R/W=1
PORTD = (PORTD & B01000011); // DATA BUS LOW RS= 0;
delayMicros(50);
PORTD = PORTD | B00111100 ;   // PULL UP RESISTOR INPUT MODE 
DDRD = DDRD & B11000011 ; // RS = OUTPUT, PIN 7  DATABITS INPUT, PINS 2,3,4,5
     
}

void displaySecondLine (String text){




    char * msgchar = & text[0];

    uint8_t charBits [sizeof(text)-1][1];

    uint8_t center = (16 - (sizeof(text)-1))/2+63 ; // IN SECOND LINE DDRAM ADDRESS START IN 40 HEXA OR 64 DECIMAL THIS WILL BE FIRS MEMORY POSITION

    // THIS IS TO CHECK BUSY FLAG BEFORE CONTINUE

PORTB = (PORTB | B00110000); // ENABLE=1 R/W=1
PORTD = (PORTD & B01000011); 
 
while (bitON(PORTD,5)){
// WAIT UNTIL BUSY FLAG IS 0
PORTB = PORTB & B11011111 ; // ENABLE=0 R/W=0
delayMicros(120);
PORTB = PORTB | B00100000 ; // ENABLE=1 R/W=0 
delayMicros(120);
 
} 

PORTB = (PORTB & B11101111); // ENABLE=1 R/W=0
DDRD = (DDRD | B00111100); // RS = OUTPUT, PIN 7  DATABITS OUTPUT, PINS 2,3,4,5

PORTD = ((PORTD | B00100000) & B01100011)| (center>>4)<<2 ; // RS= 0 D7=1 WRITE ADDRESSS OF START MEMORY HIGH BITS;
delayMicros(120);
PORTB = PORTB & B11011111 ; // ENABLE=0 R/W=0
delayMicros(120);


PORTB = PORTB | B00100000 ; // ENABLE=1 R/W=0       
PORTD = (PORTD & B01000011)| ((center<<4)>>4)<<2 ; // RS= 0 WRITE ADDRESSS OF START MEMORY LOW BITS;
PORTB = PORTB & B11011111 ; // ENABLE=0 R/W=0
delayMicros(120);


for (int i=0; i<sizeof(text)-1; i++){
msgchar+=i;
charBits [i][0] = uint8_t(msgchar) >>4;
charBits [i][1] = (uint8_t(msgchar) <<4) >> 4;       

 // THIS IS TO CHECK BUSY FLAG BEFORE CONTINUE

 PORTB = (PORTB | B00110000); // ENABLE=1 R/W=1
 PORTD = (PORTD & B01000011); 
 DDRD = (DDRD & B11000011); // RS = OUTPUT, PIN 7  DATABITS INPUT, PINS 2,3,4,5
 
 while (bitON(PORTD,5)){
// WAIT UNTIL BUSY FLAG IS 0
PORTB = PORTB & B11011111 ; // ENABLE=0 R/W=0
delayMicros(120);
PORTB = PORTB | B00100000 ; // ENABLE=1 R/W=0 
delayMicros(120);
 
} 

PORTB = (PORTB & B11101111); // ENABLE=1 R/W=0
DDRD = (DDRD | B00111100); // RS = OUTPUT, PIN 7  DATABITS OUTPUT, PINS 2,3,4,5

PORTD = (PORTD | B10000000) | charBits [i][0]<<2 ; // RS= 1 WRITE DATA HIGH BITS;
delayMicros(120);
PORTB = PORTB & B11011111 ; // ENABLE=0 R/W=0
delayMicros(120);

PORTB = PORTB | B00100000 ; // ENABLE=1 R/W=0 
PORTD = ((PORTD | B10000000) & B11000011)| charBits [i][1]<<2 ; // RS= 1 D7=1 WRITE DATA LOW BITS;
delayMicros(120);
PORTB = PORTB & B11011111 ; // ENABLE=0 R/W=0
delayMicros(120);

      }

PORTB = (PORTB & B11011111) | B00010000; // ENABLE=0 R/W=1
PORTD = (PORTD & B01000011); // DATA BUS LOW RS= 0;
delayMicros(50);
PORTD = PORTD | B00111100 ;   // PULL UP RESISTOR INPUT MODE 
DDRD = DDRD & B11000011 ; // RS = OUTPUT, PIN 7  DATABITS INPUT, PINS 2,3,4,5
     

}

void setup() {

Serial.begin(9600);
  
  operationMode();
displayFirtLine ("Hola");
displaySecondLine ("Es una prueba");



}

void loop() {
  // put your main code here, to run repeatedly:
Serial.println("Loop ");

delayMicros(120);
}
