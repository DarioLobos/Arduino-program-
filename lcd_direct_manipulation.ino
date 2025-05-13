#include <avr/io.h>
#include <avr/interrupt.h>

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
 * VO is contrast to a potenciomenter 100 ohm t0 Vcc 220 ohm rersitor to ground
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
//                0000 1100 SET ENTIRE DISPLAY ON, CURSOR OFF
//                0000 1110 SET ENTIRE DISPLAY ON, CURSOR ON
//                0000 1111 SET ENTIRE DISPLAY ON, CURSOR BLINKING
// CURSOR OR 
// DISPLAY SHIFT  0001 00xx MOVE CURSOR TO THE LEFT WITHOUT CHANGE MEMORY DATA
//                0001 10xx MOVE DISPLAY TO THE LEFT WITOUT CHANGE MEMORY DATA
//                0001 01xx MOVE CURSOR TO THE RIGHT WITHOUT CHANGE MEMORY DATA
//                0001 11xx MOVE DISPLAY TO THE RIGHT WITHOUT CHANGE MEMORY DATA
// FUNCTION SET   0010 OOxx MODE 4 BITS DATA BUS, ONE LINE, 5X8 BITS CHARACTERS
//                0011 00xx MODE 8 BITS DATA BUS, ONE LINE, 5X8 BITS CHARACTERS
//                0010 10xx MODE 4 BITS DATA BUS, TWO LINES 5X8 BITS CHARACTERS
//                0011 10xx MODE 8 BITS DATA BUS, TWO LINES 5X8 BITS CHARACTERS   
//                0010 11xx MODE 4 BITS DATA BUS, TWO LINES 5X10 BITS CHARACTERS
//                0011 11xx MODE 8 BITS DATA BUS, TWO LINES 5X10 BITS CHARACTERS
// SET CGRAM 
// ADDRESS        01AA AAAA A= A IS THE ADDRESS OF CUSTOM CHARACTER
// SET DDRAM 
// ADDRESS        1AAA AAAA A= A IS THE ADDRESS OF THE POSITION OR SHIFTED POSITION ( SEE DATASHEET FOR MORE DETAILS)
// BUSY FLAG R/W = 1 (READ) AND D7=1
// WRITE DATA E=1, RS=1 R/W=0
// READ DATA E=1 RS=1 R/W=1
 
// I WILL DO JUST EASY SIMPLE FUNCTIONS FAST STRING SHOULD BE >16

void operationMode(){

      DDRB = (DDRB | B10000000) & B10111111 ; // ENABLE = OUTPUT, PIN 13 AND R/W=INPUT, PIN 12 TO READ BUSY FLAG
      DDRD = (DDRD | B10111100); // RS = OUTPUT, PIN 7  DATABITS OUTPUT, PINS 2,3,4,5

      PORTB = (PORTB | B11000000); // ENABLE=1 R/W=1
      PORTD = (PORTD | B10000000);

      while (bitON(PORTD,5)){
        // WAIT UNTIL BUSY FLAG IS 0
      }

      DDRB = (DDRB | B11000000); // ENABLE = OUTPUT, PIN 13 AND R/W=OUTPUT, PIN 12 TO WRITE FUNCTION
      PORTB = (PORTB | B10000000) & B10111111 ; // ENABLE=1 R/W=0

      PORTD = (PORTD | B00001000)& B11111011 ; // SET 16 X 2 AND 5X8 CHARACTERS HIGH BITS

      delay(0.05);

      PORTD = (PORTD | B0010000)& B11101111 ; // SET 16 X 2 AND 5X8 CHARACTERS LOW BITS

      delay(0.05);
}

void displayFirtLine (String text){

    char * msgchar = & text[0];

    uint8_t charBits [sizeof(text)-1][1];

    uint8_t center = (16 - (sizeof(text)-1))/2-1; // IN FIRST LINE DDRAM ADDRESS START IN CERO SO THIS WILL BE FIRS MEMORY POSITION

      DDRB = (DDRB | B10000000) & B10111111 ; // ENABLE = OUTPUT, PIN 13 AND R/W=INPUT, PIN 12 TO READ BUSY FLAG
      DDRD = (DDRD | B10111100); // RS = OUTPUT, PIN 7  DATABITS OUTPUT, PINS 2,3,4,5

      PORTB = (PORTB | B11000000); // ENABLE=1 R/W=1
      PORTD = (PORTD | B10000000);
      
      while (bitON(PORTD,6)){
        // WAIT UNTIL BUSY FLAG IS 0
      }

      DDRB = (DDRB | B11000000); // ENABLE = OUTPUT, PIN 13 AND R/W=OUTPUT, PIN 12 TO WRITE DDRAM
      PORTB = (PORTB | B10000000) & B10111111 ; // ENABLE=1 R/W=0

     PORTD = (PORTD & B01000011); // DISPLAY ON CURSOR OFF HIGH  BITS 0 PIN 6 IS RESERVED FOR PWM, PIN 0 AND 1 ARE SERIAL
      
      delay(0.05);

     PORTD = (PORTD & B01110011) | B00110000 ; // DISPLAY ON CURSOR OFF LOW BITS 

      delay(0.05);
     
     
     PORTD = (PORTD & B01000011); // ENTRY MODE HIGH  BITS 0 PIN 6 IS RESERVED FOR PWM, PIN 0 AND 1 ARE SERIAL
      
      delay(0.05);

     PORTD = (PORTD & B01011011)| B00011000 ; // ENTRY MODE LOW BITS 

      delay(0.05);

     PORTD = ((PORTD | B00100000) & B01100011)| (center>>4)<<2 ; // RS= 0 D7=1 WRITE ADDRESSS OF START MEMORY HIGH;

      delay(0.05);

      PORTD = (PORTD & B01000011)| ((center<<4)>>4)<<2 ; // RS= 0 D7=1 WRITE ADDRESSS OF START MEMORY LOW;

      delay(0.05);

      for (int i=0; i<sizeof(text)-1; i++){
      msgchar+=i;
      charBits [i][0] = uint8_t(msgchar) >>4;
      charBits [i][1] = (uint8_t(msgchar) <<4) >> 4;       
          
 
      PORTD = (PORTD | B10000000) | charBits [i][0]<<2 ; // RS= 1 WRITE DATA HIGH BITS;

      delay(0.05);

      PORTD = (PORTD | B10000000)| charBits [i][1]<<2 ; // RS= 1 D7=1 WRITE DATA LOW BITS;

      delay(0.05);

      }

      PORTB = (PORTB & B01000011); // ENABLE=0 R/W=1

     delay(0.05);
}

void displaySecondLine (String text){

    char * msgchar = & text[0];

    uint8_t charBits [sizeof(text)-1][1];

    uint8_t center = (16 - (sizeof(text)-1))/2+63 ; // IN SECOND LINE DDRAM ADDRESS START IN 40 HEXA OR 64 DECIMAL THIS WILL BE FIRS MEMORY POSITION

      DDRB = (DDRB | B10000000) & B10111111 ; // ENABLE = OUTPUT, PIN 13 AND R/W=INPUT, PIN 12 TO READ BUSY FLAG
      DDRD = (DDRD | B10111100); // RS = OUTPUT, PIN 7  DATABITS OUTPUT, PINS 2,3,4,5

      PORTB = (PORTB | B11000000); // ENABLE=1 R/W=1
      PORTD = (PORTD | B10000000);
      
      while (bitON(PORTD,6)){
        // WAIT UNTIL BUSY FLAG IS 0
      }

      DDRB = (DDRB | B11000000); // ENABLE = OUTPUT, PIN 13 AND R/W=OUTPUT, PIN 12 TO WRITE DDRAM
      PORTB = (PORTB | B10000000) & B10111111 ; // ENABLE=1 R/W=0

     PORTD = (PORTD & B01000011); // DISPLAY ON CURSOR OFF HIGH  BITS 0 PIN 6 IS RESERVED FOR PWM, PIN 0 AND 1 ARE SERIAL
      
      delay(0.05);

     PORTD = (PORTD & B01110011) | B00110000 ; // DISPLAY ON CURSOR OFF LOW BITS 

      delay(0.05);
     
     
     PORTD = (PORTD & B01000011); // ENTRY MODE HIGH  BITS 0 PIN 6 IS RESERVED FOR PWM, PIN 0 AND 1 ARE SERIAL
      
      delay(0.05);

     PORTD = (PORTD & B01011011)| B00011000 ; // ENTRY MODE LOW BITS 

      delay(0.05);

     PORTD = ((PORTD | B00100000) & B01100011)| (center>>4)<<2 ; // RS= 0 D7=1 WRITE ADDRESSS OF START MEMORY HIGH;

      delay(0.05);

      PORTD = (PORTD & B01000011)| ((center<<4)>>4)<<2 ; // RS= 0 D7=1 WRITE ADDRESSS OF START MEMORY LOW;

      delay(0.05);

      for (int i=0; i<sizeof(text)-1; i++){
      msgchar+=i;
      charBits [i][0] = uint8_t(msgchar) >>4;
      charBits [i][1] = (uint8_t(msgchar) <<4) >> 4;       
          
 
      PORTD = (PORTD | B10000000) | charBits [i][0]<<2 ; // RS= 1 WRITE DATA HIGH BITS;

      delay(0.05);

      PORTD = (PORTD | B10000000)| charBits [i][1]<<2 ; // RS= 1 D7=1 WRITE DATA LOW BITS;

      delay(0.05);

      }

      PORTB = (PORTB & B01000011); // ENABLE=0 R/W=1

     delay(0.05);
}


void setup() {
  operationMode();
displayFirtLine ("Hola");
displaySecondLine ("Es una prueba");



}

void loop() {
  // put your main code here, to run repeatedly:

}
