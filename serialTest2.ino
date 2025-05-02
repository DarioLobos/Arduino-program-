/* THIS IS TO TEST THIS SERIAL TRANSFER USING ARDUINO IDE MONITOR
 * ANY KEY TURN ON A LED IN PIN 11 AND A IN PIN 5 AND B TURN IT OF  
 * THE NUMBERS WORK FOR SEND IF YOU SEND 97 MONITOR SHOW a, but monitor
 * can't send numbers
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

// PARITY NEEDS MANUAL CONFIGURATION OF COMPUTER CONTROLLER, TO AVOID THIS I CHOOSE NONE PARITY
#define ASYNCHRONOUS (0<<UMSEL00)
#define PARITY_MODE (0<<UPM00) // NON PARITY MODE,  EVEN=2 ODD=3
#define STOP_BIT (0<<USBS0) // ONE STOP BIT, TWO STOP BITS =1
#define DATA_BIT (3<<UCSZ00) // EIGHT BITS, FIVE BITS=0, SIX BITS=1, SEVEN BITS =2

void initSerial(void){

  UBRR0H = (uint8_t)(BAUD_PRESCALLER>>8); // HIGH PRESCALLER BITS
  UBRR0L = (uint8_t)(BAUD_PRESCALLER);  // LOW PRESCALER BITS
/* IN OUR CASE RXEND IS BIT 0 AND 1 CAN BE USED 01 AND 10
 * BOTH POSITIONS ARE SER TO 1 IN THE OR STATEMENT 
 */
 // THIS ENABLES RECEIVER AND TRANSMITTER
  UCSR0B = (1<<RXEN0) | (1<<TXEN0);
  
// ALL THE FLAGS DEFINED TO SETTING REGISTER UCSR0C
  UCSR0C = ASYNCHRONOUS | PARITY_MODE | STOP_BIT | DATA_BIT  ;
  
  } 

// THIS ARE THE SECONDS THAT THE PROGRAM UPDATE DATA FROM ARDUINO
// IS IMPORTANT TO SYNC ARDUINO MUST HAVE SAME VALUE IN THIS VARIABLE.
// IN THIS CASE DEFINE TIMEOUT

const int TIME_UPDATE = 1;

int receiveChar(){

unsigned long timetoUpdate = millis();

unsigned long prevtimeUpdate = 0;

unsigned long pastTime; 

int intpastTime;
  
  int attemps_with_delay=5;
  int counter=0;
  while((UCSR0A & (1<<RXC0))==0){
      prevtimeUpdate= millis();
      pastTime= timetoUpdate - prevtimeUpdate;
      if (pastTime <0){
        pastTime=4294967295-prevtimeUpdate + timetoUpdate; // overflow value - previous time + time from cero to present
      }
      intpastTime= int(pastTime*100/1000); // 100 to increase precision
      if(intpastTime > (TIME_UPDATE*100) ){
        return -1;
        break;
        }

      }
    
//  if ( UCSR0A & (1<<4)|(1<<3)|(1<<2) ){   // 4= FRAME ERROR 3= OVERRUN ERROR 2= PARITY ERROR
//return -1;
//}
  return UDR0;  // THIS IS THE BUFFER 3 BYTES REGISTER TO SEND/RECEIVE DATA 
}

int sendChar(uint8_t data){

  unsigned long timetoUpdate = millis();

unsigned long prevtimeUpdate = 0;

unsigned long pastTime; 

int intpastTime;

  while((UCSR0A & (1<<UDRE0))==0){
      prevtimeUpdate= millis();
      pastTime= timetoUpdate - prevtimeUpdate;
      if (pastTime <0){
        pastTime=4294967295-prevtimeUpdate + timetoUpdate; // overflow value - previous time + time from cero to present
      }
      intpastTime= int(pastTime*100/1000); // 100 to increase precision
      if(intpastTime > (TIME_UPDATE*100) ){
        return -1;
        break;
        }

      }
  UDR0= data;  // THIS IS THE BUFFER 3 BYTES REGISTER TO SEND/RECEIVE DATA 
  return 1;
}



int sendString(String literal){
  
  char* StringPtr = literal[0];
  while (*StringPtr != 0x00){ // HERE THE TRANSMISSION FINISHES IN A NULL CHARACTER CAN BE CHANGED
    if (sendChar(*StringPtr)==-1){
      return -1;
    StringPtr++;
    }    
  }
  return 1;

}

void flush(){
unsigned char dummy;
while ( UCSR0A & (1<<RXC0) ) dummy = UDR0;  
}

void closeSerial(){

unsigned char dummy;
while ( UCSR0A & (1<<RXC0) ){ 
  dummy = UDR0;  
}

// THIS DISSABLED RECEIVER AND TRANSMITTER
  UCSR0B &= ~((1<<RXEN0) | (1<<TXEN0));

}


void setup() {
  // put your setup code here, to run once:
initSerial();


pinMode (11,OUTPUT);
pinMode(5,OUTPUT);


}

int prevdata=0;

int data =0;
int count=0;
void loop() {
  // put your main code here, to run repeatedly:


if (count==0){
sendChar(char(49)); // all same simbol 1
sendChar(49);// all same simbol 1
sendChar('1');// all same simbol 1
sendChar(97); // sending 'a' as askII 
sendChar('a');
sendChar(25); // sending &
sendChar(10); // sending \n feed line
sendChar('b');
sendChar('\n');
sendChar('c');

}

data= receiveChar();

if (count==0){

sendChar('d');
sendChar(data); // data is not cero 
sendChar('d');
delay(1000);
}

if(data>0 ){
  prevdata=data;
  sendChar(data);
  data=0;
  count=0;
}

if (prevdata>0){
   digitalWrite(11,HIGH);
}
else{
  digitalWrite(11,HIGH);
  delay(1000);
  digitalWrite(11,LOW);
  delay(1000);
  }
  

if (prevdata=='a' & count==0){
  if(sendChar('a')==-1)
  {
    digitalWrite(5,HIGH);
  
  delay(1000);
  digitalWrite(5,LOW);
  delay(1000);
}
else{
  delay(1000);
  sendChar('t');
  digitalWrite(5,HIGH);
  }
}
if (prevdata=='b' & count==0){
  if(sendChar('b')==-1)
  {
  digitalWrite(5,HIGH);
  delay(1000);
  digitalWrite(5,LOW);
  delay(1000);
}
else{
  
  sendChar('b');
  digitalWrite(5,LOW);
}

}
count=1;
}
