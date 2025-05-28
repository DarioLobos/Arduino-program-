# flags for Arduino nano
AVRFLAGS = -F -D -v -p m328p -c arduino -C /etc/avrdude.conf -P com4 -U flash:w:main.hex 

#-V dissable verification of upload.

default: main.elf 
      avr-objcopy -O ihex main.elf main.hex 

main.c: avr/io.h avr/interrupt.h avr/util/delay.h 

main.o: 
      avr-gcc -g -Os -DF_CPU=16000000UL -mmcu=atmega328p -Wall -Wextra -c -o main.o main.c 

main.elf: main.o 

avr-gcc -o main.elf main.o 

flash: sudo avrdude  ${AVRFLAGS} main.hex 

clean: rm -rf *.o *.hex *.elf
