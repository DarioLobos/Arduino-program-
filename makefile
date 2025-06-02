# make file simple can be more complicated

lcd_direct_manipulation.hex: lcd_direct_manipulation.elf
	avr-objcopy -O ihex lcd_direct_manipulation.elf lcd_direct_manipulation.hex

lcd_direct_manipulation.o: main.c C:\WinAVR-20100110\avr\include\avr\io.h C:\WinAVR-20100110\avr\include\avr\interrupt.h C:\WinAVR-20100110\avr\include\util\delay.h
	avr-gcc -g -Os -DF_CPU=16000000UL -mmcu=atmega328p -Wall -Wextra -pedantic -c -o lcd_direct_manipulation.o lcd_direct_manipulation.c

lcd_direct_manipulation.elf: lcd_direct_manipulation.o
	avr-gcc -o lcd_direct_mainpulation.elf lcd_direct_manipulation.o

flash: 
	avrdude -F -v -p atmega328p -b57600 -c arduino -P COM4 -D -U flash:w:lcd_direct_manipulation.hex:i

clean:
	rm -rf *.o *.hex *.elf

# -V dissable verificacion of upload comprared with the file. I don't use it. Make faster upload.

# -F dissable device signature

# -D dissable autoerase fr flash. Atmega328p can erase by page. description don't mention if autoerase erase eeprom.  

# -v verbose output 
	
