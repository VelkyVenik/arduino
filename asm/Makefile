AVR=-p m328p -c stk500v1 -b 115200 -P /dev/cu.usbmodem1421 -F -U flash:w:

MCU = m328p
TARGET = stk500v1
DEVICE = /dev/cu.usbmodem1421
INCPATH = . 
SRCFILE = console

$(SRCFILE).hex: $(SRCFILE).asm
	avra -I $(INCPATH) $(SRCFILE).asm 

flash:
	avrdude -c $(TARGET) -p $(MCU) -P $(DEVICE) -F -U flash:w:$(SRCFILE).hex:i

clean:
	rm -f $(SRCFILE).hex $(SRCFILE).obj $(SRCFILE).cof $(SRCFILE).eep.hex
