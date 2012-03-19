PROG=chip8

$(PROG): $(PROG).o
	gcc $(PROG).o -o $(PROG)
$(PROG).o: $(PROG).asm
	nasm -f elf $(PROG).asm

clean:
	-rm -f *.o $(PROG)
