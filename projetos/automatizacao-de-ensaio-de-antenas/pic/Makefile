controle-pic.asm: controle-pic.c
	sdcc -V -mpic14 -p16f877a \
		-I /usr/local/share/sdcc/non-free/include \
 		-L /usr/local/share/sdcc/non-free/lib/pic $<
