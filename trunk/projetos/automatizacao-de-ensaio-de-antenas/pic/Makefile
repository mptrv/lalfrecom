# Compilação de "Controle-PIC".

CC=sdcc
CFLAGS=-V -mpic14 -p16f877a

includedir=/usr/local/share/sdcc/non-free/include
libdir=/usr/local/share/sdcc/non-free/lib/pic

EXEC=controle-pic.hex
SRC=controle-pic.c

all: $(EXEC)

$(EXEC): $(SRC)
	@echo -e "\nCompilando...\n"
	@$(CC) $(CFLAGS) -I $(includedir) -L $(libdir) $< \
		&& echo -e "\nSUCESSO: compilação concluída!\n" \
		|| echo -e "\n* ERRO: Compilação não concluída! *\n"

demo: all
# Acrescentar para a regra 'demo' a chave '-DDEMO' em 'CFLAGS'.
