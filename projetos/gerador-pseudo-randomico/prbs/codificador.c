#include <P18F452.h>
#include <timers.h>

#define BOT_PADRAO	PORTBbits.RB0
#define BOT_TAXA	PORTBbits.RB1
#define DATA_OUT	PORTCbits.RC5
#define DATA_CLK    PORTCbits.RC2

#define N_PADROES 	2
#define N_TAXAS 	2


unsigned char PRBS15(unsigned int *pseed);
unsigned char PRBS3(unsigned int *pseed);

void low_isr(void);
void high_isr(void);

#pragma code low_vector=0x18
void interrupt_at_low_vector(void)
{
_asm GOTO low_isr _endasm
}

#pragma code /* return to the default code section */

#pragma code high_vector=0x08
void interrupt_at_high_vector(void)
{
_asm GOTO high_isr _endasm
}

#pragma code /* return to the default code section */

#pragma interrupt low_isr
void low_isr (void)
{
/* ... */
}

// criação das variaveis 
unsigned char saida =0;
unsigned int seed=0x04; // inicializa seed com um bit setado
unsigned char taxa=0x00; // escolha da taxa de transmissão
unsigned char padrao=0x00; // variavel de escolha de padrão
unsigned int V0T1[2]= {64536 , 63536}; // define o valor V0 da equaçao do timer 1

void main (void) {
	
	// configurando o hardware
	// definindo entradas
	TRISB =	0x03; // 0 define uma saida e 1 entrada
				 // RB0-RB1 são entradas - demais sao saidas (RB2-RB7)
	TRISC = 0x00; // definindo porta C como saída - vou usar o RC5
	
	// configura timer1
	OpenTimer1(TIMER_INT_ON & T1_16BIT_RW & T1_SOURCE_INT & T1_OSC1EN_ON & T1_PS_1_1 & T1_SYNC_EXT_OFF );
	WriteTimer1( V0T1[taxa] ); 
	// limpa flag de estouro do Timer1 para evitar interupção acidental
	PIR1bits.TMR1IF =0; 
	// habilita as interrupções gerais
	INTCONbits.PEIE=1;
	INTCONbits.GIE=1;
	
	while (1) {
		if (!BOT_PADRAO ) {
			padrao++;
			if (padrao > (N_PADROES-1) )
			padrao = 0;		
		}			
		if(!BOT_TAXA ) {
			taxa++;
			if (taxa > (N_TAXAS-1) )
			taxa = 0;	
		}			
	
	} // while						
	
}	// main

#pragma interruptlow high_isr
void high_isr (void)
{
	WriteTimer1( V0T1[taxa] ); // recarrega  o timer
	PIR1bits.TMR1IF = 0; // desliga o flag

	if (!(DATA_CLK = !DATA_CLK) ) {
		DATA_OUT = (saida && 0x01); // assumindo que saída será somente 0x00 ou 0x01

		switch (padrao) {
			case 0:
				saida = PRBS15(&seed);
				break;
			case 1:
				saida = PRBS3(&seed);
				break;
		}		
	}
	
	
	
}

unsigned char PRBS15(unsigned int *pseed) {
	static unsigned int temp;
	temp = seed;
	temp = ( (temp>>1)^(temp>>2) ) & 0x01;
	seed = (seed >> 1) | (temp << 15);
	return (unsigned char) temp;
}	

unsigned char PRBS3(unsigned int *pseed) {
	unsigned char temp;
	temp = (unsigned  char)seed;
	temp = ( (temp>>1)^(temp>>2) ) & 0x01;
	temp >> 1;
	seed= seed >> 1;
	return temp;
}	


