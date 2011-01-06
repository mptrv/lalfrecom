#include <pic/pic16f877a.h>

// Palavra de configuração.

#ifndef CONFIG
	#define CONFIG	_CP_OFF & _DEBUG_OFF & _WRT_OFF & _CPD_OFF & _LVP_OFF & _BODEN_OFF & _PWRTE_ON & _WDT_OFF & _XT_OSC
#endif
unsigned int __at 0x2007  __CONFIG = CONFIG;
 


void TrataInterrupcoes(void) __interrupt (0);



void TrataInterrupcoes(void) __interrupt (0)
{

	static unsigned char leRCSTA;
	static unsigned char leRCREG;
	static unsigned char erro_rx;
	static unsigned char erro_tx;

	if (RCIF)
	 {
		 leRCSTA = RCSTA;
		 erro_rx = leRCSTA | 0x06;
		 leRCREG = RCREG;
		 if (erro_rx)
		 {
			 CREN = 0;
			 CREN = 1;
		 }
		 switch (leRCREG)
		 {
			 case 'a': RB0 = 1; break;
			 case 'b': RB0 = 0; break;
			 case 'c': RB1 = 1; break;
			 case 'd': RB1 = 0; break;
		 }
	 }

	if (TXIF)
	 {

	 }


 
}


void main(void)

{
	
	ADCON1 = 0x07;
	TRISA = 0x00;
	TRISB = 0x00;
	TRISC = 0b10000000;
	TRISD = 0x00;
	
	SPBRG = 25;
	BRGH = 1;
	SYNC = 0;
	SPEN = 1;
	RCIE = 1;
	TXIE = 1;
	RX9 = 0;
	TX9 = 0;
	ADDEN = 0;
	RX9D = 0;
	TX9D = 0;
	TXEN = 1;	
	CREN = 1;
	
	PEIE = 1;
	GIE = 0;

	PORTB = 0xAA;

	PORTD = 0xFF;
	RB7 = 1;

	
	while(1) // laço infinito
    {
    }
}
  
