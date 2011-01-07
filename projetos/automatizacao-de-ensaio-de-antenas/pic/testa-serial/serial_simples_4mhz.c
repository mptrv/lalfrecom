#include <pic/pic16f877a.h>

// Palavra de configuração.

#ifndef CONFIG
	#define CONFIG	_CP_OFF & _DEBUG_OFF & _WRT_OFF & _CPD_OFF & _LVP_OFF & _BODEN_OFF & _PWRTE_ON & _WDT_OFF & _XT_OSC
#endif
unsigned int __at 0x2007  __CONFIG = CONFIG;


void TrataInterrupcoes(void) __interrupt (0);
void Atraso_10ms(unsigned char fator);
void putchar (unsigned char c);
void putstring (unsigned char *s);
unsigned char getchar();


void TrataInterrupcoes(void) __interrupt (0)
{

	if (RCIF)
	 {
		switch (getchar())
		 {
			 case 'a': RB0 = 1; break;
			 case 'b': RB0 = 0; break;
			 case 'c': RB1 = 1; break;
			 case 'd': RB1 = 0; break;
		 }
	 }

	if (TXIF)
	 {
		while (!TRMT) ;
		TXREG = 0;
		TXEN = 0;
	 }
 
}


/**
 * Gera um atraso múltiplo de 10ms @ fosc = 4 MHz.
 */
void Atraso_10ms(unsigned char fator) {

	static unsigned char temp00;
	static unsigned char temp01;
	static unsigned char temp02;

	temp02 = fator;
	
	while (temp02--)
		for (temp01 = 10; temp01--; )
			for (temp00 = 60; temp00--; ) ;

}

void putchar (unsigned char c) {

	TXEN = 1;
	TXREG = c;

}

unsigned char getchar() {

	static unsigned char leRCSTA;
	static unsigned char leRCREG;
	static unsigned char erro_rx;

	leRCSTA = RCSTA;
	erro_rx = leRCSTA | 0x06;
	leRCREG = RCREG;
	if (erro_rx)
	{
		CREN = 0;
		CREN = 1;
	}

	return (leRCREG);

}

void putstring (unsigned char *s) {

	static unsigned char *l_s;

	l_s = s;

	while (*l_s) {
		putchar(*l_s++);
	}

}


void main(void)

{

	static unsigned char caracter;
	
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
	GIE = 1;
	
	PORTA = 0x00;
	PORTB = 0x00;
	PORTC = 0x00;
	PORTD = 0x00;
	PORTE = 0x00;
		
	Atraso_10ms(100);

	PORTB = 0xAA;
	PORTD = 0xFF;
	
	caracter = 'a';
	while(1) // laço infinito
    {
		putstring(">>O caracter e': - ");
		putchar(caracter++);
		putstring("||");
		putstring(" --- ");
		putchar(10);
		putchar(13);
		Atraso_10ms(100);
    }

}
  
