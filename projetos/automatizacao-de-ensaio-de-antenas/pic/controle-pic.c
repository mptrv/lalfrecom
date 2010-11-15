/**
 * Controle PIC - Interface de controle de mastro de antena por meio do PIC.
 *
 * Copyright (C) 2010  Marcelo Porto Trevizan, Felipe Montagneri
 *                 
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * Controle PIC
 *
 * Este arquivo poderá ser compilado com o SDCC (sdcc.sourceforge.net) e o
 * GPUtils (gputils.sourceforge.net). Se for compilado utilizando outro
 * compilador, algumas alterações poderão ser necessárias. A simulação poderá
 * ser realizada com o GPSim (gpsim.sourceforge.net), tando do código-fonte em C
 * quanto do correspondente \Assembly\ gerado.
 *
 * Comando atual para compilação:
 *
 *		$ make
 *
 */

/**
 * Descrição das Variáveis
 * --------- --- ---------
 *
 *	mpr		: motor-de-passo para rotação
 *	mpi		: motor-de-passo para inclinação (ainda não existente na planta)
 *	mpol	: motor CC para polarização (ainda não existente na planta)
 *	me		: motor CA para elevação
 *
 *	Sza		: sensor de zero graus absoluto para a rotação
 *	Sfcs	: sensor de fim-de-curso superior
 *	Sfci	: sensor de fim-de-curso inferior
 *	Sgme	: sensor de giro do motor de elevação
 *
 *	ab		: atuador da botoeira
 *	afcs	: atuador do fim-de-curso superior
 *	afci	: atuador do fim-de-curso inferior
 *	ampr1	: atuador do motor-de-passo de rotação - fase 1
 *	ampr2	: atuador do motor-de-passo de rotação - fase 2
 *	ampr3	: atuador do motor-de-passo de rotação - fase 3
 *	ampr4	: atuador do motor-de-passo de rotação - fase 4
 *
 *	Notas:
 *
 *		.Os sensores quando ativos fornecem nível lógico 0.
 *		.Os atuadores serão ativados em nível lógico 1.
 *
 *
 * Estados de Comando do Motor de Elevação
 * ------- -- ------- -- ----- -- --------
 *
 *		S0 : subir
 *		S1 : pára subida
 *		S2 : descer
 *		S4 : pára descida
 */


#include <pic/pic16f877a.h>


// DEFINIÇÕES


// Palavra de configuração.

#ifndef CONFIG
	#define CONFIG	_CP_OFF & _DEBUG_OFF & _WRT_OFF & _CPD_OFF & _LVP_OFF & _BODEN_OFF & _PWRTE_ON & _WDT_OFF & _XT_OSC
#endif
unsigned int __at 0x2007  __CONFIG = CONFIG;


// Sensores.

#define	_Sza	(RA0)
#define _Sfcs	(RA1)
#define _Sfci	(RA2)
#define _Sgme	(RA3)

// Atuadores.

#define _ab		(RB0)
#define _afcs	(RB1)
#define _afci	(RB2)
#define _ampr1	(RB4)
#define _ampr2	(RB5)
#define _ampr3	(RB6)
#define _ampr4	(RB7)

#define _mpr	(PORTB)

// Falso e verdadeiro.

#define false	0
#define true	!false


// Protótipos

void AtrasoRotacao(void);
void AcionarMpr(void);
void Rotacionar(signed char passos);
void RotacaoZero(void);
void TrataInterrupcoes(void) __interrupt (0);
void IniciaBaseTempo(unsigned int ms);
void PulsarBotoeira(void);
void Recolher(void);
void Elevar(signed char passos);

// Variáveis Globais

static unsigned char passoMpr[4] = {0x70, 0xB0, 0xD0, 0xE0};
static unsigned char passoAtual = 0;
static unsigned char Sobe = true;
static volatile unsigned char EstouroTempo = false;
static volatile unsigned char ContIntTmr1 = 125;

typedef union {
	struct {
		unsigned char H;
		unsigned char L;
	};
	unsigned int HL;
} __UintHL_t;

static volatile __UintHL_t Vi_Tmr1;

//static volatile unsigned int __sfr __at(TMR1L_ADDR) TMR1;

/*
 * typedef union {
  struct {
    unsigned char CCP1M0:1;
    unsigned char CCP1M1:1;
    unsigned char CCP1M2:1;
    unsigned char CCP1M3:1;
    unsigned char CCP1Y:1;
    unsigned char CCP1X:1;
    unsigned char :1;
    unsigned char :1;
  };
} __CCP1CON_bits_t;
extern volatile __CCP1CON_bits_t __at(CCP1CON_ADDR) CCP1CON_bits;
*/




/*******************************
 * INTERRUPÇÕES
 ******************************/

void TrataInterrupcoes(void) __interrupt (0) {

	/*
	 * Após 125 estouros de TMR1, a variável 'EstouroTempo' será setada e
	 * a contagem de TMR1 será paralizada. Esta poderá ser reabilitada
	 * ao se chamar 'IniciaBaseTempo()'.
	 */
	if (TMR1IF) {
		TMR1IF = 0;
		if (!(--ContIntTmr1)) {
			EstouroTempo = true;
			ContIntTmr1 = 125;
			TMR1ON = 0;
		} else {
			TMR1H = Vi_Tmr1.H;
			TMR1L = Vi_Tmr1.L;
		}
	}

}


/*******************************
 * FUNÇÕES
 ******************************/

/**
 * Gera um atraso múltiplo de 10ms @ fosc = 4 MHz.
 *	TODO: Conferir se o atraso confere.
 */
void Atraso_10ms(unsigned char fator) {

	unsigned char temp00;
	unsigned char temp01;

	while (fator--)
		for (temp01 = 10; temp01; temp01--)
			for (temp00 = 250; temp00; temp00--) ;

}

/**
 * Inicia a base de tempo definida pelo TMR1. Após a quantidade de milisegundos
 * fornecida, a variável global 'EstouroTempo' será setada.
 */
void IniciaBaseTempo(unsigned int ms) {
	Vi_Tmr1.HL = 0xFFFF - ms + 1;
	TMR1H = Vi_Tmr1.H;
	TMR1L = Vi_Tmr1.L;
	EstouroTempo = false;
	TMR1ON = 1;
}

/**
 * Gera um atraso específico para a rotação.
 */
void AtrasoRotacao(void) {
	Atraso_10ms(5);
}

/**
 * Atualiza as saídas ligadas ao Mpr.
 */
void AcionarMpr(void) {
	_mpr = ((_mpr & 0x0F) | (passoMpr[passoAtual]));
}

/**
 * Rotaciona o mastro a quantidade de passos especificada. Se for um
 * valor positivo, rotaciona no sentido anti-horário.
 */
void Rotacionar(signed char passos) {
	
	signed char sinal;

	sinal = passos > 0 ? 1 : -1;

	while (passos) {
		passoAtual = (passoAtual + sinal) & 0x03;
		passos -= sinal;
		AtrasoRotacao();
		AcionarMpr();
	}

}

/**
 * Rotaciona o mastro até atingir o sensor de zero-absoluto.
 * Nota: atualmente, considera-se que o sensor esteja funcionando
 * adequadamente.
 */
void RotacaoZero(void) {
	while (_Sza) {
		Rotacionar(1);
	}
}

/**
 * Executa um pulso na botoeira.
 */
void PulsarBotoeira(void) {
	_ab = 1;
	Atraso_10ms(10);
	_ab = 0;
}

/**
 * Recolhe o mastro, isto é, fá-lo descer até o 'fci'. Ao término,
 * o estado de comando do 'me' será S2 (descida parada).
 */
void Recolher(void) {
	
	if (_Sfci) {
		
		_afcs = 1;
		_afci = 0;

		do {
			PulsarBotoeira();
			IniciaBaseTempo(2000);
			while (_Sgme && (!EstouroTempo)) ;
		} while (_Sgme);

		while (_Sfci) ;

		Sobe = false;

	}

}

/**
 * Eleva ou abaixa o mastro de acordo com a quantidade de pulsos desejada ou
 * até atingir o 'fcs'. Se a quantidade de passos for negativa, o mastro será
 * abaixado.
 */
void Elevar(signed char passos) {

	if (passos > 0) {
		if (_Sfcs) {
			if (!Sobe) {
				PulsarBotoeira();
				PulsarBotoeira();
				Sobe = true;
			}
			_afcs = 0;
			_afci = 0;
			while (passos--) {
				while (_Sgme && _Sfcs) ;
				while (!_Sgme && _Sfcs) ;
			}
		}
	} else {
		if (_Sfci) {
			if (Sobe) {
				PulsarBotoeira();
				PulsarBotoeira();
				Sobe = false;
			}
			_afcs = 0;
			_afci = 0;
			while (passos++) {
				while (_Sgme && _Sfci) ;
				while (!_Sgme && _Sfci) ;
			}
		}
	}

}



/*******************************
 * PROGRAMA PRINCIPAL
 ******************************/

void main(void) {

	// Configurações de entrada e saída.
	
	ADCON1 = 0x07;
	TRISA = 0xFF;
	TRISB = 0x00;

	// Configurações do TMR1.
	T1CON = 0x30;

	// Habilitação das interrupções.
	TMR1IE = 1;
	PEIE = 1;
	GIE = 1;	

	// Laço principal.
	
	while (1) {

		//Rotacionar(150);
		//Rotacionar(150);
		//Rotacionar(-150);
		///Rotacionar(-150);
		//RotacaoZero();
		Recolher();

	}

}
