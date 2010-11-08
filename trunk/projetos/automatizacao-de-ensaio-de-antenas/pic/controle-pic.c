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
 *	$ sdcc -V -mpic14 -p16f877a -I /usr/local/share/sdcc/non-free/include \
 *		-L /usr/local/share/sdcc/non-free/lib/pic $* \
 *      && echo -e "\nSUCESSO: Compilação concluída!\n" \
 *		|| echo -e "\n* ERRO: Compilação não concluída! *\n"
 *
 */

/**
 * Descrição das Variáveis
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
 */

#include <pic/pic16f877a.h>


// Palavra de configuração.

#ifndef CONFIG
	#define CONFIG	_CP_OFF & _DEBUG_OFF & _WRT_OFF & _CPD_OFF & _LVP_OFF & _BODEN_OFF & _PWRTE_ON & _WDT_OFF & _XT_OSC
#endif
unsigned int __at 0x2007  __CONFIG = CONFIG;


//Definições dos sensores.

#define	_Sza	(RA0)
#define _Sfcs	(RA1)
#define _Sfci	(RA2)
#define _Sgme	(RA3)

//Definições dos atuadores.

#define _ab		(RB0)
#define _afcs	(RB1)
#define _afci	(RB2)
#define _ampr1	(RB4)
#define _ampr2	(RB5)
#define _ampr3	(RB6)
#define _ampr4	(RB7)

#define _mpr	(PORTB)


// Protótipos

void AtrasoRotacao(void);
void AcionarMpr(void);
void Rotacionar(signed char passos);
void RotacaoZero(void);


// Variáveis Globais

static unsigned char passoMpr[4] = {0x80, 0x40, 0x20, 0x10};
static unsigned char passoAtual = 0;


/*******************************
 * FUNÇÕES
 ******************************/

/**
 * Gera um atraso específico para a rotação.
 */
void AtrasoRotacao(void) {
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


/*******************************
 * PROGRAMA PRINCIPAL
 ******************************/

void main(void) {

	//Configurações de entrada e saída.
	
	ADCON1 = 0x07;
	TRISA = 0xFF;
	TRISB = 0x00;

	// Laço principal.
	
	while (1) {

		Rotacionar(10);
		Rotacionar(-10);
		RotacaoZero();

	}

}
