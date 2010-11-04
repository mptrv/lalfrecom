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
 */

#include <pic/pic16f877a.h>

// Palavra de configuração.

#ifndef CONFIG
	#define CONFIG	_CP_OFF & _DEBUG_OFF & _WRT_OFF & _CPD_OFF & _LVP_OFF & _BODEN_OFF & _PWRTE_ON & _WDT_OFF & _XT_OSC
#endif
unsigned int __at 0x2007  __CONFIG = CONFIG;

// Protótipos




/*******************************
 * FUNÇÕES
 ******************************/


/*******************************
 * PROGRAMA PRINCIPAL
 ******************************/

void main(void) {

}
