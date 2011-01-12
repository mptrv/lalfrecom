/**
 * Controle PIC - Interface de controle de mastro de antena por meio do PIC.
 *
 * Copyright (C) 2010  Marcelo Porto Trevizan
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
 * ser realizada com o GPSim (gpsim.sourceforge.net), tanto do código-fonte
 * em C quanto do correspondente /Assembly/ gerado. Um bom ambiente para
 * desenvolvimento e gravação, inclusive com suporte ao ICD2 via USB, é o
 * PikLab (piklab.sourceforge.net).
 *
 * Comando atual para compilação:
 *
 *		$ make
 *
 * Comando atual para a programação, no modo interativo:
 *
 *		$ piklab-prog -i -p icd2 -d 16f877a -t usb \
 *			--firmware-dir /usr/local/share/ICD2/ --target-self-powered
 *
 * ou pelo /shell script/:
 *
 *		$ prog-controle-pic.sh
 *
 * e, em seguida:
 *
 *		> connect
 *		> program controle-pic.hex
 *
 */

/**
 * Referências:
 *
 * [1] Configuração para gravação com ICD2 no Linux:
 *		<http://acassis.wordpress.com/2007/09/28/usando-o-programador-de-pic-icd2-no-linux>
 *		Acessado em 22-12-2010.
 *
 * [2] Uso do compilador SDCC e do simulador GpSim:
 *		<http://www.micahcarrick.com/pic-c-programming-linux.html>
 *		Acessado em 22-12-2010.
 *
 */

/**
 * Observações:
 *
 * Uso do ICD2:
 *
 *		O usuário deve ter direito de escrita nos dispositivos USB. Para isto,
 *		pode-se acrescentar o usuário ao grupo 'usb', definir '/dev/bus/usb' e
 *		seus arquivos e subdiretórios como pertencentes a 'root:usb' e aplicar o
 *		modo '775' a eles. Depois, fazer 'logout' e 'login', para entrar em vigor
 *		as alterações de grupo do usuário.
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
 *	Sza		: sensor de zero graus absoluto para a rotação (0)
 *	Sgme	: sensor de giro do motor de elevação (1)
 *
 *	ab		: atuador da botoeira (1)
 *	afcs	: atuador do fim-de-curso superior (1)
 *	afci	: atuador do fim-de-curso inferior (1)
 *	ampr1	: atuador do motor-de-passo de rotação - fase 1 (0)
 *	ampr2	: atuador do motor-de-passo de rotação - fase 2 (0)
 *	ampr3	: atuador do motor-de-passo de rotação - fase 3 (0)
 *	ampr4	: atuador do motor-de-passo de rotação - fase 4 (0)
 *
 *	Notas:
 *
 *		.Entre parêntesis, o nível lógico de "ativado".
 *
 *
 * Estados de Comando do Motor de Elevação
 * ------- -- ------- -- ----- -- --------
 *
 *		S0 : subir
 *		S1 : pára subida
 *		S2 : descer
 *		S3 : pára descida
 */

#include <pic/pic16f877a.h>


// DEFINIÇÕES

// Diretivas de compilação.

#define DEMO
#undef DEMO

// Tipos definidos pelo usuário.

typedef unsigned char bool;


// Palavra de configuração.

#ifndef CONFIG
	#define CONFIG	_CP_OFF & _DEBUG_OFF & _WRT_OFF & _CPD_OFF & _LVP_OFF & _BODEN_OFF & _PWRTE_ON & _WDT_OFF & _XT_OSC
#endif
unsigned int __at 0x2007  __CONFIG = CONFIG;


// Sensores.

#define	_Sza	(RA0)
#define _Sgme	(!RA1)

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

// Constantes

#define viContIntTmr1	125


// Protótipos

void Executar(void *Comando, signed char arg);

void Atraso_10ms(unsigned char fator);
void IniciaBaseTempo(unsigned int ms);

void AcionarMpr(void);
void PulsarBotoeira(void);

bool Inicializar(signed char sem_uso);
bool Finalizar(signed char sem_uso);
bool Rotacionar(signed char passos);
bool RotacaoZero(signed char sem_uso);
bool Recolher(signed char sem_uso);
bool Elevar(signed char passos);

void Ola(void);
void Entendido(void);
void Ocupado(void);
void Feito(void);
void ErroComando(void);
void ErroExecucao(void);
void Abortado(void);

void putchar (unsigned char c);
void putstring (unsigned char *s);
unsigned char getchar();

void TrataInterrupcoes(void) __interrupt (0);


// Variáveis Globais

static unsigned char passoMpr[4] = {0x70, 0xB0, 0xD0, 0xE0};
static unsigned char passoAtual = 0;
static unsigned char Sobe = true;
static volatile unsigned char EstouroTempo = false;
static volatile unsigned char ContIntTmr1 = viContIntTmr1;
static volatile unsigned char Abortar = false;
static volatile unsigned char Executando = false;

typedef union {
	struct {
		unsigned char L;
		unsigned char H;
	};
	unsigned int HL;
} __UintHL_t;

static volatile __UintHL_t Vi_Tmr1;


/*******************************
 * INTERRUPÇÕES
 ******************************/

void TrataInterrupcoes(void) __interrupt (0) {

	static signed char com; // Comando.
	static signed char arg; // Argumento.

	/*
	 * Após 125 estouros de TMR1, a variável 'EstouroTempo' será setada e
	 * a contagem de TMR1 será paralizada. Esta poderá ser reabilitada
	 * ao se chamar 'IniciaBaseTempo()'.
	 */
	if (TMR1IF) {
		TMR1IF = 0;
		if (!(--ContIntTmr1)) {
			EstouroTempo = true;
			ContIntTmr1 = viContIntTmr1;
			TMR1ON = 0;
		} else {
			TMR1L = 0;
			TMR1H = Vi_Tmr1.H;
			TMR1L = Vi_Tmr1.L;
		}
	}

	/*
	 * Verifica o código transmitido via RS232 e executa a requisição.
	 */
	if (RCIF) {

		com = getchar();
		arg = getchar();

		switch (com) {
			 case 'O':
				 Ola();
				 break;
			 case 'I':
				 Abortar = false;
				 Executar((char *) &Inicializar, arg);
				 break;
			 case 'Z':
				 Executar((char *) &RotacaoZero, arg);
				 break;
			 case 'L':
				 Executar((char *) &Recolher, arg);
				 break;
			 case 'R':
				 Executar((char *) &Rotacionar, arg);
				 break;
			 case 'E':
				 Executar((char *) &Elevar, arg);
				 break;
			 case 'F':
				 Executar((char *) &Finalizar, arg);
				 break;
			 case 'A':
				 Entendido();
				 Abortar = true;
				 break;
			default:
				 ErroComando();
		}

	 }

}


/*******************************
 * FUNÇÕES
 ******************************/

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

/**
 * Inicia a base de tempo definida pelo TMR1. Após a quantidade de milisegundos
 * fornecida, a variável global 'EstouroTempo' será setada.
 */
void IniciaBaseTempo(unsigned int ms) {	
	GIE = 0;
	TMR1ON = 0;
	Vi_Tmr1.HL = 0xFFFF - ms + 1;
	TMR1H = Vi_Tmr1.H;
	TMR1L = Vi_Tmr1.L;
	ContIntTmr1 = viContIntTmr1;
	EstouroTempo = false;
	TMR1IF = 0;
	GIE = 1;
	TMR1ON = 1;
}

/**
 * Pára a base de tempo definida pelo TMR1.
 */
#define ParaBaseTempo()		TMR1ON=0

/**
 * Gera um atraso específico para a rotação.
 */
#define AtrasoRotacao()		Atraso_10ms(5)

/**
 * Atualiza as saídas ligadas ao Mpr.
 */
void AcionarMpr(void) {
	_mpr = ((_mpr & 0x0F) | (passoMpr[passoAtual]));
}

/**
 * Transmite um caracter via RS232.
 */
void putchar (unsigned char c) {

	while (!TRMT) ;
	TXREG = c;
	//TXEN = 1;

}

/**
 * Obtém um caracter via RS232.
 */
unsigned char getchar() {

	static unsigned char leRCSTA;
	static unsigned char leRCREG;
	static unsigned char erro_rx;

	// Espera a recepção de um próximo dado, caso este ainda não esteja
	// disponível.
	while (!RCIF) ;

	// Obtém informações da situação do receptor.
	leRCSTA = RCSTA;
	erro_rx = leRCSTA | 0x06;

	// Lê o dado, limpando RCIF.
	leRCREG = RCREG;

	// Em caso de erro (/overrun/ ou de /frame/, serão simplesmente limpados
	// os bits de sinalização, ao pulsar CREN.
	if (erro_rx)
	{
		CREN = 0;
		CREN = 1;
	}

	// Retorna o dado lido.
	return (leRCREG);

}

/**
 * Transmite uma /string/ via RS232.
 */
void putstring (unsigned char *s) {

	static unsigned char *l_s;

	l_s = s;

	while (*l_s) {
		putchar(*l_s++);
	}

}

/**
 * Invoca o comando e envia as respostas "Entendido", "Feito" ou
 * "Erro de execução" via RS232.
 */
void Executar(void *Comando, signed char arg) {

	bool (*l_Comando)(signed char);

	l_Comando = Comando;

	Entendido();

	if (Abortar) {
		Abortado();
		return;
	}
	
	if (!Executando) {

		Executando = true;

		if (l_Comando(arg)) {
			Feito();
			Executando = false;
		} else {
			ErroExecucao();
		}

		if (Abortar) Abortado();
			
		Executando = false;

	} else {

		Ocupado();

	}

}

/**
 * Rotaciona o mastro a quantidade de passos especificada. Se for um
 * valor positivo, rotaciona no sentido anti-horário.
 */
bool Rotacionar(signed char passos) {
	
	static signed char sinal;
	static signed char l_passos;

	sinal = passos > 0 ? 1 : -1;
	l_passos = passos;

	while (l_passos & !Abortar) {
		passoAtual = (passoAtual + sinal) & 0x03;
		l_passos -= sinal;
		AtrasoRotacao();
		AcionarMpr();
	}

	return (!Abortar);

}

/**
 * Inicialização padrão do sistema: posicionamento da parte mecânica nas
 * referências.
 */
bool Inicializar(signed char sem_uso) {
	sem_uso = 0;
	return (RotacaoZero(0) && Recolher(0));
}

/**
 * Finalização padrão do sistem: posicionamento da parte mecânica de
 * forma a não fatigar sensores mecânicos.
 */
bool Finalizar(signed char sem_uso) {
	sem_uso = 0;
	return (Recolher(0) && RotacaoZero(0) && Rotacionar(20));
}

/**
 * Rotaciona o mastro até atingir o sensor de zero-absoluto.
 * Nota: atualmente, considera-se que o sensor esteja funcionando
 * adequadamente.
 */
bool RotacaoZero(signed char sem_uso) {
	sem_uso = 0;
	while (_Sza && !Abortar) {
		Rotacionar(1);
	}
	return (!Abortar);
}

/**
 * Executa um pulso na botoeira.
 */
void PulsarBotoeira(void) {
	_ab = 1;
	Atraso_10ms(50);
	_ab = 0;
	Atraso_10ms(50);
}

/**
 * Recolhe o mastro, isto é, fá-lo descer até o 'fci'. Ao término,
 * o estado de comando do 'me' será S3 (descida parada).
 */
bool Recolher(signed char sem_uso) {

	static unsigned char i;
	
	sem_uso = 0;
		
	// Somente aceita a condição de descida.
	_afcs = 1;
	_afci = 0;

	// Põe o mastro em movimento de descida.
	for (i = 0; i < 4; i++ ) {
		PulsarBotoeira();
		IniciaBaseTempo(2000);
		while (_Sgme && (!EstouroTempo) && (!Abortar)) ;
		while ((!_Sgme) && (!EstouroTempo) && (!Abortar)) ;
		if (!EstouroTempo || Abortar) break;
	}

	// Espera o mastro parar de se movimentar para baixo -- isto é,
	// supõe-se que ele atingiu o 'fci'.
	do {
		IniciaBaseTempo(2000);
		while (_Sgme && (!EstouroTempo) && (!Abortar)) ;
		while ((!_Sgme) && (!EstouroTempo) && (!Abortar)) ;
	} while ((!EstouroTempo) && (!Abortar));

	// Desativa atuador de emulação de fim-de-curso superior.
	_afcs = 0;

	// Faz com que o mastro suba um pouco.
	for (i = 0; i < 4; i++ ) {
		PulsarBotoeira();
		IniciaBaseTempo(2000);
		while (_Sgme && (!EstouroTempo) && (!Abortar)) ;
		while ((!_Sgme) && (!EstouroTempo) && (!Abortar)) ;
		if (!EstouroTempo || Abortar) break;
	}

	// Faz o mastro parar e mover-se para baixo.
	PulsarBotoeira();
	PulsarBotoeira();

	// Novamente, espera o mastro parar de se movimentar para
	// baixo -- agora, será garantido que a central de controle
	// do 'me' encontra-se no estado S2.
	do {
		IniciaBaseTempo(2000);
		while (_Sgme && (!EstouroTempo) && (!Abortar)) ;
		while ((!_Sgme) && (!EstouroTempo) && (!Abortar)) ;
	} while ((!EstouroTempo) && (!Abortar));

	// Pára a base de tempo.
	ParaBaseTempo();

	// Deixa os emuladores de fim-de-curso ativados.
	_afcs = 1;
	_afci = 1;

	// Aciona o estado S3.
	PulsarBotoeira();

	// Indica que o mastro desceu e está pronto para subir.
	Sobe = !Abortar;

	return (!Abortar);

}

/**
 * Eleva ou abaixa o mastro de acordo com a quantidade de pulsos desejada ou
 * até atingir o 'fcs'. Se a quantidade de passos for negativa, o mastro será
 * abaixado, limitando-se ao 'fci'. Ao término, o estado da central do 'me'
 * será um dos dois estados de parada e tal que, com mais um pulso na botoeira,
 * ele se movimentaria no mesmo sentido.
 */
bool Elevar(signed char passos) {
	
	static signed char l_passos;
	static signed char sinal;
	static unsigned char i;

	// Verifica se irá subir ou descer e ajusta a quantidade de passos.
	sinal = passos > -1 ? 1 : -1;
	l_passos = passos + sinal;

	// Altera o estado da central de controle para um de movimento.
	//		NOTA: o mastro ainda não se movimentará, pois é esperado
	//		que ambos os sensores de fim-de-curso estejam acionados.
	PulsarBotoeira();

	// Inverte o sentido de movimentação, se necessário.
	if (((sinal > 0) && !Sobe) || (sinal < 0) && Sobe) {
		PulsarBotoeira();
		PulsarBotoeira();
		Sobe = !Sobe;
	}

	// Permite o movimento apenas para o sentido desejado.
	_afcs = !Sobe;
	_afci = Sobe;
	
	// Espera a elevação ou abaixamento do mastro pela quantidade de pulsos
	// especificada.
	EstouroTempo = false;
	while ((l_passos-=sinal) && (!EstouroTempo) && (!Abortar)) {
		IniciaBaseTempo(2000);
		while (_Sgme && (!EstouroTempo) && (!Abortar)) ;
		while (!_Sgme && (!EstouroTempo) && (!Abortar)) ;
	}

	// Pára a base de tempo.
	ParaBaseTempo();

	// Pára a movimentação do mastro.	
	_afcs = 1;
	_afci = 1;

	// Ajusta o estado da central de controle do 'me'.
	PulsarBotoeira();
	PulsarBotoeira();
	PulsarBotoeira();

	return (!Abortar);

}

/**
 * Respostas enviadas pelo microcontrolador.
 */

void Ola(void) {
	putstring("Ola\n\r");
}

void Entendido(void) {
	putstring("Entendido\n\r");
}

void Ocupado(void) {
	putstring("Ocupado\n\r");
}

void Feito(void) {
	putstring("Feito\n\r");
}

void ErroComando(void) {
	putstring("Erro: comando\n\r");
}

void ErroExecucao(void) {
	putstring("Erro: execucao\n\r");
}

void Abortado(void) {
	putstring("Abortado\n\r");
}


/*******************************
 * PROGRAMA PRINCIPAL
 ******************************/

void main(void) {
	
	static unsigned char i;
	static unsigned char j;
	static signed char passos;

	// Configurações de entrada e saída.	
	ADCON1 = 0x07;
	TRISA = 0x03;
	TRISB = 0x00;	
	TRISC = 0x80;	
	TRISD = 0x00;	
	TRISE = 0x00;

	// Configurações da comunicação serial RS232.
	//		assíncrono, 9600 bps, 8 bits, sem paridade, com interrupção rx e tx.
	SPBRG = 25;
	BRGH = 1;
	SYNC = 0;
	SPEN = 1;
	RCIE = 1;
	TXIE = 0;
	RX9 = 0;
	TX9 = 0;
	ADDEN = 0;
	RX9D = 0;
	TX9D = 0;
	TXEN = 1;	
	CREN = 1;
	
	// Configurações do TMR1.
	T1CON = 0x30;
	TMR1IE = 1;

	// Inicializações.
	
	PORTA = 0x00;
	PORTB = 0x00;
	PORTC = 0x00;
	PORTD = 0x00;
	PORTE = 0x00;

	_ab = 0;
	_afcs = 1;
	_afci = 1;
	_ampr1 = 1;
	_ampr2 = 1;
	_ampr3 = 1;
	_ampr4 = 1;

	// Habilitação das interrupções.
	PEIE = 1;
	GIE = 1;	

	// Atraso de inicialização.
	Atraso_10ms(100);

#ifndef DEMO

	Ola();
	//Executar((char *) &Inicializar, 0);
	// Aguarda comandos via RS232.
	while (1) ;

#endif // DEMO
	
	/* --- Código de demonstração. ------------------------ */

#ifdef DEMO

	// Posicionamento do mastro em suas referências.
	Recolher(0);
	RotacaoZero(0);

	// Uma volta completa no sentido anti-horário.
	for (i = 0; i < 8; i++) {
		Atraso_10ms(100);
		Rotacionar(125);
	}

	// Uma volta completa no sentido horário.
	for (i = 0; i < 8; i++) {
		Atraso_10ms(100);
		Rotacionar(-125);
	}

	// Sobe até o 'fcs'.
	Atraso_10ms(100);
	Elevar(21);
		
	// Desce até o 'fci'.
	Atraso_10ms(100);
	Elevar(-21);

	// Sobe até a metade da excursão.
	Atraso_10ms(100);
	Elevar(10);

	// Rotaciona um pouco no sentido horário.
	Rotacionar(-100);

	// Sobe pulso-a-pulso até os 'fcs'.
	//		Este código em execução com a planta apresenta perda de pulsos,
	//		conforme relatado pela revisão r82 do projeto 'lalfrecom'.
	/*
	for (i = 21; i--; ) {
		Atraso_10ms(100);
		Elevar(1);
	}
	*/

	// Laço infinito.

	while (1) {

		passos = -14;

		Atraso_10ms(100);

		Recolher(0);
		RotacaoZero(0);

		Atraso_10ms(100);
		Rotacionar(7*14);

		for (j = 3; j--; ) {
			Elevar(5);
			for (i = 14; i--; ) {
				Atraso_10ms(100);
				Rotacionar(passos);
			}
			passos = -passos;
		}

	}

#endif // DEMO

	/* --- Fim do código de demonstração. ----------------- */

}
