#include<p16f877a.inc>

#define VOT1 .65036

	CBLOCK 0x20 
	DEZ 
	UNID 
	TIPO
	ATIVO_TXT
	ATIVO
	CP_PORTB
	CONT1
	CONT2
	W_TEMP
	STATUS_TEMP
	DEFINE_CODIGO
	AUX
	AUXT
	AUX1
	CONTA_BIT
	CHAVE
	

	ENDC 

; ====================================
	
	ORG 0X0000

	GOTO inicio

; ====================================

	ORG 0X0004
	
	MOVWF W_TEMP
	SWAPF STATUS,W
	CLRF STATUS
	MOVWF STATUS_TEMP

	BTFSC INTCON,TMR0IF
	CALL RTI_TMR0

	BTFSC T1CON,TMR1IF
	CALL RTI_TMR1

	SWAPF STATUS_TEMP,W
	MOVWF STATUS
	SWAPF W_TEMP,F
	SWAPF W_TEMP,W
	
	RETFIE

; ====================================

inicio
	;seleciona o banco de mem�ria...
	BSF STATUS,RP0
	BCF STATUS,RP1

	;configura as portas..
	MOVLW 0X00
	MOVWF TRISD
	MOVLW B'00001111'
	MOVWF TRISB
    MOVLW B'11000111'
	MOVWF TRISC


	; Configuracao do timer 0.
	MOVLW 0x83 ;4 MILISEGUNDOS
	MOVWF OPTION_REG

	; Habilita interrup�ao por TIMER 1.
	
	BSF PIE1,TMR1IE	


	;SELECIONA O BANCO  ZERO ( 0 )
	BCF STATUS,RP0

	; Configura�ao do Timer 1.
	MOVLW 0x0C
	MOVWF T1CON

	; Valor inicial do TIMER1
	
	MOVLW HIGH(VOT1)
	MOVWF TMR1H
	MOVLW LOW(VOT1)
	MOVWF TMR1L

	BCF PIR1,	TMR1IF
	BCF INTCON, TMR0IF
	BSF INTCON, TMR0IE
	BSF	INTCON,	PEIE
	BSF INTCON, GIE

;Valores inicias das vari�veis

	MOVLW .1
	MOVWF TIPO
	CLRF ATIVO	
	CLRF DEZ
	CLRF UNID
	CLRF PORTB
	CLRF CONTA_BIT
	BSF PORTB,7
	CLRF CHAVE	
	MOVLW B'01110011'
	MOVWF ATIVO_TXT 
	
Teste_Botao
	
	BTFSS PORTB,0
	CALL  Dezena
	BTFSS PORTB,1
	CALL  Unidade
	BTFSS PORTB,2
	CALL Start_Pause
	BTFSS PORTB,3
	CALL  Codigo
	MOVFW PORTB
	GOTO Teste_Botao
	
Dezena
	
	CALL ATRASO
	BTFSS ATIVO,0 ; SE A DEZENA FOR MODIFICADA DURANTE A TRANSMISSAO, ESSA NAO SERA ALTERADA
	INCF DEZ,F
	MOVFW DEZ
	ANDLW 0x0F
	MOVWF DEZ
	return
	
Unidade
	
	CALL ATRASO
	BTFSS ATIVO,0 ; SE A UNIDADE FOR MODIFICADA DURANTE A TRANSMISSAO, ESSA NAO SERA ALTERADA	
	INCF UNID,F
	MOVFW UNID
	ANDLW 0x0F
	MOVWF UNID
	return
	
Start_Pause
	
	CALL ATRASO
	COMF ATIVO, F
	BTFSC ATIVO,0
	CALL Ligado
	BTFSS ATIVO,0
	CALL Desligado
	return
	
Codigo
	CALL ATRASO
	BTFSS ATIVO,0
	INCF TIPO,F
	MOVLW .4
	SUBWF TIPO,W
	BTFSS STATUS,Z
	RETURN
	MOVLW .1
	MOVWF TIPO
	RETURN
	
Ligado
	CLRF CONTA_BIT
	BSF T1CON,TMR1ON
	MOVLW B'00111000'
	MOVWF ATIVO_TXT
	return
	
Desligado
	BCF T1CON,TMR1ON
	MOVLW B'01110011'
	MOVWF ATIVO_TXT 
	return
		
ATRASO
	
	MOVLW .255
	MOVWF CONT1
ATRASO2
	MOVLW .255
	MOVWF CONT2
	DECFSZ CONT2,F
	GOTO $-1
	DECFSZ CONT1,F
	GOTO ATRASO2 
	return

; ====================================	

RTI_TMR0


	BCF INTCON, TMR0IF 
	

	;Apaga todos os displays
	
	MOVFW PORTB
	MOVWF CP_PORTB
	MOVLW .0 
	MOVWF PORTB
	 
	;Teste display selecionado
	
RB4
	
	BTFSS CP_PORTB,4
	GOTO RB7
	MOVFW DEZ
	CALL DECODIFICA
	MOVWF PORTD
	
RB7
	
	BTFSS CP_PORTB,7
	GOTO RB6
	MOVFW UNID
	CALL DECODIFICA
	MOVWF PORTD
	
RB6
	
	BTFSS CP_PORTB,6
	GOTO RB5
	MOVFW ATIVO_TXT
	MOVWF PORTD
	
RB5
	
	BTFSS CP_PORTB,5
	GOTO  FIM_TIMER0
	MOVFW TIPO
	CALL DECODIFICA
	MOVWF PORTD
	
FIM_TIMER0
	
	BTFSC CP_PORTB,4
	BSF STATUS,C
	BTFSS CP_PORTB,4
	BCF STATUS,C
	RRF CP_PORTB, F ; ROTACIONAR
	MOVFW CP_PORTB
	MOVWF PORTB
	
	RETURN

; ====================================

	
RTI_TMR1

	BCF PIR1,TMR1IF

	MOVLW HIGH(VOT1)
	MOVWF TMR1H
	MOVLW LOW(VOT1)
	MOVWF TMR1L
	
	MOVF CONTA_BIT,F

	BTFSS STATUS,Z ; TESTA SE CONTA_BIT � 0
	GOTO DESLOCA

	SWAPF DEZ,W
	IORWF UNID,W
	MOVWF AUX
	MOVWF AUXT

	
TIPO1

	MOVLW	.1	
	SUBWF	TIPO,W
	BTFSS	STATUS,Z ;QDO DA 0 O FLAG DE ZERO � 1
	GOTO	TIPO2
	MOVFW	AUX
	MOVWF	AUX1

TIPO2
	
	MOVLW	.2
	SUBWF	TIPO,W
	BTFSC	STATUS,Z
	CLRF	AUX1

TIPO3

	MOVLW	.3
	SUBWF	TIPO,W
	BTFSS	STATUS,Z
	GOTO	CARREGA_CONTA_BIT	
	COMF	AUX,W	;INVERTE AUX E W RECEBE O VALOR
	MOVWF	AUX1	
	
CARREGA_CONTA_BIT

	MOVLW .8
	MOVWF CONTA_BIT
	GOTO PULA_DESLOCA

DESLOCA
	
	BTFSS CHAVE,0
	GOTO DESLOCA_PRIMEIRA_METADE

DESLOCA_SEGUNDA_METADE

	MOVFW AUX1
	MOVWF AUXT
	DECF CONTA_BIT,F ; P/ SABER QTOS BITS FORAM TRANSMITIDOS.
	GOTO PULA_DESLOCA

DESLOCA_PRIMEIRA_METADE

	RLF AUX,F
	RLF AUX1,F ; ROTACIONA AUX1, SEGUNDO MEIO TEMPO DE BIT.
	MOVFW AUX
	MOVWF AUXT

PULA_DESLOCA

	BTFSC AUXT,7 ; TESTE SE O BIT ATUAL � 1 OU 0
	GOTO TESTA_SETADO ; PARA CONSEGUIR GERAR TENS�O POSITIVA E NEGATIVA
	BCF PORTC,3
	BSF PORTC,4 ;VALOR INVERSO DO PORTC,3 

TESTA_SETADO

	BTFSS AUXT,7 ; TESTE SE O BIT ATUAL � 1 OU 0
	GOTO COMPLEMENTO_CHAVE
	BSF PORTC,3 ; JOGA O BIT MAIS SIGNIFICATIVO PARA O PORTC
	BCF PORTC,4

COMPLEMENTO_CHAVE

	COMF CHAVE,F ; SE FOR 0 VIRA 1 E VICE-VERSA.

	
	RETURN

; ====================================

DECODIFICA
	ANDLW B'00001111'
	ADDWF PCL,F
	RETLW B'00111111' ;0
	RETLW B'00000110' ;1
	RETLW B'01011011' ;2
	RETLW B'01001111' ;3
	RETLW B'01100110' ;4
	RETLW B'01101101' ;5
	RETLW B'01111101' ;6
	RETLW B'00000111' ;7
	RETLW B'01111111' ;8
	RETLW B'01100111' ;9
	RETLW B'01110111'; A
	RETLW B'01111100' ;B
	RETLW B'00111001' ;C
	RETLW B'01011110' ;D
	RETLW B'01111001' ;E
  	RETLW B'01110001' ;F

; ====================================

	END






	

	


		