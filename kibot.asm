;
;		Exploración del tatami a baja velocidad, detección de borde, detección de enemigo próximo.
;
;		Linea 0 - puerto B detector de linea infrarojo Delantero Derecho.
;		Linea 1 - puerto B detector de linea infrarojo Trasero.
;		Linea 2 - puerto B detector de enemigo infrarojo y bumper.
;		Linea 3 - puerto B detector de linea infrarojo Delantero Derecho.
;		Linea 4 = Puerto B bobina motor PaP
;		Linea 5 - puerto B bobina motor PaP
;		Linea 6 - puerto B bobina motor PaP
;		Linea 7 - puerto B bobina motor PaP
;
;		Linea 1 - puerto A motores DC
;		Linea 1 - puerto A motores DC
;		Linea 2 = Puerto A motores DC
;		Linea 3 - puerto A motores DC
;		Linea 4 = Puerto A detector fin de carrera en subida de las paletas	
;
;		Diseño Automatico de Sistemas Digitales.
;
;		José María Rodríguez Sáez y Miguel Angel Muñoz Ortega
;

		list 	 p = 16f84
		RADIX	HEX	  

		include	"P16F84.INC"

#define ANCHO .05		; Anchura total: Ancho activado + Ancho desactivado. 10ms * ANCHO
#define ADELANTE b'00001101'	; Patron para que los motores DC muevan el robot hacia adelante
#define GIRA_DCHA b'00001111'	; Patron para que los motores DC giren el robot a la derecha
#define ATRAS b'00001110'	; Patron para que los motores DC muevan el robot hacia atras
#define GIRA_IZQ b'000011001'	; Patron para que los motores DC giren el robot a la izquierda
#define ALTURA .100		; Número de pasos que dará el motor PaP para bajar las palas.
#define ALTURA_2 .120		; Número de pasos que dará el motor PaP para subir las palas.
#define LONGITUD .3		; Numero de pasos que dará el robot intentando sacar al oponente del tatami

Delay_Cont	EQU	0x4F	; Numero de veces que se repetirá el bucle de 10ms.

; Variables para el desplazamiento, motores DC

velocidad	EQU	0X4E	; Ancho que los motores estarán activados, ANCHO - Ancho desactivado = velocidad.
pasos		EQU 0x4D	; Numero de veces que se enviara el pulso de ANCHO a los motores.
direccion	EQU 0x4C	; Patron de bits para el sentido en el que giran los motores.

; Variables para las paletas, motor PaP
Num_Pasos	EQU	0x4B
paso_PaP	EQU 0x4A	; Paso en el que encuentra el motor, desplazamiento en la tabla Pasos


		ORG	0x00	      ;El programa comienza en la dirección 0.	
				goto	Inicio

		ORG	0x05             ;Se salta el vector interrupción.


;*********************************************************************************
;Tabla con los patrones para accionamiento del motor PaP

Pasos	addwf	PCL,F	;Desplazamiento sobre la tabla
		retlw	0x90	; 10010000
		retlw	0xA0	; 10100000
		retlw	0x60	; 01100000
		retlw	0x50	; 01010000
;*********************************************************************************

;*********************************************************************************
;Delay_var: Esta rutina de propósito general realiza una temporización variable
;entre 10 mS y 2550ms. Se emplea un preescaler de 256 y al TMR0 se le carga con 39.
;La velocidad de trabajo es de 4Mhz y por tanto el TMR0 se incrementa cada uS. De 
;esta forma, el TMR0 debe contar 39 eventos que, con un preescaler de 256 hace una
;intervalo total de 10000 uS/10 mS (39 * 256). El valor 39 hay que expresarlo
;en Hex. (27) y como el TMR0 es ascendente habrá que cargar su complemento (D8 hex.)
;Dicho intervalo de 10 mS se repite tantes veces como indique la variable "Delay_cont",
;es por ello que el delay mínimo es de 10 mS ("Delay_cont=1) y el máxima de 2550ms 
;(Delay_cont=255).

Delay_var:
		bcf	INTCON,T0IF			;Desconecta el flag de rebosamiento
		movlw	0xD8			;Complemento hex. de 39
		movwf	TMR0			;carga el TMR0

		clrwdt					; Refrescar el WDT
		movf	Delay_Cont,F
		btfsc	STATUS, Z		; Fin de la temporizacion?
		return					; Si, vuelve
Timer		
		btfss	INTCON,T0IF		; No, espera Rebasamiento del TMR0 ??
		goto	Timer 
		decfsz	Delay_Cont,F	;Decrementa contador de intervalos
		goto	Delay_var		;Repite el intervalo
		return	

;*************************************************************************************
; Subir: Hace que el motor PaP suba las paletas.
; El detector de fin de carrera en la linea 3 del puerto B determina si las paletas han llegado al final del recorrido.

Subir:
		movlw	ALTURA_2
		movwf	Num_Pasos		

		clrf	paso_PaP			; Puesta a 0

Subir_loop
		clrwdt
		btfss	PORTA, 4		; Detectado tope?
		return					; Si, retorna

		movf	Num_Pasos, F
		btfsc	STATUS,Z		; Es 0
		return
		decf	Num_Pasos, F
		btfsc	STATUS,Z		; Es 0
		return		; Si, acabar


		movlw	.1				; No, temporiza
		movwf	Delay_Cont
		call 	Delay_var

		incf	paso_PaP,F		
		movlw	.4
		subwf	paso_PaP, W
		btfss	STATUS,Z		; Es 4?
		goto	Subir_2			; No, da el Paso
		clrf	paso_PaP		; Si, puesta a 0
		goto	Subir_2			; da el Paso
		
Subir_2						; Porner en el puerto el patron
		movf	paso_PaP, W
		call 	Pasos
		movwf 	PORTB
		goto	Subir_loop

		return
;*************************************************************************************

;*************************************************************************************
; Bajar: Hace que el motor PaP baje las paletas.
; Se bajarán las paletas y se debloqueará el motor dejandolo sin tension en las bobinas.

Bajar:
		movlw	ALTURA
		movwf	Num_Pasos		

Bajar_loop
		movf	Num_Pasos, F
		btfsc	STATUS,Z		; Es 0
		goto 	Fin_Bajar
		decf	Num_Pasos, F
		btfsc	STATUS,Z		; Es 0
		goto	Fin_Bajar		; Si, acabar

;		btfss	PORTB, 3		; Detectado tope?
;		goto	Fin_Bajar		; Si, acabar

		movlw	.04				; No
		movf	paso_PaP, F		; Observar paso
		btfsc	STATUS,Z		; Es 0
		movwf	paso_PaP		; Si, poner a 4
		decf	paso_PaP, F		; No, paso anterior

		movlw	.02				; Temporiza
		movwf	Delay_Cont
		call 	Delay_var

Bajar_2							; Porner en el puerto el patron
		movf	paso_PaP, W
		call 	Pasos
		movwf 	PORTB
		goto	Bajar_loop

Fin_Bajar
		clrf	PORTB			; Liberar el motor
		return
;*************************************************************************************

;*************************************************************************************
; Mover_Robot: Una vez configurado el movimiento en las variables, velocidad, direccion y pasos
; esta rutina pone el robot en movimiento.

Mover_Robot:
		movf	velocidad, W	
		movwf	Delay_Cont		
		movf	direccion, W		; Cargar el ancho activo y la dirección
		movwf	PORTA			; Comenzar movimiento
		call	Delay_var		; Retardo
		movf	velocidad,w
		sublw	ANCHO					
		movwf	Delay_Cont		; Cargar ancho inactivo, ANCHO-activo
		clrf	PORTA			; Parar motores
		call	Delay_var		; Retardo

		decf 	pasos			
		btfss	STATUS, Z		; Repetir segun numero de pasos
		goto	Mover_Robot

		return
;*************************************************************************************

;*************************************************************************************
; Media_Vuelta: Hace que el robot retroceda y gire sobre si mismo.
; Empleada cuando se detecta el borde del tatami de frente

Media_Vuelta:
		movlw 	ATRAS			; Retroceso
		movwf	direccion
		movlw	.03
		movwf	velocidad
		movlw	.15
		movwf	pasos
		call	Mover_Robot

		movlw 	GIRA_DCHA		; Girar
		movwf	direccion
		movlw	.30
		movwf	pasos
		call	Mover_Robot

		return
;*************************************************************************************

;*************************************************************************************
; Reentrada: Hace que el robot avance rapido para volver a entrar en el tatami.
; Ademas sube y baja las paletas como defensa

Reentrada:
		movlw 	ADELANTE
		movwf	direccion
		movlw	.05
		movwf	velocidad
		movlw	.15
		movwf	pasos
		call	Mover_Robot

		return
;*************************************************************************************

;*************************************************************************************
; Ataque: Hace que el robot avance rapido para volver a entrar en el tatami.
; Ademas sube y baja las paletas como defensa

Ataque:
		movlw 	ADELANTE
		movwf	direccion
		movlw	.05
		movwf	velocidad
		movlw	.05
		movwf	pasos
		call	Mover_Robot		; Embestir al oponente

		call	Subir			; Intentar levantarlo

Ataque_sacar
		movlw	.01
		movwf	pasos
		call	Mover_Robot

		btfsc	PORTB, 0		; Avanzar hasta que encuentre el borde
		goto	Fin_Atacar

		btfsc	PORTB, 1		; Si lo estan sacando intentar entrar
		goto	Fin_Atacar
		
		btfsc	PORTB, 3		; Avanzar hasta que encuentre el borde
		goto	Fin_Atacar
				
		goto	Ataque_sacar

Fin_Atacar
		call 	Bajar
		;call 	Media_Vuelta

		return
;*************************************************************************************

Inicio	
		bcf		STATUS, RP0		; Selecciona banco 0
		clrf	PORTA
		clrf	PORTB

		bsf		STATUS, RP0		; Selecciona banco 1
		movlw	0x10
		movwf	TRISA			; Puerto A,  lineas [0:3] salidas, [4] entrada
		movlw	0x0F
		movwf	TRISB			; Puerto B, lineas [0:3] entradas, [4:7] salidas
		movlw	0x07
		movwf	OPTION_REG		; Preescaler de 256 para el TMR0

		bcf		STATUS, RP0		; Selecciona banco 0

Ajuste_Mov
		movlw 	ADELANTE		; Avance
		movwf	direccion
		movlw	.01				; Velocidad que mantendria el robot si no se detecta nada
		movwf	velocidad
		movlw	.01				; "Pasos" que avanzará el robot
		movwf	pasos

Mira_Del_Drcha
		btfss	PORTB, 0		; Detectado algo delante a la derecha?
		goto	Mira_Delante_Izq		; No, Comprueba que no se sale por detras
		call 	Media_Vuelta
		goto	Ajuste_Mov

Mira_Delante_Izq
		btfss	PORTB, 3		; Detectado algo delante a la izquierda?
		goto	Mira_Detras		; No, Comprueba que no se sale por detras
		call 	Media_Vuelta
		goto	Ajuste_Mov


Mira_Detras
		btfss	PORTB, 1		; Detectado algo detras?
		goto	Busca_Enemigo		; No, Comprueba si se ha detectado enemigo con el infrarojo
		call 	Reentrada
		goto	Ajuste_Mov

Busca_Enemigo
		btfss	PORTB, 2		; Detectado Enemigo por infrarojos o bumper?
		goto	Exploracion		
		call	Ataque
		goto	Ajuste_Mov

Exploracion
		call	Mover_Robot
		goto	Ajuste_Mov


Fin
		END						; Fin del programa.  

;**********************************************************************************************





		
