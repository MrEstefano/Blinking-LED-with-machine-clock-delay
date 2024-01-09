;******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
;* File Name          : Bliking LED with machine cycle delay.s
;* Author             : Stefan zakutansky
;* Version            : V1.0
;* Date               : 15 - November - 2023
;* Description        : STM32F100RB Blinking LED using machine clock as delay, 
;*                      utilizing conditional branch to subroutine concept in low level assembly code. 
;*                      When user button not pressed LED blinks every second, 
;*                      when user button is pressed LED blinks every half second.
;*******************************************************************************

; Amount of memory (in bytes) allocated for Stack
; Tailor this value to your application needs
; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size      EQU     0x00000400

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x00000200

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit

                PRESERVE8
                THUMB
					
GPIOC_BASE 		EQU 	0x40011000		; GPIO Port C base register addres
GPIOA_BASE 		EQU 	0x40010800		; GPIO Port A base register addres
GPIOC_ODR 		EQU 	0x0C			; Port output data register Offset
GPIOA_IDR		EQU 	0x08			; Port read input data register Offset
RCC_Base		EQU		0x40021000	   	; RCC base registar addres
RCC_APB2ENR		EQU		0x018          	; RCC_APB2ENR offset value 
IOPCEN			EQU		0x1<<4         	; Shift 1 four places to align it with bit 4
IOPAEN 			EQU		0x1<<2         	; Shift 1 four places to align it with bit 2
GPIOC_CRH		EQU		0x04 	       	; CRH register Address offset
GPIOA_CRL		EQU		0x00 	       	; CRL register Address offset	
CNF0_1			EQU 	0x1<<1			; position of a bit in CRL register
CNF0_0			EQU 	0x1<<0			; position of a bit in CRL register			
MODE8_1			EQU 	0x1<<1			; position of a bit in CRH register
MODE8_0			EQU 	0x1<<0			; position of a bit in CRH register
CNF8_1			EQU 	0x1<<3			; position of a bit in CRH register
CNF8_0			EQU 	0x1<<2			; position of a bit in CRH register	
HALF_SEC_DELAY  EQU     0x14584D		; defining the delay interval with calculated value for half second delay
ONE_SEC_DELAY   EQU		0x28B0A2		; defining the delay interval with calculated value for one second delay


; Vector Table Mapped to Address 0 at Reset
                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

__Vectors       DCD     __initial_sp                    ; Top of Stack
                DCD     Reset_Handler                   ; Reset Handler
                DCD     NMI_Handler                     ; NMI Handler
                DCD     HardFault_Handler               ; Hard Fault Handler
                DCD     MemManage_Handler               ; MPU Fault Handler
                DCD     BusFault_Handler                ; Bus Fault Handler
                DCD     UsageFault_Handler              ; Usage Fault Handler
                DCD     0                               ; Reserved
                DCD     0                               ; Reserved
                DCD     0                               ; Reserved
                DCD     0                               ; Reserved
                DCD     SVC_Handler                     ; SVCall Handler
                DCD     DebugMon_Handler                ; Debug Monitor Handler
                DCD     0                               ; Reserved
                DCD     PendSV_Handler                  ; PendSV Handler
                DCD     SysTick_Handler                 ; SysTick Handler

                ; External Interrupts
                DCD     WWDG_IRQHandler                 ; Window Watchdog
                DCD     PVD_IRQHandler                  ; PVD through EXTI Line detect
                DCD     TAMPER_IRQHandler               ; Tamper
                DCD     RTC_IRQHandler                  ; RTC
                DCD     FLASH_IRQHandler                ; Flash
                DCD     RCC_IRQHandler                  ; RCC
                DCD     EXTI0_IRQHandler                ; EXTI Line 0
                DCD     EXTI1_IRQHandler                ; EXTI Line 1
                DCD     EXTI2_IRQHandler                ; EXTI Line 2
                DCD     EXTI3_IRQHandler                ; EXTI Line 3
                DCD     EXTI4_IRQHandler                ; EXTI Line 4
                DCD     DMA1_Channel1_IRQHandler        ; DMA1 Channel 1
                DCD     DMA1_Channel2_IRQHandler        ; DMA1 Channel 2
                DCD     DMA1_Channel3_IRQHandler        ; DMA1 Channel 3
                DCD     DMA1_Channel4_IRQHandler        ; DMA1 Channel 4
                DCD     DMA1_Channel5_IRQHandler        ; DMA1 Channel 5
                DCD     DMA1_Channel6_IRQHandler        ; DMA1 Channel 6
                DCD     DMA1_Channel7_IRQHandler        ; DMA1 Channel 7
                DCD     ADC1_IRQHandler                 ; ADC1
                DCD     0                               ; Reserved
                DCD     0                               ; Reserved
                DCD     0                               ; Reserved
                DCD     0                               ; Reserved
                DCD     EXTI9_5_IRQHandler              ; EXTI Line 9..5
                DCD     TIM1_BRK_TIM15_IRQHandler       ; TIM1 Break and TIM15
                DCD     TIM1_UP_TIM16_IRQHandler        ; TIM1 Update and TIM16
                DCD     TIM1_TRG_COM_TIM17_IRQHandler   ; TIM1 Trigger and Commutation and TIM17
                DCD     TIM1_CC_IRQHandler              ; TIM1 Capture Compare
                DCD     TIM2_IRQHandler                 ; TIM2
                DCD     TIM3_IRQHandler                 ; TIM3
                DCD     TIM4_IRQHandler                 ; TIM4
                DCD     I2C1_EV_IRQHandler              ; I2C1 Event
                DCD     I2C1_ER_IRQHandler              ; I2C1 Error
                DCD     I2C2_EV_IRQHandler              ; I2C2 Event
                DCD     I2C2_ER_IRQHandler              ; I2C2 Error
                DCD     SPI1_IRQHandler                 ; SPI1
                DCD     SPI2_IRQHandler                 ; SPI2
                DCD     USART1_IRQHandler               ; USART1
                DCD     USART2_IRQHandler               ; USART2
                DCD     USART3_IRQHandler               ; USART3
                DCD     EXTI15_10_IRQHandler            ; EXTI Line 15..10
                DCD     RTCAlarm_IRQHandler             ; RTC Alarm through EXTI Line
                DCD     CEC_IRQHandler                  ; HDMI-CEC
                DCD     0                               ; Reserved
                DCD     0                               ; Reserved
                DCD     0                               ; Reserved
                DCD     0                               ; Reserved 
                DCD     0                               ; Reserved
                DCD     0                               ; Reserved
                DCD     0                               ; Reserved
                DCD     0                               ; Reserved 
                DCD     0                               ; Reserved
                DCD     0                               ; Reserved
                DCD     0                               ; Reserved
                DCD     TIM6_DAC_IRQHandler             ; TIM6 and DAC underrun
                DCD     TIM7_IRQHandler                 ; TIM7
__Vectors_End

__Vectors_Size  EQU  __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY

; Reset handler
Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
				
				LDR r0, 	 =RCC_Base			; Load RCC base address
				LDR r1, [r0, #RCC_APB2ENR] 		; adding offset to base address
				ORR r1, r1,  #IOPCEN          	; Set bit 4 to enable Port C clock
				ORR r1, r1,  #IOPAEN          	; Set bit 2 to enable Port A clock
				STR r1, [r0, #RCC_APB2ENR] 		; Store new value to memory 	
				
								
				LDR r0,      =GPIOC_BASE		; Load GPIO Port C base address
				LDR r1, [r0, #GPIOC_CRH] 		; adding offset to base address
						
				ORR r1, r1,  #MODE8_0			; PC9 output mode max speed 10MHz by setting bit 0
				BIC r1, r1,  #MODE8_1			; PC9 output mode max speed 10MHz by clearing bit 1					
				
				BIC r1, r1,  #CNF8_0			; PC8 general purpose output push pull by clearing bit 0
				BIC r1, r1,  #CNF8_1			; PC8 general purpose output push pull by clearing bit 1
				STR r1, [r0, #GPIOC_CRH]  		; Store new value to memory 
				
				LDR r2,      =GPIOA_BASE		; Load GPIO Port A base address
				LDR r3, [r2, #GPIOA_CRL] 		; adding offset to base address of GPIOA	
				BIC r3, r3,  #CNF0_0			; PA0 general purpose Input with pull-up / pull-down by clearing bit 0
				ORR r3, r3,  #CNF0_1			; PA0 general purpose Input with pull-up / pull-down by setting	bit 1		
				STR r3, [r2, #GPIOA_CRL]  		; Store new value to memory 	
				
				
				MOV r1,#0x000 					;move value of zero to register one to initialize the statre of LED's off
LOOP
				LDR r5,[r2,#GPIOA_IDR]			;Load what's at address GPIO_IDR to register five 
				AND r5,r5, #1					;mask bit 0 and ignore all other bits coming into register register five
				STR r5,[r2,#GPIOA_IDR]			;Store the value what’s at address GPIO_IDR and put it to register five
				TST r5,#0x01					;check if push button is pressed / if bit 0 was set high
				BNE HALF_SEC_SUBROUTINE			;branch if register five is not equals to zero / push button not pressed so,
				                                ;it is equal to zero, than carry into the next line of code				
				BL  ONE_SEC_SUBROUTINE			;unconditional branch		
				
				B LOOP							;return back to start of the loop
		
		
HALF_SEC_SUBROUTINE
				LDR r4,= HALF_SEC_DELAY 		;load desired value to registrer four 
DELAY_1
				SUBS r4,r4,#0x001 				;subtract one from whats in register four and store it in same register / Suffix S affects the flag
				BNE DELAY_1 					;branch if not equal zero
				MVN r1,r1						;Move value from register one and complement it, then store it in same registetr
				STR r1, [r0, #GPIOC_ODR]		;store value of register one at ODR address 		
				
				BX	LR							;branch indirect to address storred at link registetr

ONE_SEC_SUBROUTINE
				LDR r4,= ONE_SEC_DELAY 			;load desired value to registrer four 
DELAY_2
				SUBS r4,r4,#0x001 				;subtract one from whats in register four and store it in same register / Suffix S affects the flag
				BNE DELAY_2 					;branch if not equal zero
				MVN r1,r1						;Move value from register one and complement it, then store it in same registetr
				STR r1, [r0, #GPIOC_ODR]		;store value if Register one at ODR address				
				BX	LR							;branch indirect to address storred at link registetr
				


;     IMPORT  __main
;     IMPORT  SystemInit
;                LDR     R0, =SystemInit
;                BLX     R0
;                LDR     R0, =__main
;                BX      R0
                 ENDP

; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler                      [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler                [WEAK]
                B       .
                ENDP
MemManage_Handler\
                PROC
                EXPORT  MemManage_Handler                [WEAK]
                B       .
                ENDP
BusFault_Handler\
                PROC
                EXPORT  BusFault_Handler                 [WEAK]
                B       .
                ENDP
UsageFault_Handler\
                PROC
                EXPORT  UsageFault_Handler               [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler                      [WEAK]
                B       .
                ENDP
DebugMon_Handler\
                PROC
                EXPORT  DebugMon_Handler                 [WEAK]
                B       .
                ENDP
PendSV_Handler  PROC
                EXPORT  PendSV_Handler                   [WEAK]
                B       .
                ENDP
SysTick_Handler PROC
                EXPORT  SysTick_Handler                  [WEAK]
                B       .
                ENDP

Default_Handler PROC

                EXPORT  WWDG_IRQHandler                  [WEAK]
                EXPORT  PVD_IRQHandler                   [WEAK]
                EXPORT  TAMPER_IRQHandler                [WEAK]
                EXPORT  RTC_IRQHandler                   [WEAK]
                EXPORT  FLASH_IRQHandler                 [WEAK]
                EXPORT  RCC_IRQHandler                   [WEAK]
                EXPORT  EXTI0_IRQHandler                 [WEAK]
                EXPORT  EXTI1_IRQHandler                 [WEAK]
                EXPORT  EXTI2_IRQHandler                 [WEAK]
                EXPORT  EXTI3_IRQHandler                 [WEAK]
                EXPORT  EXTI4_IRQHandler                 [WEAK]
                EXPORT  DMA1_Channel1_IRQHandler         [WEAK]
                EXPORT  DMA1_Channel2_IRQHandler         [WEAK]
                EXPORT  DMA1_Channel3_IRQHandler         [WEAK]
                EXPORT  DMA1_Channel4_IRQHandler         [WEAK]
                EXPORT  DMA1_Channel5_IRQHandler         [WEAK]
                EXPORT  DMA1_Channel6_IRQHandler         [WEAK]
                EXPORT  DMA1_Channel7_IRQHandler         [WEAK]
                EXPORT  ADC1_IRQHandler                  [WEAK]
                EXPORT  EXTI9_5_IRQHandler               [WEAK]
                EXPORT  TIM1_BRK_TIM15_IRQHandler        [WEAK]
                EXPORT  TIM1_UP_TIM16_IRQHandler         [WEAK]
                EXPORT  TIM1_TRG_COM_TIM17_IRQHandler    [WEAK]
                EXPORT  TIM1_CC_IRQHandler               [WEAK]
                EXPORT  TIM2_IRQHandler                  [WEAK]
                EXPORT  TIM3_IRQHandler                  [WEAK]
                EXPORT  TIM4_IRQHandler                  [WEAK]
                EXPORT  I2C1_EV_IRQHandler               [WEAK]
                EXPORT  I2C1_ER_IRQHandler               [WEAK]
                EXPORT  I2C2_EV_IRQHandler               [WEAK]
                EXPORT  I2C2_ER_IRQHandler               [WEAK]
                EXPORT  SPI1_IRQHandler                  [WEAK]
                EXPORT  SPI2_IRQHandler                  [WEAK]
                EXPORT  USART1_IRQHandler                [WEAK]
                EXPORT  USART2_IRQHandler                [WEAK]
                EXPORT  USART3_IRQHandler                [WEAK]
                EXPORT  EXTI15_10_IRQHandler             [WEAK]
                EXPORT  RTCAlarm_IRQHandler              [WEAK]
                EXPORT  CEC_IRQHandler                   [WEAK]
                EXPORT  TIM6_DAC_IRQHandler              [WEAK]
                EXPORT  TIM7_IRQHandler                  [WEAK]

WWDG_IRQHandler
PVD_IRQHandler
TAMPER_IRQHandler
RTC_IRQHandler
FLASH_IRQHandler
RCC_IRQHandler
EXTI0_IRQHandler
EXTI1_IRQHandler
EXTI2_IRQHandler
EXTI3_IRQHandler
EXTI4_IRQHandler
DMA1_Channel1_IRQHandler
DMA1_Channel2_IRQHandler
DMA1_Channel3_IRQHandler
DMA1_Channel4_IRQHandler
DMA1_Channel5_IRQHandler
DMA1_Channel6_IRQHandler
DMA1_Channel7_IRQHandler
ADC1_IRQHandler
EXTI9_5_IRQHandler
TIM1_BRK_TIM15_IRQHandler
TIM1_UP_TIM16_IRQHandler
TIM1_TRG_COM_TIM17_IRQHandler
TIM1_CC_IRQHandler
TIM2_IRQHandler
TIM3_IRQHandler
TIM4_IRQHandler
I2C1_EV_IRQHandler
I2C1_ER_IRQHandler
I2C2_EV_IRQHandler
I2C2_ER_IRQHandler
SPI1_IRQHandler
SPI2_IRQHandler
USART1_IRQHandler
USART2_IRQHandler
USART3_IRQHandler
EXTI15_10_IRQHandler
RTCAlarm_IRQHandler
CEC_IRQHandler
TIM6_DAC_IRQHandler
TIM7_IRQHandler
                B       .

                ENDP

                ALIGN

;*******************************************************************************
; User Stack and Heap initialization
;*******************************************************************************
                 IF      :DEF:__MICROLIB           
                
                 EXPORT  __initial_sp
                 EXPORT  __heap_base
                 EXPORT  __heap_limit
                
                 ELSE
                
                 IMPORT  __use_two_region_memory
                 EXPORT  __user_initial_stackheap
                 
__user_initial_stackheap

                 LDR     R0, =  Heap_Mem
                 LDR     R1, =(Stack_Mem + Stack_Size)
                 LDR     R2, = (Heap_Mem +  Heap_Size)
                 LDR     R3, = Stack_Mem
                 BX      LR

                 ALIGN

                 ENDIF

                 END

;******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE*****
