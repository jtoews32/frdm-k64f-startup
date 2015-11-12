    .syntax unified
    .arch armv7-m
    
    .section .stack
    .align 3
    .equ    Stack_Size, 0x400

    .globl    __StackTop
    .globl    __StackLimit
__StackLimit:
    .space    Stack_Size
    .size __StackLimit, . - __StackLimit
__StackTop:
    .size __StackTop, . - __StackTop

    .section .isr_vector,"a",%progbits
    .align 2
    .globl __isr_vector
__isr_vector:
    .long   __StackTop                  /* Top of Stack */
    .long   Reset_Handler               /* Reset Handler */
    .long   NMI_Handler                 /* NMI Handler                  */
    .long   HardFault_Handler           /* Hard Fault Handler           */
    .long   MemManage_Handler           /* MPU Fault Handler            */
    .long   BusFault_Handler            /* Bus Fault Handler            */
    .long   UsageFault_Handler          /* Usage Fault Handler          */
    .long   0                           /* Reserved                     */
    .long   0                           /* Reserved                     */
    .long   0                           /* Reserved                     */
    .long   0                           /* Reserved                     */
    .long   SVC_Handler                 /* SVCall Handler               */
    .long   DebugMon_Handler            /* Debug Monitor Handler        */
    .long   0                           /* Reserved                     */
    .long   PendSV_Handler              /* PendSV Handler               */
    .long   SysTick_Handler             /* SysTick Handler              */

    .size    __isr_vector, . - __isr_vector

    .text 
    .thumb
    .thumb_func
    .equ	PORTE_BASE,	 	0x4004D000
    .equ 	PORTB_BASE,  	0x4004A000
    .equ	PTB_BASE, 		0x400FF040
    .equ 	PTE_BASE,		0x400FF100
    .equ	PORT_PCR22,  	0x58
    .equ	PORT_PCR21,		0x54
	.equ	PORT_PCR26,		0x68
    .equ	GPIO_PDOR,  	0x00   @; Port Data Output Register, offset: 0x0
	.equ	GPIO_PSOR,		0x04   @; Port Set Output Register, offset: 0x4
	.equ	GPIO_PCOR,		0x08   @; Port Clear Output Register, offset: 0x8
	.equ	GPIO_PTOR,		0x0C   @; Port Toggle Output Register, offset: 0xC
	.equ	GPIO_PDIR,		0x10   @; Port Data Input Register, offset: 0x10
	.equ	GPIO_PDDR,		0x14   @; Port Data Direction Register, offset: 0x14

	
    .align  2
    .globl   Reset_Handler
    .type    Reset_Handler, %function
Reset_Handler:
	@ Set PTB 22 as General GPIO pin  
	LDR r1, =PORTB_BASE
	LDR r0, [r1, #PORT_PCR22]
	ORR r0, r0, #(1<<8) 
	STR r0, [r1, #PORT_PCR22]
		
	@ Set PTB 21 as General GPIO pin  
	LDR r0, [r1, #PORT_PCR21]
	ORR r0, r0, #(1<<8) 
	STR r0, [r1, #PORT_PCR21]
		
	@ Set PTE 26 as General GPIO pin
	LDR r1, =PORTE_BASE
	LDR r0, [r1, #PORT_PCR26]
	ORR r0, r0, #(1<<8) 
	STR r0, [r1, #PORT_PCR26]     
	
	@ Port Data Direction Register (PDDR)
	@ 0 = digital input, 1 = digital output
 
	LDR r1, =PTB_BASE
	LDR r0, [r1, #GPIO_PDDR]
	ORR r0, r0, #(1<<21)     @ PTB 21 as output
	ORR r0,	r0, #(1 <<22)    @ PTB 22 as output
	STR r0, [r1, #GPIO_PDDR]

	LDR r1, =PTE_BASE
	LDR r0, [r1, #GPIO_PDDR]
	ORR r0, r0, #(1<<26) 
	STR r0, [r1, #GPIO_PDDR]

	@ Port Data Output Register (PDOR) 
	@ Output low to light up LED
	LDR r1, =PTB_BASE
	LDR r0, [r1, #GPIO_PDOR]
	ORR r0, r0, #(1<<21)       @ Turn off Blue
	BIC r0, r0, #(1<<22)       @ Turn on Red
	STR r0, [r1, #GPIO_PDOR]

	LDR r1, =PTE_BASE
	LDR r0, [r1, #GPIO_PDOR]
	ORR r0, r0, #(1<<26)       @ Turn off Green
	STR r0, [r1, #GPIO_PDOR]	  

loop:	B	loop   @ ; Dead loop 	
    .size Reset_Handler, . - Reset_Handler


    .text
/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loops. They can be
 *    overwritten by other handlers */
    .macro    def_default_handler    handler_name
    .align 1
    .thumb_func
    .weak    \handler_name
    .type    \handler_name, %function
\handler_name :
    b    .
    .size    \handler_name, . - \handler_name
    .endm

    def_default_handler    NMI_Handler
    def_default_handler    HardFault_Handler
    def_default_handler    MemManage_Handler
    def_default_handler    BusFault_Handler
    def_default_handler    UsageFault_Handler
    def_default_handler    SVC_Handler
    def_default_handler    DebugMon_Handler
    def_default_handler    PendSV_Handler
    def_default_handler    SysTick_Handler
    def_default_handler    Default_Handler

