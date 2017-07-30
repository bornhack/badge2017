/*
 * This file is part of geckonator.
 * Copyright 2017 Emil Renner Berthing <esmil@esmil.dk>
 *
 * geckonator is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * geckonator is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with geckonator. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __STACK_SIZE
#define __STACK_SIZE 1024
#endif

.syntax unified
.fpu softvfp

.global __Vectors
.global __halt
.extern main

.weak Default_Handler
.set Default_Handler, __halt

.section .stack
__StackLimit:
	.space __STACK_SIZE
	.size __StackLimit, . - __StackLimit

.macro interrupt name
	.weak \name
	.set \name, __halt
	.long \name
.endm

.section .vectors, "a", %progbits
.type __Vectors, %object
__Vectors:
	/* the first vector in Cortex is the stack pointer */
	.long __StackTop                /* top of stack */
	.long Reset_Handler             /* reset handler */
	interrupt NMI_Handler           /* nmi handler */
	interrupt HardFault_Handler     /* hard fault handler */
	.long Default_Handler           /* reserved */
	.long Default_Handler           /* reserved */
	.long Default_Handler           /* reserved */
	.long Default_Handler           /* reserved */
	.long Default_Handler           /* reserved */
	.long Default_Handler           /* reserved */
	.long Default_Handler           /* reserved */
	interrupt SVC_Handler           /* svcall handler */
	.long Default_Handler           /* reserved */
	.long Default_Handler           /* reserved */
	interrupt PendSV_Handler        /* pendsv handler */
	interrupt SysTick_Handler       /* systick handler */
	/* external interrupts */
	interrupt DMA_IRQHandler        /*  0 - DMA */
	interrupt GPIO_EVEN_IRQHandler  /*  1 - GPIO_EVEN */
	interrupt TIMER0_IRQHandler     /*  2 - TIMER0 */
	interrupt ACMP0_IRQHandler      /*  3 - ACMP0 */
	interrupt ADC0_IRQHandler       /*  4 - ADC0 */
	interrupt I2C0_IRQHandler       /*  5 - I2C0 */
	interrupt GPIO_ODD_IRQHandler   /*  6 - GPIO_ODD */
	interrupt TIMER1_IRQHandler     /*  7 - TIMER1 */
	interrupt USART1_RX_IRQHandler  /*  8 - USART1_RX */
	interrupt USART1_TX_IRQHandler  /*  9 - USART1_TX */
	interrupt LEUART0_IRQHandler    /* 10 - LEUART0 */
	interrupt PCNT0_IRQHandler      /* 11 - PCNT0 */
	interrupt RTC_IRQHandler        /* 12 - RTC */
	interrupt CMU_IRQHandler        /* 13 - CMU */
	interrupt VCMP_IRQHandler       /* 14 - VCMP */
	interrupt MSC_IRQHandler        /* 15 - MSC */
	interrupt AES_IRQHandler        /* 16 - AES */
	interrupt USART0_RX_IRQHandler  /* 17 - USART0_RX */
	interrupt USART0_TX_IRQHandler  /* 18 - USART0_TX */
	interrupt USB_IRQHandler        /* 19 - USB */
	interrupt TIMER2_IRQHandler     /* 20 - TIMER2 */
.size __Vectors, . - __Vectors

.section .text.Reset_Handler, "ax", %progbits
.thumb_func
.weak Reset_Handler
.type Reset_Handler, %function
.func Reset_Handler
Reset_Handler:
	/* copy data with *r1++ = *r0++ while r1 <= r2 */
	ldr	r0, =__etext
	ldr	r1, =__data_start__
	ldr	r2, =__data_end__
	b	2f
.ltorg
1:	ldmia	r0!, {r4}
	stmia	r1!, {r4}
2:	cmp	r1, r2
	bls	1b

	/* clear bss with *r1++ = r0 (== 0) while r1 <= r2 */
	movs	r0, #0
	ldr	r1, =__bss_start__
	ldr	r2, =__bss_end__
	b	4f
.ltorg
3:	stmia	r1!, {r0}
4:	cmp	r1, r2
	bls	3b
	bl	main
.size Reset_Handler, . - Reset_Handler
.endfunc

.section .text.__halt, "ax", %progbits
.thumb_func
.type __halt, %function
.func __halt
__halt:
	cpsid	i
0:	b	0b
.size __halt, . - __halt
.endfunc
