/* File: startup_da1469x.S
 * Purpose: startup file for DA1469x devices. Should use with
 *   GCC for ARM Embedded Processors
 * Version: V2.0
 * Date: 16 August 2013
 */

/* Copyright (c) 2011 - 2013 ARM LIMITED

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of ARM nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
   ---------------------------------------------------------------------------*/
#include "custom_config.h"

#define SYS_CTRL_REG                            0x50000024
#define SYS_CTRL_REG_DEBUGGER_ENABLE_MSK        0x80
#define SYS_CTRL_REG_REMAP_ADR0_MSK             0x7

#define WATCHDOG_CTRL_REG                       0x50000704
#define WATCHDOG_CTRL_REG_WDOG_FREEZE_EN_MSK    0x4
#define SYS_WDOG_WATCHDOG_CTRL_REG_NMI_RST_Msk  0x1

#define RESET_FREEZE_REG                        0x50040304
#define RESET_FREEZE_REG_FRZ_SYS_WDOG_MSK       0x08

#define SET_FREEZE_REG                          0x50040300
#define SET_FREEZE_REG_FRZ_SYS_WDOG_MSK         0x08

/* Micro Trace Buffer registers */
#define MTB_POSITION_REG                        0xE0043000
#define MTB_FLOW_REG                            0xE0043008
#define MTB_MASTER_REG                          0xE0043004
#define MTB_MASTER_REG_ENABLE_MSK               0x80000009

	.syntax	unified
	.arch	armv7-m

#if ( (dg_configSKIP_MAGIC_CHECK_AT_START == 0) && (dg_configIMAGE_SETUP == PRODUCTION_MODE) )
#error "Building Production image with MAGIC Value check enabled! Possible security risk..."
#endif

	.equ RESET_STAT_REG, 0x500000BC

#if (dg_configCODE_LOCATION == NON_VOLATILE_IS_NONE)
	.section .text
#else
	.section text_retained
#endif
	.thumb
	.thumb_func
	.align	2
	.globl	Wakeup_Reset_Handler
	.type	Wakeup_Reset_Handler, %function
Wakeup_Reset_Handler:
        /*
         *  Determine if this is a cold reset (e.g. POR, watchdog reset etc.) or
         *  a wakeup from deep sleep.
         *
         *  If RESET_STAT_REG = 0, it is a wakeup from deep sleep without state retention
         *  since RESET_STAT_REG is zeroed by goto_deepsleep().
         *  In that case, the Reset_Handler jumps to wakeup_from_deepsleep,
         *  instead of going through the normal startup procedure.
         */
        ldr     r1, =RESET_STAT_REG
        ldr     r2, [r1]
        cbnz	r2, .cold_reset            /* if RESET_STAT_REG != 0 then jump to .cold_reset */

        ldr     r3, =wakeup_from_deepsleep
        bx      r3
.cold_reset:
        ldr     r3, =Reset_Handler
        bx      r3
        .size	Wakeup_Reset_Handler, . - Wakeup_Reset_Handler


        .section text_reset
        .thumb
        .thumb_func
        .align  2
        .globl  Reset_Handler
        .type   Reset_Handler, %function
Reset_Handler:
        ldr     r0, =0x20000004
        ldr     r1, =Wakeup_Reset_Handler
        str     r1, [r0]

#if (dg_configENABLE_MTB)
        ldr     r0, =MTB_POSITION_REG
        ldr     r1, =0
        str     r1, [r0]
        ldr     r0, =MTB_FLOW_REG
        ldr     r1, =0
        str     r1, [r0]
        ldr     r0, =MTB_MASTER_REG
        ldr     r1, =MTB_MASTER_REG_ENABLE_MSK
        str     r1, [r0]
#endif

#if (dg_configSKIP_MAGIC_CHECK_AT_START == 0)

/* If the Magic Word {0xDEADBEEF, 0xDEADBEEF, 0xDEADBEEF, 0xDEAD10CC} is found at 0x20010000 then the
 * execution will block for a while in order to give time to a debugger to attach. */
        ldr     r4, =0x20010000
        ldmia   r4!, {r0-r3}
        ldr     r4, =0xDEADBEEF
        cmp     r0, r4
        bne     .code_starts
        cmp     r1, r4
        bne     .code_starts
        cmp     r2, r4
        bne     .code_starts
        ldr     r4, =0xDEAD10CC
        cmp     r3, r4
        bne     .code_starts
        /* Enable debugger */
        ldr     r4, =SYS_CTRL_REG
        ldrh    r5, [r4]
        mov     r6, r5
        movs    r1, #SYS_CTRL_REG_DEBUGGER_ENABLE_MSK
        orrs    r5, r5, r1
        strh    r5, [r4]
        /* Block for 2'' */
        ldr     r0, =0x150000
.delay_loop:
        subs    r0, #1
        bgt     .delay_loop
        /* Make sure that this will happen only once! */
        ldr     r4, =0x20010000
        movs    r0, #0
        str     r0, [r4]
        /* Restore debugger setting */
        ldr     r4, =SYS_CTRL_REG
        strh    r6, [r4]

.code_starts:
#endif

/* RAM projects should disable the cache controller */
#if (dg_configCODE_LOCATION == NON_VOLATILE_IS_NONE)
        ldr     r4, =SYS_CTRL_REG
        ldrh    r0, [r4]
        movs    r1, #1
        lsls    r1, r1, #10
        bics    r0, r0, r1
        /* Update SYS_CTRL_REG. */
        strh    r0, [r4]
#endif

#ifndef __NO_SYSTEM_INIT
        bl		SystemInitPre
#endif /* __NO_SYSTEM_INIT */

/*  Firstly it copies data from read only memory to RAM. There are two schemes
 *  to copy. One can copy more than one sections. Another can only copy
 *  one section.  The former scheme needs more instructions and read-only
 *  data to implement than the latter.
 *  Macro __STARTUP_COPY_MULTIPLE is used to choose between two schemes.  */
#define __STARTUP_COPY_MULTIPLE

#ifdef __STARTUP_COPY_MULTIPLE
/*  Multiple sections scheme.
 *
 *  Between symbol address __copy_table_start__ and __copy_table_end__,
 *  there are array of triplets, each of which specify:
 *    offset 0: LMA of start of a section to copy from
 *    offset 4: VMA of start of a section to copy to
 *    offset 8: size of the section to copy. Must be multiply of 4
 *
 *  All addresses must be aligned to 4 bytes boundary.
 */
	ldr	r4, =__copy_table_start__
	ldr	r5, =__copy_table_end__

.L_loop0:
	cmp	r4, r5
	bge	.L_loop0_done
	ldr	r1, [r4]
	ldr	r2, [r4, #4]
	ldr	r3, [r4, #8]

/*
 *  Copy (src(r1), dst(r2), len(r3))
 */
	push    {r4, r5}
.L_loop0_0:
	subs	r3, #4
	blt     .L_loop0_end
	ldr     r4, [r1], #4
	str     r4, [r2], #4
	lsls    r4, r3, #26	/* while (r3 % 64) */
	bne     .L_loop0_0

.L_loop0_1:
	subs    r3, #64
	blt     .L_loop0_end
	ldmia   r1!, {r4-r11}
	stmia   r2!, {r4-r11}
	ldmia   r1!, {r4-r11}
	stmia   r2!, {r4-r11}
	b       .L_loop0_1

.L_loop0_end:
	pop    {r4, r5}

	adds	r4, #12
	b       .L_loop0

.L_loop0_done:
#else
/*  Single section scheme.
 *
 *  The ranges of copy from/to are specified by following symbols
 *    __etext: LMA of start of the section to copy from. Usually end of text
 *    __data_start__: VMA of start of the section to copy to
 *    __data_end__: VMA of end of the section to copy to
 *
 *  All addresses must be aligned to 4 bytes boundary.
 */
	ldr	r1, =__etext
	ldr	r2, =__data_start__
	ldr	r3, =__data_end__

.L_loop1:
	cmp	r2, r3
	ittt	lt
	ldrlt	r0, [r1], #4
	strlt	r0, [r2], #4
	blt	.L_loop1
#endif /*__STARTUP_COPY_MULTIPLE */

/*  This part of work usually is done in C library startup code. Otherwise,
 *  define this macro to enable it in this startup.
 *
 *  There are two schemes too. One can clear multiple BSS sections. Another
 *  can only clear one section. The former is more size expensive than the
 *  latter.
 *
 *  Define macro __STARTUP_CLEAR_BSS_MULTIPLE to choose the former.
 *  Otherwise efine macro __STARTUP_CLEAR_BSS to choose the later.
 */
 #define __STARTUP_CLEAR_BSS_MULTIPLE

#ifdef __STARTUP_CLEAR_BSS_MULTIPLE
/*  Multiple sections scheme.
 *
 *  Between symbol address __copy_table_start__ and __copy_table_end__,
 *  there are array of tuples specifying:
 *    offset 0: Start of a BSS section
 *    offset 4: Size of this BSS section. Must be multiply of 4
 */
	ldr	r3, =__zero_table_start__
	ldr	r4, =__zero_table_end__

.L_loop2:
	cmp	r3, r4
	bge	.L_loop2_done
	ldr	r1, [r3]
	ldr	r2, [r3, #4]

/*
 *  Zero (from(r1), size(r2))
 */
	push    {r4}
	mov     r4, 0
	mov     r5, r4
	mov     r6, r4
	mov     r7, r4
	mov     r8, r4
	mov     r9, r4
	mov     r10, r4
	mov     r11, r4
.L_loop2_0:
	subs	r2, #4
	blt     .L_loop2_end
	str     r4, [r1], #4
	lsls    r0, r2, #26 /* while (r2 % 64) */
	bne     .L_loop2_0

.L_loop2_1:
	subs	r2, #64
	blt     .L_loop2_end
	stmia   r1!, {r4-r11}
	stmia   r1!, {r4-r11}
	b       .L_loop2_1

.L_loop2_end:
	pop    {r4}

	adds	r3, #8
	b       .L_loop2
.L_loop2_done:
#elif defined (__STARTUP_CLEAR_BSS)
/*  Single BSS section scheme.
 *
 *  The BSS section is specified by following symbols
 *    __bss_start__: start of the BSS section.
 *    __bss_end__: end of the BSS section.
 *
 *  Both addresses must be aligned to 4 bytes boundary.
 */
	ldr	r1, =__bss_start__
	ldr	r2, =__bss_end__

	movs	r0, 0
.L_loop3:
	cmp	r1, r2
	itt	lt
	strlt	r0, [r1], #4
	blt	.L_loop3
#endif /* __STARTUP_CLEAR_BSS_MULTIPLE || __STARTUP_CLEAR_BSS */

#ifndef __NO_SYSTEM_INIT
    bl		SystemInit
#endif /* __NO_SYSTEM_INIT */

/*
#ifndef __START
#define __START _start
#endif
	bl	__START
*/

	.pool
	.size	Reset_Handler, . - Reset_Handler

    .text
    .align  2
	.thumb_func
	.weak	Default_Handler
	.type	Default_Handler, %function
Default_Handler:
        /* Enable debugger */
        ldr     r1, =SYS_CTRL_REG
        ldrh    r2, [r1, #0]
        movs    r3, #SYS_CTRL_REG_DEBUGGER_ENABLE_MSK
        orrs    r2, r2, r3
        strh    r2, [r1, #0]
	b	.
	.size	Default_Handler, . - Default_Handler

/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loops. They can be
 *    overwritten by other handlers */
	.macro	def_irq_handler	handler_name
	.weak	\handler_name
	.set	\handler_name, Default_Handler
	.endm

	.thumb
	.thumb_func
	.align	1
	.weak	SVC_Handler
	.type	SVC_Handler, %function
SVC_Handler:
	b	.
	.size	SVC_Handler, . - SVC_Handler
    /*def_irq_handler MemManage_Handler
    def_irq_handler BusFault_Handler
    def_irq_handler UsageFault_Handler*/
    def_irq_handler SecureFault_Handler
    def_irq_handler DebugMon_Handler
    def_irq_handler PendSV_Handler
    def_irq_handler SysTick_Handler

	def_irq_handler Sensor_Node_Handler
    def_irq_handler DMA_Handler
    def_irq_handler Charger_State_Handler
    def_irq_handler Charger_Error_Handler
    def_irq_handler CMAC2SYS_Handler
    def_irq_handler UART_Handler
    def_irq_handler UART2_Handler
    def_irq_handler UART3_Handler
    def_irq_handler I2C_Handler
    def_irq_handler I2C2_Handler
    def_irq_handler SPI_Handler
    def_irq_handler SPI2_Handler
    def_irq_handler PCM_Handler
    def_irq_handler SRC_In_Handler
    def_irq_handler SRC_Out_Handler
    def_irq_handler USB_Handler
    def_irq_handler Timer_Handler
    def_irq_handler Timer2_Handler
    def_irq_handler RTC_Handler
    def_irq_handler Key_Wkup_GPIO_Handler
    def_irq_handler PDC_Handler
    def_irq_handler VBUS_Handler
    def_irq_handler MRM_Handler
    def_irq_handler Motor_Controller_Handler
    def_irq_handler TRNG_Handler
    def_irq_handler DCDC_Handler
    def_irq_handler XTAL32M_Ready_Handler
    def_irq_handler ADC_Handler
    def_irq_handler ADC2_Handler
    def_irq_handler Crypto_Handler
    def_irq_handler CAPTIMER1_Handler
    def_irq_handler RFDIAG_Handler
    def_irq_handler LCD_Controller_Handler
    def_irq_handler PLL_Lock_Handler
    def_irq_handler Timer3_Handler
    def_irq_handler Timer4_Handler
    def_irq_handler LRA_Handler
    def_irq_handler RTC_Event_Handler
    def_irq_handler GPIO_P0_Handler
    def_irq_handler GPIO_P1_Handler
    def_irq_handler Reserver40_Handler
    def_irq_handler Reserver41_Handler
    def_irq_handler Reserver42_Handler
    def_irq_handler Reserver43_Handler
    def_irq_handler Reserver44_Handler
    def_irq_handler Reserver45_Handler
    def_irq_handler Reserver46_Handler
    def_irq_handler Reserver47_Handler

#if (dg_configCODE_LOCATION == NON_VOLATILE_IS_FLASH)
        .section text_retained
#endif
        .align  2
        .thumb
        .thumb_func
        .globl  NMI_Handler
        .type   NMI_Handler, %function
NMI_Handler:
        ldr     r3, =NMI_HandlerC
        movs    r0, #4
        mov     r1, lr
        tst     r0, r1
        beq     NMI_stacking_using_MSP
        mrs     r0, psp
        b       stack_check
NMI_stacking_using_MSP:
        mrs     r0, msp
        b       stack_check
        .size   NMI_Handler, . - NMI_Handler

		.align	2
		.thumb
		.thumb_func
		.globl	MemManage_Handler
		.type	MemManage_Handler, %function
MemManage_Handler: 
		ldr 	r3, =MemManage_HandlerC
		movs	r0, #4
		mov 	r1, lr
		tst 	r0, r1
		beq 	MEM_stacking_using_MSP
		mrs 	r0, psp
MEM_stacking_using_MSP:
		mrs 	r0, msp
		b	    stack_check
		.size	MemManage_Handler, . - MemManage_Handler

		.align	2
		.thumb
		.thumb_func
		.globl	BusFault_Handler
		.type	BusFault_Handler, %function
BusFault_Handler: 
		ldr 	r3, =BusFault_HandlerC
		movs	r0, #4
		mov 	r1, lr
		tst 	r0, r1
		beq 	BUS_stacking_using_MSP
		mrs 	r0, psp
BUS_stacking_using_MSP:
		mrs 	r0, msp
		b	    stack_check
		.size	BusFault_Handler, . - BusFault_Handler

        .align  2
        .thumb
        .thumb_func
        .globl  UsageFault_Handler
        .type   UsageFault_Handler, %function
UsageFault_Handler: 
		ldr 	r3, =UsageFault_HandlerC
		movs	r0, #4
		mov 	r1, lr
		tst 	r0, r1
		beq 	USAGE_stacking_using_MSP
		mrs 	r0, psp
USAGE_stacking_using_MSP:
		mrs 	r0, msp
		b	    stack_check
        .size   UsageFault_Handler, . - UsageFault_Handler

        .align  2
        .thumb
        .thumb_func
        .globl  HardFault_Handler
        .type   HardFault_Handler, %function
HardFault_Handler:
        ldr     r3, =HardFault_HandlerC
        movs    r0, #4
        mov     r1, lr
        tst     r0, r1
        beq     HF_stacking_using_MSP
        mrs     r0, psp
        b       stack_check
HF_stacking_using_MSP:
        mrs     r0, msp
stack_check:
        /* If address 0x0 is remapped to SYSRAM (SYS_CTRL_REG[REMAP_ADR0] is 0x3),
         * check if SP (r0) belongs to one of the following ranges:
         * - [0x0, 0x80000)
         * - [0x800000, 0x880000)
         * - [0x20000000, 0x20080000)
         * Else, check if SP (r0) belongs to one of the following ranges:
         * - [0x800000, 0x880000)
         * - [0x20000000, 0x20080000)
         */
        ldr     r2, =SYS_CTRL_REG
        ldr     r2, [r2]
        and     r2, r2, #SYS_CTRL_REG_REMAP_ADR0_MSK
        cmp     r2, #3
        beq     stack_check_range_0x0
        b       stack_check_range_0x800000

stack_check_range_0x0:
        /* Check if SP (r0) belongs to [0x0, 0x80000)  */
        ldr     r2, =0x80000-32
        cmp     r0, r2
        bgt 	stack_check_range_0x800000
        bx      r3 /* Valid SP */

stack_check_range_0x800000:
        /* Check if SP (r0) belongs to [0x800000, 0x880000) */
        ldr     r2, =0x800000
        cmp     r0, r2
        blt     Wrong_SP
        ldr     r2, =0x880000-32
        cmp     r0, r2
        bgt 	stack_check_range_0x20000000
        bx      r3 /* Valid SP */

stack_check_range_0x20000000:
        /* Check if SP (r0) belongs to [0x20000000, 0x20080000) */
        ldr     r2, =0x20000000
        cmp     r0, r2
        blt     Wrong_SP
        ldr     r2, =0x20080000-32
        cmp     r0, r2
        bgt     stack_check_range_0x32000000
        bx      r3 /* Valid SP */

stack_check_range_0x32000000:
		/* Check if SP (r0) belongs to [0x32000000, 0x34000000) */
		ldr     r2, =0x32000000
		cmp     r0, r2
		blt     Wrong_SP
		ldr     r2, =0x34000000-32
		cmp     r0, r2
		bgt     Wrong_SP
		bx      r3 /* Valid SP */

Wrong_SP:
#if (dg_configIMAGE_SETUP == DEVELOPMENT_MODE) || defined (SEC_MODEN)
        /*
        * Set       WATCHDOG_CTRL_REG[WDOG_FREEZE_EN]
        * and reset WATCHDOG_CTRL_REG[NMI_RST]
        * so that watchdog can be frozen.
        */
        ldr     r0, =WATCHDOG_CTRL_REG
        ldr     r1, =WATCHDOG_CTRL_REG_WDOG_FREEZE_EN_MSK
        str     r1, [r0]
        /* Freeze WDog at all times */
        ldr     r0, =SET_FREEZE_REG
        ldr     r1, =SET_FREEZE_REG_FRZ_SYS_WDOG_MSK
        strh    r1, [r0, #0]
        /* Enable debugger at all times */
        ldr     r4, =SYS_CTRL_REG
        ldrh    r0, [r4, #0]
        movs    r1, #SYS_CTRL_REG_DEBUGGER_ENABLE_MSK
        orrs    r0, r0, r1
        /* Update SYS_CTRL_REG */
        strh    r0, [r4, #0]
#else
        /* Unfreeze WDog at all times */
        ldr     r0, =RESET_FREEZE_REG
        ldr     r1, =RESET_FREEZE_REG_FRZ_SYS_WDOG_MSK
        strh    r1, [r0, #0]
#if (dg_configENABLE_DEBUGGER == 1)
        /* Enable debugger */
        ldr     r4, =SYS_CTRL_REG
        ldrh    r0, [r4, #0]
        movs    r1, #SYS_CTRL_REG_DEBUGGER_ENABLE_MSK
        orrs    r0, r0, r1
        /* Update SYS_CTRL_REG */
        strh    r0, [r4, #0]
#endif
#endif /* (dg_configIMAGE_SETUP == DEVELOPMENT_MODE) */
        /* Wait for the WDog to hit or a debug session to start */
        b       .
        .size   HardFault_Handler, . - HardFault_Handler

	.end
