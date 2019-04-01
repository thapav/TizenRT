/**
 * \addtogroup BSP
 * \{
 * \addtogroup SYSTEM
 * \{
 * \addtogroup Exception_Handlers
 * \{
*/

/**
 ****************************************************************************************
 *
 * @file hw_hard_fault.c
 *
 * @brief HardFault handler.
 *
 * Copyright (C) 2015-2018 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */
#include <stdint.h>
#include <stdio.h>
#include "sdk_defs.h"
#include "hw_hard_fault.h"
#include "hw_watchdog.h"
#include "hw_cpm.h"
#include "hw_sys.h"

#if defined(SEC_MODEN)
#include "system.h"
#include "boot_info.h"
#include <osal.h>
#include <FreeRTOSConfig.h>
#endif
#include "sec_fault.h"

#define MTB_MASTER_REG                  ((uint32_t *) (0xE0043004))
#define MTB_MASTER_DISABLE_MSK          (0x00000009)



/*
 * Global variables
 */
volatile uint32_t hardfault_event_data[9] __attribute__((section("hard_fault_info")));

/*
 * Local variables
 */


/*
 * This is the base address in Retention RAM where the stacked information will be copied.
 */
#define STATUS_BASE (0x20005600)

/*
 * Compilation switch to enable verbose output on HardFault
 */
#ifndef VERBOSE_HARDFAULT
#       define VERBOSE_HARDFAULT        0
#endif

#if (dg_configFAULT_DEBUG_DUMP==1)

FAULT_header sys_fault_header;

void* FAULT_header_addr;
void* FAULT_regs_addr;
void* FAULT_ram_addr;

#if (dg_configCODE_LOCATION == NON_VOLATILE_IS_FLASH)
__attribute__((section("text_retained")))
#endif
void save_reg(SS_FAULT_REG *core_reg, unsigned long *fault_args)
{
        hw_watchdog_freeze(); //for dump
    __asm volatile (
    		"str r0, [%0,#0]\n\t"		/* R0 is pushed first to core_reg */
    		"mov r0, %0\n\t"		/* R0 will be alias for core_reg */
                "str r1, [r0,#4]\n\t"		/* R1 */
                "str r2, [r0,#8]\n\t"		/* R2 */
                "str r3, [r0,#12]\n\t"		/* R3 */
                "str r4, [r0,#16]\n\t"		/* R4 */
                "str r5, [r0,#20]\n\t"		/* R5 */
                "str r6, [r0,#24]\n\t"		/* R6 */
                "str r7, [r0,#28]\n\t"		/* R7 */
                "str r8, [r0,#32]\n\t"		/* R8 */
                "str r9, [r0,#36]\n\t"		/* R9 */
                "str r10, [r0,#40]\n\t"		/* R10 */
                "str r11, [r0,#44]\n\t"		/* R11 */
                "str r12, [r0,#48]\n\t"		/* R12 */
    		"str r13, [r0,#52]\n\t"		/* R13 - SP */
    		"str r14, [r0,#56]\n\t"		/* R14 - LR*/

    		"sub r1, r15, #0x30\n\t"		/* PC */
    		"str r1, [r0,#60]\n\t"

    		"mrs r1, xpsr\n\t"		/* xPSR */
    		"str r1, [r0,#64]\n\t":
                :"r"(core_reg)
                :"%r0", "%r1"

    );
    core_reg->Reg_CONTORL = __get_CONTROL();
    core_reg->Reg_MSP = __get_MSP();
    core_reg->Reg_PSP = __get_PSP();

    if(fault_args != NULL)
    {
            core_reg->Reg_real_PC = fault_args[6];
            core_reg->Reg_real_LR = fault_args[5];
            core_reg->Reg_real_SP = (unsigned long) fault_args;
    }
}

#if (dg_configCODE_LOCATION == NON_VOLATILE_IS_FLASH)
__attribute__((section("text_retained")))
#endif
uint32_t FAULT_memcpy(void* dest, void* src, uint32_t len)
{
        uint32_t i;

        for (i=0; i<len; i++) {
                ((uint8_t*)dest)[i] = ((uint8_t*)src)[i];
        }

        return i;
}

#if (dg_configCODE_LOCATION == NON_VOLATILE_IS_FLASH)
__attribute__((section("text_retained")))
#endif
uint32_t FAULT_memcpy32(void* dest, void* src, uint32_t len)
{
        uint32_t i;

        for (i=0; i<len; i++) {
                ((uint32_t*)dest)[i] = ((uint32_t*)src)[i];
        }

        return i;
}


#if (dg_configCODE_LOCATION == NON_VOLATILE_IS_FLASH)
__attribute__((section("text_retained")))
#endif
uint32_t FAULT_Regs(void* dest)
{
        void* regs_dump_target = dest;

        FAULT_memcpy32(regs_dump_target, (void*)AES_HASH, sizeof(AES_HASH_Type)/4);
        regs_dump_target += sizeof(AES_HASH_Type);

        FAULT_memcpy32(regs_dump_target, (void*)ANAMISC, sizeof(ANAMISC_Type)/4);
        regs_dump_target += sizeof(ANAMISC_Type);

        FAULT_memcpy32(regs_dump_target, (void*)APU, sizeof(APU_Type)/4);
        regs_dump_target += sizeof(APU_Type);

        FAULT_memcpy32(regs_dump_target, (void*)CACHE, sizeof(CACHE_Type)/4);
        regs_dump_target += sizeof(CACHE_Type);

        FAULT_memcpy32(regs_dump_target, (void*)CHARGER, sizeof(CHARGER_Type)/4);
        regs_dump_target += sizeof(CHARGER_Type);

        FAULT_memcpy32(regs_dump_target, (void*)CHIP_VERSION, sizeof(CHIP_VERSION_Type)/4);
        regs_dump_target += sizeof(CHIP_VERSION_Type);

        FAULT_memcpy32(regs_dump_target, (void*)CRG_COM, sizeof(CRG_COM_Type)/4);
        regs_dump_target += sizeof(CRG_COM_Type);

        FAULT_memcpy32(regs_dump_target, (void*)CRG_PER, sizeof(CRG_PER_Type)/4);
        regs_dump_target += sizeof(CRG_PER_Type);

        FAULT_memcpy32(regs_dump_target, (void*)CRG_SYS, sizeof(CRG_SYS_Type)/4);
        regs_dump_target += sizeof(CRG_SYS_Type);

        FAULT_memcpy32(regs_dump_target, (void*)CRG_TOP, sizeof(CRG_TOP_Type)/4);
        regs_dump_target += sizeof(CRG_TOP_Type);

        FAULT_memcpy32(regs_dump_target, (void*)CRG_XTAL, sizeof(CRG_XTAL_Type)/4);
        regs_dump_target += sizeof(CRG_XTAL_Type);

        FAULT_memcpy32(regs_dump_target, (void*)DCDC, sizeof(DCDC_Type)/4);
        regs_dump_target += sizeof(DCDC_Type);

        FAULT_memcpy32(regs_dump_target, (void*)DEM, sizeof(DEM_Type)/4);
        regs_dump_target += sizeof(DEM_Type);

        FAULT_memcpy32(regs_dump_target, (void*)DMA, sizeof(DMA_Type)/4);
        regs_dump_target += sizeof(DMA_Type);

        FAULT_memcpy32(regs_dump_target, (void*)GPADC, sizeof(GPADC_Type)/4);
        regs_dump_target += sizeof(GPADC_Type);

        FAULT_memcpy32(regs_dump_target, (void*)GPIO, sizeof(GPIO_Type)/4);
        regs_dump_target += sizeof(GPIO_Type);

        FAULT_memcpy32(regs_dump_target, (void*)GPREG, sizeof(GPREG_Type)/4);
        regs_dump_target += sizeof(GPREG_Type);

        FAULT_memcpy32(regs_dump_target, (void*)I2C, sizeof(I2C_Type)/4);
        regs_dump_target += sizeof(I2C_Type);

        FAULT_memcpy32(regs_dump_target, (void*)I2C2, sizeof(I2C2_Type)/4);
        regs_dump_target += sizeof(I2C2_Type);

        FAULT_memcpy32(regs_dump_target, (void*)LCDC, sizeof(LCDC_Type)/4);
        regs_dump_target += sizeof(LCDC_Type);

        FAULT_memcpy32(regs_dump_target, (void*)LRA, sizeof(LRA_Type)/4);
        regs_dump_target += sizeof(LRA_Type);

        FAULT_memcpy32(regs_dump_target, (void*)MEMCTRL, sizeof(MEMCTRL_Type)/4);
        regs_dump_target += sizeof(MEMCTRL_Type);

        FAULT_memcpy32(regs_dump_target, (void*)OTPC, sizeof(OTPC_Type)/4);
        regs_dump_target += sizeof(OTPC_Type);

        FAULT_memcpy32(regs_dump_target, (void*)PATCH, sizeof(PATCH_Type)/4);
        regs_dump_target += sizeof(PATCH_Type);

        FAULT_memcpy32(regs_dump_target, (void*)PDC, sizeof(PDC_Type)/4);
        regs_dump_target += sizeof(PDC_Type);

        FAULT_memcpy32(regs_dump_target, (void*)PWMLED, sizeof(PWMLED_Type)/4);
        regs_dump_target += sizeof(PWMLED_Type);

        FAULT_memcpy32(regs_dump_target, (void*)QSPIC, sizeof(QSPIC_Type)/4);
        regs_dump_target += sizeof(QSPIC_Type);

        FAULT_memcpy32(regs_dump_target, (void*)QSPIC2, sizeof(QSPIC2_Type)/4);
        regs_dump_target += sizeof(QSPIC2_Type);

        FAULT_memcpy32(regs_dump_target, (void*)RFCU, sizeof(RFCU_Type)/4);
        regs_dump_target += sizeof(RFCU_Type);

        FAULT_memcpy32(regs_dump_target, (void*)RFCU_POWER, sizeof(RFCU_POWER_Type)/4);
        regs_dump_target += sizeof(RFCU_POWER_Type);

        FAULT_memcpy32(regs_dump_target, (void*)RFMON, sizeof(RFMON_Type)/4);
        regs_dump_target += sizeof(RFMON_Type);

        FAULT_memcpy32(regs_dump_target, (void*)RTC, sizeof(RTC_Type)/4);
        regs_dump_target += sizeof(RTC_Type);

        FAULT_memcpy32(regs_dump_target, (void*)SDADC, sizeof(SDADC_Type)/4);
        regs_dump_target += sizeof(SDADC_Type);

        FAULT_memcpy32(regs_dump_target, (void*)SMOTOR, sizeof(SMOTOR_Type)/4);
        regs_dump_target += sizeof(SMOTOR_Type);

        FAULT_memcpy32(regs_dump_target, (void*)SNC, sizeof(SNC_Type)/4);
        regs_dump_target += sizeof(SNC_Type);

        FAULT_memcpy32(regs_dump_target, (void*)SPI, sizeof(SPI_Type)/4);
        regs_dump_target += sizeof(SPI_Type);

        FAULT_memcpy32(regs_dump_target, (void*)SPI2, sizeof(SPI2_Type)/4);
        regs_dump_target += sizeof(SPI2_Type);

        FAULT_memcpy32(regs_dump_target, (void*)SYS_WDOG, sizeof(SYS_WDOG_Type)/4);
        regs_dump_target += sizeof(SYS_WDOG_Type);

        FAULT_memcpy32(regs_dump_target, (void*)TIMER, sizeof(TIMER_Type)/4);
        regs_dump_target += sizeof(TIMER_Type);

        FAULT_memcpy32(regs_dump_target, (void*)TIMER2, sizeof(TIMER2_Type)/4);
        regs_dump_target += sizeof(TIMER2_Type);

        FAULT_memcpy32(regs_dump_target, (void*)TIMER3, sizeof(TIMER3_Type)/4);
        regs_dump_target += sizeof(TIMER3_Type);

        FAULT_memcpy32(regs_dump_target, (void*)TIMER4, sizeof(TIMER4_Type)/4);
        regs_dump_target += sizeof(TIMER4_Type);

        FAULT_memcpy32(regs_dump_target, (void*)TRNG, sizeof(TRNG_Type)/4);
        regs_dump_target += sizeof(TRNG_Type);

        FAULT_memcpy32(regs_dump_target, (void*)UART, sizeof(UART_Type)/4);
        regs_dump_target += sizeof(UART_Type);

        FAULT_memcpy32(regs_dump_target, (void*)UART2, sizeof(UART2_Type)/4);
        regs_dump_target += sizeof(UART2_Type);

        FAULT_memcpy32(regs_dump_target, (void*)UART3, sizeof(UART3_Type)/4);
        regs_dump_target += sizeof(UART3_Type);

        FAULT_memcpy32(regs_dump_target, (void*)USB, sizeof(USB_Type)/4);
        regs_dump_target += sizeof(USB_Type);

        FAULT_memcpy32(regs_dump_target, (void*)WAKEUP, sizeof(WAKEUP_Type)/4);
        regs_dump_target += sizeof(WAKEUP_Type);

        return (regs_dump_target - dest);
}

#if (dg_configCODE_LOCATION == NON_VOLATILE_IS_FLASH)
__attribute__((section("text_retained")))
#endif
bool FAULT_is_valid_dump(void)
{
        FAULT_header* tmp_FAULT_header_addr = (FAULT_header*)FAULT_get_header_addr();

        return (tmp_FAULT_header_addr->is_valid == FAULT_MAGIC_NUMBER);
}


#if (dg_configCODE_LOCATION == NON_VOLATILE_IS_FLASH)
__attribute__((section("text_retained")))
#endif
void* FAULT_get_header_addr(void)
{
        FAULT_header_addr = (void*)(MEMORY_QSPIR_BASE + (dg_configQSPIC2_DEV_CONFIG.memory_size/8) - FAULT_RAM_SIZE - FAULT_REGS_SIZE - sizeof(FAULT_header));

        return FAULT_header_addr;
}

#if (dg_configCODE_LOCATION == NON_VOLATILE_IS_FLASH)
__attribute__((section("text_retained")))
#endif
void* FAULT_get_regs_addr(void)
{
        FAULT_regs_addr = (void*)(MEMORY_QSPIR_BASE + (dg_configQSPIC2_DEV_CONFIG.memory_size/8) - FAULT_RAM_SIZE - FAULT_REGS_SIZE);

        return FAULT_regs_addr;
}

#if (dg_configCODE_LOCATION == NON_VOLATILE_IS_FLASH)
__attribute__((section("text_retained")))
#endif
void* FAULT_get_ram_addr(void)
{
        FAULT_ram_addr = (void*)(MEMORY_QSPIR_BASE + (dg_configQSPIC2_DEV_CONFIG.memory_size/8) - FAULT_RAM_SIZE);

        return FAULT_ram_addr;
}

#endif /*dg_configFAULT_DEBUG_DUMP*/
/*
 * Function definitions
 */

/**
* \brief HardFault handler implementation. During development it will copy the system's status
*        to a predefined location in memory. In release mode, it will cause a system reset.
*
* \param [in] hardfault_args The system's status when the HardFault event occurred.
*
* \return void
*
*/
#if (dg_configCODE_LOCATION == NON_VOLATILE_IS_FLASH)
__attribute__((section("text_retained")))
#endif
void HardFault_HandlerC(unsigned long *hardfault_args)
{
#if (dg_configFAULT_DEBUG_DUMP==1)
	save_reg(&ss_fault_register, hardfault_args);
#endif
#if dg_configENABLE_MTB
        /* Disable MTB */
        *MTB_MASTER_REG = MTB_MASTER_DISABLE_MSK;
#endif /* dg_configENABLE_MTB */

        // Stack frame contains:
        // r0, r1, r2, r3, r12, r14, the return address and xPSR
        // - Stacked R0 = hf_args[0]
        // - Stacked R1 = hf_args[1]
        // - Stacked R2 = hf_args[2]
        // - Stacked R3 = hf_args[3]
        // - Stacked R12 = hf_args[4]
        // - Stacked LR = hf_args[5]
        // - Stacked PC = hf_args[6]
        // - Stacked xPSR= hf_args[7]
#if (dg_configFAULT_DEBUG_DUMP==1)
        FAULT_header_addr = FAULT_get_header_addr();
        FAULT_regs_addr = FAULT_get_regs_addr();
        FAULT_ram_addr = FAULT_get_ram_addr();

        memset((void *)&sys_fault_header, 0x0, sizeof(struct __FAULT_header));
        sys_fault_header.is_valid = FAULT_MAGIC_NUMBER;
        sys_fault_header.fault_type = HARD_FAULT;
        sys_fault_header.hard_fault_args.Reg_R0  = hardfault_args[0];
        sys_fault_header.hard_fault_args.Reg_R1  = hardfault_args[1];
        sys_fault_header.hard_fault_args.Reg_R2  = hardfault_args[2];
        sys_fault_header.hard_fault_args.Reg_R3  = hardfault_args[3];
        sys_fault_header.hard_fault_args.Reg_R12 = hardfault_args[4];
        sys_fault_header.hard_fault_args.Reg_LR  = hardfault_args[5];
        sys_fault_header.hard_fault_args.Reg_PC  = hardfault_args[6];
        sys_fault_header.hard_fault_args.Reg_PSR = hardfault_args[7];
        sys_fault_header.hard_fault_args.SP      = (unsigned long) hardfault_args;
        sys_fault_header.hard_fault_args.Reg_CFSR = (*((volatile unsigned long *)(0xE000ED28)));    // CFSR
        sys_fault_header.hard_fault_args.Reg_HFSR = (*((volatile unsigned long *)(0xE000ED2C)));    // HFSR
        sys_fault_header.hard_fault_args.Reg_DFSR = (*((volatile unsigned long *)(0xE000ED30)));    // DFSR
        sys_fault_header.hard_fault_args.Reg_AFSR = (*((volatile unsigned long *)(0xE000ED3C)));    // AFSR
        sys_fault_header.hard_fault_args.Reg_MMAR = (*((volatile unsigned long *)(0xE000ED34)));    // MMAR
        sys_fault_header.hard_fault_args.Reg_BFAR = (*((volatile unsigned long *)(0xE000ED38)));    // BFAR

        // Obtain the handle of a task from its name.
        OS_TASK current_task;
        current_task = OS_GET_CURRENT_TASK();
        sys_fault_header.task_info.cnt = 1;
        memcpy(sys_fault_header.task_info.task_name[0], pcTaskGetName(current_task), configMAX_TASK_NAME_LEN);
        sys_fault_header.task_info.task_name[0][configMAX_TASK_NAME_LEN]='\0';

        // display summarized information
        printf_debug_info(1,"Fault Type     : Hard Fault\r\n");
        printf_debug_info(1,"Foreground Task: %s\r\n",sys_fault_header.task_info.task_name[0]);
		sec_fault_report_fault_info(hardfault_args,0);
        vTaskGetCallStack( current_task, 0 ); // for print current task call stack;

        FAULT_memcpy(FAULT_header_addr, (void*)&sys_fault_header, sizeof(sys_fault_header));
        FAULT_regs_addr = FAULT_get_regs_addr();
        if (FAULT_REGS_SIZE < FAULT_Regs(FAULT_regs_addr)) {
                ASSERT_WARNING(0); //the reserved size for the Registers dump is not big enough
        }
        FAULT_ram_addr = FAULT_get_ram_addr();
        FAULT_memcpy32(FAULT_ram_addr, (void*)RAM_DUMP_ADDR, FAULT_RAM_SIZE/4);
        FAULT_memcpy32(CONFIG_PSRAM_GDI_BACKUP_ADDR, (void*)CONFIG_PSRAM_DUMP_BASE, CONFIG_PSRAM_DUMP_SIZE/4);
        FAULT_memcpy32(CONFIG_PSRAM_BSS_BACKUP_ADDR, (void*)CONFIG_PSRAM_BSS_BASE, CONFIG_PSRAM_BSS_BACKUP_SIZE/4);

#endif

#ifdef SEC_MODEN
        sys_set_reset_reason(RESET_REASON_FAULT_HARD);
        printf_debug_info(0,"reboot \r\n");
#endif
        hw_cpm_reboot_system();                         // Force reset

        if (dg_configIMAGE_SETUP == DEVELOPMENT_MODE) {

                ENABLE_DEBUGGER;

                *(volatile unsigned long *)(STATUS_BASE       ) = hardfault_args[0];    // R0
                *(volatile unsigned long *)(STATUS_BASE + 0x04) = hardfault_args[1];    // R1
                *(volatile unsigned long *)(STATUS_BASE + 0x08) = hardfault_args[2];    // R2
                *(volatile unsigned long *)(STATUS_BASE + 0x0C) = hardfault_args[3];    // R3
                *(volatile unsigned long *)(STATUS_BASE + 0x10) = hardfault_args[4];    // R12
                *(volatile unsigned long *)(STATUS_BASE + 0x14) = hardfault_args[5];    // LR
                *(volatile unsigned long *)(STATUS_BASE + 0x18) = hardfault_args[6];    // PC
                *(volatile unsigned long *)(STATUS_BASE + 0x1C) = hardfault_args[7];    // PSR
                *(volatile unsigned long *)(STATUS_BASE + 0x20) = (unsigned long)hardfault_args;    // Stack Pointer

                *(volatile unsigned long *)(STATUS_BASE + 0x24) = (*((volatile unsigned long *)(0xE000ED28)));    // CFSR
                *(volatile unsigned long *)(STATUS_BASE + 0x28) = (*((volatile unsigned long *)(0xE000ED2C)));    // HFSR
                *(volatile unsigned long *)(STATUS_BASE + 0x2C) = (*((volatile unsigned long *)(0xE000ED30)));    // DFSR
                *(volatile unsigned long *)(STATUS_BASE + 0x30) = (*((volatile unsigned long *)(0xE000ED3C)));    // AFSR
                *(volatile unsigned long *)(STATUS_BASE + 0x34) = (*((volatile unsigned long *)(0xE000ED34)));    // MMAR
                *(volatile unsigned long *)(STATUS_BASE + 0x38) = (*((volatile unsigned long *)(0xE000ED38)));    // BFAR

                if (VERBOSE_HARDFAULT) {
                        printf("HardFault Handler:\r\n");
                        printf("- R0  = 0x%08lx\r\n", hardfault_args[0]);
                        printf("- R1  = 0x%08lx\r\n", hardfault_args[1]);
                        printf("- R2  = 0x%08lx\r\n", hardfault_args[2]);
                        printf("- R3  = 0x%08lx\r\n", hardfault_args[3]);
                        printf("- R12 = 0x%08lx\r\n", hardfault_args[4]);
                        printf("- LR  = 0x%08lx\r\n", hardfault_args[5]);
                        printf("- PC  = 0x%08lx\r\n", hardfault_args[6]);
                        printf("- xPSR= 0x%08lx\r\n", hardfault_args[7]);
                }

                hw_sys_assert_trigger_gpio();

                while (1);
        }
        else {
# ifdef PRODUCTION_DEBUG_OUTPUT
# if (USE_WDOG)
                WDOG->WATCHDOG_REG = 0xC8;                      // Reset WDOG! 200 * 10.24ms active time for UART to finish printing!
# endif

                dbg_prod_output(1, hardfault_args);
# endif // PRODUCTION_DEBUG_OUTPUT

                hardfault_event_data[0] = HARDFAULT_MAGIC_NUMBER;
                hardfault_event_data[1] = hardfault_args[0];    // R0
                hardfault_event_data[2] = hardfault_args[1];    // R1
                hardfault_event_data[3] = hardfault_args[2];    // R2
                hardfault_event_data[4] = hardfault_args[3];    // R3
                hardfault_event_data[5] = hardfault_args[4];    // R12
                hardfault_event_data[6] = hardfault_args[5];    // LR
                hardfault_event_data[7] = hardfault_args[6];    // PC
                hardfault_event_data[8] = hardfault_args[7];    // PSR

                hw_cpm_reboot_system();                         // Force reset

                while (1);
        }
}


#if (dg_configCODE_LOCATION == NON_VOLATILE_IS_FLASH)
__attribute__((section("text_retained")))
#endif
void MemManage_HandlerC(unsigned long *fault_args, unsigned int exc)
{
#if (dg_configFAULT_DEBUG_DUMP==1)
		save_reg(&ss_fault_register, fault_args);
#endif
        volatile uint8_t mem_fault_status_reg;
        volatile uint32_t mem_fault_addr __UNUSED;

#if dg_configENABLE_MTB
        /* Disable MTB */
        *MTB_MASTER_REG = MTB_MASTER_DISABLE_MSK;
#endif
        mem_fault_status_reg = (SCB->CFSR & SCB_CFSR_MEMFAULTSR_Msk) >> SCB_CFSR_MEMFAULTSR_Pos;
        if (mem_fault_status_reg & 0x80) {
                mem_fault_addr = SCB->MMFAR;
        }

#if (dg_configFAULT_DEBUG_DUMP==1)
        sec_mpu_enable(0, 0);
        FAULT_header_addr = FAULT_get_header_addr();
        FAULT_regs_addr = FAULT_get_regs_addr();
        FAULT_ram_addr = FAULT_get_ram_addr();

        memset((void *)&sys_fault_header, 0x0, sizeof(struct __FAULT_header));
        sys_fault_header.is_valid = FAULT_MAGIC_NUMBER;
        sys_fault_header.fault_type = MEM_FAULT;
        sys_fault_header.mem_fault_args.fault_addr = mem_fault_addr;
        sys_fault_header.mem_fault_args.fault_status_reg = mem_fault_status_reg;
		sys_fault_header.hard_fault_args.Reg_LR = ((unsigned long) fault_args[5]);
		sys_fault_header.hard_fault_args.Reg_PC = ((unsigned long) fault_args[6]);

        // Obtain the handle of a task from its name.
        OS_TASK current_task;
        current_task = OS_GET_CURRENT_TASK();
        sys_fault_header.task_info.cnt = 1;
        memcpy(sys_fault_header.task_info.task_name[0], pcTaskGetName(current_task), configMAX_TASK_NAME_LEN);
        sys_fault_header.task_info.task_name[0][configMAX_TASK_NAME_LEN]='\0';

        // display summarized information
        printf_debug_info(1,"Fault Type     : MemManage Fault\r\n");
		sec_fault_report_fault_info(fault_args, exc);
        printf_debug_info(1,"Foreground Task: %s\r\n",sys_fault_header.task_info.task_name[0]);
        vTaskGetCallStack( current_task, 0 ); // for print current task call stack;

        FAULT_memcpy(FAULT_header_addr, (void*)&sys_fault_header, sizeof(sys_fault_header));
        FAULT_regs_addr = FAULT_get_regs_addr();
        if (FAULT_REGS_SIZE < FAULT_Regs(FAULT_regs_addr)) {
                ASSERT_WARNING(0); //the reserved size for the Registers dump is not big enough
        }
        FAULT_ram_addr = FAULT_get_ram_addr();
        FAULT_memcpy32(FAULT_ram_addr, (void*)RAM_DUMP_ADDR, FAULT_RAM_SIZE/4);
        FAULT_memcpy32(CONFIG_PSRAM_GDI_BACKUP_ADDR, (void*)CONFIG_PSRAM_DUMP_BASE, CONFIG_PSRAM_DUMP_SIZE/4);
        FAULT_memcpy32(CONFIG_PSRAM_BSS_BACKUP_ADDR, (void*)CONFIG_PSRAM_BSS_BASE, CONFIG_PSRAM_BSS_BACKUP_SIZE/4);

#endif
#if defined(SEC_MODEN)
        sys_set_reset_reason(RESET_REASON_FAULT_MEM);
        printf_debug_info(0,"reboot \r\n");
#endif
        hw_cpm_reboot_system();

        while (1) {}
}


#if (dg_configCODE_LOCATION == NON_VOLATILE_IS_FLASH)
__attribute__((section("text_retained")))
#endif
void BusFault_HandlerC(unsigned long *fault_args, unsigned int exc)
{
#if (dg_configFAULT_DEBUG_DUMP==1)
		save_reg(&ss_fault_register, fault_args);
#endif
        volatile uint8_t bus_fault_status_reg;
        volatile uint32_t bus_fault_addr __UNUSED;

#if dg_configENABLE_MTB
        /* Disable MTB */
        *MTB_MASTER_REG = MTB_MASTER_DISABLE_MSK;
#endif

        bus_fault_status_reg = (SCB->CFSR & SCB_CFSR_BUSFAULTSR_Msk) >> SCB_CFSR_BUSFAULTSR_Pos;
        if (bus_fault_status_reg & 0x80) {
                bus_fault_addr = SCB->BFAR;
        } else {
                bus_fault_addr = 0x0;
        }

#if (dg_configFAULT_DEBUG_DUMP==1)
        FAULT_header_addr = FAULT_get_header_addr();
        FAULT_regs_addr = FAULT_get_regs_addr();
        FAULT_ram_addr = FAULT_get_ram_addr();

        memset((void *)&sys_fault_header, 0x0, sizeof(struct __FAULT_header));
        sys_fault_header.is_valid = FAULT_MAGIC_NUMBER;
        sys_fault_header.fault_type = BUS_FAULT;
        sys_fault_header.bus_fault_args.fault_addr = bus_fault_addr;
        sys_fault_header.bus_fault_args.fault_status_reg = bus_fault_status_reg;
		sys_fault_header.hard_fault_args.Reg_LR = ((unsigned long) fault_args[5]);
		sys_fault_header.hard_fault_args.Reg_PC = ((unsigned long) fault_args[6]);

        // Obtain the handle of a task from its name.
        OS_TASK current_task;
        current_task = OS_GET_CURRENT_TASK();
        sys_fault_header.task_info.cnt = 1;
        memcpy(sys_fault_header.task_info.task_name[0], pcTaskGetName(current_task), configMAX_TASK_NAME_LEN);
        sys_fault_header.task_info.task_name[0][configMAX_TASK_NAME_LEN]='\0';

        // display summarized information
        printf_debug_info(1,"Fault Type     : Bus Fault\r\n");
		sec_fault_report_fault_info(fault_args, exc);
        printf_debug_info(1,"Foreground Task: %s\r\n",sys_fault_header.task_info.task_name[0]);
        vTaskGetCallStack( current_task, 0 ); // for print current task call stack;

        FAULT_memcpy(FAULT_header_addr, (void*)&sys_fault_header, sizeof(sys_fault_header));
        FAULT_regs_addr = FAULT_get_regs_addr();
        if (FAULT_REGS_SIZE < FAULT_Regs(FAULT_regs_addr)) {
                ASSERT_WARNING(0); //the reserved size for the Registers dump is not big enough
        }
        FAULT_ram_addr = FAULT_get_ram_addr();
        FAULT_memcpy32(FAULT_ram_addr, (void*)RAM_DUMP_ADDR, FAULT_RAM_SIZE/4);
        FAULT_memcpy32(CONFIG_PSRAM_GDI_BACKUP_ADDR, (void*)CONFIG_PSRAM_DUMP_BASE, CONFIG_PSRAM_DUMP_SIZE/4);
        FAULT_memcpy32(CONFIG_PSRAM_BSS_BACKUP_ADDR, (void*)CONFIG_PSRAM_BSS_BASE, CONFIG_PSRAM_BSS_BACKUP_SIZE/4);

#endif
#if defined(SEC_MODEN)
        sys_set_reset_reason(RESET_REASON_FAULT_BUS);
        printf_debug_info(0,"reboot \r\n");
#endif
        hw_cpm_reboot_system();

        while (1) {}
}

#if (dg_configCODE_LOCATION == NON_VOLATILE_IS_FLASH)
__attribute__((section("text_retained")))
#endif
void UsageFault_HandlerC(unsigned long *fault_args, unsigned int exc)
{
#if (dg_configFAULT_DEBUG_DUMP==1)
		save_reg(&ss_fault_register, fault_args);
#endif
        volatile uint16_t usage_fault_status_reg __UNUSED;

        usage_fault_status_reg = (SCB->CFSR & SCB_CFSR_USGFAULTSR_Msk) >> SCB_CFSR_USGFAULTSR_Pos;

#if dg_configENABLE_MTB
        /* Disable MTB */
        *MTB_MASTER_REG = MTB_MASTER_DISABLE_MSK;
#endif

#if (dg_configFAULT_DEBUG_DUMP==1)
        FAULT_header_addr = FAULT_get_header_addr();
        FAULT_regs_addr = FAULT_get_regs_addr();
        FAULT_ram_addr = FAULT_get_ram_addr();

        memset((void *)&sys_fault_header, 0x0, sizeof(struct __FAULT_header));
        sys_fault_header.is_valid = FAULT_MAGIC_NUMBER;
        sys_fault_header.fault_type = USAGE_FAULT;
        sys_fault_header.usage_fault_args.fault_status_reg = usage_fault_status_reg;
		sys_fault_header.hard_fault_args.Reg_LR = ((unsigned long) fault_args[5]);
		sys_fault_header.hard_fault_args.Reg_PC = ((unsigned long) fault_args[6]);

        // Obtain the handle of a task from its name.
        OS_TASK current_task;
        current_task = OS_GET_CURRENT_TASK();
        sys_fault_header.task_info.cnt = 1;
        memcpy(sys_fault_header.task_info.task_name[0], pcTaskGetName(current_task), configMAX_TASK_NAME_LEN);
        sys_fault_header.task_info.task_name[0][configMAX_TASK_NAME_LEN]='\0';

        // display summarized information
        printf_debug_info(1,"Fault Type     : Usage Fault\r\n");
		sec_fault_report_fault_info(fault_args, exc);
		printf_debug_info(1,"Foreground Task: %s\r\n",sys_fault_header.task_info.task_name[0]);
        vTaskGetCallStack( current_task, 0 ); // for print current task call stack;

        FAULT_memcpy(FAULT_header_addr, (void*)&sys_fault_header, sizeof(sys_fault_header));
        FAULT_regs_addr = FAULT_get_regs_addr();
        if (FAULT_REGS_SIZE < FAULT_Regs(FAULT_regs_addr)) {
                ASSERT_WARNING(0); //the reserved size for the Registers dump is not big enough
        }
        FAULT_ram_addr = FAULT_get_ram_addr();
        FAULT_memcpy32(FAULT_ram_addr, (void*)RAM_DUMP_ADDR, FAULT_RAM_SIZE/4);
        FAULT_memcpy32(CONFIG_PSRAM_GDI_BACKUP_ADDR, (void*)CONFIG_PSRAM_DUMP_BASE, CONFIG_PSRAM_DUMP_SIZE/4);
        FAULT_memcpy32(CONFIG_PSRAM_BSS_BACKUP_ADDR, (void*)CONFIG_PSRAM_BSS_BASE, CONFIG_PSRAM_BSS_BACKUP_SIZE/4);

#endif
#if defined(SEC_MODEN)
        sys_set_reset_reason(RESET_REASON_FAULT_USAGE);
        printf_debug_info(0,"reboot \r\n");
#endif
        hw_cpm_reboot_system();

        while(1) {}
}

#if (dg_configCODE_LOCATION == NON_VOLATILE_IS_FLASH)
__attribute__((section("text_retained")))
#endif
void DebugMon_Handler(void)
{
#if dg_configENABLE_MTB
        /* Disable MTB */
        *MTB_MASTER_REG = MTB_MASTER_DISABLE_MSK;
#endif

        while (1) {}
}

/**
 * \}
 * \}
 * \}
 * */
