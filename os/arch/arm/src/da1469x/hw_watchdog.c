/**
 * \addtogroup BSP
 * \{
 * \addtogroup DEVICES
 * \{
 * \addtogroup Watchdog_Timer
 * \{
 */

/**
 ****************************************************************************************
 *
 * @file hw_watchdog.c
 *
 * @brief Implementation of the Watchdog timer Low Level Driver.
 *
 * Copyright (C) 2015-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */
#include <stdio.h>
#include "sw_version.h"
#ifdef SEC_MODEN
#include <FreeRTOSConfig.h>
#include "system.h"
#include "boot_info.h"
#endif

#include "hw_watchdog.h"
#include "hw_sys.h"

#if defined(CONFIG_WD_MONITOR)
extern void check_monitor(void);
#endif
#if (dg_configFAULT_DEBUG_DUMP==1)
#include "hw_hard_fault.h"
#endif


/*
 * Global variables
 */
volatile uint32_t nmi_event_data[9] __attribute__((section("nmi_info")));

/*
 * Local variables
 */
static hw_watchdog_interrupt_cb int_handler __RETAINED = NULL;

/*
 * This is the base address in Retention RAM where the stacked information will be copied.
 */
#define STATUS_BASE (0x20005600)

__RETAINED_CODE bool hw_watchdog_freeze(void)
{

        GPREG->SET_FREEZE_REG = GPREG_SET_FREEZE_REG_FRZ_SYS_WDOG_Msk;

        return (REG_GETF(SYS_WDOG, WATCHDOG_CTRL_REG, WDOG_FREEZE_EN) &&
                !REG_GETF(SYS_WDOG, WATCHDOG_CTRL_REG, NMI_RST));

}

__RETAINED_CODE bool hw_watchdog_unfreeze(void)
{

        GPREG->RESET_FREEZE_REG = GPREG_RESET_FREEZE_REG_FRZ_SYS_WDOG_Msk;

        return (REG_GETF(SYS_WDOG, WATCHDOG_CTRL_REG, WDOG_FREEZE_EN) &&
                !REG_GETF(SYS_WDOG, WATCHDOG_CTRL_REG, NMI_RST));

}

HW_WDG_RESET hw_watchdog_is_irq_or_rst_gen(void)
{
        if (REG_GETF(SYS_WDOG, WATCHDOG_CTRL_REG, NMI_RST)) {
                return HW_WDG_RESET_RST;
        }

        return HW_WDG_RESET_NMI;
}

void hw_watchdog_register_int(hw_watchdog_interrupt_cb handler)
{
        int_handler = handler;
}

void hw_watchdog_unregister_int(void)
{
        int_handler = NULL;
}

__RETAINED_CODE void hw_watchdog_handle_int(unsigned long *exception_args)
{
#ifdef SEC_MODEN
#if defined(CONFIG_WD_MONITOR)
		check_monitor();
#endif

        if (dg_configIMAGE_SETUP == DEVELOPMENT_MODE) {
            printf("\r\n!!!hw_watchdog!!!\r\n");
            printf("R0  : 0x%08lx\r\n", exception_args[0]);
            printf("R1  : 0x%08lx\r\n", exception_args[1]);
            printf("R2  : 0x%08lx\r\n", exception_args[2]);
            printf("R3  : 0x%08lx\r\n", exception_args[3]);
            printf("R12 : 0x%08lx\r\n", exception_args[4]);
            printf("LR  : 0x%08lx\r\n", exception_args[5]);
            printf("PC  : 0x%08lx\r\n", exception_args[6]);
            printf("PSR : 0x%08lx\r\n", exception_args[7]);
        }


#endif


        hw_watchdog_unfreeze();

        // Reached this point due to a WDOG timeout
        uint16_t pmu_ctrl_reg = CRG_TOP->PMU_CTRL_REG;
        pmu_ctrl_reg |= ((1 << CRG_TOP_PMU_CTRL_REG_TIM_SLEEP_Pos)     |        /* turn off timer Power Domain */
                         (1 << CRG_TOP_PMU_CTRL_REG_COM_SLEEP_Pos)     |        /* turn off communication PD */
                         (1 << CRG_TOP_PMU_CTRL_REG_RADIO_SLEEP_Pos)   |        /* turn off radio PD */
                         (1 << CRG_TOP_PMU_CTRL_REG_PERIPH_SLEEP_Pos));         /* turn off peripheral PD */
        CRG_TOP->PMU_CTRL_REG = pmu_ctrl_reg;

#if (dg_configIMAGE_SETUP == DEVELOPMENT_MODE)
        ENABLE_DEBUGGER;

        if (exception_args != NULL) {
                *(volatile unsigned long *)(STATUS_BASE       ) = exception_args[0];    // R0
                *(volatile unsigned long *)(STATUS_BASE + 0x04) = exception_args[1];    // R1
                *(volatile unsigned long *)(STATUS_BASE + 0x08) = exception_args[2];    // R2
                *(volatile unsigned long *)(STATUS_BASE + 0x0C) = exception_args[3];    // R3
                *(volatile unsigned long *)(STATUS_BASE + 0x10) = exception_args[4];    // R12
                *(volatile unsigned long *)(STATUS_BASE + 0x14) = exception_args[5];    // LR
                *(volatile unsigned long *)(STATUS_BASE + 0x18) = exception_args[6];    // PC
                *(volatile unsigned long *)(STATUS_BASE + 0x1C) = exception_args[7];    // PSR
                *(volatile unsigned long *)(STATUS_BASE + 0x20) = (unsigned long)exception_args;    // Stack Pointer

                *(volatile unsigned long *)(STATUS_BASE + 0x24) = (*((volatile unsigned long *)(0xE000ED28)));    // CFSR
                *(volatile unsigned long *)(STATUS_BASE + 0x28) = (*((volatile unsigned long *)(0xE000ED2C)));    // HFSR
                *(volatile unsigned long *)(STATUS_BASE + 0x2C) = (*((volatile unsigned long *)(0xE000ED30)));    // DFSR
                *(volatile unsigned long *)(STATUS_BASE + 0x30) = (*((volatile unsigned long *)(0xE000ED3C)));    // AFSR
                *(volatile unsigned long *)(STATUS_BASE + 0x34) = (*((volatile unsigned long *)(0xE000ED34)));    // MMAR
                *(volatile unsigned long *)(STATUS_BASE + 0x38) = (*((volatile unsigned long *)(0xE000ED38)));    // BFAR
        }

        if (REG_GETF(CRG_TOP, SYS_STAT_REG, DBG_IS_ACTIVE)) {
                __BKPT(0);
        }
        else {
                while (1);
        }

#else // dg_configIMAGE_SETUP == DEVELOPMENT_MODE
        if (exception_args != NULL) {
                nmi_event_data[0] = NMI_MAGIC_NUMBER;
                nmi_event_data[1] = exception_args[0];          // R0
                nmi_event_data[2] = exception_args[1];          // R1
                nmi_event_data[3] = exception_args[2];          // R2
                nmi_event_data[4] = exception_args[3];          // R3
                nmi_event_data[5] = exception_args[4];          // R12
                nmi_event_data[6] = exception_args[5];          // LR
                nmi_event_data[7] = exception_args[6];          // PC
                nmi_event_data[8] = exception_args[7];          // PSR
        }

        // Wait for the reset to occur
        while (1);
#endif // dg_configIMAGE_SETUP == DEVELOPMENT_MODE
}

__RETAINED_CODE void NMI_HandlerC(unsigned long *exception_args);

void NMI_HandlerC(unsigned long *exception_args)
{
#if (dg_configFAULT_DEBUG_DUMP==1)
	save_reg(&ss_fault_register, exception_args);
    printf("\r\n!!!NMI_HandlerC!!!\r\n");
#endif

    sys_set_reset_reason(RESET_REASON_FAULT_WATCHDOG);

    if ( 0 /* int_handler */) {
    		printf("[%s %05d]\r\n",__func__,__LINE__);
            int_handler(exception_args);
    } else {
#if (dg_configFAULT_DEBUG_DUMP==1)
        OS_TASK current_task;
        current_task = OS_GET_CURRENT_TASK();
        char cnt=0;

    	char tasks2ble[128] = {0};
    	unsigned int nr = 0, nsize = sizeof(tasks2ble);
    	
        FAULT_header_addr = FAULT_get_header_addr();
        FAULT_regs_addr = FAULT_get_regs_addr();
        FAULT_ram_addr = FAULT_get_ram_addr();

        memset((void *)&sys_fault_header, 0x0, sizeof(struct __FAULT_header));
        sys_fault_header.is_valid = FAULT_MAGIC_NUMBER;
        sys_fault_header.fault_type = WATCHDOG_NMI;
        sys_fault_header.watchdog_args.Reg_R0  = exception_args[0];
        sys_fault_header.watchdog_args.Reg_R1  = exception_args[1];
        sys_fault_header.watchdog_args.Reg_R2  = exception_args[2];
        sys_fault_header.watchdog_args.Reg_R3  = exception_args[3];
        sys_fault_header.watchdog_args.Reg_R12 = exception_args[4];
        sys_fault_header.watchdog_args.Reg_LR  = exception_args[5];
        sys_fault_header.watchdog_args.Reg_PC  = exception_args[6];
        sys_fault_header.watchdog_args.Reg_PSR = exception_args[7];
        sys_fault_header.watchdog_args.SP      = (unsigned long) exception_args;
        sys_fault_header.watchdog_args.Reg_CFSR = (*((volatile unsigned long *)(0xE000ED28)));    // CFSR
        sys_fault_header.watchdog_args.Reg_HFSR = (*((volatile unsigned long *)(0xE000ED2C)));    // HFSR
        sys_fault_header.watchdog_args.Reg_DFSR = (*((volatile unsigned long *)(0xE000ED30)));    // DFSR
        sys_fault_header.watchdog_args.Reg_AFSR = (*((volatile unsigned long *)(0xE000ED3C)));    // AFSR
        sys_fault_header.watchdog_args.Reg_MMAR = (*((volatile unsigned long *)(0xE000ED34)));    // MMAR
        sys_fault_header.watchdog_args.Reg_BFAR = (*((volatile unsigned long *)(0xE000ED38)));    // BFAR
        sys_fault_header.watchdog_args.tasks_monitored_mask = 0;
        sys_fault_header.watchdog_args.notified_mask = 0;

        // Obtain the handle of a task from its name.
        //strncpy(sys_fault_header.task_name[0], pcTaskGetName(current_task), configMAX_TASK_NAME_LEN);
        printf("[%s %05d]\r\n",__func__,__LINE__);
        memset(sys_fault_header.task_info.task_name,0, sizeof(sys_fault_header.task_info.task_name));
        get_suspected_task(&sys_fault_header.task_info);
        for(int i=0; i < sys_fault_header.task_info.cnt; i++) {
                printf_debug_info(1,"[%s %05d] task[%d] : %s\r\n",__func__,__LINE__,i, sys_fault_header.task_info.task_name[i]);
                nr += snprintf(tasks2ble + nr, nsize - nr, "%s ", sys_fault_header.task_info.task_name[i]);
        }

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
        hw_watchdog_handle_int(exception_args);
    }
}

/**
 * \}
 * \}
 * \}
 */
