/**
 * \addtogroup PLA_DRI_PER_ANALOG
 * \{
 * \addtogroup HW_HARD_FAULT Hard Fault Handler
 * \{
 * \brief Generic Exception Handlers
 */

/**
 ****************************************************************************************
 *
 * @file hw_hard_fault.h
 *
 * @brief Hard-Fault Handler include file.
 *
 * Copyright (C) 2015-2017 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef HW_HARD_FAULT_H_
#define HW_HARD_FAULT_H_


#include <stdbool.h>
#include <stdint.h>

#include "sdk_defs.h"

#define HARDFAULT_MAGIC_NUMBER          0xBADC0FFE

/**
 * \brief Holds the stack contents when a hard-fault occurs.
 *
 * \details The stack contents are copied at this variable when a hard-fault occurs. The first
 *        position is marked with a special "flag" (0xBADC0FFE) to indicate that the data that
 *        follow are valid.
 */
extern volatile uint32_t hardfault_event_data[9];

#if (dg_configFAULT_DEBUG_DUMP==1)
#include "hw_clk.h"
#include "qspi_automode.h"
#include "hw_qspi.h"
#include dg_configQSPIC2_DEV_HEADER_FILE

#define FAULT_MAGIC_NUMBER                 0xBAD0C0DE

#define FAULT_RAM_SIZE                     0x80000 //512K
#define FAULT_REGS_SIZE                    6720    //This is the size of dumped REGs.

#define MAX_SUSPECTED_TASK_CNT             3

struct task_info_t {
	unsigned char cnt;
	char task_name[MAX_SUSPECTED_TASK_CNT][configMAX_TASK_NAME_LEN+1];
};

typedef enum __FAULT_type {
        HARD_FAULT = 0x10,
        MEM_FAULT,
        BUS_FAULT,
        USAGE_FAULT,
        WATCHDOG_NMI,
        USER_ASSERT,
        SYS_ASSERT,
        UESR_MALLOC,
        USER_STACKOVER,
        MAX_FAULT_SIZE
} FAULT_type;

typedef struct __FAULT_header {
        uint32_t        is_valid;

        FAULT_type      fault_type;

        //start of kookjin code
        struct {
        	char	filename[32];
        	int		linenumber;
        } sys_assert_args;

        struct {
        	char	taskName[32];
        } user_malloc_args;

        struct {
        	char	taskName[32];
        } user_stackover_args;
        //end of kookjin code

        struct{
                char file[32];   //fault file
                int line;   //fault line
        }user_assert_args;

        struct {
                unsigned long Reg_R0;      // R0
                unsigned long Reg_R1;      // R1
                unsigned long Reg_R2;      // R2
                unsigned long Reg_R3;      // R3
                unsigned long Reg_R12;     // R12
                unsigned long Reg_LR;      // LR
                unsigned long Reg_PC;      // PC
                unsigned long Reg_PSR;     // PSR
                unsigned long SP;          // Stack Pointer

                unsigned long Reg_CFSR;    // CFSR
                unsigned long Reg_HFSR;    // HFSR
                unsigned long Reg_DFSR;    // DFSR
                unsigned long Reg_AFSR;    // AFSR
                unsigned long Reg_MMAR;    // MMAR
                unsigned long Reg_BFAR;    // BFAR
        } hard_fault_args;

        struct {
                uint8_t       fault_status_reg;
                uint32_t      fault_addr;
        } mem_fault_args;

        struct {
                uint8_t       fault_status_reg;
                uint32_t      fault_addr;
        } bus_fault_args;

        struct {
                uint16_t      fault_status_reg;
        } usage_fault_args;

        struct {
                unsigned long Reg_R0;      // R0
                unsigned long Reg_R1;      // R1
                unsigned long Reg_R2;      // R2
                unsigned long Reg_R3;      // R3
                unsigned long Reg_R12;     // R12
                unsigned long Reg_LR;      // LR
                unsigned long Reg_PC;      // PC
                unsigned long Reg_PSR;     // PSR
                unsigned long SP;          // Stack Pointer

                unsigned long Reg_CFSR;    // CFSR
                unsigned long Reg_HFSR;    // HFSR
                unsigned long Reg_DFSR;    // DFSR
                unsigned long Reg_AFSR;    // AFSR
                unsigned long Reg_MMAR;    // MMAR
                unsigned long Reg_BFAR;    // BFAR
#if (dg_configWDOG_EXTENDED == 1)
                /* bitmask of tasks identifiers which are monitored (registered and not suspended) */
                uint64_t tasks_monitored_mask;

                /* bitmask of tasks which notified during last period */
                uint64_t notified_mask;
#else
                /* bitmask of tasks identifiers which are monitored (registered and not suspended) */
                uint32_t tasks_monitored_mask;

                /* bitmask of tasks which notified during last period */
                uint32_t notified_mask;
#endif
        } watchdog_args;

        struct task_info_t task_info;

}FAULT_header;

//samsung falut header for ramdump
#define RAM_DUMP_ADDR 0x20000000
typedef struct SS_FAULT_REG{
                unsigned long Reg_R0;      // R0
                unsigned long Reg_R1;      // R1
                unsigned long Reg_R2;      // R2
                unsigned long Reg_R3;      // R3
                unsigned long Reg_R4;      // R4
                unsigned long Reg_R5;      // R5
                unsigned long Reg_R6;      // R6
                unsigned long Reg_R7;      // R7
                unsigned long Reg_R8;      // R8
                unsigned long Reg_R9;      // R9
                unsigned long Reg_R10;     // R10
                unsigned long Reg_R11;     // R11
                unsigned long Reg_R12;     // R12
                unsigned long SP;          // Stack Pointer  R13
                unsigned long Reg_LR;      // LR   R14
                unsigned long Reg_PC;      // PC   R15
                unsigned long Reg_PSR;     // PSR  xpsr

                unsigned long Reg_CONTORL;
                unsigned long Reg_MSP;
                unsigned long Reg_PSP;

                unsigned long Reg_real_PC;    // CFSR
                unsigned long Reg_real_LR;    // HFSR
                unsigned long Reg_real_SP;    // DFSR
} SS_FAULT_REG;

SS_FAULT_REG ss_fault_register;

#if (dg_configCODE_LOCATION == NON_VOLATILE_IS_FLASH)
__attribute__((section("text_retained")))
#endif
void save_reg(SS_FAULT_REG *core_reg, unsigned long *fault_args);

/////

extern void* FAULT_header_addr;
extern void* FAULT_regs_addr;
extern void* FAULT_ram_addr;

extern FAULT_header sys_fault_header;

#if (dg_configCODE_LOCATION == NON_VOLATILE_IS_FLASH)
__attribute__((section("text_retained")))
#endif
uint32_t FAULT_memcpy32(void* dest, void* src, uint32_t len);

#if (dg_configCODE_LOCATION == NON_VOLATILE_IS_FLASH)
__attribute__((section("text_retained")))
#endif
uint32_t FAULT_memcpy(void* dest, void* src, uint32_t len);

#if (dg_configCODE_LOCATION == NON_VOLATILE_IS_FLASH)
__attribute__((section("text_retained")))
#endif
uint32_t FAULT_Regs(void* dest);

#if (dg_configCODE_LOCATION == NON_VOLATILE_IS_FLASH)
__attribute__((section("text_retained")))
#endif
bool FAULT_is_valid_dump(void);

#if (dg_configCODE_LOCATION == NON_VOLATILE_IS_FLASH)
__attribute__((section("text_retained")))
#endif
void* FAULT_get_header_addr(void);

#if (dg_configCODE_LOCATION == NON_VOLATILE_IS_FLASH)
__attribute__((section("text_retained")))
#endif
void* FAULT_get_regs_addr(void);

#if (dg_configCODE_LOCATION == NON_VOLATILE_IS_FLASH)
__attribute__((section("text_retained")))
#endif
void* FAULT_get_ram_addr(void);

#endif /* dg_configFAULT_DEBUG_DUMP */

#endif /* HW_HARD_FAULT_H_ */
/**
 * \}
 * \}
 */
