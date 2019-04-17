/**
 ****************************************************************************************
 *
 * @file sys_watchdog.c
 *
 * @brief Watchdog Service
 *
 * Copyright (C) 2015-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sdk_defs.h"
#include "hw_watchdog.h"

#include "sys_watchdog.h"
#if dg_configUSE_WDOG

//#include "timers.h"
//#include "LogUtil.h"

#if (dg_configFAULT_DEBUG_DUMP==1)
#include "hw_hard_fault.h"
#endif

/* mutex to synchronize access to wdog data */
//__RETAINED static OS_MUTEX lock;

/* number of tasks registered for wdog */
__RETAINED static int8_t max_task_id;

#if (dg_configWDOG_EXTENDED == 1)
/* bitmask of tasks identifiers which are registered */
__RETAINED static uint64_t tasks_mask;

/* bitmask of tasks identifiers which are monitored (registered and not suspended) */
__RETAINED static uint64_t tasks_monitored_mask;

/* bitmask of tasks which notified during last period */
__RETAINED volatile static uint64_t notified_mask;
#else
/* bitmask of tasks identifiers which are registered */
__RETAINED static uint32_t tasks_mask;

/* bitmask of tasks identifiers which are monitored (registered and not suspended) */
__RETAINED static uint32_t tasks_monitored_mask;

/* bitmask of tasks which notified during last period */
__RETAINED volatile static uint32_t notified_mask;
#endif

/* allowed latency set by tasks, if any */
__RETAINED static uint8_t tasks_latency[dg_configWDOG_MAX_TASKS_CNT];

//#if (dg_configIMAGE_SETUP == DEVELOPMENT_MODE)
/* handles of monitored tasks */
__RETAINED OS_TASK tasks_handle[dg_configWDOG_MAX_TASKS_CNT];
//#endif

/* the wdog id of the IDLE task */
__RETAINED_RW static int8_t idle_task_id = -1;

#if dg_configWDOG_NOTIFY_TRIGGER_TMO
#if (dg_configWDOG_EXTENDED == 1)
/* bitmask of tasks which requested notification trigger from common timer */
__RETAINED static uint64_t tasks_notify_mask;
#else
/* bitmask of tasks which requested notification trigger from common timer */
__RETAINED static uint32_t tasks_notify_mask;
#endif

/* task handle for tasks which requested notification trigger from common timer */
__RETAINED static OS_TASK tasks_notify_handle[dg_configWDOG_MAX_TASKS_CNT];

/* timer handle for common notification trigger */
__RETAINED static OS_TIMER tasks_notify_timer;
#endif

#if defined(CONFIG_WD_MONITOR)

#define MAX_TASK_SIZE       dg_configWDOG_MAX_TASKS_CNT
#define MAX_BUF_SIZE			dg_configWDOG_MAX_TASKS_CNT

struct dog_t {
	uint8_t     id;
	char        task_name[MAX_TASK_NAME_LEN + 1];
};

struct dog_monitor_t {
	struct dog_t    list[MAX_TASK_SIZE];
	uint32_t        mask;
	uint8_t         count;
};

static struct dog_monitor_t  dog_monitor = {0};

void init_monitor(void) {

}
    
void add_monitor(struct dog_t dog) {
	if ( dog_monitor.count >= MAX_TASK_SIZE ) {
		return;
	}

	memcpy( (void *)&dog_monitor.list[dog_monitor.count], (void *)&dog, sizeof(dog));
	dog_monitor.count++;
	dog_monitor.mask |= (1 << dog.id);
}

void getFormat(uint8_t id, char buf[]) {
	for (int i = MAX_TASK_SIZE -1; i >= 0; i--) {
		if (i == id) {
			buf[MAX_TASK_SIZE -1 - i] = 'X';
		} else {
			buf[MAX_TASK_SIZE -1 - i] = '-';
		}
	}
}

void check_monitor(void) {
	int id;
//	char buf[MAX_BUF_SIZE+1] = { 0, };

#if (dg_configWDOG_EXTENDED == 1)
//	printf("                    ---6---------5---------4---------3---------2---------1---------0\r\n");
	printf("                       registered     suspended       notified      suspected\r\n");
	for (id = 0; id < dog_monitor.count; id++) {
//		memset(buf, 0, MAX_BUF_SIZE+1);
//		getFormat(id, buf);
		printf("[%-16s]        [%c]            [%c]            [%c]            [%c]\r\n", dog_monitor.list[id].task_name,
				((tasks_mask >> id) & 0x01) ? 'O' : 'X',
				((tasks_monitored_mask >> id) & 0x01) ? 'X' : 'O',
				((notified_mask >> id) & 0x01) ? 'O' : 'X',
				((tasks_mask >> id) & 0x01) & ((tasks_monitored_mask >> id) & 0x01) & !((notified_mask >> id) & 0x01) ? 'v' : ' ');
	}
#else
	printf("                    -3---------2---------1---------0\r\n");
	for (id = 0; id < dog_monitor.count; id++) {
		memset(buf, 0, MAX_BUF_SIZE+1);
		getFormat(id, buf);
		printf("[%-16s : %s] = [%c]\r\n", dog_monitor.list[id].task_name, buf,
				((notified_mask >> id) & 0x01) ? 'O' : 'X');
	}
#endif
}

void get_suspected_task(struct task_info_t *task_info) {
	int id;

	task_info->cnt = 0;
	for (id = 0; id < dog_monitor.count; id++) {
		if ( ((tasks_monitored_mask >> id & 0x01) == 0) || ((notified_mask >> id) & 0x01) )
			continue;

//		printf_debug_info(0,"[%s %05d] count=%d\r\n",__func__,__LINE__, task_info->cnt);
		if (task_info->cnt >= MAX_SUSPECTED_TASK_CNT) {
		       task_info->cnt = MAX_SUSPECTED_TASK_CNT;
			return;
		}
		strncpy(task_info->task_name[task_info->cnt], dog_monitor.list[id].task_name, configMAX_TASK_NAME_LEN );
		task_info->task_name[task_info->cnt][configMAX_TASK_NAME_LEN] = '\0';
		//printf("[%s %05d] suspected_task%d=[%s]\r\n",__func__,__LINE__,task_info->cnt, task_info->task_name[task_info->cnt]);

//#if (INCLUDE_xTaskGetHandle == 1)
//			vTaskGetCallStackWithName( task_info->task_name[task_info->cnt] ); // for print current task call stack;
//#endif

                for(int i = 0; i < dg_configWDOG_MAX_TASKS_CNT; i++)
                {
                        if(strcmp(task_info->task_name[task_info->cnt], pcTaskGetName(tasks_handle[i])) == 0) {
                                vTaskGetCallStack(tasks_handle[i], 0);
                                break;
                        }
                }

                task_info->cnt++;
        }
}
#endif	/* CONFIG_WD_MONITOR */



/* Store the last reload value set to the watchdog */
__RETAINED static uint16_t watchdog_reload_value;

#define VALIDATE_ID(id) \
        do { \
                if ((id) < 0 || (id) >= dg_configWDOG_MAX_TASKS_CNT) { \
                        OS_ASSERT(0); \
                        return; \
                } \
        } while (0)

#if dg_configUSE_WDOG
__RETAINED_CODE static void reset_watchdog(void)
{
	notified_mask = 0;

        sys_watchdog_set_pos_val(dg_configWDOG_RESET_VALUE);
#if defined(CONFIG_WD_MONITOR)
        init_monitor();
#endif        
}

__RETAINED_CODE static void watchdog_cb(unsigned long *exception_args)
{
#if (dg_configFAULT_DEBUG_DUMP==1)
        save_reg(&ss_fault_register, exception_args);
#endif
#if (dg_configWDOG_EXTENDED == 1)
        uint64_t tmp_mask = tasks_monitored_mask;
        uint64_t latency_mask = 0;
#else
        uint32_t tmp_mask = tasks_monitored_mask;
        uint32_t latency_mask = 0;
#endif
        int i;

        /*
         * watchdog is reset immediately when we detect that all tasks notified during period so
         * no need to check this here
         *
         * but if we're here, then check if some tasks have non-zero latency and remove them from
         * notify mask check and also decrease latency for each task
         */
        for (i = 0; i <= max_task_id; i++) {
                if (tasks_latency[i] == 0) {
                        continue;
                }

                tasks_latency[i]--;
                latency_mask |= (1 << i);
        }

        /*
         * check if all remaining tasks notified and reset hw_watchdog in such case
         */
        tmp_mask &= ~latency_mask;
        if ((notified_mask & tmp_mask) == tmp_mask)
                goto reset_wdog;

#if (dg_configIMAGE_SETUP == DEVELOPMENT_MODE)
        else {
                volatile unsigned n;
                printf("watch dog monitor tasks\r\n");

                for(n = 0; n <= max_task_id; n++) {
                        if((tmp_mask >> n) & 0x01) {
                                printf("%s ( %c )\r\n", pcTaskGetName( tasks_handle[n] ),
                                        (((notified_mask >> n) & 0x01)? 'O' : 'X'));
                        }
                }
        }
#endif

#if (dg_configFAULT_DEBUG_DUMP==1)
        FAULT_header_addr = FAULT_get_header_addr();
        FAULT_regs_addr = FAULT_get_regs_addr();
        FAULT_ram_addr = FAULT_get_ram_addr();

        memset((void *)&sys_fault_header, 0x0, sizeof(sys_fault_header));
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
        sys_fault_header.watchdog_args.tasks_monitored_mask = tmp_mask;
        sys_fault_header.watchdog_args.notified_mask = notified_mask;

        // Obtain the handle of a task from its name.
        memset(sys_fault_header.task_info.task_name,0, sizeof(sys_fault_header.task_info.task_name));
        get_suspected_task(&sys_fault_header.task_info);
        for(int i=0; i < sys_fault_header.task_info.cnt; i++) {
//                printf_debug_info(0,"[%s %05d] task[%d] : %s\r\n",__func__,__LINE__,i, sys_fault_header.task_info.task_name[i]);
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

#ifdef SEC_MODEN
#if (dg_configWDOG_EXTENDED == 1)
        if (dg_configIMAGE_SETUP == DEVELOPMENT_MODE) {
            printf("notified_mask=0x%16lx\r\n", notified_mask);
            printf("monitoring_mask=0x%16lx\r\n", tmp_mask);
        }
#else
        if (dg_configIMAGE_SETUP == DEVELOPMENT_MODE) {
        printf_debug_info(0,"notified_mask=0x%08lx\r\n", notified_mask);
        printf_debug_info(0,"monitoring_mask=0x%08lx\r\n", tmp_mask);
        }
#endif
#endif
        /*
         * latency for all tasks expired and some of them still did not notify sys_watchdog
         * we'll let watchdog reset the system
         *
         * note that hw_watchdog_handle_int() never returns
         */
        hw_watchdog_handle_int(exception_args);

reset_wdog:
        reset_watchdog();
}
#endif

#if dg_configWDOG_NOTIFY_TRIGGER_TMO
static void watchdog_auto_notify_cb(OS_TIMER timer)
{
        int i;

        OS_MUTEX_GET(lock, OS_MUTEX_FOREVER);

        for (i = 0; i <= max_task_id; i++) {
                if (tasks_notify_handle[i]) {
                        OS_TASK_NOTIFY(tasks_notify_handle[i], SYS_WATCHDOG_TRIGGER, OS_NOTIFY_SET_BITS);
                }
        }

        OS_MUTEX_PUT(lock);
}
#endif

#endif

void sys_watchdog_init(void)
{
#if dg_configUSE_WDOG
        max_task_id = 0;
        notified_mask = 0;

        sys_watchdog_set_pos_val(dg_configWDOG_RESET_VALUE);

        OS_MUTEX_CREATE(lock);

#if dg_configWDOG_NOTIFY_TRIGGER_TMO
        tasks_notify_timer = OS_TIMER_CREATE("wdog",
                        OS_MS_2_TICKS(dg_configWDOG_NOTIFY_TRIGGER_TMO),
                        pdTRUE, NULL, watchdog_auto_notify_cb);
#endif
#endif
}

int8_t sys_watchdog_register(bool notify_trigger)
{

#if defined(CONFIG_WD_MONITOR)
    char *ptr = NULL;
    struct dog_t dog;
#endif

#if dg_configUSE_WDOG
        int8_t id = 0;

        OS_MUTEX_GET(lock, OS_MUTEX_FOREVER);

        while (tasks_mask & (1 << id)) {
                id++;
        }

        if (id >= dg_configWDOG_MAX_TASKS_CNT) {
                /* Don't allow registration of more than dg_configWDOG_MAX_TASKS_CNT */
                OS_ASSERT(0);
                return -1;
        }

        	tasks_mask |= (1 << id);
        	tasks_monitored_mask |= (1 << id);

#if (dg_configIMAGE_SETUP == DEVELOPMENT_MODE)
        tasks_handle[id] = OS_GET_CURRENT_TASK();
#endif

        if (id > max_task_id) {
                max_task_id = id;
        }

        if (id == 0) {
                hw_watchdog_register_int(watchdog_cb);
        }

#if dg_configWDOG_NOTIFY_TRIGGER_TMO
        if (notify_trigger) {
                /* this is first task to request trigger - start timer */
                if (!tasks_notify_mask) {
                        OS_TIMER_START(tasks_notify_timer, OS_TIMER_FOREVER);
                }
                tasks_notify_mask |= (1 << id);
                tasks_notify_handle[id] = OS_GET_CURRENT_TASK();
        }
#endif

#if defined(CONFIG_WD_MONITOR)
        ptr = pcTaskGetName( tasks_handle[id] );
        if (ptr != NULL ) {
            dog.id = id;
            strncpy(dog.task_name, ptr, strlen(ptr));
            dog.task_name[strlen(ptr)] = '\0';
            add_monitor(dog);
        } else {
            printf_ex(DBG_MAIN,DBG_PATH_UART,"[%s %06d] couldn't get taskname\r\n",__FILE__,__LINE__);
        }
#endif

        OS_MUTEX_PUT(lock);

        return id;
#else
        return 0;
#endif
}

void sys_watchdog_unregister(int8_t id)
{
#if dg_configUSE_WDOG
#if (dg_configWDOG_EXTENDED == 1)
	uint64_t tmp_mask;
#else
        uint32_t tmp_mask;
#endif
        int8_t new_max = 0;

        VALIDATE_ID(id);

        OS_MUTEX_GET(lock, OS_MUTEX_FOREVER);

        tasks_mask &= ~(1 << id);
        tasks_monitored_mask &= ~(1 << id);
        tasks_latency[id] = 0;

#if (dg_configIMAGE_SETUP == DEVELOPMENT_MODE)
        tasks_handle[id] = NULL;
#endif

#if dg_configWDOG_NOTIFY_TRIGGER_TMO
        tasks_notify_handle[id] = 0;
        tasks_notify_mask &= ~(1 << id);
        /* this was last task to request trigger - stop timer */
        if (!tasks_notify_mask) {
                OS_TIMER_STOP(tasks_notify_timer, OS_TIMER_FOREVER);
        }
#endif

        /* recalculate max task id */
        tmp_mask = tasks_mask;
        while (tmp_mask) {
                tmp_mask >>= 1;
                new_max++;
        }

        max_task_id = new_max;

        OS_MUTEX_PUT(lock);
#endif
}

int8_t sys_watchdog_get_reg_id(OS_TASK task_handle)
{
        int8_t id = -1;

#if dg_configUSE_WDOG
        while (tasks_handle[id] != task_handle && id < dg_configWDOG_MAX_TASKS_CNT) {
                id++;
        }
#endif
        return id;
}

void sys_watchdog_configure_idle_id(int8_t id)
{
#if dg_configUSE_WDOG
        idle_task_id = id;

#if (dg_configIMAGE_SETUP == DEVELOPMENT_MODE)
        char *ptr = NULL;

        tasks_handle[id] = xTaskGetIdleTaskHandle();
        ptr = pcTaskGetName(tasks_handle[id]);
        strncpy(dog_monitor.list[id].task_name, ptr, strlen(ptr));
        dog_monitor.list[id].task_name[strlen(ptr)] = '\0';
#endif
#endif
}

void sys_watchdog_suspend(int8_t id)
{
#if dg_configUSE_WDOG
        VALIDATE_ID(id);

        OS_MUTEX_GET(lock, OS_MUTEX_FOREVER);

        tasks_monitored_mask &= ~(1 << id);

        OS_MUTEX_PUT(lock);
#endif
}

#if dg_configUSE_WDOG
__STATIC_INLINE void resume_monitoring(int8_t id)
{
        tasks_monitored_mask |= (1 << id);
        tasks_monitored_mask &= tasks_mask;
}
#endif

void sys_watchdog_resume(int8_t id)
{
#if dg_configUSE_WDOG
        VALIDATE_ID(id);

        OS_MUTEX_GET(lock, OS_MUTEX_FOREVER);

        resume_monitoring(id);

        OS_MUTEX_PUT(lock);
#endif
}

#if dg_configUSE_WDOG
__STATIC_INLINE void notify_about_task(int8_t id)
{
    /* Make sure that the requested task is one of the watched tasks */
    OS_ASSERT(tasks_mask & (1 << id ));

    if (tasks_mask & (1 << id)) {
            notified_mask |= (1 << id);

            /*
             * we also reset latency here because it's ok for app to notify before latency
             * expired, but it should start with zero latency for next notification interval
             */
            tasks_latency[id] = 0;

            if ((notified_mask & tasks_monitored_mask) == tasks_monitored_mask) {
                    reset_watchdog();
            }
        }
}

__STATIC_INLINE void notify_idle(int8_t id)
{
        /* Notify the IDLE task every time one of the monitored tasks notifies the service. */
        if ((id != idle_task_id) && (idle_task_id != -1)) {
                sys_watchdog_notify(idle_task_id);
        }
}
#endif

void sys_watchdog_notify(int8_t id)
{
#if dg_configUSE_WDOG
        VALIDATE_ID(id);

        OS_MUTEX_GET(lock, OS_MUTEX_FOREVER);

        notify_about_task(id);

        OS_MUTEX_PUT(lock);

        notify_idle(id);
#endif
}

void sys_watchdog_notify_and_resume(int8_t id)
{
#if dg_configUSE_WDOG
        VALIDATE_ID(id);

        OS_MUTEX_GET(lock, OS_MUTEX_FOREVER);

        resume_monitoring(id);
        notify_about_task(id);

        OS_MUTEX_PUT(lock);

        notify_idle(id);
#endif
}

void sys_watchdog_set_latency(int8_t id, uint8_t latency)
{
#if dg_configUSE_WDOG
        VALIDATE_ID(id);

        OS_MUTEX_GET(lock, OS_MUTEX_FOREVER);

        tasks_latency[id] = latency;

        OS_MUTEX_PUT(lock);
#endif
}

__RETAINED_CODE bool sys_watchdog_monitor_mask_empty()
{
#if dg_configUSE_WDOG
        return ((idle_task_id != -1) && (tasks_monitored_mask == (1 << idle_task_id)));
#else
        return true;
#endif
}

__RETAINED_CODE void sys_watchdog_set_pos_val(uint16_t value)
{
#if (dg_configDEVICE == DEVICE_D2522 && dg_configUSE_WDOG)
        if (REG_GETF(SYS_WDOG, WATCHDOG_CTRL_REG, WRITE_BUSY)) {
                // WATCHDOG is still busy writing the previous value

                // Previous value was written using sys_watchdog_set_pos_val()
                ASSERT_WARNING(watchdog_reload_value != 0);

                if (watchdog_reload_value == value) {
                       /* The previous value was the same. No need to wait for
                          WRITE_BUSY and write it again. */
                        return;
                }
        }

        watchdog_reload_value = value;
#endif
        hw_watchdog_set_pos_val(value);
}

uint16_t sys_watchdog_get_val(void)
{
#if (dg_configDEVICE == DEVICE_D2522 && dg_configUSE_WDOG)
        if (REG_GETF(SYS_WDOG, WATCHDOG_CTRL_REG, WRITE_BUSY)) {
                // WATCHDOG is still busy writing the previous value

                // Previous value was written using sys_watchdog_set_pos_val()
                ASSERT_WARNING(watchdog_reload_value != 0);

                return watchdog_reload_value;
        }
#endif
        return hw_watchdog_get_val();
}
