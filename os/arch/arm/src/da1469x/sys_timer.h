/**
 * \addtogroup MID_SYS_SERVICES
 * \{
 * \addtogroup SYS_TIMER System Timer
 *
 * \brief System timer
 *
 * \{
 */

/**
 ****************************************************************************************
 *
 * @file sys_timer.h
 *
 * @brief System timer header file.
 *
 * Copyright (C) 2017-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef SYS_TIMER_H_
#define SYS_TIMER_H_


#include "sdk_defs.h"

/**
 * \brief Start the OS timer.
 *
 * \param period The timer period in LP clock cycles
 *
 * \note This function must be called from vPortSetupTimerInterrupt()
 *
 */
void sys_timer_start(uint32_t period);

/*
 * \brief Stop the OS timer.
 *
 */
void sys_timer_stop(void);

/**
 * \brief Set OS timer trigger
 *
 * \param[in] trigger The new timer trigger
 */
void sys_timer_set_trigger(uint32_t trigger);

/**
 * \brief Get OS timer offset in the tick period
 *
 * \return The offset in the tick period
 */
uint32_t sys_timer_get_tick_offset(void);


/**
 * \brief Get timestamp value.
 *
 * \return The current timestamp. This is expressed in ticks of the clock that clocks OS timer
 *         (e.g. XTAL32K). For example, if XTAL32K drives OS timer, each timestamp tick will be
 *         1000000 / 32768 = 30.5uS
 *
 * \warning This function is called only from OS Tasks.
 *
 */
uint64_t sys_timer_get_timestamp(void);

/**
 * \brief Get timestamp value.
 *
 * \return The current timestamp. This is expressed in ticks of the clock that clocks OS timer
 *         (e.g. XTAL32K). For, example, if XTAL32K drives OS timer, each timestamp tick will be
 *         1000000 / 32768 = 30.5uS
 *
 * \warning This function is called only when interrupts are disabled.
 *
 */
__RETAINED_CODE uint64_t sys_timer_get_timestamp_fromISR(void);

#endif /* SYS_TIMER_H_ */

/**
\}
\}
*/
