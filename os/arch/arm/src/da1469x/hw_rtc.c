/**
 * \addtogroup BSP
 * \{
 * \addtogroup DEVICES
 * \{
 * \addtogroup Real Time Clock (RTC)
 * \{
 */

/**
 ****************************************************************************************
 *
 * @file hw_rtc.c
 *
 * @brief Implementation of the Real Time Clock Low Level Driver.
 *
 * Copyright (C) 2017-2018 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#include <tinyara/config.h>

#include <stdio.h>

#include "sdk_defs.h"
#include "hw_rtc.h"
#ifdef SEC_MODEN
#include "hw_pdc.h"
#endif

#if (dg_configSYSTEMVIEW == 1)
#  include "SEGGER_SYSVIEW_FreeRTOS.h"
#else
#  define SEGGER_SYSTEMVIEW_ISR_ENTER()
#  define SEGGER_SYSTEMVIEW_ISR_EXIT()
#endif /* (dg_configSYSTEMVIEW == 1) */

static hw_rtc_interrupt_cb rtc_interrupt_cb;

#ifdef SEC_MODEN
/*
 * Timezone parameters. Hold values Timezone.
 */
static int32_t current_timezone = 0;
 
/*
 * Macro for enabling the RTC alarm demonstration. Valid values are:
 *
 * 1 --> Enables RTC alarm events
 * 0 --> Disabled RTC alarm events
 *
 */
#define ENABLE_RTC_ALARM_EVENT         (0)

bool rtcInitFlag = false;

#endif

/**
 * \brief Converts time from decimal to binary-coded decimal (BCD)
 *
 * \param[in] time pointer to RTC time in decimal number format
 *
 * \return time in BCD format
 */

#ifdef SEC_MODEN
static uint32_t time_to_bcd(const rtc_time_t *time) 
#else
static uint32_t time_to_bcd(const rtc_time *time)
#endif
{
        uint32_t time_bcd;

        time_bcd = ((time->hsec % 10) << 0);
        time_bcd += ((time->hsec / 10) << 4);

        time_bcd += ((time->sec % 10) << 8);
        time_bcd += ((time->sec / 10) << 12);

        time_bcd += ((time->minute % 10) << 16);
        time_bcd += ((time->minute / 10) << 20);

        if (time->hour_mode == RTC_24H_CLK) {
                hw_rtc_set_hour_clk_mode(RTC_24H_CLK); //24hr mode
                time_bcd += ((time->hour % 10) << 24);
                time_bcd += (( time->hour / 10) << 28);
        } else if (time->hour_mode == RTC_12H_CLK) {
                hw_rtc_set_hour_clk_mode(RTC_12H_CLK); //12hr mode
                time_bcd += ((time->hour % 10 ) << 24);
                time_bcd += ((time->hour / 10) << 28);
                time_bcd += ((time->pm_flag) << 30);
        } else {
                ASSERT_WARNING(0);//Invalid argument
        }

        return time_bcd;
}

/**
 * \brief Converts alarm time from decimal to binary-coded decimal (BCD)
 *
 * \param[in] time pointer to alarm time in decimal number format
 *
 * \return time in BCD format
 */
#ifdef SEC_MODEN 
static uint32_t alarm_time_to_bcd(const rtc_time_t *time)
#else
static uint32_t alarm_time_to_bcd(const rtc_time *time)
#endif
{
        uint32_t time_bcd;

        time_bcd = ((time->hsec % 10) << 0);
        time_bcd += ((time->hsec / 10) << 4);

        time_bcd += ((time->sec % 10) << 8);
        time_bcd += ((time->sec / 10) << 12);

        time_bcd += ((time->minute % 10) << 16);
        time_bcd += ((time->minute / 10) << 20);

        if (hw_rtc_get_hour_clk_mode() == RTC_24H_CLK) {
                time_bcd += ((time->hour % 10) << 24);
                time_bcd += (( time->hour / 10) << 28);
        } else if (hw_rtc_get_hour_clk_mode() == RTC_12H_CLK) {
                time_bcd += ((time->hour % 10 ) << 24);
                time_bcd += ((time->hour / 10) << 28);
                time_bcd += ((time->pm_flag) << 30);
        } else {
                ASSERT_WARNING(0);//Invalid argument
        }

        return time_bcd;
}

/**
 * \brief Converts Calendar date from decimal to binary-coded decimal (BCD)
 *
 * \param[in] clndr pointer to RTC calendar in decimal number format
 *
 * \return calendar date in BCD format
 */
static uint32_t calendar_to_bcd(const rtc_calendar *clndr)
{
        uint32_t clndr_bcd;

        clndr_bcd = clndr->wday & 0x7;

        if (clndr->month > 9) {
                clndr_bcd += (0x80 + ((clndr->month-10) << 3));
        } else {
                clndr_bcd += (clndr->month << 3);
        }

        clndr_bcd += ((clndr->mday % 10) << 8);
        clndr_bcd += ((clndr->mday / 10) << 12);

        clndr_bcd += (((clndr->year % 100) % 10) << 16);
        clndr_bcd += (((clndr->year % 100) / 10) << 20);

        clndr_bcd += (((clndr->year/100) % 10) << 24);
        clndr_bcd += (((clndr->year/100) / 10) << 28);

        return clndr_bcd;
}

/**
 * \brief Converts alarm Calendar date from decimal to binary-coded decimal (BCD)
 *
 * \param[in] clndr pointer to RTC alarm calendar in decimal number format
 *
 * \return alarm calendar date in BCD format
 */
static uint32_t alarm_calendar_to_bcd(const rtc_alarm_calendar *clndr)
{
        uint32_t clndr_bcd;

        if (clndr->month > 9) {
                clndr_bcd = (0x80 + ((clndr->month-10) << 3));
        } else {
                clndr_bcd = (clndr->month << 3);
        }

        clndr_bcd += ((clndr->mday % 10) << 8);
        clndr_bcd += ((clndr->mday / 10) << 12);

        return clndr_bcd;
}

/**
 * \brief Converts RTC time from binary-coded decimal (BCD) to decimal
 *
 * \param[in] time_bcd  time in BCD
 * \param[out] time pointer to RTC time in decimal number format
 *
 */
#ifdef SEC_MODEN
static void bdc_to_time(uint32_t time_bcd, rtc_time_t *time)
#else
static void bdc_to_time(uint32_t time_bcd, rtc_time *time)
#endif
{
        time->pm_flag = (time_bcd & RTC_RTC_TIME_REG_RTC_TIME_PM_Msk) >> 30;
        time->hour = (((time_bcd & RTC_RTC_TIME_REG_RTC_TIME_HR_T_Msk) >> 28) * 10) +  ((time_bcd & RTC_RTC_TIME_REG_RTC_TIME_HR_U_Msk) >> 24);
        time->minute = (((time_bcd & RTC_RTC_TIME_REG_RTC_TIME_M_T_Msk) >> 20) * 10) +  ((time_bcd & RTC_RTC_TIME_REG_RTC_TIME_M_U_Msk) >> 16);
        time->sec = (((time_bcd & RTC_RTC_TIME_REG_RTC_TIME_S_T_Msk) >> 12) * 10) +  ((time_bcd & RTC_RTC_TIME_REG_RTC_TIME_S_U_Msk) >> 8);
        time->hsec = (((time_bcd & RTC_RTC_TIME_REG_RTC_TIME_H_T_Msk) >> 4) * 10) +  (time_bcd & RTC_RTC_TIME_REG_RTC_TIME_H_U_Msk);
}

/**
 * \brief Converts Calendar date from binary-coded decimal (BCD) to decimal
 *
 * \param[in] date_bcd  calendar date in BCD
 * \param[out] clndr pointer to RTC calendar in decimal number format
 *
 */
static void bdc_to_clndr(uint32_t date_bcd, rtc_calendar *clndr)
{
        clndr->year = (((date_bcd & RTC_RTC_CALENDAR_REG_RTC_CAL_C_T_Msk) >> 28) * 1000) +
                      (((date_bcd & RTC_RTC_CALENDAR_REG_RTC_CAL_C_U_Msk) >> 24) * 100) +
                      (((date_bcd & RTC_RTC_CALENDAR_REG_RTC_CAL_Y_T_Msk) >> 20) * 10) +
                      ((date_bcd & RTC_RTC_CALENDAR_REG_RTC_CAL_Y_U_Msk) >> 16);
        clndr->mday = (((date_bcd & RTC_RTC_CALENDAR_REG_RTC_CAL_D_T_Msk) >> 12) * 10) +  ((date_bcd & RTC_RTC_CALENDAR_REG_RTC_CAL_D_U_Msk) >> 8);
        clndr->month = (((date_bcd & RTC_RTC_CALENDAR_REG_RTC_CAL_M_T_Msk) >> 7) * 10) +  ((date_bcd & RTC_RTC_CALENDAR_REG_RTC_CAL_M_U_Msk) >> 3);
        clndr->wday = date_bcd & RTC_RTC_CALENDAR_REG_RTC_DAY_Msk;
}

void hw_rtc_init(const rtc_config *cfg)
{
        hw_rtc_set_hour_clk_mode(cfg->hour_clk_mode);
        hw_rtc_set_keep_reg_on_reset(cfg->keep_rtc);

        hw_rtc_motor_event_disable();
        if (cfg->motor_evt.motor_evt_en) {
                hw_rtc_set_motor_event_period(cfg->motor_evt.motor_evt_period);
                hw_rtc_motor_event_enable();
        }

        hw_rtc_pdc_event_disable();
        if (cfg->pdc_evt.pdc_evt_en) {
                hw_rtc_set_pdc_event_period(cfg->pdc_evt.pdc_evt_period);
                hw_rtc_pdc_event_enable();
        }
}

void hw_rtc_register_intr(hw_rtc_interrupt_cb handler, uint8_t mask)
{
        rtc_interrupt_cb = handler;
        hw_rtc_interrupt_enable(mask);
        NVIC_EnableIRQ(RTC_IRQn);
}

void hw_rtc_unregister_intr(void)
{
        rtc_interrupt_cb = NULL;
        hw_rtc_interrupt_disable(0xff);
        NVIC_ClearPendingIRQ(RTC_IRQn);
        NVIC_DisableIRQ(RTC_IRQn);
}

#ifdef SEC_MODEN
HW_RTC_SET_REG_STATUS hw_rtc_set_time_clndr(const rtc_time_t*time, const rtc_calendar *clndr)
#else
HW_RTC_SET_REG_STATUS hw_rtc_set_time_clndr(const rtc_time *time, const rtc_calendar *clndr)
#endif
{
        uint8_t status;
        // stores the current RTC time. If the new time value causes an entry error, this time value will be re-written
        uint32_t time_cur_val;
        // stores the current RTC calendar. If the new calendar value causes an entry error, this time will be re-written
        uint32_t clndr_cur_val;

        printf("%s set to %4u-%02u-%02u %u %02u:%02u:%02u:%02u\r\n", __func__,
                clndr->year, clndr->month, clndr->mday, clndr->wday, time->hour, time->minute, time->sec, time->hsec);

        if ((time != NULL) && (clndr != NULL)) {
                // set both time and calendar. Stop and start counters at the same time
                hw_rtc_stop();
                time_cur_val = hw_rtc_get_time_bcd();
                clndr_cur_val = hw_rtc_get_clndr_bcd();
                hw_rtc_set_time_bcd(time_to_bcd(time));
                hw_rtc_set_clndr_bcd(calendar_to_bcd(clndr));
                status = hw_rtc_get_status();
                if ((status & (HW_RTC_VALID_TIME | HW_RTC_VALID_CLNDR)) == 0x0) {
                        hw_rtc_set_clndr_bcd(clndr_cur_val);
                        hw_rtc_set_time_bcd(time_cur_val);
                        hw_rtc_start();
                        return HW_RTC_INVALID_TIME_CLNDR;
                } else if ((status & HW_RTC_VALID_TIME) != HW_RTC_VALID_TIME) {
                        hw_rtc_set_time_bcd(time_cur_val);
                        hw_rtc_start();
                        return HW_RTC_INVALID_TIME;
                } else if ((status & HW_RTC_VALID_CLNDR) != HW_RTC_VALID_CLNDR) {
                        hw_rtc_set_clndr_bcd(clndr_cur_val);
                        hw_rtc_start();
                        return HW_RTC_INVALID_CLNDR;
                }
                hw_rtc_start();
        } else if (time != NULL) {
                hw_rtc_time_stop();
                time_cur_val = hw_rtc_get_time_bcd();
                hw_rtc_set_time_bcd(time_to_bcd(time));
                status = hw_rtc_get_status();
                if ((status & HW_RTC_VALID_TIME) != HW_RTC_VALID_TIME) {
                        hw_rtc_set_time_bcd(time_cur_val);
                        hw_rtc_time_start();
                        return HW_RTC_INVALID_TIME;
                }
                hw_rtc_time_start();
        } else if (clndr != NULL) {
                hw_rtc_clndr_stop();
                clndr_cur_val = hw_rtc_get_clndr_bcd();
                hw_rtc_set_clndr_bcd(calendar_to_bcd(clndr));
                status = hw_rtc_get_status();
                if ((status & HW_RTC_VALID_CLNDR) != HW_RTC_VALID_CLNDR) {
                        hw_rtc_set_clndr_bcd(clndr_cur_val);
                        hw_rtc_clndr_start();
                        return HW_RTC_INVALID_CLNDR;
                }
                hw_rtc_clndr_start();
        }

        return HW_RTC_VALID_ENTRY;
}

#ifdef SEC_MODEN
void hw_rtc_get_time_clndr(rtc_time_t *time, rtc_calendar *clndr)
#else
void hw_rtc_get_time_clndr(rtc_time *time, rtc_calendar *clndr)
#endif
{
        if ((time != NULL) && (clndr != NULL)) {
                uint32_t val_bcd;

                // In order to obtain a coherent view of time and date, the counters must be stopped
                // while reading them. This avoids the situation where the date counter increments
                // between reading the time register and reading the calendar register.
                hw_rtc_stop();
                val_bcd = hw_rtc_get_time_bcd();
                bdc_to_time(val_bcd, time);
                time->hour_mode = hw_rtc_get_hour_clk_mode();
                val_bcd = hw_rtc_get_clndr_bcd();
                bdc_to_clndr(val_bcd, clndr);
                hw_rtc_start();
        } else if (time != NULL) {
                uint32_t time_bcd;

                time_bcd = hw_rtc_get_time_bcd();
                bdc_to_time(time_bcd, time);
                time->hour_mode = hw_rtc_get_hour_clk_mode();
        } else if (clndr != NULL) {
                uint32_t date_bcd;

                date_bcd = hw_rtc_get_clndr_bcd();
                bdc_to_clndr(date_bcd, clndr);
        }
}

#ifdef SEC_MODEN
HW_RTC_SET_REG_STATUS hw_rtc_set_alarm(const rtc_time_t *time, const rtc_alarm_calendar *clndr, const uint8_t mask)
#else
HW_RTC_SET_REG_STATUS hw_rtc_set_alarm(const rtc_time *time, const rtc_alarm_calendar *clndr, const uint8_t mask)
#endif
{
        uint8_t status;
        uint32_t current_val;
        uint8_t tmp_msk;


        tmp_msk = hw_rtc_get_alarm_enable_msk();  // Get a copy of the alarm enable register to restore the value in case of invalid entry

        hw_rtc_interrupt_disable(HW_RTC_INT_ALRM);
        hw_rtc_alarm_enable(0x0); // disable alarm events



        if (time != NULL) {
                if (time->hour_mode != hw_rtc_get_hour_clk_mode()) {
                        hw_rtc_alarm_enable(tmp_msk);
                        hw_rtc_interrupt_enable(HW_RTC_INT_ALRM);
                        return HW_RTC_INVALID_TIME_HOUR_MODE_ALM; // Do not allow alarm with different hour clock mode from time
                }
                current_val = hw_rtc_get_alarm_time_bcd();
                hw_rtc_set_alarm_time_bcd(alarm_time_to_bcd(time));
                status = hw_rtc_get_status();
                if ((status & HW_RTC_VALID_TIME_ALM) != HW_RTC_VALID_TIME_ALM) {
                        hw_rtc_set_alarm_time_bcd(current_val);
                        hw_rtc_alarm_enable(tmp_msk);
                        hw_rtc_interrupt_enable(HW_RTC_INT_ALRM);
                        return HW_RTC_INVALID_TIME_ALM;
                }
        }

        if (clndr != NULL) {
                current_val = hw_rtc_get_alarm_clndr_bcd();
                hw_rtc_set_alarm_clndr_bcd(alarm_calendar_to_bcd(clndr));
                status = hw_rtc_get_status();
                if ((status & HW_RTC_VALID_CLNDR_ALM) != HW_RTC_VALID_CLNDR_ALM) {
                        hw_rtc_set_alarm_clndr_bcd(current_val);
                        hw_rtc_alarm_enable(tmp_msk);
                        hw_rtc_interrupt_enable(HW_RTC_INT_ALRM);
                        return HW_RTC_INVALID_CLNDR_ALM;
                }
        }

        hw_rtc_alarm_enable(mask);
        hw_rtc_interrupt_enable(HW_RTC_INT_ALRM);

        return HW_RTC_VALID_ENTRY;
}

#ifdef SEC_MODEN
void hw_rtc_get_alarm(rtc_time_t *time, rtc_alarm_calendar *clndr, uint8_t *mask)
#else
void hw_rtc_get_alarm(rtc_time *time, rtc_alarm_calendar *clndr, uint8_t *mask)
#endif
{
        if (time != NULL) {
                uint32_t time_bcd;

                time_bcd = hw_rtc_get_alarm_time_bcd();

                time->pm_flag = (time_bcd & RTC_RTC_TIME_ALARM_REG_RTC_TIME_PM_Msk) >> 30;
                time->hour = (((time_bcd & RTC_RTC_TIME_ALARM_REG_RTC_TIME_HR_T_Msk) >> 28) * 10) +  ((time_bcd & RTC_RTC_TIME_ALARM_REG_RTC_TIME_HR_U_Msk) >> 24);
                time->minute = (((time_bcd & RTC_RTC_TIME_ALARM_REG_RTC_TIME_M_T_Msk) >> 20) * 10) +  ((time_bcd & RTC_RTC_TIME_ALARM_REG_RTC_TIME_M_U_Msk) >> 16);
                time->sec = (((time_bcd & RTC_RTC_TIME_ALARM_REG_RTC_TIME_S_T_Msk) >> 12) * 10) +  ((time_bcd & RTC_RTC_TIME_ALARM_REG_RTC_TIME_S_U_Msk) >> 8);
                time->hsec = (((time_bcd & RTC_RTC_TIME_ALARM_REG_RTC_TIME_H_T_Msk) >> 4) * 10) +  (time_bcd & RTC_RTC_TIME_ALARM_REG_RTC_TIME_H_U_Msk);
                time->hour_mode = hw_rtc_get_hour_clk_mode();
        }

        if (clndr != NULL) {
                uint32_t date_bcd;

                date_bcd = hw_rtc_get_alarm_clndr_bcd();

                clndr->mday = (((date_bcd & RTC_RTC_CALENDAR_ALARM_REG_RTC_CAL_D_T_Msk) >> 12) * 10) +  ((date_bcd & RTC_RTC_CALENDAR_ALARM_REG_RTC_CAL_D_U_Msk) >> 8);
                clndr->month = (((date_bcd & RTC_RTC_CALENDAR_ALARM_REG_RTC_CAL_M_T_Msk) >> 7) * 10) +  ((date_bcd & RTC_RTC_CALENDAR_ALARM_REG_RTC_CAL_M_U_Msk) >> 3);
        }

        if (mask != NULL) {
                *mask = hw_rtc_get_alarm_enable_msk();
        }

}

void RTC_Handler(void)
{
        SEGGER_SYSTEMVIEW_ISR_ENTER();

        if (rtc_interrupt_cb != NULL) {
                uint8_t event;
                event = hw_rtc_get_event_flags();
                rtc_interrupt_cb(event);
        }

        SEGGER_SYSTEMVIEW_ISR_EXIT();
}

#ifdef SEC_MODEN
/*
 * @brief: RTC event callback handler
 *
 * \param [output] events A bitmask of event values (HW_RTC_EVENT). A value set to '1'
 *                        indicates that the event occurred since the last reset:
 *
 *
 *  Bit:    |  6     |    5   |   4   |   3   |  2   |  1   |   0   |
 *          +--------+--------+-------+-------+------+------+-------+
 *  Event:  |on alarm|on month|on mday|on hour|on min|on sec|on hsec|
 *          +--------+--------+-------+-------+------+------+-------+
 */
static void _rtc_event_cb(uint8_t events)
{
        /* Check for the RTC trigger source using the HW_RTC_EVENT enum as bitmask */
        if ((events & HW_RTC_EVENT_SEC) == HW_RTC_EVENT_SEC) {
                /* Notify the main task to print the current date and time */
#ifndef ONLY_SDK_BUILD
                if (gdi_display_is_powered()) {
                        ui_send_event_from_isr(UI_EVENT_RTC_TIME_EXPIRED);
                }
#endif
        }

#if (ENABLE_RTC_ALARM_EVENT)

        /* Check if the callback was triggered because of an RTC alarm event */
        if ((events & HW_RTC_EVENT_ALRM) == HW_RTC_EVENT_ALRM) {
                /* Notify the main task to disable triggering RTC alarm events */
                ui_send_event_from_isr(UI_EVENT_RTC_ALARM_EXPIRED);
        }

#endif

}



/*
 * @brief RTC initialization function
 *
 *
 * This function initializes the RTC with default time and calendar values as well as enables the
 * RTC interrupt to hit every 1 second (seconds unit rollover). If ENABLE_RTC_ALARM_EVENT macro
 * is set, an RTC alarm event is also enabled to trigger an RTC interrupt.
 *
 *
 * \note The 100 Hz RTC (10 milliseconds granularity) is clocked by the low power (lp) clock used.
 *       So far, the SDK supports only the XTAL32K crystal as lp clock.
 *
 * \note By default, the RTC is trimmed to work with the XTAL32K clock source. If different
 *       trimming is required, the following API should be used:
 *
 *       hw_rtc_clk_config(HW_RTC_DIV_DENOM div_denom, uint16_t div_int, uint16_t div_fract)
 *
 *       The value of \p div_int must be equal to the integer divisor, part of the formula:
 *       (FLP_CLK / 100) e.g. (32768 / 100) = 327.680 (327 --> 0x147)
 *
 *       The value of \p div_fract must be equal to the fractional divisor, part of the formula:
 *       (FLP_CLK / 100) e.g. (32768 / 100) = 327.680 (680 --> 0x2A8)
 *
 */
void _rtc_init(void)
{
      rtc_config  cfg = { 0 };

      // Here you can define the hour clock format, either in 24-mode or 12-mode
      cfg.hour_clk_mode = RTC_24H_CLK;

      /*
       * Here you can define whether or note the date and time register
       * values should be reset after a SW reset.
       */
      cfg.keep_rtc = true;

      // Initialize the RTC with the previous defined configurations
      hw_rtc_init(&cfg);


      // Enable the RTC peripheral clock
      hw_rtc_clock_enable();

      /*
       * Declare time parameters using the 24-hour clock format (these values will be the
       * starting point for the time functionality).
       */
      rtc_time_t rtc_time_cfg = {
              /*
               * Valid values from 0 to 23 in 24-hour mode,
               * or from 1 to 12 in 12-hour mode.
               */
              .hour   = 0,
              .minute = 0,
              .sec    = 0,
              .hsec   = 0,
      };


      /*
       * Declare calendar parameters (these values will be the starting point
       * for the calendar functionality).
       */
      rtc_calendar rtc_clndr_cfg = {
              .year  = 2019,
              .month = 1,
              .mday  = 1,

              /*
               * Day of week from 1 to 7. The user is free to map these values
               * with the days of the week as preferred. For instance:
               * 1 --> Monday
               * 2 --> Tuesday
               * ...
               * 7 --> Sunday
               */
              .wday  = 2,
      };

      rtc_time_t rtc_time_rd = { 0 };
      rtc_calendar rtc_clndr_rd = { 0 };
      hw_rtc_get_time_clndr(&rtc_time_rd, &rtc_clndr_rd);

      if(is_rtc_init_time(rtc_time_rd, rtc_clndr_rd))
      {
          /*
           * Set the previous defined time and calendar parameters. This function will call
           * hw_rtc_start() function that starts both the time and calendar functionality.
           * It is essential that hw_rtc_clock_enable() function has already been invoked.
           */
          HW_RTC_SET_REG_STATUS rtc_error = hw_rtc_set_time_clndr(&rtc_time_cfg, &rtc_clndr_cfg);
          OS_ASSERT( rtc_error == HW_RTC_VALID_ENTRY );
          printf("Apply RTC initial time\r\n");

          nvms_t app_param_nvms = ad_nvms_open(NVMS_APP_PARAM_PART);
          if(is_exist_rtc_nvms_flash(app_param_nvms) && is_exist_timezone_nvms_flash(app_param_nvms))
          {
              /* Get Previous RTC date & time if possible */
              rtc_read_from_nvms();

              /* Get Timezone if possible */
              timezone_read_from_nvms();
          }
      }

#if (ENABLE_RTC_ALARM_EVENT)

      /* Given the current time values, declare time alarm parameters */
      rtc_time_t rtc_alarm_time_cfg = {
              .hour   = 18,
              .minute = 1,
              .sec    = 5,
              .hsec   = 0,
      };


      /*
       * Since we want to trigger an RTC alarm event within the same day, there is no need to
       * declare calendar alarm parameters invoking the 'rtc_alarm_calendar' structure.
       */


      /*
       * Set the previous defined time alarm parameters. You can enable one or more alarm sources
       * using the HW_RTC_ALARM_EN structure and the Bitwise OR operator. In this example, since
       * both 'min' and 'sec' alarms are used, an alarm event should be triggered after 1 minute
       * and 5 seconds.
       *
       * \note The alarm interrupt is enabled automatically and thus an interrupt
       *       will be generated when an RTC alarm event occurs.
       *
       */
      HW_RTC_SET_REG_STATUS rtc_alarm_error = hw_rtc_set_alarm(&rtc_alarm_time_cfg, NULL,
                                                                      HW_RTC_ALARM_SEC | HW_RTC_ALARM_MIN);
      OS_ASSERT( rtc_alarm_error == HW_RTC_VALID_ENTRY );
#endif


      /*
       * Reading the event flag register will clear the register
       * and thus any pending interrupt.
       */
      hw_rtc_get_event_flags();


      /*
       * Enable RTC to issue an RTC interrupt when minutes roll over, that is, every time the
       * 'minutes' unit changes value. If enabled, an interrupt will also be generated upon
       * an RTC alarm event.
       *
       * \note: The 2nd input parameter declares all the RTC interrupt sources. You can select one
       *        or more interrupt sources using the HW_RTC_INTR enum and the Bitwise OR operator.
       *
       */
      hw_rtc_register_intr( _rtc_event_cb, (HW_RTC_INT_SEC) |
                              ((ENABLE_RTC_ALARM_EVENT) ? HW_RTC_INT_ALRM : 0x0) );

    rtcInitFlag = true;

    printf("\r\n****** RTC INITIALIZED ******\n\r");

    /*
     * Add an PDC LUT entry so that the CM33 can wakeup upon an RTC event.
     * This is important because every time the CM33 goes for sleep, it is
     * powered off completely! The Power Domain Controller (PDC) will
     * trigger the CM33 to wake up and thus to handle the RTC event.
     *
     * \note The 2nd input parameter declares that the PDC should be triggered following an
     *       RTC event (either an alarm or rollover event).
     *
     * \note The 3rd input parameter declares that the PDC should notify the CM33 to wake up
     *       following an RTC event.
     */
    uint32_t pdc_rtc_id = hw_pdc_add_entry(HW_PDC_LUT_ENTRY_VAL(HW_PDC_TRIG_SELECT_PERIPHERAL,
                                              HW_PDC_PERIPH_TRIG_ID_RTC_ALARM, HW_PDC_MASTER_CM33,
                                              0));

    OS_ASSERT(pdc_rtc_id != HW_PDC_INVALID_LUT_INDEX);

}

#endif

#ifdef SEC_MODEN
bool is_rtc_init_time(rtc_time_t time, rtc_calendar clndr)
{
    if(clndr.year < 2019)
    {
        return true;
    }
    return false;
}

/**********************************************************
*********************       RTC APIs       ***********************
***********************************************************/
void write_rtc_to_flash(rtc_time_t time, rtc_calendar clndr)
{
    time_t last_epoch = {0};
    int ret = 0;
    struct tm tim = {0};

    if(rtcInitFlag)
    {
        // Make UTC epoch time.
        uint8_t hh =  time.hour;
        uint8_t mm =  time.minute;
        uint8_t ss =  time.sec;
        uint8_t wd =  clndr.wday;
        uint8_t d  =  clndr.mday;
        uint8_t m  =  clndr.month;
        uint16_t y =  clndr.year;
        uint16_t yr = (uint16_t)(y-1900);

        tim.tm_year = yr;
        tim.tm_mon  = m - 1;
        tim.tm_mday = d;
        tim.tm_wday = wd;
        tim.tm_hour = hh;
        tim.tm_min  = mm;
        tim.tm_sec  = ss;

        last_epoch = mktime(&tim);
    }

    ret = set_epoch_to_flash(last_epoch);
    if(ret < 0)
    {
        printf("%s ret: %d\r\n", __func__, ret);
    }
}

/*
 * @brief: set UTC+0 time
 */
void sys_settime(time_t secs_since_epoch)
{
    rtc_time_t rtc_time = { 0 };
    rtc_calendar rtc_clndr = { 0 };    
    struct tm *time_tm;
    HW_RTC_SET_REG_STATUS rtc_error = 0;
    time_tm = gmtime(&secs_since_epoch);

    rtc_clndr.year = (uint16_t)(time_tm->tm_year+1900);  // time.h is years since 1900, chip is years since 2000
    rtc_clndr.month = (uint8_t)time_tm->tm_mon+1;        //momth 1-12. This is why date math is frustrating.
    rtc_clndr.mday= (uint8_t)time_tm->tm_mday;
    rtc_clndr.wday = (uint8_t)time_tm->tm_wday;

    rtc_time.hour = (uint8_t)time_tm->tm_hour;
    rtc_time.minute = (uint8_t)time_tm->tm_min;
    rtc_time.sec = (uint8_t)time_tm->tm_sec;
    rtc_time.hsec = 0;
    
    rtc_error = hw_rtc_set_time_clndr(&rtc_time, &rtc_clndr);
    //OS_ASSERT( rtc_error == HW_RTC_VALID_ENTRY );
    if(rtc_error != HW_RTC_VALID_ENTRY)
    {
        printf("%s rtc_error: %d\r\n", __func__, rtc_error);
    }
}

/*
 * @brief: set UTC+0 time calendar
 */
void sys_settime_calendar(rtc_date_time_t cal)
{
    hw_rtc_set_time_clndr(&cal.time, &cal.date);
}

/*
 * @brief: get UTC+0 time
 */
bool sys_gettime(time_t* sec_since_epoch)
{
    if(rtcInitFlag)
    {
        rtc_time_t rtc_time_rd = { 0 };
        rtc_calendar rtc_clndr_rd = { 0 };
        time_t currentTime = {0};
        struct tm tim = {0};

        // Get current calendar from hw rtc block at call time.
        hw_rtc_get_time_clndr(&rtc_time_rd, &rtc_clndr_rd);

        // Make UTC epoch time.
        uint8_t hh =  rtc_time_rd.hour;
        uint8_t mm =  rtc_time_rd.minute;
        uint8_t ss =  rtc_time_rd.sec;
        uint8_t d  =  rtc_clndr_rd.mday;
        uint8_t m  =  rtc_clndr_rd.month;
        uint16_t y =  rtc_clndr_rd.year;
        uint16_t yr = (uint16_t)(y-1900);

        tim.tm_year = yr;
        tim.tm_mon  = m - 1;
        tim.tm_mday = d;
        tim.tm_hour = hh;
        tim.tm_min  = mm;
        tim.tm_sec  = ss;
        
        currentTime = mktime(&tim);
        *sec_since_epoch = currentTime;
        return true;
    }

    return false;
}

bool sys_gettime_msec(uint64_t* msec_since_epoch)
{
    time_t sec_time;
    if(sys_gettime(&sec_time))
    {
        rtc_time_t rtc_time_rd = { 0 };
        rtc_calendar rtc_clndr_rd = { 0 };
        uint64_t msec_time = 0;

        // Get current calendar from hw rtc block at call time.
        hw_rtc_get_time_clndr(&rtc_time_rd, &rtc_clndr_rd);
        
        msec_time = ((uint64_t)sec_time * 1000) + (rtc_time_rd.hsec*10);
        
        *msec_since_epoch = msec_time;
        return true;
    }
    return false;
}

/*
 * @brief: get local time
 */
bool sys_gettime_msec_local(uint64_t* local_msec_since_epoch)
{
    uint64_t utcTimeEpoch = 0;
    int32_t currTimezone = 0;

    bool ret = sys_gettime_msec(&utcTimeEpoch);

    if(ret == false)
        return false;

    currTimezone = sys_gettimezone();

    *local_msec_since_epoch = utcTimeEpoch + (currTimezone*1000);
    return true;
}

bool sys_gettime_str(char* buf, uint8_t buf_size)
{
    time_t now_time;
    if(sys_gettime(&now_time))
    {
        strftime (buf, buf_size, "[DATE] %a %Y-%m-%d [TIME] %H:%M:%S", localtime(&now_time));
        return true;
    }
    return false;
}

bool sys_gettime_calendar(rtc_date_time_t* date_time)
{
    if(rtcInitFlag)
    {
        rtc_time_t rtc_time_rd = { 0 };
        rtc_calendar rtc_clndr_rd = { 0 };
        
        // Get current calendar from hw rtc block at call time.
        hw_rtc_get_time_clndr(&rtc_time_rd, &rtc_clndr_rd);
        
        date_time->date.year = rtc_clndr_rd.year;
        date_time->date.month = rtc_clndr_rd.month;
        date_time->date.mday = rtc_clndr_rd.mday;
        date_time->date.wday = rtc_clndr_rd.wday;

        date_time->time.hour = rtc_time_rd.hour;
        date_time->time.minute = rtc_time_rd.minute;
        date_time->time.sec = rtc_time_rd.sec;
        date_time->time.hsec= rtc_time_rd.hsec;
        
        return true;
    }

    return false;
}

void show_current_calendar(void)
{
    char buf[100];
    if(sys_gettime_str(buf,  sizeof(buf)))
    	printf("%s\r\n", buf);
    else
		printf("RTC not initialized.\r\n");

}

rtc_date_time_t convert_epoch_to_calendar(uint64_t msec_since_epoch)
{
    rtc_date_time_t covertCalendar;
    struct tm *time_tm ;
    time_t secs_since_epoch = (time_t)(msec_since_epoch/1000);
    time_tm = gmtime(&secs_since_epoch);

    covertCalendar.date.year = (uint16_t)(time_tm->tm_year+1900);
    covertCalendar.date.month = (uint8_t)time_tm->tm_mon+1;
    covertCalendar.date.mday = (uint8_t)time_tm->tm_mday;
    covertCalendar.date.wday = (uint8_t)time_tm->tm_wday;

    covertCalendar.time.hour = (uint8_t)time_tm->tm_hour;
    covertCalendar.time.minute = (uint8_t)time_tm->tm_min;
    covertCalendar.time.sec = (uint8_t)time_tm->tm_sec;
    covertCalendar.time.hsec = (msec_since_epoch%1000)/10;

    return covertCalendar;
}

#ifndef ONLY_SDK_BUILD
/**********************************************************
*********************       NVMS APIs       ***********************
***********************************************************/
bool set_epoch_to_flash(time_t epoch)
{
    printf("%s epoch: %lu\r\n", __func__, epoch);
    nvms_t app_param_nvms = ad_nvms_open(NVMS_APP_PARAM_PART);
    uint8_t epoch_array[NVMS_APP_PARAMS_TAG_RTC_SIZE] = {0};
    epoch_to_byte_array(epoch, epoch_array);

    return (app_param_nvms && NVMS_APP_PARAMS_TAG_RTC_SIZE == ad_nvms_write(app_param_nvms, NVMS_APP_PARAMS_TAG_RTC_OFFSET, epoch_array, NVMS_APP_PARAMS_TAG_RTC_SIZE));
}


bool get_epoch_from_flash(time_t* flash_epoch)
{
    nvms_t app_param_nvms = ad_nvms_open(NVMS_APP_PARAM_PART);
    uint8_t epoch_array[NVMS_APP_PARAMS_TAG_RTC_SIZE] = {0};
    time_t epoch = 0;
    if (app_param_nvms && NVMS_APP_PARAMS_TAG_RTC_SIZE == ad_nvms_read(app_param_nvms, NVMS_APP_PARAMS_TAG_RTC_OFFSET, epoch_array, NVMS_APP_PARAMS_TAG_RTC_SIZE))
    {
        byte_array_to_epoch(&epoch, epoch_array);
        *flash_epoch = epoch;
        return true;
    }
    else
    {
        return false;
    }
}

bool set_timezone_flash(int32_t timezone)
{
    printf("set_timezone_flash [%d] \r\n", timezone);
    nvms_t app_param_nvms = ad_nvms_open(NVMS_APP_PARAM_PART);
    uint8_t timezone_array[NVMS_APP_PARAMS_TAG_TIMEZONE_SIZE] = {0};
    epoch_to_byte_array((time_t)timezone, timezone_array);

    return (app_param_nvms && NVMS_APP_PARAMS_TAG_TIMEZONE_SIZE == ad_nvms_write(app_param_nvms, NVMS_APP_PARAMS_TAG_TIMEZONE_OFFSET, timezone_array, NVMS_APP_PARAMS_TAG_TIMEZONE_SIZE));
}

bool get_timezone_from_flash(int32_t* flash_timezone)
{
    nvms_t app_param_nvms = ad_nvms_open(NVMS_APP_PARAM_PART);
    uint8_t timezone_array[NVMS_APP_PARAMS_TAG_TIMEZONE_SIZE] = {0};
    time_t timezone = 0;

    if (app_param_nvms && NVMS_APP_PARAMS_TAG_TIMEZONE_SIZE == ad_nvms_read(app_param_nvms, NVMS_APP_PARAMS_TAG_TIMEZONE_OFFSET, timezone_array, NVMS_APP_PARAMS_TAG_TIMEZONE_SIZE))
    {
        byte_array_to_epoch(&timezone, timezone_array);
        *flash_timezone = timezone;
        return true;
    }
    else
    {
        printf(DBG_RTC, DBG_PATH_UART|DBG_PATH_FLASH,"[RTC] get_timezone_from_flash error!\r\n");
        return false;
    }
}
#endif

/*
 * @brief: get timezone for secs
 */
int32_t sys_gettimezone()
{
    return current_timezone;
}

/*
 * @brief: set timezone for secs
 */
void sys_settimezone(int32_t timezone)
{
    printf("%s set to tz: %d\r\n", __func__, timezone);
    current_timezone = timezone;
    if(!set_timezone_flash(timezone))
    {
        printf("%s save to flash failed\r\n", __func__);
    }
}

void epoch_to_byte_array(time_t epoch, uint8_t buf[])
{
    buf[0] = (uint8_t)((epoch & 0xFF000000) >> 24);
    buf[1] = (uint8_t)((epoch & 0x00FF0000) >> 16);
    buf[2] = (uint8_t)((epoch & 0x0000FF00) >> 8);
    buf[3] = (uint8_t)(epoch);
}

void byte_array_to_epoch(time_t* epoch, uint8_t buf[])
{
    *epoch = ((buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | (buf[3]));
}

#ifndef ONLY_SDK_BUILD
bool is_exist_rtc_nvms_flash(nvms_t hld)
{
    uint8_t nv[NVMS_APP_PARAMS_TAG_RTC_SIZE] = { 0 };

    if (hld && NVMS_APP_PARAMS_TAG_RTC_SIZE == ad_nvms_read(hld, NVMS_APP_PARAMS_TAG_RTC_OFFSET, nv, NVMS_APP_PARAMS_TAG_RTC_SIZE))
    {
        uint8_t ff_cnt = 0;

        for(uint8_t i = 0; i < NVMS_APP_PARAMS_TAG_RTC_SIZE; i++)
        {
            if (0xFF == nv[i])
                ff_cnt++;
            else
                break;
        }

        if (NVMS_APP_PARAMS_TAG_RTC_SIZE == ff_cnt)
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    return false;
}

void rtc_read_from_nvms(void)
{
    nvms_t app_param_nvms = ad_nvms_open(NVMS_APP_PARAM_PART);
    
    if(is_exist_rtc_nvms_flash(app_param_nvms))
    {
        time_t flash_epoch;
    	printf("Restore Previous RTC Date & Time\r\n");		
        get_epoch_from_flash(&flash_epoch);
        sys_settime(flash_epoch);
    }
    else
    {
    	printf("Apply Factory RTC Date & Time\r\n");				
    }
}

bool is_exist_timezone_nvms_flash(nvms_t hld)
{
    uint8_t nv[NVMS_APP_PARAMS_TAG_TIMEZONE_SIZE]={ 0 };

    if (hld && NVMS_APP_PARAMS_TAG_TIMEZONE_SIZE == ad_nvms_read(hld, NVMS_APP_PARAMS_TAG_TIMEZONE_OFFSET, nv, NVMS_APP_PARAMS_TAG_TIMEZONE_SIZE))
    {
        uint8_t ff_cnt = 0;
        for(uint8_t i = 0; i < NVMS_APP_PARAMS_TAG_TIMEZONE_SIZE; i++)
        {
            if (0xFF == nv[i])
                ff_cnt++;
            else
                break;
        }
        if (NVMS_APP_PARAMS_TAG_TIMEZONE_SIZE==ff_cnt)
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    return false;
}

void timezone_read_from_nvms(void)
{
    nvms_t app_param_nvms = ad_nvms_open(NVMS_APP_PARAM_PART);

    if(is_exist_timezone_nvms_flash(app_param_nvms))
    {
        int32_t flash_timezone=0;
        printf("Restore Previous Timezone\r\n");
        get_timezone_from_flash(&flash_timezone);
        sys_settimezone(flash_timezone);
    }
    else
    {
        current_timezone = 0;
        printf("Apply Factory Timezone\r\n");
    }
}

bool read_app_param_nvms(uint8_t buf[], uint8_t size)
{
    nvms_t app_param_nvms = ad_nvms_open(NVMS_APP_PARAM_PART);
    return (app_param_nvms && size == ad_nvms_read(app_param_nvms, NVMS_APP_PARAMS_TAG_RTC_OFFSET, buf, size));
}

void reset_rtc_timezone_flash(void)
{
    printf(DBG_RTC, DBG_PATH_UART|DBG_PATH_FLASH,"[RTC] Reset rtc and timezone flash\r\n");
    set_timezone_flash(0);
}
#endif

void terminate_rtc_task(void)
{
    time_t last_epoch = 0;
    int32_t last_timezone = 0;
    bool ret = false;

    // Save last epoch to flash
    if(!sys_gettime(&last_epoch))
    {
        return;
    }

    ret = set_epoch_to_flash(last_epoch);
    if(!ret)
        printf("Can't save rtc epoch\r\n");
    else
    {
        // Save last timezone to flash
        last_timezone = sys_gettimezone();
        ret = set_timezone_flash(last_timezone);
        if(!ret)
            printf("Can't save rtc timezone\r\n");
    }

    // Delete rtc interrupt, timer and task
    hw_rtc_unregister_intr();
}

#endif
