/**
\addtogroup BSP
\{
\addtogroup DEVICES
\{
\addtogroup CLK
\{
*/

/**
****************************************************************************************
*
* @file hw_clk_da1469x.c
*
* @brief Clock Driver
*
* Copyright (C) 2015-2018 Dialog Semiconductor.
* This computer program includes Confidential, Proprietary Information
* of Dialog Semiconductor. All Rights Reserved.
*
****************************************************************************************
*/

#define USE_FAST_STARTUP        0

#include <stdint.h>
#include "hw_clk.h"

typedef struct
{
        int16_t  tc[4];                 // contains temperature compensation coefficients
        uint16_t trim;                  // frequency trimming at room temperature
        uint8_t  cxcomp_trim_cap;       // cxcomp capacitance value
        uint8_t  capbank_sens;          // capacitor bank sensitivity
        uint16_t temp_refpt;            // reference point for temperature compensation
#if USE_FAST_STARTUP
        uint16_t base_drive_cycles;     // base drive cycles at room xtal32m_temp_refpt
#endif
} xtalm_otp_t;

__RETAINED static xtalm_otp_t xtalm_otp_values;

/*
 * Function definitions
 */

__RETAINED_CODE void hw_clk_start_calibration(cal_clk_t clk_type, cal_ref_clk_t clk_ref_type, uint16_t cycles)
{
        uint32_t val = 0;

        /* Must be disabled */
        ASSERT_WARNING(!REG_GETF(ANAMISC, CLK_REF_SEL_REG, REF_CAL_START));

        ANAMISC->CLK_REF_CNT_REG = cycles;                      // # of cal clock cycles

        if (clk_ref_type == CALIBRATE_REF_EXT) {
                REG_SET_FIELD(ANAMISC, CLK_REF_SEL_REG, EXT_CNT_EN_SEL, val, 1);
                REG_SET_FIELD(ANAMISC, CLK_REF_SEL_REG, CAL_CLK_SEL, val, 0); /* DivN to be calibrated */
        } else {
                REG_SET_FIELD(ANAMISC, CLK_REF_SEL_REG, CAL_CLK_SEL, val, clk_ref_type);
        }
        REG_SET_FIELD(ANAMISC, CLK_REF_SEL_REG, REF_CLK_SEL, val, clk_type);
        ANAMISC->CLK_REF_SEL_REG = val;

        REG_SET_BIT(ANAMISC, CLK_REF_SEL_REG, REF_CAL_START);
}

uint32_t hw_clk_get_calibration_data(void)
{
        /* Wait until it's finished */
        while (REG_GETF(ANAMISC, CLK_REF_SEL_REG, REF_CAL_START)) {
        }

        return ANAMISC->CLK_REF_VAL_REG;
}

#define CLK_DELAY_SANITY_CHECKS
#pragma GCC push_options
#pragma GCC optimize ("O3")

void hw_clk_delay_usec(uint32_t usec)
{
        static const uint32_t DIVIDER = 1000000;

#ifdef CLK_DELAY_SANITY_CHECKS
        ASSERT_WARNING((dg_configXTAL32M_FREQ % DIVIDER) == 0);
        ASSERT_WARNING((dg_configPLL96M_FREQ % DIVIDER) == 0);
        ASSERT_WARNING((HW_CLK_DELAY_OVERHEAD_CYCLES % HW_CLK_CYCLES_PER_DELAY_REP) == 0);
#endif

        static const uint8_t OVERHEAD_REPS = HW_CLK_DELAY_OVERHEAD_CYCLES / HW_CLK_CYCLES_PER_DELAY_REP;
        static volatile uint32_t sys_freq_table[] = {
                dg_configXTAL32M_FREQ/DIVIDER,  // SYS_CLK_IS_XTAL32M
                dg_configXTAL32M_FREQ/DIVIDER,  // SYS_CLK_IS_RC32: Use XTAL32M frequency. This is the maximum frequency of RC32M
                                                // therefore it is guaranteed that the delay will be greater or equal to the
                                                // requested period.
                0,                              // SYS_CLK_IS_LP is not supported
                dg_configPLL96M_FREQ/DIVIDER    // SYS_CLK_IS_PLL
        };

        const uint32_t cycles_per_usec = sys_freq_table[hw_clk_get_sysclk()] >> hw_clk_get_hclk_div(),
                       reps = cycles_per_usec * usec / HW_CLK_CYCLES_PER_DELAY_REP;

#ifdef CLK_DELAY_SANITY_CHECKS
        ASSERT_WARNING(usec <= 0xFFFFFFFF/cycles_per_usec);     // The requested delay is greater than the maximum delay this function can achieve
        ASSERT_WARNING(reps > OVERHEAD_REPS);                   // The requested delay is smaller than the minimum delay this function can achieve.
#endif

        if (reps <= OVERHEAD_REPS) {
                return;
        }

        asm volatile(
                "       nop                             \n"
                "       nop                             \n"
                "       nop                             \n"
                "       nop                             \n"
                "       nop                             \n"
                "loop:  nop                             \n"
                "       subs %[reps], %[reps], #1       \n"
                "       bne loop                        \n"
                :                                       // outputs
                : [reps] "r" (reps - OVERHEAD_REPS)     // inputs
                :                                       // clobbers
        );
}

#pragma GCC pop_options

static void xtal32m_readOTP(void)
{
        xtalm_otp_values.tc[0] = -201;
        xtalm_otp_values.tc[1] = -3084;
        xtalm_otp_values.tc[2] = -1130;
        xtalm_otp_values.tc[3] = 3988;
        xtalm_otp_values.capbank_sens = 91;                             // capacitor bank sensitivity, Steps/pF
        xtalm_otp_values.cxcomp_trim_cap = 0;                           // cxcomp capacitance value
        xtalm_otp_values.temp_refpt = 100;                              // reference point for temperature compensation
#if USE_FAST_STARTUP
        xtalm_otp_values.base_drive_cycles = 175;                       // base drive cycles at room xtal32m_temp_refpt
#endif
}

static uint16_t xtal32m_adcread(void)
{
        // currently returns the 'default' value of 100, which disables the temperature compensation
        return 100;
}

#if (dg_configENABLE_DA1469x_AA_SUPPORT)
/* Workaround for bug2522A_050: SW needed to overrule the XTAL calibration state machine */

void hw_clk_perform_init_rcosc_calibration(void)
{
        uint16_t band_stat;
        uint16_t band_sel;
        uint16_t trim_sns;

        ASSERT_WARNING(REG_GETF(CRG_TOP, CLK_CTRL_REG, RUNNING_AT_XTAL32M) || REG_GETF(CRG_TOP, CLK_CTRL_REG, RUNNING_AT_PLL96M));


        REG_SET_BIT(CRG_XTAL, XTAL32M_CTRL0_REG, XTAL32M_RCOSC_CALIBRATE);
        hw_clk_delay_usec(100);
        band_sel = REG_GETF(CRG_XTAL, XTAL32M_CTRL3_REG, XTAL32M_RCOSC_BAND_SELECT);
        do {
                band_stat = REG_GETF(CRG_XTAL, XTAL32M_STAT0_REG, XTAL32M_RCOSC_BAND_SELECT_STAT);
        } while ((band_stat == band_sel) && !REG_GETF(CRG_XTAL, XTAL32M_STAT0_REG, XTAL32M_RCOSC_CALIBRATION_DONE));

        if (band_stat != band_sel) {
                REG_SETF(CRG_XTAL, XTAL32M_CTRL3_REG, XTAL32M_FREQ_DET_START, 0x1); // keep frequency counter running
                trim_sns =  REG_GETF(CRG_XTAL, XTAL32M_CTRL2_REG, XTAL32M_RCOSC_TRIM_SNS);
                REG_SETF(CRG_XTAL, XTAL32M_CTRL2_REG, XTAL32M_RCOSC_TRIM_SNS, 0x03);
                REG_CLR_BIT(CRG_XTAL, XTAL32M_CTRL0_REG, XTAL32M_RCOSC_CALIBRATE); // stop calibration
                while (REG_GETF(CRG_XTAL, XTAL32M_STAT1_REG, XTAL32M_CAL_STATE) != 0x08); // wait for the calibration sm to reach the IDLE state.
                REG_SETF(CRG_XTAL, XTAL32M_CTRL3_REG, XTAL32M_FREQ_DET_START, 0x0); // stop frequency detector
                REG_SETF(CRG_XTAL, XTAL32M_CTRL2_REG, XTAL32M_RCOSC_TRIM_SNS, trim_sns);

                if (band_stat > band_sel) {
                        REG_SETF(CRG_XTAL, XTAL32M_CTRL3_REG, XTAL32M_RCOSC_BAND_SELECT, band_sel-1);
                }
                else {
                        REG_SETF(CRG_XTAL, XTAL32M_CTRL3_REG, XTAL32M_RCOSC_BAND_SELECT, band_sel+1);
                }
                REG_SETF(CRG_XTAL, XTAL32M_CTRL3_REG, XTAL32M_RCOSC_TRIM, 512);

                REG_SETF(CRG_XTAL, XTAL32M_CTRL3_REG, XTAL32M_RCOSC_TRIM_STROBE, 1);
                REG_SETF(CRG_XTAL, XTAL32M_CTRL3_REG, XTAL32M_RCOSC_TRIM_STROBE, 0);
//                REG_SET_BIT(CRG_XTAL, XTAL32M_CTRL0_REG, XTAL32M_RCOSC_CALIBRATE);
//                if (REG_GETF(CRG_XTAL, XTAL32M_CTRL3_REG, XTAL32M_SW_CTRL_MODE)) {
//                        REG_CLR_BIT(CRG_XTAL, XTAL32M_CTRL0_REG, XTAL32M_RCOSC_CALIBRATE);
//                }
                hw_clk_xtalm_calibrate_rcosc();

                return;
        }
}

void hw_clk_xtalm_calibrate_rcosc(void)
{
#if USE_FAST_STARTUP
        hw_clk_perform_init_rcosc_calibration();
#endif
}
#endif /*dg_configENABLE_DA1469x_AA_SUPPORT */

int16_t hw_clk_xtalm_update_rdy_cnt(void)
{
        int16_t xtalrdy_stat = 0;
        uint8_t xtalrdy_cnt = 0;

        //update IRQ time when in NORMAL startup mode:
        //(mis) use irq counter setting to do this
        if (REG_GETF(CRG_XTAL, XTALRDY_CTRL_REG, XTALRDY_CLK_SEL) == 0) {
                xtalrdy_cnt = REG_GETF(CRG_XTAL, XTALRDY_CTRL_REG, XTALRDY_CNT);
                xtalrdy_stat = 3 - REG_GETF(CRG_XTAL, XTALRDY_STAT_REG, XTALRDY_STAT);
                xtalrdy_cnt += xtalrdy_stat;
                REG_SETF(CRG_XTAL, XTALRDY_CTRL_REG, XTALRDY_CNT, xtalrdy_cnt);
        }
        return xtalrdy_stat;
}

void hw_clk_xtalm_compensate_amp(void)
{
#if USE_FAST_STARTUP
        uint16_t T_drive, T_drive_lsb;
        uint8_t N;
        // perform amplitude compensation
        if (REG_GETF(CRG_XTAL, XTAL32M_CTRL0_REG, XTAL32M_RCOSC_XTAL_DRIVE) == 1) {
                uint16_t divisor = (0x8 << (0x7 - REG_GETF(CRG_XTAL, XTAL32M_CTRL1_REG, XTAL32M_STARTUP_TDRIVE)));

                T_drive = (xtalm_otp_values.base_drive_cycles * xtal32m_adcread()) / xtalm_otp_values.temp_refpt;
                N = T_drive / divisor;
                T_drive_lsb = T_drive - divisor * N;

                REG_SETF(CRG_XTAL, XTAL32M_CTRL1_REG, XTAL32M_STARTUP_TDRIVE_LSB, T_drive_lsb);
                REG_SETF(CRG_XTAL, XTAL32M_CTRL1_REG, XTAL32M_DRIVE_CYCLES, N+1);
        }
#endif
}

void hw_clk_xtalm_compensate_freq(void)
{
        int16_t d_temp;
        int32_t k_trim;
        uint16_t cnt = 0;

        if (!(REG_GETF(CRG_TOP, CLK_CTRL_REG, RUNNING_AT_XTAL32M) || REG_GETF(CRG_TOP, CLK_CTRL_REG, RUNNING_AT_PLL96M))) {
                // apply temperature compensation:

                d_temp = xtal32m_adcread() - xtalm_otp_values.temp_refpt;
                k_trim = xtalm_otp_values.tc[0] + ((xtalm_otp_values.tc[1]*d_temp)*64)/1e2 +
                        (xtalm_otp_values.tc[2]*d_temp*d_temp*64)/1e4 + (xtalm_otp_values.tc[3]*d_temp*d_temp*d_temp*64)/1e6;
                // k_trim needs to be divided by 2<<16: this is the first 8
                k_trim = (k_trim >> 8);

                // capbank sense needs to be divided by (64*(2<<8)) == >> 14
                // and another 8 bits for k_trim
                k_trim = xtalm_otp_values.trim + ((k_trim * xtalm_otp_values.capbank_sens) >> 22);

                // switch trim-setting to settle
                // apply new trim setting:
                if (k_trim < 703 && k_trim > 0) {
                        while (cnt < 10) {
                                REG_SETF(CRG_XTAL, CLK_FREQ_TRIM_REG, XTAL32M_TRIM, k_trim);
                                cnt += 1;
                        }
                        hw_clk_delay_usec(10);
                }
                if (REG_GETF(CRG_XTAL, XTAL32M_CTRL0_REG, XTAL32M_CXCOMP_ENABLE)) {
                        hw_clk_delay_usec(10);
                }
        }
}

void hw_clk_xtalm_configure(void)
{
        uint8_t settling_time;
        uint8_t cxcomp_phi_trim;

        // load preferred settings
        REG_SETF(CRG_XTAL, XTAL32M_CTRL2_REG, XTAL32M_CXCOMP_BIAS_TRIM, 0x2);
        REG_SETF(CRG_XTAL, XTAL32M_CTRL2_REG, XTAL32M_FREQ_DET_LEN, 0x2);
        REG_SET_BIT(CRG_XTAL, XTAL32M_CTRL4_REG, XTAL32M_CORE_CUR_PTAT);
        REG_SET_BIT(CRG_XTAL, XTAL32M_CTRL4_REG, XTAL32M_CORE_CUR_NTAT_DIS);

        xtal32m_readOTP();

        REG_SETF(CRG_XTAL, XTAL32M_CTRL0_REG, XTAL32M_CORE_CUR_SET, 3);                          // gmopt cur set.

        uint32_t reg = CRG_XTAL->XTAL32M_CTRL1_REG;
        REG_SET_FIELD(CRG_XTAL, XTAL32M_CTRL1_REG, XTAL32M_RCOSC_SYNC_DELAY_TRIM, reg, 0);      // synchronization delay trim
        REG_SET_FIELD(CRG_XTAL, XTAL32M_CTRL1_REG, XTAL32M_STARTUP_TDISCHARGE,    reg, 0);      // discharge time in drive sequence
        REG_SET_FIELD(CRG_XTAL, XTAL32M_CTRL1_REG, XTAL32M_STARTUP_TSETTLE,       reg, 5);      // required settling time
        settling_time = 6;
        REG_SET_FIELD(CRG_XTAL, XTAL32M_CTRL1_REG, XTAL32M_STARTUP_TDRIVE,        reg, settling_time);      // unit drive length
        CRG_XTAL->XTAL32M_CTRL1_REG = reg;

        REG_SETF(CRG_XTAL, XTAL32M_CTRL2_REG, XTAL32M_RCOSC_TRIM_SNS, 118);                     // sensitivity of rcosc

        reg = CRG_XTAL->XTAL32M_CTRL3_REG;
        REG_SET_FIELD(CRG_XTAL, XTAL32M_CTRL3_REG, XTAL32M_RCOSC_TRIM, reg, 655);               // initial condition for RCOSC_TRIM
        REG_SET_FIELD(CRG_XTAL, XTAL32M_CTRL3_REG, XTAL32M_RCOSC_BAND_SELECT, reg, 4);          // initial condition for RCOSC_BAND_SELECT
        CRG_XTAL->XTAL32M_CTRL3_REG = reg;

        reg = CRG_XTAL->CLK_FREQ_TRIM_REG;
        if ((reg & CRG_XTAL_CLK_FREQ_TRIM_REG_XTAL32M_TRIM_Msk) == 0x170) {                     // if the register has the default value
                REG_SET_FIELD(CRG_XTAL, CLK_FREQ_TRIM_REG, XTAL32M_TRIM, reg, 0x120);           // set frequency trimming to default 0x120
        }
        REG_SET_FIELD(CRG_XTAL, CLK_FREQ_TRIM_REG, XTAL32M_START, reg, 0);                      // start at lowest load capacitance (fast startup)
        CRG_XTAL->CLK_FREQ_TRIM_REG = reg;



        REG_SET_BIT(CRG_XTAL, XTAL32M_CTRL3_REG, XTAL32M_RCOSC_TRIM_STROBE);
        REG_CLR_BIT(CRG_XTAL, XTAL32M_CTRL3_REG, XTAL32M_RCOSC_TRIM_STROBE);

        // sets TSETTLE to half of TDRIVE
        REG_SETF(CRG_XTAL, XTAL32M_CTRL1_REG, XTAL32M_STARTUP_TSETTLE, settling_time);

        REG_SETF(CRG_XTAL, XTAL32M_CTRL0_REG, XTAL32M_RCOSC_XTAL_DRIVE, 0x1);

#if USE_FAST_STARTUP
        uint16_t T_drive, T_drive_lsb;
        uint8_t N;

        uint16_t divisor = (0x8 << (0x7 - REG_GETF(CRG_XTAL, XTAL32M_CTRL1_REG, XTAL32M_STARTUP_TDRIVE)));

        // equate FASTBOOT drive-cycles
        T_drive = xtalm_otp_values.base_drive_cycles;
        N = T_drive / divisor;
        T_drive_lsb = T_drive - divisor * N;

        REG_SETF(CRG_XTAL, XTAL32M_CTRL1_REG, XTAL32M_STARTUP_TDRIVE_LSB, T_drive_lsb);
        REG_SETF(CRG_XTAL, XTAL32M_CTRL1_REG, XTAL32M_DRIVE_CYCLES, N + 1);

        // fast startup mode
        REG_SETF(CRG_XTAL, TRIM_CTRL_REG, XTAL_TRIM_SELECT, 0x2);                       // always use direct trimming (disable legacy)

        // ~10us for ldo settling and 2 cycles for xtal voltage settling. IRQ handler also takes some time.
        // hclk divider needs to be set to 0
        uint16_t xtalrdy_cnt = T_drive/83 + 3 + 2;

        // Setup IRQ:
        REG_SET_BIT(CRG_XTAL, XTALRDY_CTRL_REG, XTALRDY_CLK_SEL);                     // use 256kHz clock
        REG_SETF(CRG_XTAL, XTALRDY_CTRL_REG, XTALRDY_CNT, xtalrdy_cnt);
        REG_SET_BIT(CRG_XTAL, XTAL32M_CTRL0_REG, XTAL32M_RCOSC_CALIBRATE);
        REG_CLR_BIT(CRG_XTAL, XTAL32M_CTRL0_REG, XTAL32M_CXCOMP_ENABLE);
#else
        // Configure OSF BOOST
        REG_SETF(CRG_XTAL, XTAL32M_CTRL0_REG, XTAL32M_CXCOMP_ENABLE, xtalm_otp_values.cxcomp_trim_cap != 0x00);

        // set phi compensation
        if (xtalm_otp_values.cxcomp_trim_cap < 37) {
                cxcomp_phi_trim = 3;
        } else {
                if (xtalm_otp_values.cxcomp_trim_cap < 123)
                        cxcomp_phi_trim = 2;
                else {
                        if (xtalm_otp_values.cxcomp_trim_cap < 170) {
                                cxcomp_phi_trim = 1;
                        }
                        else {
                                cxcomp_phi_trim = 0;
                        }
                }
        }

        REG_SETF(CRG_XTAL, XTAL32M_CTRL2_REG, XTAL32M_CXCOMP_TRIM_CAP, xtalm_otp_values.cxcomp_trim_cap);
        REG_SETF(CRG_XTAL, XTAL32M_CTRL2_REG, XTAL32M_CXCOMP_PHI_TRIM, cxcomp_phi_trim);

        // slow startup mode - use a small burst of the rcosc; very low accuracy is then required
        REG_SETF(CRG_XTAL, TRIM_CTRL_REG, XTAL_TRIM_SELECT, 0x1);               // use two-step trimming (with amplitude detection/xtal_ok filter)
        REG_SETF(CRG_XTAL, XTAL32M_CTRL1_REG, XTAL32M_STARTUP_TDRIVE_LSB, 0x5);
        REG_SETF(CRG_XTAL, XTAL32M_CTRL1_REG, XTAL32M_DRIVE_CYCLES, 1);


        //Setup IRQ:
        REG_SETF(CRG_XTAL, XTALRDY_CTRL_REG, XTALRDY_CLK_SEL, 0x0);   // use 32kHz clock
        REG_SETF(CRG_XTAL, XTALRDY_CTRL_REG, XTALRDY_CNT, 64);        // initial value; is updated in the IRQ handler
        REG_SETF(CRG_XTAL, XTAL32M_CTRL0_REG, XTAL32M_RCOSC_CALIBRATE, 0x0);
#endif
}
/**
\}
\}
\}
*/
