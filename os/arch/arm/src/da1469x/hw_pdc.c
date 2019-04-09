/**
 * \addtogroup BSP
 * \{
 * \addtogroup DEVICES
 * \{
 * \addtogroup PDC
 * \{
 */

/**
 ****************************************************************************************
 *
 * @file hw_pdc.c
 *
 * @brief Implementation of the Power Domains Controller Low Level Driver.
 *
 * Copyright (C) 2017-2018 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#include "hw_pdc.h"

__RETAINED_CODE uint32_t hw_pdc_add_entry(uint32_t lut_entry)
{
        uint32_t idx = HW_PDC_INVALID_LUT_INDEX;

        for (int i = 0; i < HW_PDC_LUT_SIZE; ++i) {
                if (*(&PDC->PDC_CTRL0_REG + i) == HW_PDC_UNUSED_LUT_ENTRY_VALUE) {
                        idx = i;
                        *(&PDC->PDC_CTRL0_REG + i) = lut_entry;
                        break;
                }
        }

        return idx;
}

uint32_t hw_pdc_remove_entry(uint32_t idx)
{
        uint32_t old_value = hw_pdc_read_entry(idx);

        hw_pdc_write_entry(idx, HW_PDC_UNUSED_LUT_ENTRY_VALUE);

        return old_value;
}

void hw_pdc_ack_all_pending_cm33(void)
{
        uint32_t pending = hw_pdc_get_pending_cm33();

        for (int i = 0; i < HW_PDC_LUT_SIZE; ++i) {
                if (pending & (1 << i) ) {
                        hw_pdc_acknowledge(i);
                }
        }
}

void hw_pdc_lut_reset(void)
{
        for (int i = 0; i < HW_PDC_LUT_SIZE; ++i) {
                *(&PDC->PDC_CTRL0_REG + i) = HW_PDC_UNUSED_LUT_ENTRY_VALUE;
                hw_pdc_acknowledge(i);
        }
}
/**
 * \}
 * \}
 * \}
 */
