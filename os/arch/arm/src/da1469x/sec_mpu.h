/**
 ****************************************************************************************
 *
 * @file sec_mpu.h
 *
 * @brief Samsung MPU header file.
 *
 * Copyright (C) 2018-2019 Samsung Electronics Co. Ltd.
 * This computer program includes Confidential, Proprietary Information
 * of Samsung Electronics Co. Ltd.. All Rights Reserved.
 *
 ****************************************************************************************
 */


#ifndef SEC_MPU_H_
#define SEC_MPU_H_

#include <stdint.h>
#include <stdbool.h>


#ifdef CONFIG_USE_SEC_MPU

void sec_mpu_init_bss_ext_ram(void);
void sec_mpu_enable(int enable, int use_default_memory_map);
void sec_mpu_setup_default_memory_region(void);
void sec_mpu_store_memory_region(void *memory_regions);

#else

static inline void sec_mpu_init_bss_ext_ram(void) {}
static inline void sec_mpu_enable(int enable, int use_default_memory_map) {}
static inline void sec_mpu_setup_default_memory_region(void) {}
static inline void sec_mpu_store_memory_region(void *memory_regions) {}

#endif /* CONFIG_USE_SEC_MPU */

#endif /* SEC_MPU_H_ */
