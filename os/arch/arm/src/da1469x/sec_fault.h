/**
 ****************************************************************************************
 *
 * @file sec_fault.h
 *
 * @brief Samsung Fault Handler header file.
 *
 * Copyright (C) 2018-2019 Samsung Electronics Co. Ltd.
 * This computer program includes Confidential, Proprietary Information
 * of Samsung Electronics Co. Ltd.. All Rights Reserved.
 *
 ****************************************************************************************
 */


#ifndef SEC_FAULT_H_
#define SEC_FAULT_H_

#include <stdint.h>
#include <stdbool.h>


#ifdef CONFIG_USE_SEC_FAULT

static inline void sec_fault_test_fault(void);
static inline void sec_fault_report_fault_info(unsigned long *fault_args, unsigned int exc);

#else

static inline void sec_fault_test_fault(void) {}
static inline void sec_fault_report_fault_info(unsigned long *fault_args, unsigned int exc) {}

#endif /* CONFIG_USE_SEC_FAULT */

#endif /* SEC_FAULT_H_ */
