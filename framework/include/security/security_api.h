/****************************************************************************
 *
 * Copyright 2019 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ****************************************************************************/
#ifndef __SECURITY_API_H__
#define __SECURITY_API_H__

#include "security_auth.h"
#include "security_crypto.h"
#include "security_keymgr.h"
#include "security_ss.h"

#if (defined(CONFIG_ARCH_BOARD_ARTIK053S) || defined(CONFIG_ARCH_BOARD_ARTIK053)) \
			&& defined(CONFIG_S5J_HAVE_SSS)
#include "../../../os/arch/arm/src/s5j/sss/isp_custom.h"
#endif

#endif // __SECURITY_API_H__
