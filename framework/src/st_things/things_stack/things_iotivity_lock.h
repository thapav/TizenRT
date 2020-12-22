/****************************************************************************
 *
 * Copyright 2018 Samsung Electronics All Rights Reserved.
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

#ifndef THINGS_IOTIVITY_LOCK_H_
#define THINGS_IOTIVITY_LOCK_H_

#include <pthread.h>

/* Initializes the mutex variables. Returns 'true' if initialization is successful.
This function should not be called more than once. */
void init_iotivity_api_lock(void);

/* Locks the mutex. Returns 'true' if lock is successful.
This function should be called after init_iotivity_api_lock(). */
void iotivity_api_lock(void);

/* Unlocks the mutex. Returns 'true' if unlock is successful.
This function should be called after init_iotivity_api_lock(). */
void iotivity_api_unlock(void);

/* De-initializes the mutex variables. Returns 'true' if de-initialization is successful.
This function should be called after init_iotivity_api_lock(). */
void deinit_iotivity_api_lock(void);

#endif // THINGS_IOTIVITY_LOCK_H_
