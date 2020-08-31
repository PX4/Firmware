/****************************************************************************
 *
 *   Copyright (C) 2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file shutdown.h
 * Power-related API
 */

#pragma once

#include <px4_platform_common/px4_config.h>

#include <stdbool.h>
#include <inttypes.h>

__BEGIN_DECLS

/**
 * Shutdown hook callback method (@see px4_register_shutdown_hook()).
 * @return true if it's ok to shutdown, false if more time needed for cleanup
 */
typedef bool (*shutdown_hook_t)(void);


/**
 * Register a method that should be called when powering off (and also on reboot).
 * @param hook callback method. It must not block, but return immediately.
 *        When the system is requested to shutdown, the registered hooks will be
 *        called regularily until either all of them return true, or a timeout
 *        is reached.
 * @return 0 on success, <0 on error
 */
__EXPORT int px4_register_shutdown_hook(shutdown_hook_t hook);


/**
 * Unregister a shutdown hook
 * @param hook callback method to be removed
 * @return 0 on success, <0 on error
 */
__EXPORT int px4_unregister_shutdown_hook(shutdown_hook_t hook);


/**
 * Request the system to reboot.
 * Note the following:
 * - The system might not support reboot. In that case -EINVAL will
 *   be returned.
 * - The system might not shutdown immediately, so expect this method to return even
 *   on success.
 * @param to_bootloader reboot into bootloader mode (only used if reboot is true)
 * @param delay_us optional delay in microseconds
 * @return 0 on success, <0 on error
 */
#if defined(CONFIG_BOARDCTL_RESET)
__EXPORT int px4_reboot_request(bool to_bootloader = false, uint32_t delay_us = 0);
#endif // CONFIG_BOARDCTL_RESET


/**
 * Request the system to shut down or reboot.
 * Note the following:
 * - The system might not support shutdown. In that case -EINVAL will
 *   be returned.
 * - The system might not shutdown immediately, so expect this method to return even
 *   on success.
 * @param delay_us optional delay in microseconds
 * @return 0 on success, <0 on error
 */
#if defined(CONFIG_BOARDCTL_POWEROFF) || defined(__PX4_POSIX)
__EXPORT int px4_shutdown_request(uint32_t delay_us = 0);
#endif // CONFIG_BOARDCTL_POWEROFF


/**
 * Grab the shutdown lock. It will prevent the system from shutting down until the lock is released.
 * It is safe to call this recursively.
 * @return 0 on success, <0 on error
 */
__EXPORT int px4_shutdown_lock(void);


/**
 * Release the shutdown lock.
 * @return 0 on success, <0 on error
 */
__EXPORT int px4_shutdown_unlock(void);


__END_DECLS
