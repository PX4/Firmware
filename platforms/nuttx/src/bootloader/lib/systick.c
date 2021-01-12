/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "arm_arch.h"
#include "systick.h"
#include <nvic.h>

uint8_t systick_get_countflag(void)
{
	return (getreg32(NVIC_SYSTICK_CTRL) & NVIC_SYSTICK_CTRL_COUNTFLAG) ? 1 : 0;
}

// See 2.2.3 SysTick external clock is not HCLK/8
uint32_t g_divisor = 1;
void systick_set_reload(uint32_t value)
{
	putreg32((((value * g_divisor) << NVIC_SYSTICK_RELOAD_SHIFT) & NVIC_SYSTICK_RELOAD_MASK), NVIC_SYSTICK_RELOAD);
}


void systick_set_clocksource(uint8_t clocksource)
{
	g_divisor = (clocksource == CLKSOURCE_EXTERNAL) ? 8 : 1;
	modifyreg32(NVIC_SYSTICK_CTRL, NVIC_SYSTICK_CTRL_CLKSOURCE, clocksource & NVIC_SYSTICK_CTRL_CLKSOURCE);
}

void systick_counter_enable(void)
{
	modifyreg32(NVIC_SYSTICK_CTRL, 0, NVIC_SYSTICK_CTRL_ENABLE);
}

void systick_counter_disable(void)
{
	modifyreg32(NVIC_SYSTICK_CTRL, NVIC_SYSTICK_CTRL_ENABLE, 0);
	putreg32(0, NVIC_SYSTICK_CURRENT);
}

void systick_interrupt_enable(void)
{
	modifyreg32(NVIC_SYSTICK_CTRL, 0, NVIC_SYSTICK_CTRL_TICKINT);
}

void systick_interrupt_disable(void)
{
	modifyreg32(NVIC_SYSTICK_CTRL, NVIC_SYSTICK_CTRL_TICKINT, 0);
}
