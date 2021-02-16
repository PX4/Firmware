/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan_stm32h7/build_config.hpp>
#include "board_config.h"

#if UAVCAN_STM32H7_NUTTX
# include <nuttx/arch.h>
# include <arch/board/board.h>
# include <hardware/stm32_tim.h>
# include <syslog.h>
#else
# error "Unknown OS"
#endif

/**
 * Debug output
 */
#ifndef UAVCAN_STM32H7_LOG
// syslog() crashes the system in this context
// # if UAVCAN_STM32H7_NUTTX && CONFIG_ARCH_LOWPUTC
# if 0
#  define UAVCAN_STM32H7_LOG(fmt, ...)  printf("uavcan_stm32: \n" fmt "\n", ##__VA_ARGS__)
# else
#  define UAVCAN_STM32H7_LOG(...)       ((void)0)
# endif
#endif

/**
 * IRQ handler macros
 */
#if UAVCAN_STM32H7_NUTTX
# define UAVCAN_STM32H7_IRQ_HANDLER(id)  int id(int irq, FAR void* context, FAR void *arg)
# define UAVCAN_STM32H7_IRQ_PROLOGUE()
# define UAVCAN_STM32H7_IRQ_EPILOGUE()    return 0;
#endif

/**
 * Glue macros
 */
#define UAVCAN_STM32H7_GLUE2_(A, B)       A##B
#define UAVCAN_STM32H7_GLUE2(A, B)        UAVCAN_STM32H7_GLUE2_(A, B)

#define UAVCAN_STM32H7_GLUE3_(A, B, C)    A##B##C
#define UAVCAN_STM32H7_GLUE3(A, B, C)     UAVCAN_STM32H7_GLUE3_(A, B, C)

namespace uavcan_stm32h7
{
#if UAVCAN_STM32H7_NUTTX

struct CriticalSectionLocker {
	const irqstate_t flags_;

	CriticalSectionLocker()
		: flags_(enter_critical_section())
	{ }

	~CriticalSectionLocker()
	{
		leave_critical_section(flags_);
	}
};

#endif

namespace clock
{
uavcan::uint64_t getUtcUSecFromCanInterrupt();
}
}
