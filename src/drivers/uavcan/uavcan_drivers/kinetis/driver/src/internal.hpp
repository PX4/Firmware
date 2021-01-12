/*
 * Copyright (C) 2014, 2018 Pavel Kirienko <pavel.kirienko@gmail.com>
 * Kinetis Port Author David Sidrane <david_s5@nscdg.com>
 */

#pragma once

#include <uavcan_kinetis/build_config.hpp>

#if UAVCAN_KINETIS_NUTTX
# include <nuttx/arch.h>
# include "arm_arch.h"
# include <arch/board/board.h>
# include <hardware/kinetis_pit.h>
# include <hardware/kinetis_sim.h>
# include <syslog.h>
#else
# error "Unknown OS"
#endif

/**
 * Debug output
 */
#ifndef UAVCAN_KINETIS_LOG
# if 1
#  define UAVCAN_KINETIS_LOG(fmt, ...)  syslog(LOG_INFO, "uavcan_kinetis: " fmt "\n", ## __VA_ARGS__)
# else
#  define UAVCAN_KINETIS_LOG(...)       ((void)0)
# endif
#endif

/**
 * IRQ handler macros
 */
#define UAVCAN_KINETIS_IRQ_HANDLER(id)  int id(int irq, FAR void* context, FAR void *arg)


/**
 * Glue macros
 */
#define UAVCAN_KINETIS_GLUE2_(A, B)       A ## B
#define UAVCAN_KINETIS_GLUE2(A, B)        UAVCAN_KINETIS_GLUE2_(A, B)

#define UAVCAN_KINETIS_GLUE3_(A, B, C)    A ## B ## C
#define UAVCAN_KINETIS_GLUE3(A, B, C)     UAVCAN_KINETIS_GLUE3_(A, B, C)

namespace uavcan_kinetis
{
#if UAVCAN_KINETIS_NUTTX

struct CriticalSectionLocker {
	const irqstate_t flags_;

	CriticalSectionLocker()
		: flags_(enter_critical_section())
	{
	}

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
