/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
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
 * @file serial.c
 *
 * Serial communication for the PX4IO module.
 */

#include <stdint.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <string.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

/* XXX might be able to prune these */
#include <chip.h>
#include <arm_internal.h>
#include <arm_arch.h>
#include <stm32.h>

//#define DEBUG
#include "px4io.h"

#if defined(PX4IO_PERF)
# include <perf/perf_counter.h>

static perf_counter_t	pc_txns;
static perf_counter_t	pc_errors;
static perf_counter_t	pc_ore;
static perf_counter_t	pc_fe;
static perf_counter_t	pc_ne;
static perf_counter_t	pc_idle;
static perf_counter_t	pc_badidle;
static perf_counter_t	pc_regerr;
static perf_counter_t	pc_crcerr;
#endif

static void		rx_handle_packet(void);
static void		rx_dma_callback(DMA_HANDLE handle, uint8_t status, void *arg);
static DMA_HANDLE	tx_dma;
static DMA_HANDLE	rx_dma;

static int		serial_interrupt(int irq, void *context, FAR void *arg);
static void		dma_reset(void);

static struct IOPacket	dma_packet;

/* serial register accessors */
#define REG(_x)		(*(volatile uint32_t *)(PX4FMU_SERIAL_BASE + _x))
#define rSR		REG(STM32_USART_SR_OFFSET)
#define rDR		REG(STM32_USART_DR_OFFSET)
#define rBRR		REG(STM32_USART_BRR_OFFSET)
#define rCR1		REG(STM32_USART_CR1_OFFSET)
#define rCR2		REG(STM32_USART_CR2_OFFSET)
#define rCR3		REG(STM32_USART_CR3_OFFSET)
#define rGTPR		REG(STM32_USART_GTPR_OFFSET)

void
interface_init(void)
{
#if defined(PX4IO_PERF)
	pc_txns = perf_alloc(PC_ELAPSED, "txns");
	pc_errors = perf_alloc(PC_COUNT, "errors");
	pc_ore = perf_alloc(PC_COUNT, "overrun");
	pc_fe = perf_alloc(PC_COUNT, "framing");
	pc_ne = perf_alloc(PC_COUNT, "noise");
	pc_idle = perf_alloc(PC_COUNT, "idle");
	pc_badidle = perf_alloc(PC_COUNT, "badidle");
	pc_regerr = perf_alloc(PC_COUNT, "regerr");
	pc_crcerr = perf_alloc(PC_COUNT, "crcerr");
#endif

	/* allocate DMA */
	tx_dma = stm32_dmachannel(PX4FMU_SERIAL_TX_DMA);
	rx_dma = stm32_dmachannel(PX4FMU_SERIAL_RX_DMA);

	/* configure pins for serial use */
	px4_arch_configgpio(PX4FMU_SERIAL_TX_GPIO);
	px4_arch_configgpio(PX4FMU_SERIAL_RX_GPIO);

	/* reset and configure the UART */
	rCR1 = 0;
	rCR2 = 0;
	rCR3 = 0;

	/* clear status/errors */
	(void)rSR;
	(void)rDR;

	/* configure line speed */
	uint32_t usartdiv32 = PX4FMU_SERIAL_CLOCK / (PX4FMU_SERIAL_BITRATE / 2);
	uint32_t mantissa = usartdiv32 >> 5;
	uint32_t fraction = (usartdiv32 - (mantissa << 5) + 1) >> 1;
	rBRR = (mantissa << USART_BRR_MANT_SHIFT) | (fraction << USART_BRR_FRAC_SHIFT);

	/* connect our interrupt */
	irq_attach(PX4FMU_SERIAL_VECTOR, serial_interrupt, NULL);
	up_enable_irq(PX4FMU_SERIAL_VECTOR);

	/* enable UART and error/idle interrupts */
	rCR3 = USART_CR3_EIE;
	rCR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_UE | USART_CR1_IDLEIE;

#if 0	/* keep this for signal integrity testing */

	for (;;) {
		while (!(rSR & USART_SR_TXE))
			;

		rDR = 0xfa;

		while (!(rSR & USART_SR_TXE))
			;

		rDR = 0xa0;
	}

#endif

	/* configure RX DMA and return to listening state */
	dma_reset();

	debug("serial init");
}

static void
rx_handle_packet(void)
{
	/* check packet CRC */
	uint8_t crc = dma_packet.crc;
	dma_packet.crc = 0;

	if (crc != crc_packet(&dma_packet)) {
#if defined(PX4IO_PERF)
		perf_count(pc_crcerr);
#endif

		/* send a CRC error reply */
		dma_packet.count_code = PKT_CODE_CORRUPT;
		dma_packet.page = 0xff;
		dma_packet.offset = 0xff;

		return;
	}

	if (PKT_CODE(dma_packet) == PKT_CODE_WRITE) {

		/* it's a blind write - pass it on */
		if (registers_set(dma_packet.page, dma_packet.offset, &dma_packet.regs[0], PKT_COUNT(dma_packet))) {
#if defined(PX4IO_PERF)
			perf_count(pc_regerr);
#endif

			dma_packet.count_code = PKT_CODE_ERROR;

		} else {
			dma_packet.count_code = PKT_CODE_SUCCESS;
		}

		return;
	}

	if (PKT_CODE(dma_packet) == PKT_CODE_READ) {

		/* it's a read - get register pointer for reply */
		unsigned count;
		uint16_t *registers;

		if (registers_get(dma_packet.page, dma_packet.offset, &registers, &count) < 0) {
#if defined(PX4IO_PERF)
			perf_count(pc_regerr);
#endif

			dma_packet.count_code = PKT_CODE_ERROR;

		} else {
			/* constrain reply to requested size */
			if (count > PKT_MAX_REGS) {
				count = PKT_MAX_REGS;
			}

			if (count > PKT_COUNT(dma_packet)) {
				count = PKT_COUNT(dma_packet);
			}

			/* copy reply registers into DMA buffer */
			memcpy((void *)&dma_packet.regs[0], registers, count * 2);
			dma_packet.count_code = count | PKT_CODE_SUCCESS;
		}

		return;
	}

	/* send a bad-packet error reply */
	dma_packet.count_code = PKT_CODE_CORRUPT;
	dma_packet.page = 0xff;
	dma_packet.offset = 0xfe;
}

static void
rx_dma_callback(DMA_HANDLE handle, uint8_t status, void *arg)
{
	/*
	 * We are here because DMA completed, or UART reception stopped and
	 * we think we have a packet in the buffer.
	 */
#if defined(PX4IO_PERF)
	perf_begin(pc_txns);
#endif

	/* disable UART DMA */
	rCR3 &= ~(USART_CR3_DMAT | USART_CR3_DMAR);

	/* handle the received packet */
	rx_handle_packet();

	/* re-set DMA for reception first, so we are ready to receive before we start sending */
	dma_reset();

	/* send the reply to the just-processed request */
	dma_packet.crc = 0;
	dma_packet.crc = crc_packet(&dma_packet);
	stm32_dmasetup(
		tx_dma,
		(uint32_t)&rDR,
		(uint32_t)&dma_packet,
		PKT_SIZE(dma_packet),
		DMA_CCR_DIR		|
		DMA_CCR_MINC		|
		DMA_CCR_PSIZE_8BITS	|
		DMA_CCR_MSIZE_8BITS);
	stm32_dmastart(tx_dma, NULL, NULL, false);
	rCR3 |= USART_CR3_DMAT;

#if defined(PX4IO_PERF)
	perf_end(pc_txns);
#endif
}

static int
serial_interrupt(int irq, void *context, FAR void *arg)
{
	static bool abort_on_idle = false;

	uint32_t sr = rSR;	/* get UART status register */
	(void)rDR;		/* required to clear any of the interrupt status that brought us here */

	if (sr & (USART_SR_ORE |	/* overrun error - packet was too big for DMA or DMA was too slow */
		  USART_SR_NE |		/* noise error - we have lost a byte due to noise */
		  USART_SR_FE)) {		/* framing error - start/stop bit lost or line break */

#if defined(PX4IO_PERF)
		perf_count(pc_errors);

		if (sr & USART_SR_ORE) {
			perf_count(pc_ore);
		}

		if (sr & USART_SR_NE) {
			perf_count(pc_ne);
		}

		if (sr & USART_SR_FE) {
			perf_count(pc_fe);
		}

#endif

		/* send a line break - this will abort transmission/reception on the other end */
		rCR1 |= USART_CR1_SBK;

		/* when the line goes idle, abort rather than look at the packet */
		abort_on_idle = true;
	}

	if (sr & USART_SR_IDLE) {

		/*
		 * If we saw an error, don't bother looking at the packet - it should have
		 * been aborted by the sender and will definitely be bad. Get the DMA reconfigured
		 * ready for their retry.
		 */
		if (abort_on_idle) {

			abort_on_idle = false;
			dma_reset();
			return 0;
		}

		/*
		 * The sender has stopped sending - this is probably the end of a packet.
		 * Check the received length against the length in the header to see if
		 * we have something that looks like a packet.
		 */
		unsigned length = sizeof(dma_packet) - stm32_dmaresidual(rx_dma);

		if ((length < 1) || (length < PKT_SIZE(dma_packet))) {

			/* it was too short - possibly truncated */
#if defined(PX4IO_PERF)
			perf_count(pc_badidle);
#endif
			dma_reset();
			return 0;
		}

		/*
		 * Looks like we received a packet. Stop the DMA and go process the
		 * packet.
		 */
#if defined(PX4IO_PERF)
		perf_count(pc_idle);
#endif
		stm32_dmastop(rx_dma);
		rx_dma_callback(rx_dma, DMA_STATUS_TCIF, NULL);
	}

	return 0;
}

static void
dma_reset(void)
{
	rCR3 &= ~(USART_CR3_DMAT | USART_CR3_DMAR);
	(void)rSR;
	(void)rDR;
	(void)rDR;

	/* kill any pending DMA */
	stm32_dmastop(tx_dma);
	stm32_dmastop(rx_dma);

	/* reset the RX side */
	stm32_dmasetup(
		rx_dma,
		(uint32_t)&rDR,
		(uint32_t)&dma_packet,
		sizeof(dma_packet),
		DMA_CCR_MINC		|
		DMA_CCR_PSIZE_8BITS	|
		DMA_CCR_MSIZE_8BITS     |
		DMA_CCR_PRIVERYHI);

	/* start receive DMA ready for the next packet */
	stm32_dmastart(rx_dma, rx_dma_callback, NULL, false);
	rCR3 |= USART_CR3_DMAR;
}

