/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#pragma once

/*
|    DMA1    | Stream 0         | Stream 1         | Stream 2         | Stream 3         | Stream 4         | Stream 5         | Stream 6         | Stream 7         |
|------------|------------------|------------------|------------------|------------------|------------------|------------------|------------------|------------------|
| Channel 0  | SPI3_RX_1        | SPDIFRX_DT       | SPI3_RX_2        | SPI2_RX          | SPI2_TX          | SPI3_TX_1        | SPDIFRX_CS       | SPI3_TX_2        |
| Channel 1  | I2C1_RX          | I2C3_RX          | TIM7_UP_1        | -                | TIM7_UP_2        |  I2C1_RX_1       | I2C1_TX          | I2C1_TX_1        |
| Channel 2  | TIM4_CH1         | -                | I2C4_RX          | TIM4_CH2         | -                | I2C4_RX          | TIM4_UP          | TIM4_CH3         |
| Channel 3  | -                | TIM2_UP_1        | I2C3_RX_1        | -                | I2C3_TX          | TIM2_CH1         | TIM2_CH2         | TIM2_UP_2        |
|            |                  |   TIM2_CH3       |                  |                  |                  |                  |   TIM2_CH4_1     |   TIM2_CH4_2     |
| Channel 4  | UART5_RX         | USART3_RX        | UART4_RX         | USART3_TX_1      | UART4_TX         | USART2_RX        | USART2_TX        | UART5_TX         |
| Channel 5  | UART8_TX         | UART7_TX         | TIM3_CH4         | UART7_RX         | TIM3_CH1         | TIM3_CH2         | UART8_RX         | TIM3_CH3         |
|            |                  |                  |   TIM3_UP        |                  |   TIM3_TRIG      |                  |                  |                  |
| Channel 6  | TIM5_CH3         | TIM5_CH4_1       | TIM5_CH1         | TIM5_CH4_2       | TIM5_CH2         | -                | TIM5_UP_2        | -                |
|            |   TIM5_UP_1      |   TIM5_TRIG_1    |                  |   TIM5_TRIG_2    |                  |                  |                  |                  |
| Channel 7  | -                | TIM6_UP          | I2C2_RX          | I2C2_RX_1        | USART3_TX_2      | DAC1             | DAC2             | I2C2_TX          |
| Channel 8  | I2C3_TX          | I2C4_RX          | -                | -                | I2C2_TX          | -                | I2C4_TX          | -                |
| Channel 9  | -                | SPI2_RX          | -                | -                | -                | -                | SPI2_TX          | -                |
|            |                  |                  |                  |                  |                  |                  |                  |                  |
| Usage      |                  | SPI2_RX          | SPI3_RX_2        | UART7_RX         | SPI2_TX          | SPI3_TX_1        | TIM4_UP          |                  |


|    DMA2    | Stream 0         | Stream 1         | Stream 2         | Stream 3         | Stream 4         | Stream 5         | Stream 6         | Stream 7         |
|------------|------------------|------------------|------------------|------------------|------------------|------------------|------------------|------------------|
| Channel 0  | ADC1_1           | SAI1_A           | TIM8_CH1_1       | SAI1_A_1         | ADC1_2           | SAI1_B_1         | TIM1_CH1_1       | SAI2_B_2         |
|            |                  |                  |   TIM8_CH2_1     |                  |                  |                  |   TIM1_CH2_1     |                  |
|            |                  |                  |   TIM8_CH3_1     |                  |                  |                  |   TIM1_CH3_1     |                  |
| Channel 1  | -                | DCMI_1           | ADC2_1           | ADC2_2           | SAI1_B           | SPI6_TX          | SPI6_RX          | DCMI_2           |
| Channel 2  | ADC3_1           | ADC3_2           | -                | SPI5_RX_1        | SPI5_TX_1        | CRYP_OUT         | CRYP_IN          | HASH_IN          |
| Channel 3  | SPI1_RX_1        | -                | SPI1_RX_2        | SPI1_TX_1        | SAI2_A           | SPI1_TX_2        | SAI2_B           | QUADSPI          |
| Channel 4  | SPI4_RX_1        | SPI4_TX_1        | USART1_RX_1      | SDMMC1_1         | -                | USART1_RX_2      | SDMMC1_2         | USART1_TX        |
| Channel 5  | -                | USART6_RX_1      | USART6_RX_2      | SPI4_RX_2        | SPI4_TX_2        | -                | USART6_TX_1      | USART6_TX_2      |
| Channel 6  | TIM1_TRIG_1      | TIM1_CH1_2       | TIM1_CH2_2       | TIM1_CH1         | TIM1_CH4         | TIM1_UP          | TIM1_CH3_2       | -                |
|            |                  |                  |                  |                  |   TIM1_TRIG_2    |                  |                  |                  |
|            |                  |                  |                  |                  |   TIM1_COM       |                  |                  |                  |
| Channel 7  | -                | TIM8_UP          | TIM8_CH1_2       | TIM8_CH2_2       | TIM8_CH3_2       | SPI5_RX_2        | SPI5_TX_2        | TIM8_CH4         |
|            |                  |                  |                  |                  |                  |                  |                  |   TIM8_TRIG      |
|            |                  |                  |                  |                  |                  |                  |                  |   TIM8_COM       |
| Channel 8  | DSFDM1_FLT0      | DSFDM1_FLT1      | DSFDM1_FLT2      | DSFDM1_FLT3      | DSFDM1_FLT0      | DSFDM1_FLT1      | DSFDM1_FLT2      | DSFDM1_FLT3      |
| Channel 9  | JPEG_IN          | JPEG_OUT         | SPI4_TX          | JPEG_IN          | JPEG_OUT         | SPI5_RX          | -                | -                |
| Channel 10 | SAI1_B           | SAI2_B           | SAI2_A           | -                | -                | -                | SAI1_A           | -                |
| Channel 11 | SDMMC2           | -                | QUADSPI          | -                | -                | SDMMC2           | -                | -                |
|            |                  |                  |                  |                  |                  |                  |                  |                  |
| Usage      | SDMMC2           | USART6_RX_1      | SPI1_RX_2        | SPI1_TX_1        |                  | TIM1_UP          |                  | USART6_TX_2      |
 */

// DMA1 Channel/Stream Selections
//--------------------------------------------//---------------------------//----------------
//      DMAMAP_UART5_RX                       // DMA1, Stream 0, Channel 4    (TELEM2 RX)
#define DMAMAP_SPI2_RX    DMAMAP_SPI2_RX_1    // DMA1, Stream 1, Channel 9    (SPI2 RX)
#define DMAMAP_SPI3_RX    DMAMAP_SPI3_RX_2    // DMA1, Stream 2, Channel 0    (SPI3 RX)
//      DMAMAP_UART7_RX                       // DMA1, Stream 3, Channel 5    (TELEM1 RX)
#define DMAMAP_SPI2_TX    DMAMAP_SPI2_TX_2    // DMA1, Stream 4, Channel 0    (SPI2 TX)
#define DMAMAP_SPI3_TX    DMAMAP_SPI3_TX_1    // DMA1, Stream 5, Channel 0    (SPI3 TX)
//      DMAMAP_TIM4_UP                        // DMA1, Stream 6, Channel 2    (DSHOT)

//  DMA2 Channel/Stream Selections
//--------------------------------------------//---------------------------//----------------
#define DMAMAP_SDMMC2     DMAMAP_SDMMC2_1     // DMA2, Stream 0, Channel 11
#define DMAMAP_USART6_RX  DMAMAP_USART6_RX_1  // DMA2, Stream 1, Channel 5
#define DMAMAP_SPI1_RX    DMAMAP_SPI1_RX_2    // DMA2, Stream 2, Channel 3    (SPI sensors RX)
#define DMAMAP_SPI1_TX    DMAMAP_SPI1_TX_1    // DMA2, Stream 3, Channel 3    (SPI sensors TX)
//      AVAILABLE                             // DMA2, Stream 4
//      DMAMAP_TIM1_UP                        // DMA2, Stream 5, Channel 6    (DSHOT)
//      AVAILABLE                             // DMA2, Stream 6
#define DMAMAP_USART6_TX  DMAMAP_USART6_TX_2  // DMA2, Stream 7, Channel 5
