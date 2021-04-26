/* ----------------------------------------------------------------------
 * Project:      CMSIS DSP Library
 * Title:        arm_rfft_q15.c
 * Description:  RFFT & RIFFT Q15 process function
 *
 * $Date:        18. March 2019
 * $Revision:    V1.6.0
 *
 * Target Processor: Cortex-M cores
 * -------------------------------------------------------------------- */
/*
 * Copyright (C) 2010-2019 ARM Limited or its affiliates. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "arm_math.h"

/* ----------------------------------------------------------------------
 * Internal functions prototypes
 * -------------------------------------------------------------------- */

void arm_split_rfft_q15(
        q15_t * pSrc,
        uint32_t fftLen,
  const q15_t * pATable,
  const q15_t * pBTable,
        q15_t * pDst,
        uint32_t modifier);

void arm_split_rifft_q15(
        q15_t * pSrc,
        uint32_t fftLen,
  const q15_t * pATable,
  const q15_t * pBTable,
        q15_t * pDst,
        uint32_t modifier);

/**
  @addtogroup RealFFT
  @{
 */

/**
  @brief         Processing function for the Q15 RFFT/RIFFT.
  @param[in]     S     points to an instance of the Q15 RFFT/RIFFT structure
  @param[in]     pSrc  points to input buffer (Source buffer is modified by this function.)
  @param[out]    pDst  points to output buffer
  @return        none

  @par           Input an output formats
                   Internally input is downscaled by 2 for every stage to avoid saturations inside CFFT/CIFFT process.
                   Hence the output format is different for different RFFT sizes.
                   The input and output formats for different RFFT sizes and number of bits to upscale are mentioned in the tables below for RFFT and RIFFT:
  @par
                   \image html RFFTQ15.gif "Input and Output Formats for Q15 RFFT"
  @par
                   \image html RIFFTQ15.gif "Input and Output Formats for Q15 RIFFT"
  @par
                   If the input buffer is of length N, the output buffer must have length 2*N.
                   The input buffer is modified by this function.
 */

void arm_rfft_q15(
  const arm_rfft_instance_q15 * S,
        q15_t * pSrc,
        q15_t * pDst)
{
#if defined(ARM_MATH_MVEI)
  const arm_cfft_instance_q15 *S_CFFT = &(S->cfftInst);
#else
  const arm_cfft_instance_q15 *S_CFFT = S->pCfft;
#endif
        uint32_t L2 = S->fftLenReal >> 1U;
        uint32_t i;

  /* Calculation of RIFFT of input */
  if (S->ifftFlagR == 1U)
  {
     /*  Real IFFT core process */
     arm_split_rifft_q15 (pSrc, L2, S->pTwiddleAReal, S->pTwiddleBReal, pDst, S->twidCoefRModifier);

     /* Complex IFFT process */
     arm_cfft_q15 (S_CFFT, pDst, S->ifftFlagR, S->bitReverseFlagR);

     for(i = 0; i < S->fftLenReal; i++)
     {
        pDst[i] = pDst[i] << 1U;
     }
  }
  else
  {
     /* Calculation of RFFT of input */

     /* Complex FFT process */
     arm_cfft_q15 (S_CFFT, pSrc, S->ifftFlagR, S->bitReverseFlagR);

     /*  Real FFT core process */
     arm_split_rfft_q15 (pSrc, L2, S->pTwiddleAReal, S->pTwiddleBReal, pDst, S->twidCoefRModifier);
  }

}

/**
  @} end of RealFFT group
 */

/**
  @brief         Core Real FFT process
  @param[in]     pSrc      points to input buffer
  @param[in]     fftLen    length of FFT
  @param[in]     pATable   points to twiddle Coef A buffer
  @param[in]     pBTable   points to twiddle Coef B buffer
  @param[out]    pDst      points to output buffer
  @param[in]     modifier  twiddle coefficient modifier that supports different size FFTs with the same twiddle factor table
  @return        none

  @par
                   The function implements a Real FFT
 */

#if defined(ARM_MATH_MVEI)
void arm_split_rfft_q15(
        q15_t * pSrc,
        uint32_t fftLen,
  const q15_t * pATable,
  const q15_t * pBTable,
        q15_t * pDst,
        uint32_t modifier)
{
    q15_t const     *pCoefA, *pCoefB; /* Temporary pointers for twiddle factors */
    q15_t           *pDst1 = &pDst[2], *pDst2 = &pDst[(4U * fftLen) - 1U - 14]; /* temp pointers for output buffer */
    q15_t const     *pSrc1 = &pSrc[2], *pSrc2 = &pSrc[(2U * fftLen) - 1U - 14]; /* temp pointers for input buffer */
    q15_t const    *pVecSrc1;
    q15_t          *pVecDst1;
    q15x8x2_t      vecIn, vecSum;
    uint32_t         blkCnt;
    uint16x8_t     vecStridesFwd, vecStridesBkwd;
    q15x8_t        vecInBkwd, vecCoefFwd0, vecCoefFwd1;

    /*
     * Init coefficient pointers
     */
    pCoefA = &pATable[modifier * 2U];
    pCoefB = &pBTable[modifier * 2U];
    /*
     * scatter / gather offsets
     * for ascending & descending addressing
     */
    vecStridesFwd = vidupq_u16((uint32_t)0, 2);    // 0, 2, 4, 6, 8, 10, 12, 14
    vecStridesBkwd = vddupq_u16(14, 2);   // 14, 12, 10, 8, 6, 4, 2, 0
    vecStridesFwd = vecStridesFwd * (uint16_t)  modifier;

    pVecSrc1 = (q15_t const *) pSrc1;
    pVecDst1 = pDst1;

    blkCnt = fftLen >> 3;
    while (blkCnt > 0U)
    {
        vecCoefFwd0 = vldrhq_gather_shifted_offset(pCoefA, vecStridesFwd);
        vecCoefFwd1 = vldrhq_gather_shifted_offset(&pCoefA[1], vecStridesFwd);
        vecIn = vld2q(pVecSrc1);
        pVecSrc1 += 16;
        /*
         * outR = *pSrc1 * CoefA1;
         */
        vecSum.val[0] = vrmulhq(vecIn.val[0], vecCoefFwd0);
        /*
         * outI = *pSrc1++ * CoefA2;
         */
        vecSum.val[1] = vrmulhq(vecIn.val[0], vecCoefFwd1);

        vecInBkwd = vldrhq_gather_shifted_offset(pSrc2, vecStridesBkwd);
        /*
         * outR -= (*pSrc1 + *pSrc2) * CoefA2;
         */
        vecInBkwd = vqaddq(vecIn.val[1], vecInBkwd);
        vecSum.val[0] = vqsubq(vecSum.val[0], vrmulhq(vecInBkwd, vecCoefFwd1));

        vecInBkwd = vldrhq_gather_shifted_offset(pSrc2, vecStridesBkwd);
        /*
         * outI += *pSrc1++ * CoefA1;
         */
        vecSum.val[1] = vqaddq(vecSum.val[1], vrmulhq(vecIn.val[1], vecCoefFwd0));

        vecCoefFwd0 = vldrhq_gather_shifted_offset(pCoefB, vecStridesFwd);
        /*
         * outI -= *pSrc2-- * CoefB1;
         */
        vecSum.val[1] = vqsubq(vecSum.val[1], vrmulhq(vecInBkwd, vecCoefFwd0));

        vecInBkwd = vldrhq_gather_shifted_offset(&pSrc2[-1], vecStridesBkwd);
        /*
         * outI -= *pSrc2 * CoefA2;
         */
        vecSum.val[1] = vqsubq(vecSum.val[1], vrmulhq(vecInBkwd, vecCoefFwd1));
        /*
         * outR += *pSrc2-- * CoefB1;
         */
        vecSum.val[0] = vqaddq(vecSum.val[0], vrmulhq(vecInBkwd, vecCoefFwd0));

        vst2q(pVecDst1, vecSum);
        pVecDst1 += 16;
        /*
         * write complex conjugate output
         */
        vecSum.val[1] = -vecSum.val[1];
        vstrhq_scatter_shifted_offset(pDst2, vecStridesBkwd, vecSum.val[1]);
        vstrhq_scatter_shifted_offset(&pDst2[-1], vecStridesBkwd, vecSum.val[0]);
        /*
         * update fwd and backwd offsets
         */
        vecStridesFwd = vecStridesFwd + (uint16_t)(modifier * 16U);
        /* cannot use negative 16-bit offsets (would lead to positive 32-65K jump*/
        //vecStridesBkwd = vecStridesBkwd - (uint16_t)16;
        pSrc2 = pSrc2 - 16;
        pDst2 = pDst2 - 16;

        blkCnt--;
    }

    pDst[2U * fftLen] = (pSrc[0] - pSrc[1]) >> 1;
    pDst[(2U * fftLen) + 1U] = 0;

    pDst[0] = (pSrc[0] + pSrc[1]) >> 1;
    pDst[1] = 0;
}
#else
void arm_split_rfft_q15(
        q15_t * pSrc,
        uint32_t fftLen,
  const q15_t * pATable,
  const q15_t * pBTable,
        q15_t * pDst,
        uint32_t modifier)
{       
        uint32_t i;                                    /* Loop Counter */
        q31_t outR, outI;                              /* Temporary variables for output */
  const q15_t *pCoefA, *pCoefB;                        /* Temporary pointers for twiddle factors */
        q15_t *pSrc1, *pSrc2;
#if defined (ARM_MATH_DSP)
        q15_t *pD1, *pD2;
#endif

  /* Init coefficient pointers */
  pCoefA = &pATable[modifier * 2];
  pCoefB = &pBTable[modifier * 2];

  pSrc1 = &pSrc[2];
  pSrc2 = &pSrc[(2U * fftLen) - 2U];

#if defined (ARM_MATH_DSP)

    i = 1U;
    pD1 = pDst + 2;
    pD2 = pDst + (4U * fftLen) - 2;

    for (i = fftLen - 1; i > 0; i--)
    {
        /*
          outR = (  pSrc[2 * i]             * pATable[2 * i]
                  - pSrc[2 * i + 1]         * pATable[2 * i + 1]
                  + pSrc[2 * n - 2 * i]     * pBTable[2 * i]
                  + pSrc[2 * n - 2 * i + 1] * pBTable[2 * i + 1]);

          outI = (  pIn[2 * i + 1]         * pATable[2 * i]
                  + pIn[2 * i]             * pATable[2 * i + 1]
                  + pIn[2 * n - 2 * i]     * pBTable[2 * i + 1]
                  - pIn[2 * n - 2 * i + 1] * pBTable[2 * i])
         */


#ifndef ARM_MATH_BIG_ENDIAN
        /* pSrc[2 * i] * pATable[2 * i] - pSrc[2 * i + 1] * pATable[2 * i + 1] */
        outR = __SMUSD(read_q15x2 (pSrc1), read_q15x2((q15_t *) pCoefA));
#else
        /* -(pSrc[2 * i + 1] * pATable[2 * i + 1] - pSrc[2 * i] * pATable[2 * i]) */
        outR = -(__SMUSD(read_q15x2 (pSrc1), read_q15x2((q15_t *) pCoefA)));
#endif /* #ifndef ARM_MATH_BIG_ENDIAN */

        /* pSrc[2 * n - 2 * i] * pBTable[2 * i] + pSrc[2 * n - 2 * i + 1] * pBTable[2 * i + 1]) */
        outR = __SMLAD(read_q15x2 (pSrc2), read_q15x2((q15_t *) pCoefB), outR) >> 16U;

        /* pIn[2 * n - 2 * i] * pBTable[2 * i + 1] - pIn[2 * n - 2 * i + 1] * pBTable[2 * i] */
#ifndef ARM_MATH_BIG_ENDIAN
        outI = __SMUSDX(read_q15x2_da (&pSrc2), read_q15x2((q15_t *) pCoefB));
#else
        outI = __SMUSDX(read_q15x2 ((q15_t *) pCoefB), read_q15x2_da (&pSrc2));
#endif /* #ifndef ARM_MATH_BIG_ENDIAN */

        /* (pIn[2 * i + 1] * pATable[2 * i] + pIn[2 * i] * pATable[2 * i + 1] */
        outI = __SMLADX(read_q15x2_ia (&pSrc1), read_q15x2 ((q15_t *) pCoefA), outI);

        /* write output */
        *pD1++ = (q15_t) outR;
        *pD1++ = outI >> 16U;

        /* write complex conjugate output */
        pD2[0] = (q15_t) outR;
        pD2[1] = -(outI >> 16U);
        pD2 -= 2;

        /* update coefficient pointer */
        pCoefB = pCoefB + (2U * modifier);
        pCoefA = pCoefA + (2U * modifier);
    }

    pDst[2U * fftLen]      = (pSrc[0] - pSrc[1]) >> 1U;
    pDst[2U * fftLen + 1U] = 0;

    pDst[0] = (pSrc[0] + pSrc[1]) >> 1U;
    pDst[1] = 0;

#else

    i = 1U;

    while (i < fftLen)
    {
        /*
          outR = (  pSrc[2 * i]             * pATable[2 * i]
                  - pSrc[2 * i + 1]         * pATable[2 * i + 1]
                  + pSrc[2 * n - 2 * i]     * pBTable[2 * i]
                  + pSrc[2 * n - 2 * i + 1] * pBTable[2 * i + 1]);
        */

        outR = *pSrc1 * *pCoefA;
        outR = outR - (*(pSrc1 + 1) * *(pCoefA + 1));
        outR = outR + (*pSrc2 * *pCoefB);
        outR = (outR + (*(pSrc2 + 1) * *(pCoefB + 1))) >> 16;

        /*
          outI = (  pIn[2 * i + 1]         * pATable[2 * i]
                  + pIn[2 * i]             * pATable[2 * i + 1]
                  + pIn[2 * n - 2 * i]     * pBTable[2 * i + 1]
                  - pIn[2 * n - 2 * i + 1] * pBTable[2 * i]);
        */

        outI = *pSrc2 * *(pCoefB + 1);
        outI = outI - (*(pSrc2 + 1) * *pCoefB);
        outI = outI + (*(pSrc1 + 1) * *pCoefA);
        outI = outI + (*pSrc1 * *(pCoefA + 1));

        /* update input pointers */
        pSrc1 += 2U;
        pSrc2 -= 2U;

        /* write output */
        pDst[2U * i] = (q15_t) outR;
        pDst[2U * i + 1U] = outI >> 16U;

        /* write complex conjugate output */
        pDst[(4U * fftLen) - (2U * i)] = (q15_t) outR;
        pDst[((4U * fftLen) - (2U * i)) + 1U] = -(outI >> 16U);

        /* update coefficient pointer */
        pCoefB = pCoefB + (2U * modifier);
        pCoefA = pCoefA + (2U * modifier);

        i++;
    }

    pDst[2U * fftLen] = (pSrc[0] - pSrc[1]) >> 1;
    pDst[2U * fftLen + 1U] = 0;

    pDst[0] = (pSrc[0] + pSrc[1]) >> 1;
    pDst[1] = 0;

#endif /* #if defined (ARM_MATH_DSP) */
}
#endif /* defined(ARM_MATH_MVEI) */

/**
  @brief         Core Real IFFT process
  @param[in]     pSrc      points to input buffer
  @param[in]     fftLen    length of FFT
  @param[in]     pATable   points to twiddle Coef A buffer
  @param[in]     pBTable   points to twiddle Coef B buffer
  @param[out]    pDst      points to output buffer
  @param[in]     modifier  twiddle coefficient modifier that supports different size FFTs with the same twiddle factor table
  @return        none

  @par
                   The function implements a Real IFFT
 */

#if defined(ARM_MATH_MVEI)

void arm_split_rifft_q15(
        q15_t * pSrc,
        uint32_t fftLen,
  const q15_t * pATable,
  const q15_t * pBTable,
        q15_t * pDst,
        uint32_t modifier)
{
    q15_t const     *pCoefA, *pCoefB; /* Temporary pointers for twiddle factors */
    q15_t const     *pSrc1 = &pSrc[0], *pSrc2 = &pSrc[(2U * fftLen) + 1U - 14U];
    q15_t           *pDst1 = &pDst[0];
    q15_t const    *pVecSrc1;
    q15_t          *pVecDst1;
    q15x8x2_t      vecIn, vecSum;
    uint32_t         blkCnt;
    uint16x8_t     vecStridesFwd, vecStridesBkwd;
    q15x8_t        vecInBkwd, vecCoefFwd0, vecCoefFwd1;

    /*
     * Init coefficient pointers
     */
    pCoefA = &pATable[0];
    pCoefB = &pBTable[0];
    /*
     * scatter / gather offsets
     * for ascending & descending addressing
     */
    vecStridesFwd = vidupq_u16((uint32_t)0, 2);    // 0, 2, 4, 6, 8, 10, 12, 14
    vecStridesBkwd = vddupq_u16(14, 2);   // 14, 12, 10, 8, 6, 4, 2, 0
    vecStridesFwd = vecStridesFwd * (uint16_t)  modifier;


    pVecSrc1 = (q15_t const *) pSrc1;
    pVecDst1 = pDst1;

    blkCnt = fftLen >> 3;
    while (blkCnt > 0U)
    {
        vecCoefFwd0 = vldrhq_gather_shifted_offset(pCoefA, vecStridesFwd);
        vecCoefFwd1 = vldrhq_gather_shifted_offset(&pCoefA[1], vecStridesFwd);
        vecIn = vld2q(pVecSrc1);
        pVecSrc1 += 16;
        /*
         * outR = *pSrc1 * CoefA1;
         */
        vecSum.val[0] = vmulhq(vecIn.val[0], vecCoefFwd0);
        /*
         * outI = -(*pSrc1++) * CoefA2;
         */
        vecIn.val[0] = vnegq(vecIn.val[0]);
        vecSum.val[1] = vmulhq(vecIn.val[0], vecCoefFwd1);

        vecInBkwd = vldrhq_gather_shifted_offset(pSrc2, vecStridesBkwd);
        /*
         * outR += (*pSrc1 + *pSrc2) * CoefA2;
         */
        vecInBkwd = vqaddq(vecIn.val[1], vecInBkwd);
        vecSum.val[0] = vqaddq(vecSum.val[0], vmulhq(vecInBkwd, vecCoefFwd1));

        vecInBkwd = vldrhq_gather_shifted_offset(pSrc2, vecStridesBkwd);
        /*
         * outI += *pSrc1++ * CoefA1;
         */
        vecSum.val[1] = vqaddq(vecSum.val[1], vmulhq(vecIn.val[1], vecCoefFwd0));

        vecCoefFwd0 = vldrhq_gather_shifted_offset(pCoefB, vecStridesFwd);
        /*
         * outI -= *pSrc2-- * CoefB1;
         */
        vecSum.val[1] = vqsubq(vecSum.val[1], vmulhq(vecInBkwd, vecCoefFwd0));

        vecInBkwd = vldrhq_gather_shifted_offset(&pSrc2[-1], vecStridesBkwd);
        /*
         * outI += *pSrc2 * CoefA2;
         */
        vecSum.val[1] = vqaddq(vecSum.val[1], vmulhq(vecInBkwd, vecCoefFwd1));
        /*
         * outR += *pSrc2-- * CoefB1;
         */
        vecSum.val[0] = vqaddq(vecSum.val[0], vmulhq(vecInBkwd, vecCoefFwd0));

        vst2q(pVecDst1, vecSum);
        pVecDst1 += 16;
        /*
         * update fwd and backwd offsets
         */
        vecStridesFwd = vecStridesFwd + (uint16_t)(modifier * 16U);

        /* cannot use negative 16-bit offsets (would lead to positive 32-65K jump*/
        //vecStridesBkwd = vecStridesBkwd - (uint16_t)16;
        pSrc2 = pSrc2 - 16;
        blkCnt--;
    }
}
#else
void arm_split_rifft_q15(
        q15_t * pSrc,
        uint32_t fftLen,
  const q15_t * pATable,
  const q15_t * pBTable,
        q15_t * pDst,
        uint32_t modifier)
{
        uint32_t i;                                    /* Loop Counter */
        q31_t outR, outI;                              /* Temporary variables for output */
  const q15_t *pCoefA, *pCoefB;                        /* Temporary pointers for twiddle factors */
        q15_t *pSrc1, *pSrc2;
        q15_t *pDst1 = &pDst[0];

  pCoefA = &pATable[0];
  pCoefB = &pBTable[0];

  pSrc1 = &pSrc[0];
  pSrc2 = &pSrc[2 * fftLen];

  i = fftLen;
  while (i > 0U)
  {
      /*
        outR = (  pIn[2 * i]             * pATable[2 * i]
                + pIn[2 * i + 1]         * pATable[2 * i + 1]
                + pIn[2 * n - 2 * i]     * pBTable[2 * i]
                - pIn[2 * n - 2 * i + 1] * pBTable[2 * i + 1]);

        outI = (  pIn[2 * i + 1]         * pATable[2 * i]
                - pIn[2 * i]             * pATable[2 * i + 1]
                - pIn[2 * n - 2 * i]     * pBTable[2 * i + 1]
                - pIn[2 * n - 2 * i + 1] * pBTable[2 * i]);
       */

#if defined (ARM_MATH_DSP)

#ifndef ARM_MATH_BIG_ENDIAN
      /* pIn[2 * n - 2 * i] * pBTable[2 * i] - pIn[2 * n - 2 * i + 1] * pBTable[2 * i + 1]) */
      outR = __SMUSD(read_q15x2(pSrc2), read_q15x2((q15_t *) pCoefB));
#else
      /* -(-pIn[2 * n - 2 * i] * pBTable[2 * i] + pIn[2 * n - 2 * i + 1] * pBTable[2 * i + 1])) */
      outR = -(__SMUSD(read_q15x2(pSrc2), read_q15x2((q15_t *) pCoefB)));
#endif /* #ifndef ARM_MATH_BIG_ENDIAN */

      /* pIn[2 * i] * pATable[2 * i] + pIn[2 * i + 1] * pATable[2 * i + 1] + pIn[2 * n - 2 * i] * pBTable[2 * i] */
      outR = __SMLAD(read_q15x2(pSrc1), read_q15x2 ((q15_t *) pCoefA), outR) >> 16U;

      /* -pIn[2 * n - 2 * i] * pBTable[2 * i + 1] + pIn[2 * n - 2 * i + 1] * pBTable[2 * i] */
      outI = __SMUADX(read_q15x2_da (&pSrc2), read_q15x2((q15_t *) pCoefB));

      /* pIn[2 * i + 1] * pATable[2 * i] - pIn[2 * i] * pATable[2 * i + 1] */
#ifndef ARM_MATH_BIG_ENDIAN
      outI = __SMLSDX(read_q15x2 ((q15_t *) pCoefA), read_q15x2_ia (&pSrc1), -outI);
#else
      outI = __SMLSDX(read_q15x2_ia (&pSrc1), read_q15x2 ((q15_t *) pCoefA), -outI);
#endif /* #ifndef ARM_MATH_BIG_ENDIAN */

      /* write output */
#ifndef ARM_MATH_BIG_ENDIAN
      write_q15x2_ia (&pDst1, __PKHBT(outR, (outI >> 16U), 16));
#else
      write_q15x2_ia (&pDst1, __PKHBT((outI >> 16U), outR, 16));
#endif /* #ifndef ARM_MATH_BIG_ENDIAN */


#else  /* #if defined (ARM_MATH_DSP) */

      outR = *pSrc2 * *pCoefB;
      outR = outR - (*(pSrc2 + 1) * *(pCoefB + 1));
      outR = outR + (*pSrc1 * *pCoefA);
      outR = (outR + (*(pSrc1 + 1) * *(pCoefA + 1))) >> 16;

      outI = *(pSrc1 + 1) * *pCoefA;
      outI = outI - (*pSrc1 * *(pCoefA + 1));
      outI = outI - (*pSrc2 * *(pCoefB + 1));
      outI = outI - (*(pSrc2 + 1) * *(pCoefB));

      /* update input pointers */
      pSrc1 += 2U;
      pSrc2 -= 2U;

      /* write output */
      *pDst1++ = (q15_t) outR;
      *pDst1++ = (q15_t) (outI >> 16);

#endif /* #if defined (ARM_MATH_DSP) */

      /* update coefficient pointer */
      pCoefB = pCoefB + (2 * modifier);
      pCoefA = pCoefA + (2 * modifier);

      i--;
  }

}
#endif /* defined(ARM_MATH_MVEI) */
