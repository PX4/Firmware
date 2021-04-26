/* ----------------------------------------------------------------------
 * Project:      CMSIS DSP Library
 * Title:        arm_cfft_q15.c
 * Description:  Combined Radix Decimation in Q15 Frequency CFFT processing function
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

#if defined(ARM_MATH_MVEI)

#include "arm_vec_fft.h"


static void arm_bitreversal_16_inpl_mve(
        uint16_t *pSrc,
  const uint16_t bitRevLen,
  const uint16_t *pBitRevTab)

{
    uint32_t       *src = (uint32_t *)pSrc;
    uint32_t        blkCnt;     /* loop counters */
    uint32x4_t      bitRevTabOff;
    uint16x8_t      one = vdupq_n_u16(1);

    blkCnt = (bitRevLen / 2) / 4;
    while (blkCnt > 0U) {
        bitRevTabOff = vldrhq_u16(pBitRevTab);
        pBitRevTab += 8;

        uint32x4_t      bitRevOff1 = vmullbq_int_u16(bitRevTabOff, one);
        uint32x4_t      bitRevOff2 = vmulltq_int_u16(bitRevTabOff, one);

        bitRevOff1 = bitRevOff1 >> 3;
        bitRevOff2 = bitRevOff2 >> 3;

        uint32x4_t      in1 = vldrwq_gather_shifted_offset_u32(src, bitRevOff1);
        uint32x4_t      in2 = vldrwq_gather_shifted_offset_u32(src, bitRevOff2);

        vstrwq_scatter_shifted_offset_u32(src, bitRevOff1, in2);
        vstrwq_scatter_shifted_offset_u32(src, bitRevOff2, in1);

        /*
         * Decrement the blockSize loop counter
         */
        blkCnt--;
    }


    /*
     * tail
     * (will be merged thru tail predication)
     */
    blkCnt = bitRevLen & 7;
    if (blkCnt > 0U) {
        mve_pred16_t    p0 = vctp16q(blkCnt);

        bitRevTabOff = vldrhq_z_u16(pBitRevTab, p0);

        uint32x4_t      bitRevOff1 = vmullbq_int_u16(bitRevTabOff, one);
        uint32x4_t      bitRevOff2 = vmulltq_int_u16(bitRevTabOff, one);

        bitRevOff1 = bitRevOff1 >> 3;
        bitRevOff2 = bitRevOff2 >> 3;

        uint32x4_t      in1 = vldrwq_gather_shifted_offset_z_u32(src, bitRevOff1, p0);
        uint32x4_t      in2 = vldrwq_gather_shifted_offset_z_u32(src, bitRevOff2, p0);

        vstrwq_scatter_shifted_offset_p_u32(src, bitRevOff1, in2, p0);
        vstrwq_scatter_shifted_offset_p_u32(src, bitRevOff2, in1, p0);
    }
}

static void _arm_radix4_butterfly_q15_mve(
    const arm_cfft_instance_q15 * S,
    q15_t   *pSrc,
    uint32_t fftLen)
{
    q15x8_t vecTmp0, vecTmp1;
    q15x8_t vecSum0, vecDiff0, vecSum1, vecDiff1;
    q15x8_t vecA, vecB, vecC, vecD;
    q15x8_t vecW;
    uint32_t  blkCnt;
    uint32_t  n1, n2;
    uint32_t  stage = 0;
    int32_t  iter = 1;
    static const uint32_t strides[4] = {
        (0 - 16) * sizeof(q15_t *), (4 - 16) * sizeof(q15_t *),
        (8 - 16) * sizeof(q15_t *), (12 - 16) * sizeof(q15_t *)
    };

    /*
     * Process first stages
     * Each stage in middle stages provides two down scaling of the input
     */
    n2 = fftLen;
    n1 = n2;
    n2 >>= 2u;

    for (int k = fftLen / 4u; k > 1; k >>= 2u)
    {
        for (int i = 0; i < iter; i++)
        {
            q15_t const *p_rearranged_twiddle_tab_stride2 =
                &S->rearranged_twiddle_stride2[
                S->rearranged_twiddle_tab_stride2_arr[stage]];
            q15_t const *p_rearranged_twiddle_tab_stride3 = &S->rearranged_twiddle_stride3[
                S->rearranged_twiddle_tab_stride3_arr[stage]];
            q15_t const *p_rearranged_twiddle_tab_stride1 =
                &S->rearranged_twiddle_stride1[
                S->rearranged_twiddle_tab_stride1_arr[stage]];
            q15_t const *pW1, *pW2, *pW3;
            q15_t    *inA = pSrc + CMPLX_DIM * i * n1;
            q15_t    *inB = inA + n2 * CMPLX_DIM;
            q15_t    *inC = inB + n2 * CMPLX_DIM;
            q15_t    *inD = inC + n2 * CMPLX_DIM;

            pW1 = p_rearranged_twiddle_tab_stride1;
            pW2 = p_rearranged_twiddle_tab_stride2;
            pW3 = p_rearranged_twiddle_tab_stride3;

            blkCnt = n2 / 4;
            /*
             * load 4 x q15 complex pair
             */
            vecA = vldrhq_s16(inA);
            vecC = vldrhq_s16(inC);
            while (blkCnt > 0U)
            {
                vecB = vldrhq_s16(inB);
                vecD = vldrhq_s16(inD);

                vecSum0 = vhaddq(vecA, vecC);
                vecDiff0 = vhsubq(vecA, vecC);

                vecSum1 = vhaddq(vecB, vecD);
                vecDiff1 = vhsubq(vecB, vecD);
                /*
                 * [ 1 1 1 1 ] * [ A B C D ]' .* 1
                 */
                vecTmp0 = vhaddq(vecSum0, vecSum1);
                vst1q(inA, vecTmp0);
                inA += 8;
                /*
                 * [ 1 -1 1 -1 ] * [ A B C D ]'
                 */
                vecTmp0 = vhsubq(vecSum0, vecSum1);
                /*
                 * [ 1 -1 1 -1 ] * [ A B C D ]'.* W2
                 */
                vecW = vld1q(pW2);
                pW2 += 8;
                vecTmp1 = MVE_CMPLX_MULT_FX_AxB(vecW, vecTmp0);

                vst1q(inB, vecTmp1);
                inB += 8;
                /*
                 * [ 1 -i -1 +i ] * [ A B C D ]'
                 */
                vecTmp0 = MVE_CMPLX_SUB_FX_A_ixB(vecDiff0, vecDiff1);
                /*
                 * [ 1 -i -1 +i ] * [ A B C D ]'.* W1
                 */
                vecW = vld1q(pW1);
                pW1 += 8;
                vecTmp1 = MVE_CMPLX_MULT_FX_AxB(vecW, vecTmp0);
                vst1q(inC, vecTmp1);
                inC += 8;

                /*
                 * [ 1 +i -1 -i ] * [ A B C D ]'
                 */
                vecTmp0 = MVE_CMPLX_ADD_FX_A_ixB(vecDiff0, vecDiff1);
                /*
                 * [ 1 +i -1 -i ] * [ A B C D ]'.* W3
                 */
                vecW = vld1q(pW3);
                pW3 += 8;
                vecTmp1 = MVE_CMPLX_MULT_FX_AxB(vecW, vecTmp0);
                vst1q(inD, vecTmp1);
                inD += 8;

                vecA = vldrhq_s16(inA);
                vecC = vldrhq_s16(inC);

                blkCnt--;
            }
        }
        n1 = n2;
        n2 >>= 2u;
        iter = iter << 2;
        stage++;
    }

    /*
     * start of Last stage process
     */
    uint32x4_t vecScGathAddr = *(uint32x4_t *) strides;
    vecScGathAddr = vecScGathAddr + (uint32_t) pSrc;

    /*
     * load scheduling
     */
    vecA = (q15x8_t) vldrwq_gather_base_wb_s32(&vecScGathAddr, 64);
    vecC = (q15x8_t) vldrwq_gather_base_s32(vecScGathAddr, 8);

    blkCnt = (fftLen >> 4);
    while (blkCnt > 0U)
    {
        vecSum0 = vhaddq(vecA, vecC);
        vecDiff0 = vhsubq(vecA, vecC);

        vecB = (q15x8_t) vldrwq_gather_base_s32(vecScGathAddr, 4);
        vecD = (q15x8_t) vldrwq_gather_base_s32(vecScGathAddr, 12);

        vecSum1 = vhaddq(vecB, vecD);
        vecDiff1 = vhsubq(vecB, vecD);
        /*
         * pre-load for next iteration
         */
        vecA = (q15x8_t) vldrwq_gather_base_wb_s32(&vecScGathAddr, 64);
        vecC = (q15x8_t) vldrwq_gather_base_s32(vecScGathAddr, 8);

        vecTmp0 = vhaddq(vecSum0, vecSum1);
        vstrwq_scatter_base_s32(vecScGathAddr, -64, (q15x8_t) vecTmp0);

        vecTmp0 = vhsubq(vecSum0, vecSum1);
        vstrwq_scatter_base_s32(vecScGathAddr, -64 + 4, (q15x8_t) vecTmp0);

        vecTmp0 = MVE_CMPLX_SUB_FX_A_ixB(vecDiff0, vecDiff1);
        vstrwq_scatter_base_s32(vecScGathAddr, -64 + 8, (q15x8_t) vecTmp0);

        vecTmp0 = MVE_CMPLX_ADD_FX_A_ixB(vecDiff0, vecDiff1);
        vstrwq_scatter_base_s32(vecScGathAddr, -64 + 12, (q15x8_t) vecTmp0);

        blkCnt--;
    }

}

static void arm_cfft_radix4by2_q15_mve(const arm_cfft_instance_q15 *S, q15_t *pSrc, uint32_t fftLen)
{
    uint32_t n2;
    q15_t *pIn0;
    q15_t *pIn1;
    const q15_t *pCoef = S->pTwiddle;
    uint32_t     blkCnt;
    q15x8_t    vecIn0, vecIn1, vecSum, vecDiff;
    q15x8_t    vecCmplxTmp, vecTw;
    q15_t  const *pCoefVec;

    n2 = fftLen >> 1;

    pIn0 = pSrc;
    pIn1 = pSrc + fftLen;
    pCoefVec = pCoef;

    blkCnt = n2 / 4;

    while (blkCnt > 0U)
    {
        vecIn0 = *(q15x8_t *) pIn0;
        vecIn1 = *(q15x8_t *) pIn1;

        vecIn0 = vecIn0 >> 1;
        vecIn1 = vecIn1 >> 1;
        vecSum = vhaddq(vecIn0, vecIn1);
        vst1q(pIn0, vecSum);
        pIn0 += 8;

        vecTw = vld1q(pCoefVec);
        pCoefVec += 8;

        vecDiff = vhsubq(vecIn0, vecIn1);
        vecCmplxTmp = MVE_CMPLX_MULT_FX_AxConjB(vecDiff, vecTw);
        vst1q(pIn1, vecCmplxTmp);
        pIn1 += 8;

        blkCnt--;
    }

    _arm_radix4_butterfly_q15_mve(S, pSrc, n2);

    _arm_radix4_butterfly_q15_mve(S, pSrc + fftLen, n2);


    pIn0 = pSrc;
    blkCnt = (fftLen << 1) >> 3;
    while (blkCnt > 0U)
    {
        vecIn0 = *(q15x8_t *) pIn0;
        vecIn0 = vecIn0 << 1;
        vst1q(pIn0, vecIn0);
        pIn0 += 8;
        blkCnt--;
    }
    /*
     * tail
     * (will be merged thru tail predication)
     */
    blkCnt = (fftLen << 1) & 7;
    if (blkCnt > 0U)
    {
        mve_pred16_t p0 = vctp16q(blkCnt);

        vecIn0 = *(q15x8_t *) pIn0;
        vecIn0 = vecIn0 << 1;
        vstrhq_p(pIn0, vecIn0, p0);
    }
}

static void _arm_radix4_butterfly_inverse_q15_mve(const arm_cfft_instance_q15 *S,q15_t *pSrc, uint32_t fftLen)
{
    q15x8_t vecTmp0, vecTmp1;
    q15x8_t vecSum0, vecDiff0, vecSum1, vecDiff1;
    q15x8_t vecA, vecB, vecC, vecD;
    q15x8_t vecW;
    uint32_t  blkCnt;
    uint32_t  n1, n2;
    uint32_t  stage = 0;
    int32_t  iter = 1;
    static const uint32_t strides[4] = {
        (0 - 16) * sizeof(q15_t *), (4 - 16) * sizeof(q15_t *),
        (8 - 16) * sizeof(q15_t *), (12 - 16) * sizeof(q15_t *)
    };


    /*
     * Process first stages
     * Each stage in middle stages provides two down scaling of the input
     */
    n2 = fftLen;
    n1 = n2;
    n2 >>= 2u;

    for (int k = fftLen / 4u; k > 1; k >>= 2u)
    {
        for (int i = 0; i < iter; i++)
        {
            q15_t const *p_rearranged_twiddle_tab_stride2 =
                &S->rearranged_twiddle_stride2[
                S->rearranged_twiddle_tab_stride2_arr[stage]];
            q15_t const *p_rearranged_twiddle_tab_stride3 = &S->rearranged_twiddle_stride3[
                S->rearranged_twiddle_tab_stride3_arr[stage]];
            q15_t const *p_rearranged_twiddle_tab_stride1 =
                &S->rearranged_twiddle_stride1[
                S->rearranged_twiddle_tab_stride1_arr[stage]];
            q15_t const *pW1, *pW2, *pW3;
            q15_t    *inA = pSrc + CMPLX_DIM * i * n1;
            q15_t    *inB = inA + n2 * CMPLX_DIM;
            q15_t    *inC = inB + n2 * CMPLX_DIM;
            q15_t    *inD = inC + n2 * CMPLX_DIM;

            pW1 = p_rearranged_twiddle_tab_stride1;
            pW2 = p_rearranged_twiddle_tab_stride2;
            pW3 = p_rearranged_twiddle_tab_stride3;

            blkCnt = n2 / 4;
            /*
             * load 4 x q15 complex pair
             */
            vecA = vldrhq_s16(inA);
            vecC = vldrhq_s16(inC);
            while (blkCnt > 0U)
            {
                vecB = vldrhq_s16(inB);
                vecD = vldrhq_s16(inD);

                vecSum0 = vhaddq(vecA, vecC);
                vecDiff0 = vhsubq(vecA, vecC);

                vecSum1 = vhaddq(vecB, vecD);
                vecDiff1 = vhsubq(vecB, vecD);
                /*
                 * [ 1 1 1 1 ] * [ A B C D ]' .* 1
                 */
                vecTmp0 = vhaddq(vecSum0, vecSum1);
                vst1q(inA, vecTmp0);
                inA += 8;
                /*
                 * [ 1 -1 1 -1 ] * [ A B C D ]'
                 */
                vecTmp0 = vhsubq(vecSum0, vecSum1);
                /*
                 * [ 1 -1 1 -1 ] * [ A B C D ]'.* W2
                 */
                vecW = vld1q(pW2);
                pW2 += 8;
                vecTmp1 = MVE_CMPLX_MULT_FX_AxConjB(vecTmp0, vecW);

                vst1q(inB, vecTmp1);
                inB += 8;
                /*
                 * [ 1 -i -1 +i ] * [ A B C D ]'
                 */
                vecTmp0 = MVE_CMPLX_ADD_FX_A_ixB(vecDiff0, vecDiff1);
                /*
                 * [ 1 -i -1 +i ] * [ A B C D ]'.* W1
                 */
                vecW = vld1q(pW1);
                pW1 += 8;
                vecTmp1 = MVE_CMPLX_MULT_FX_AxConjB(vecTmp0, vecW);
                vst1q(inC, vecTmp1);
                inC += 8;
                /*
                 * [ 1 +i -1 -i ] * [ A B C D ]'
                 */
                vecTmp0 = MVE_CMPLX_SUB_FX_A_ixB(vecDiff0, vecDiff1);
                /*
                 * [ 1 +i -1 -i ] * [ A B C D ]'.* W3
                 */
                vecW = vld1q(pW3);
                pW3 += 8;
                vecTmp1 = MVE_CMPLX_MULT_FX_AxConjB(vecTmp0, vecW);
                vst1q(inD, vecTmp1);
                inD += 8;

                vecA = vldrhq_s16(inA);
                vecC = vldrhq_s16(inC);

                blkCnt--;
            }
        }
        n1 = n2;
        n2 >>= 2u;
        iter = iter << 2;
        stage++;
    }

    /*
     * start of Last stage process
     */
    uint32x4_t vecScGathAddr = *(uint32x4_t *) strides;
    vecScGathAddr = vecScGathAddr + (uint32_t) pSrc;

    /*
     * load scheduling
     */
    vecA = (q15x8_t) vldrwq_gather_base_wb_s32(&vecScGathAddr, 64);
    vecC = (q15x8_t) vldrwq_gather_base_s32(vecScGathAddr, 8);

    blkCnt = (fftLen >> 4);
    while (blkCnt > 0U)
    {
        vecSum0 = vhaddq(vecA, vecC);
        vecDiff0 = vhsubq(vecA, vecC);

        vecB = (q15x8_t) vldrwq_gather_base_s32(vecScGathAddr, 4);
        vecD = (q15x8_t) vldrwq_gather_base_s32(vecScGathAddr, 12);

        vecSum1 = vhaddq(vecB, vecD);
        vecDiff1 = vhsubq(vecB, vecD);
        /*
         * pre-load for next iteration
         */
        vecA = (q15x8_t) vldrwq_gather_base_wb_s32(&vecScGathAddr, 64);
        vecC = (q15x8_t) vldrwq_gather_base_s32(vecScGathAddr, 8);

        vecTmp0 = vhaddq(vecSum0, vecSum1);
        vstrwq_scatter_base_s32(vecScGathAddr, -64, (q15x8_t) vecTmp0);

        vecTmp0 = vhsubq(vecSum0, vecSum1);
        vstrwq_scatter_base_s32(vecScGathAddr, -64 + 4, (q15x8_t) vecTmp0);

        vecTmp0 = MVE_CMPLX_ADD_FX_A_ixB(vecDiff0, vecDiff1);
        vstrwq_scatter_base_s32(vecScGathAddr, -64 + 8, (q15x8_t) vecTmp0);

        vecTmp0 = MVE_CMPLX_SUB_FX_A_ixB(vecDiff0, vecDiff1);
        vstrwq_scatter_base_s32(vecScGathAddr, -64 + 12, (q15x8_t) vecTmp0);

        blkCnt--;
    }
}

static void arm_cfft_radix4by2_inverse_q15_mve(const arm_cfft_instance_q15 *S, q15_t *pSrc, uint32_t fftLen)
{
    uint32_t n2;
    q15_t *pIn0;
    q15_t *pIn1;
    const q15_t *pCoef = S->pTwiddle;

    uint32_t     blkCnt;
    q15x8_t    vecIn0, vecIn1, vecSum, vecDiff;
    q15x8_t    vecCmplxTmp, vecTw;
    q15_t  const *pCoefVec;

    n2 = fftLen >> 1;

    pIn0 = pSrc;
    pIn1 = pSrc + fftLen;
    pCoefVec = pCoef;

    blkCnt = n2 / 4;

    while (blkCnt > 0U)
    {
        vecIn0 = *(q15x8_t *) pIn0;
        vecIn1 = *(q15x8_t *) pIn1;

        vecIn0 = vecIn0 >> 1;
        vecIn1 = vecIn1 >> 1;
        vecSum = vhaddq(vecIn0, vecIn1);
        vst1q(pIn0, vecSum);
        pIn0 += 8;

        vecTw = vld1q(pCoefVec);
        pCoefVec += 8;

        vecDiff = vhsubq(vecIn0, vecIn1);
        vecCmplxTmp = vqrdmlsdhq(vuninitializedq_s16() , vecDiff, vecTw);
        vecCmplxTmp = vqrdmladhxq(vecCmplxTmp, vecDiff, vecTw);
        vst1q(pIn1, vecCmplxTmp);
        pIn1 += 8;

        blkCnt--;
    }


    _arm_radix4_butterfly_inverse_q15_mve(S, pSrc, n2);

    _arm_radix4_butterfly_inverse_q15_mve(S, pSrc + fftLen, n2);

    pIn0 = pSrc;
    blkCnt = (fftLen << 1) >> 3;
    while (blkCnt > 0U)
    {
        vecIn0 = *(q15x8_t *) pIn0;
        vecIn0 = vecIn0 << 1;
        vst1q(pIn0, vecIn0);
        pIn0 += 8;
        blkCnt--;
    }
    /*
     * tail
     * (will be merged thru tail predication)
     */
    blkCnt = (fftLen << 1) & 7;
    while (blkCnt > 0U)
    {
        mve_pred16_t p0 = vctp16q(blkCnt);

        vecIn0 = *(q15x8_t *) pIn0;
        vecIn0 = vecIn0 << 1;
        vstrhq_p(pIn0, vecIn0, p0);
    }
}

/**
  @ingroup groupTransforms
 */

/**
  @addtogroup ComplexFFT
  @{
 */

/**
  @brief         Processing function for Q15 complex FFT.
  @param[in]     S               points to an instance of Q15 CFFT structure
  @param[in,out] p1              points to the complex data buffer of size <code>2*fftLen</code>. Processing occurs in-place
  @param[in]     ifftFlag       flag that selects transform direction
                   - value = 0: forward transform
                   - value = 1: inverse transform
  @param[in]     bitReverseFlag flag that enables / disables bit reversal of output
                   - value = 0: disables bit reversal of output
                   - value = 1: enables bit reversal of output
  @return        none
 */
void arm_cfft_q15(
  const arm_cfft_instance_q15 * S,
        q15_t * pSrc,
        uint8_t ifftFlag,
        uint8_t bitReverseFlag)
{                                                                             
        uint32_t fftLen = S->fftLen;     

        if (ifftFlag == 1U) {                                                            
                                                                                         
            switch (fftLen) {                                                            
            case 16:                                                                     
            case 64:                                                                     
            case 256:                                                                    
            case 1024:                                                                   
            case 4096:                                                                   
                _arm_radix4_butterfly_inverse_q15_mve(S, pSrc, fftLen); 
                break;                                                                   
                                                                                         
            case 32:                                                                     
            case 128:                                                                    
            case 512:                                                                    
            case 2048:                                                                   
                arm_cfft_radix4by2_inverse_q15_mve(S, pSrc, fftLen);              
                break;                                                                   
            }  
        } else {                                                                         
            switch (fftLen) {                                                            
            case 16:                                                                     
            case 64:                                                                     
            case 256:                                                                    
            case 1024:                                                                   
            case 4096:    
                _arm_radix4_butterfly_q15_mve(S, pSrc, fftLen);         
                break;                                                                   
                                                                                         
            case 32:                                                                     
            case 128:                                                                    
            case 512:                                                                    
            case 2048:                                                                   
                arm_cfft_radix4by2_q15_mve(S, pSrc, fftLen);                      
                break;                                                                   
            }                                                                            
        }                                                                                
                                                                                         
                                                                                         
        if (bitReverseFlag) 
        {                                                            
            
            arm_bitreversal_16_inpl_mve((uint16_t*)pSrc, S->bitRevLength, S->pBitRevTable);
       
        } 
}

#else

extern void arm_radix4_butterfly_q15(
        q15_t * pSrc,
        uint32_t fftLen,
  const q15_t * pCoef,
        uint32_t twidCoefModifier);

extern void arm_radix4_butterfly_inverse_q15(
        q15_t * pSrc,
        uint32_t fftLen,
  const q15_t * pCoef,
        uint32_t twidCoefModifier);

extern void arm_bitreversal_16(
        uint16_t * pSrc,
  const uint16_t bitRevLen,
  const uint16_t * pBitRevTable);

void arm_cfft_radix4by2_q15(
        q15_t * pSrc,
        uint32_t fftLen,
  const q15_t * pCoef);

void arm_cfft_radix4by2_inverse_q15(
        q15_t * pSrc,
        uint32_t fftLen,
  const q15_t * pCoef);

/**
  @ingroup groupTransforms
 */

/**
  @addtogroup ComplexFFT
  @{
 */

/**
  @brief         Processing function for Q15 complex FFT.
  @param[in]     S               points to an instance of Q15 CFFT structure
  @param[in,out] p1              points to the complex data buffer of size <code>2*fftLen</code>. Processing occurs in-place
  @param[in]     ifftFlag       flag that selects transform direction
                   - value = 0: forward transform
                   - value = 1: inverse transform
  @param[in]     bitReverseFlag flag that enables / disables bit reversal of output
                   - value = 0: disables bit reversal of output
                   - value = 1: enables bit reversal of output
  @return        none
 */

void arm_cfft_q15(
  const arm_cfft_instance_q15 * S,
        q15_t * p1,
        uint8_t ifftFlag,
        uint8_t bitReverseFlag)
{
  uint32_t L = S->fftLen;

  if (ifftFlag == 1U)
  {
     switch (L)
     {
     case 16:
     case 64:
     case 256:
     case 1024:
     case 4096:
       arm_radix4_butterfly_inverse_q15 ( p1, L, (q15_t*)S->pTwiddle, 1 );
       break;

     case 32:
     case 128:
     case 512:
     case 2048:
       arm_cfft_radix4by2_inverse_q15 ( p1, L, S->pTwiddle );
       break;
     }
  }
  else
  {
     switch (L)
     {
     case 16:
     case 64:
     case 256:
     case 1024:
     case 4096:
       arm_radix4_butterfly_q15  ( p1, L, (q15_t*)S->pTwiddle, 1 );
       break;

     case 32:
     case 128:
     case 512:
     case 2048:
       arm_cfft_radix4by2_q15  ( p1, L, S->pTwiddle );
       break;
     }
  }

  if ( bitReverseFlag )
    arm_bitreversal_16 ((uint16_t*) p1, S->bitRevLength, S->pBitRevTable);
}

/**
  @} end of ComplexFFT group
 */

void arm_cfft_radix4by2_q15(
        q15_t * pSrc,
        uint32_t fftLen,
  const q15_t * pCoef)
{
        uint32_t i;
        uint32_t n2;
        q15_t p0, p1, p2, p3;
#if defined (ARM_MATH_DSP)
        q31_t T, S, R;
        q31_t coeff, out1, out2;
  const q15_t *pC = pCoef;
        q15_t *pSi = pSrc;
        q15_t *pSl = pSrc + fftLen;
#else
        uint32_t l;
        q15_t xt, yt, cosVal, sinVal;
#endif

  n2 = fftLen >> 1U;

#if defined (ARM_MATH_DSP)

  for (i = n2; i > 0; i--)
  {
      coeff = read_q15x2_ia ((q15_t **) &pC);

      T = read_q15x2 (pSi);
      T = __SHADD16(T, 0); /* this is just a SIMD arithmetic shift right by 1 */

      S = read_q15x2 (pSl);
      S = __SHADD16(S, 0); /* this is just a SIMD arithmetic shift right by 1 */

      R = __QSUB16(T, S);

      write_q15x2_ia (&pSi, __SHADD16(T, S));

#ifndef ARM_MATH_BIG_ENDIAN
      out1 = __SMUAD(coeff, R) >> 16U;
      out2 = __SMUSDX(coeff, R);
#else
      out1 = __SMUSDX(R, coeff) >> 16U;
      out2 = __SMUAD(coeff, R);
#endif /* #ifndef ARM_MATH_BIG_ENDIAN */

      write_q15x2_ia (&pSl, (q31_t) ((out2) & 0xFFFF0000) | (out1 & 0x0000FFFF));
  }

#else /* #if defined (ARM_MATH_DSP) */

  for (i = 0; i < n2; i++)
  {
     cosVal = pCoef[2 * i];
     sinVal = pCoef[2 * i + 1];

     l = i + n2;

     xt =           (pSrc[2 * i] >> 1U) - (pSrc[2 * l] >> 1U);
     pSrc[2 * i] = ((pSrc[2 * i] >> 1U) + (pSrc[2 * l] >> 1U)) >> 1U;

     yt =               (pSrc[2 * i + 1] >> 1U) - (pSrc[2 * l + 1] >> 1U);
     pSrc[2 * i + 1] = ((pSrc[2 * l + 1] >> 1U) + (pSrc[2 * i + 1] >> 1U)) >> 1U;

     pSrc[2 * l]     = (((int16_t) (((q31_t) xt * cosVal) >> 16U)) +
                        ((int16_t) (((q31_t) yt * sinVal) >> 16U))  );

     pSrc[2 * l + 1] = (((int16_t) (((q31_t) yt * cosVal) >> 16U)) -
                        ((int16_t) (((q31_t) xt * sinVal) >> 16U))   );
  }

#endif /* #if defined (ARM_MATH_DSP) */

  /* first col */
  arm_radix4_butterfly_q15( pSrc,          n2, (q15_t*)pCoef, 2U);

  /* second col */
  arm_radix4_butterfly_q15( pSrc + fftLen, n2, (q15_t*)pCoef, 2U);

  n2 = fftLen >> 1U;
  for (i = 0; i < n2; i++)
  {
     p0 = pSrc[4 * i + 0];
     p1 = pSrc[4 * i + 1];
     p2 = pSrc[4 * i + 2];
     p3 = pSrc[4 * i + 3];

     p0 <<= 1U;
     p1 <<= 1U;
     p2 <<= 1U;
     p3 <<= 1U;

     pSrc[4 * i + 0] = p0;
     pSrc[4 * i + 1] = p1;
     pSrc[4 * i + 2] = p2;
     pSrc[4 * i + 3] = p3;
  }

}

void arm_cfft_radix4by2_inverse_q15(
        q15_t * pSrc,
        uint32_t fftLen,
  const q15_t * pCoef)
{
        uint32_t i;
        uint32_t n2;
        q15_t p0, p1, p2, p3;
#if defined (ARM_MATH_DSP)
        q31_t T, S, R;
        q31_t coeff, out1, out2;
  const q15_t *pC = pCoef;
        q15_t *pSi = pSrc;
        q15_t *pSl = pSrc + fftLen;
#else
        uint32_t l;
        q15_t xt, yt, cosVal, sinVal;
#endif

  n2 = fftLen >> 1U;

#if defined (ARM_MATH_DSP)

  for (i = n2; i > 0; i--)
  {
     coeff = read_q15x2_ia ((q15_t **) &pC);

     T = read_q15x2 (pSi);
     T = __SHADD16(T, 0); /* this is just a SIMD arithmetic shift right by 1 */

     S = read_q15x2 (pSl);
     S = __SHADD16(S, 0); /* this is just a SIMD arithmetic shift right by 1 */

     R = __QSUB16(T, S);

     write_q15x2_ia (&pSi, __SHADD16(T, S));

#ifndef ARM_MATH_BIG_ENDIAN
     out1 = __SMUSD(coeff, R) >> 16U;
     out2 = __SMUADX(coeff, R);
#else
     out1 = __SMUADX(R, coeff) >> 16U;
     out2 = __SMUSD(__QSUB(0, coeff), R);
#endif /* #ifndef ARM_MATH_BIG_ENDIAN */

     write_q15x2_ia (&pSl, (q31_t) ((out2) & 0xFFFF0000) | (out1 & 0x0000FFFF));
  }

#else /* #if defined (ARM_MATH_DSP) */

  for (i = 0; i < n2; i++)
  {
     cosVal = pCoef[2 * i];
     sinVal = pCoef[2 * i + 1];

     l = i + n2;

     xt =           (pSrc[2 * i] >> 1U) - (pSrc[2 * l] >> 1U);
     pSrc[2 * i] = ((pSrc[2 * i] >> 1U) + (pSrc[2 * l] >> 1U)) >> 1U;

     yt =               (pSrc[2 * i + 1] >> 1U) - (pSrc[2 * l + 1] >> 1U);
     pSrc[2 * i + 1] = ((pSrc[2 * l + 1] >> 1U) + (pSrc[2 * i + 1] >> 1U)) >> 1U;

     pSrc[2 * l]      = (((int16_t) (((q31_t) xt * cosVal) >> 16U)) -
                         ((int16_t) (((q31_t) yt * sinVal) >> 16U))  );

     pSrc[2 * l + 1] = (((int16_t) (((q31_t) yt * cosVal) >> 16U)) +
                        ((int16_t) (((q31_t) xt * sinVal) >> 16U))  );
  }

#endif /* #if defined (ARM_MATH_DSP) */

  /* first col */
  arm_radix4_butterfly_inverse_q15( pSrc,          n2, (q15_t*)pCoef, 2U);

  /* second col */
  arm_radix4_butterfly_inverse_q15( pSrc + fftLen, n2, (q15_t*)pCoef, 2U);

  n2 = fftLen >> 1U;
  for (i = 0; i < n2; i++)
  {
     p0 = pSrc[4 * i + 0];
     p1 = pSrc[4 * i + 1];
     p2 = pSrc[4 * i + 2];
     p3 = pSrc[4 * i + 3];

     p0 <<= 1U;
     p1 <<= 1U;
     p2 <<= 1U;
     p3 <<= 1U;

     pSrc[4 * i + 0] = p0;
     pSrc[4 * i + 1] = p1;
     pSrc[4 * i + 2] = p2;
     pSrc[4 * i + 3] = p3;
  }
}

#endif /* defined(ARM_MATH_MVEI) */
