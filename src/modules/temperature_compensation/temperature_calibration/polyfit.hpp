/****************************************************************************
 *
 *   Copyright (c) 2015-2016 PX4 Development Team. All rights reserved.
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

/*

This algorithm performs a curve fit of m x,y data points using a polynomial
equation of the following form:

yi = a0 + a1.xi + a2.xi^2 + a3.xi^3 + .... + an.xi^n + ei , where:

i = [0,m]
xi is the x coordinate (independant variable) of the i'th measurement
yi is the y coordinate (dependant variable) of the i'th measurement
ei is a random fit error being the difference between the i'th y coordinate
   and the value predicted by the polynomial.

In vector form this is represented as:

Y = V.A + E , where:

V is Vandermonde matrix in x -> https://en.wikipedia.org/wiki/Vandermonde_matrix
Y is a vector of length m containing the y measurements
E is a vector of length m containing the fit errors for each measurement

Use an Ordinary Least Squares derivation to minimise ∑(i=0..m)ei^2 -> https://en.wikipedia.org/wiki/Ordinary_least_squares

Note: In the wikipedia reference, the X matrix in reference is equivalent to our V matrix and the Beta matrix is equivalent to our A matrix

A = inv(transpose(V)*V)*(transpose(V)*Y)

We can accumulate VTV and VTY recursively as they are of fixed size, where:

VTV = transpose(V)*V =
 __                                                                                                                        __
|    m+1                      x0+x1+...+xm                   x0^2+x1^2+...+xm^3   ..........  x0^n+x1^n+...+xm^n             |
|x0+x1+...+xm              x0^2+x1^2+...+xm^3                x0^3+x1^3+...+xm^3   ..........  x0^(n+1)+x1^(n+1)+...+xm^(n+1) |
|      .                            .                                  .                             .                       |
|      .                            .                                  .                             .                       |
|      .                            .                                  .                             .                       |
|x0^n+x1^n+...+xm^n     x0^(n+1)+x1^(n+1)+...+xm^(n+1)  x0^(n+2)+x1^(n+2)+...+xm^(n+2) ....  x0^(2n)+x1^(2n)+...+xm^(2n)     |
|__                                                                                                                        __|

and VTY = transpose(V)*Y =
 __            __
|  ∑(i=0..m)yi   |
| ∑(i=0..m)yi*xi |
|       .        |
|       .        |
|       .        |
|∑(i=0..m)yi*xi^n|
|__            __|

*/

/*
Polygon linear fit
Author: Siddharth Bharat Purohit
*/

#pragma once
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/time.h>

#include <float.h>

#include <matrix/math.hpp>

#define DEBUG 0
#if DEBUG
#define PF_DEBUG(fmt, ...) printf(fmt, ##__VA_ARGS__);
#else
#define PF_DEBUG(fmt, ...)
#endif

template<int _forder>
class polyfitter
{
public:
	polyfitter() {}

	void update(double x, double y)
	{
		update_VTV(x);
		update_VTY(x, y);
	}

	bool fit(double res[])
	{
		//Do inverse of VTV
		matrix::SquareMatrix<double, _forder> IVTV;

		IVTV = _VTV.I();

		for (int i = 0; i < _forder; i++) {
			for (int j = 0; j < _forder; j++) {
				PF_DEBUG("%.10f ", (double)IVTV(i, j));
			}

			PF_DEBUG("\n");
		}

		for (int i = 0; i < _forder; i++) {
			res[i] = 0.0;

			for (int j = 0; j < _forder; j++) {
				res[i] += IVTV(i, j) * (double)_VTY(j);
			}

			PF_DEBUG("%.10f ", res[i]);
		}

		return true;
	}

private:
	matrix::SquareMatrix<double, _forder> _VTV;
	matrix::Vector<double, _forder> _VTY;

	void update_VTY(double x, double y)
	{
		double temp = 1.0;
		PF_DEBUG("O %.6f\n", (double)x);

		for (int i = _forder - 1; i >= 0; i--) {
			_VTY(i) += y * temp;
			temp *= x;
			PF_DEBUG("%.6f ", (double)_VTY(i));
		}

		PF_DEBUG("\n");
	}

	void update_VTV(double x)
	{
		double temp = 1.0;
		int8_t z;

		for (int i = 0; i < _forder; i++) {
			for (int j = 0; j < _forder; j++) {
				PF_DEBUG("%.10f ", (double)_VTV(i, j));
			}

			PF_DEBUG("\n");
		}

		for (int i = 2 * _forder - 2; i >= 0; i--) {
			if (i < _forder) {
				z = 0.0f;

			} else {
				z = i - _forder + 1;
			}

			for (int j = i - z; j >= z; j--) {
				int row = j;
				int col = i - j;
				_VTV(row, col) += (double)temp;
			}

			temp *= x;
		}
	}
};
