/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file BezierQuad.cpp
 * Bezier function
 *
 * @author Dennis Mannhart <dennis.mannhart@gmail.com>
 */

// The header includes this implementation, so in order to have this library shared with other
// .cpp files and avoid duplication of constants, we need to prevent including it twice
#ifndef BEZIER_QUAD_CPP
#define BEZIER_QUAD_CPP


#include "BezierQuad.hpp"

namespace bezier
{
static constexpr double GOLDEN_RATIO = 1.6180339887; //(sqrt(5)+1)/2
static constexpr double RESOLUTION = 0.0001; //End criterion for golden section search

template<typename Tp>
void BezierQuad<Tp>::setBezier(const Vector3_t &pt0, const Vector3_t &ctrl, const Vector3_t &pt1,
			       Tp duration)
{
	_pt0 = pt0;
	_ctrl = ctrl;
	_pt1 = pt1;
	_duration = duration;
	_cached_resolution = (Tp)(-1);
}

template<typename Tp>
void BezierQuad<Tp>::getBezier(Vector3_t &pt0, Vector3_t &ctrl, Vector3_t &pt1)
{
	pt0 = _pt0;
	ctrl = _ctrl;
	pt1 = _pt1;
}

template<typename Tp>
matrix::Vector<Tp, 3> BezierQuad<Tp>::getPoint(const Tp t)
{
	return (_pt0 * ((Tp)1 - t / _duration) * ((Tp)1 - t / _duration) + _ctrl * (Tp)2 * ((
				Tp)1 - t / _duration) * t / _duration +   _pt1 *
		t / _duration * t / _duration);
}

template<typename Tp>
matrix::Vector<Tp, 3> BezierQuad<Tp>::getVelocity(const Tp t)
{
	return (((_ctrl - _pt0) * _duration + (_pt0 - _ctrl * (Tp)2 + _pt1) * t) * (Tp)2 / (_duration * _duration));
}

template<typename Tp>
matrix::Vector<Tp, 3> BezierQuad<Tp>::getAcceleration()
{
	return ((_pt0 - _ctrl * (Tp)2 + _pt1) * (Tp)2 / (_duration * _duration));
}

template<typename Tp>
void BezierQuad<Tp>::getStates(Vector3_t &point, Vector3_t &vel, Vector3_t &acc, const Tp time)
{
	point = getPoint(time);
	vel = getVelocity(time);
	acc = getAcceleration();
}

template<typename Tp>
void BezierQuad<Tp>::getStatesClosest(Vector3_t &point, Vector3_t &vel, Vector3_t &acc,
				      const Vector3_t pose)
{
	// get t that corresponds to point closest on bezier point
	Tp t = _goldenSectionSearch(pose);

	// get states corresponding to t
	getStates(point, vel, acc, t);

}

template<typename Tp>
void BezierQuad<Tp>::setBezFromVel(const Vector3_t &ctrl, const Vector3_t &vel0, const Vector3_t &vel1,
				   const Tp duration)
{
	// update bezier points
	_ctrl = ctrl;
	_duration = duration;
	_pt0 = _ctrl - vel0 * _duration / (Tp)2;
	_pt1 = _ctrl + vel1 * _duration / (Tp)2;
	_cached_resolution = (Tp)(-1);
}

template<typename Tp>
Tp BezierQuad<Tp>::getArcLength(const Tp resolution)
{
	// we don't need to recompute arc length if:
	// 1. _cached_resolution is up to date; 2. _cached_resolution is smaller than desired resolution (= more accurate)
	if ((_cached_resolution > (Tp)0) && (_cached_resolution <= resolution)) {
		return _cached_arc_length;
	}

	// get number of elements
	int n = (int)(roundf(_duration / resolution));

	// check if n is even
	if (n % 2 == 1) {
		n += 1;
	}

	// step size
	Tp h = (_duration) / n;

	// get integration
	_cached_arc_length = (Tp)0;
	Vector3_t y;

	for (int i = 1; i < n; i++) {

		y = getVelocity(h * i);

		if (i % 2 == 1) {
			_cached_arc_length += (Tp)4 * y.length();

		} else {
			_cached_arc_length += (Tp)2 * y.length();
		}
	}

	// velocity length at start and end points
	Tp y0 = getVelocity((Tp)0).length();
	Tp yn = getVelocity(_duration).length();

	// 1/3 simpsons rule
	_cached_arc_length = h / (Tp)3 * (y0 + yn + _cached_arc_length);

	// update cache
	_cached_resolution = resolution;

	return _cached_arc_length;
}

template<typename Tp>
Tp BezierQuad<Tp>::getDistToClosestPoint(const Vector3_t &pose)
{
	// get t that corresponds to point closest on bezier point
	Tp t = _goldenSectionSearch(pose);

	// get closest point
	Vector3_t point = getPoint(t);
	return (pose - point).length();
}

template<typename Tp>
Tp BezierQuad<Tp>::_goldenSectionSearch(const Vector3_t &pose)
{
	Tp a, b, c, d;
	a = (Tp)0; // represents most left point
	b = _duration * (Tp)1; // represents most right point

	c = b - (b - a) / GOLDEN_RATIO;
	d = a + (b - a) / GOLDEN_RATIO;

	while (fabsf(c - d) > RESOLUTION) {
		if (_getDistanceSquared(c, pose) < _getDistanceSquared(d, pose)) {
			b = d;

		} else {
			a = c;
		}

		c = b - (b - a) / GOLDEN_RATIO;
		d = a + (b - a) / GOLDEN_RATIO;
	}

	return (b + a) / (Tp)2;
}

template<typename Tp>
Tp BezierQuad<Tp>::_getDistanceSquared(const Tp t, const Vector3_t &pose)
{
	// get point on bezier
	Vector3_t vec = getPoint(t);

	// get vector from point to pose
	vec = vec - pose;

	// norm squared
	return (vec * vec);
}
}

#endif
