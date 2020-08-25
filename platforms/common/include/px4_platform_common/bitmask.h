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

/**
 * @file bitmask.h
 * @brief Provides SFINAE for type safe templated bitmask operations
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 *
 */

#pragma once

#if defined(__cplusplus) && (defined(__PX4_POSIX) || defined(__PX4_LINUX))

#include<type_traits>

template<typename E>
struct enable_bitmask_operators {
	static const bool enable = false;
};

namespace px4
{

#define ENABLE_BIT_OPERATORS(E) \
	template<> \
	struct enable_bitmask_operators<E> \
	{ \
		static const bool enable = true; \
	};

template<typename E>
typename std::enable_if<enable_bitmask_operators<E>::enable, E>::type
operator==(E lhs, E rhs)
{
	typedef typename std::underlying_type<E>::type underlying;
	return static_cast<E>(
		       static_cast<underlying>(lhs) ==
		       static_cast<underlying>(rhs)
	       );
}

template<typename E>
typename std::enable_if<enable_bitmask_operators<E>::enable, E>::type
operator~(E lhs)
{
	typedef typename std::underlying_type<E>::type underlying;
	return static_cast<E>(
		       ~static_cast<underlying>(lhs)
	       );
}

template<typename E>
typename std::enable_if<enable_bitmask_operators<E>::enable, E>::type
operator|(E lhs, E rhs)
{
	typedef typename std::underlying_type<E>::type underlying;
	return static_cast<E>(
		       static_cast<underlying>(lhs) |
		       static_cast<underlying>(rhs)
	       );
}

template<typename E>
typename std::enable_if<enable_bitmask_operators<E>::enable, E &>::type
operator|=(E &lhs, E rhs)
{
	typedef typename std::underlying_type<E>::type underlying;
	lhs = static_cast<E>(
		      static_cast<underlying>(lhs) |
		      static_cast<underlying>(rhs)
	      );
	return lhs;
}

template<typename E>
typename std::enable_if<enable_bitmask_operators<E>::enable, E>::type
operator&(E lhs, E rhs)
{
	typedef typename std::underlying_type<E>::type underlying;
	return static_cast<E>(
		       static_cast<underlying>(lhs) &
		       static_cast<underlying>(rhs)
	       );
}

template<typename E>
typename std::enable_if<enable_bitmask_operators<E>::enable, E &>::type
operator&=(E &lhs, E rhs)
{
	typedef typename std::underlying_type<E>::type underlying;
	lhs = static_cast<E>(
		      static_cast<underlying>(lhs) &
		      static_cast<underlying>(rhs)
	      );
	return lhs;
}

template<typename E>
typename std::enable_if<enable_bitmask_operators<E>::enable, E>::type
operator^(E lhs, E rhs)
{
	typedef typename std::underlying_type<E>::type underlying;
	return static_cast<E>(
		       static_cast<underlying>(lhs) ^
		       static_cast<underlying>(rhs)
	       );
}

template<typename E>
typename std::enable_if<enable_bitmask_operators<E>::enable, E &>::type
operator^=(E &lhs, E rhs)
{
	typedef typename std::underlying_type<E>::type underlying;
	lhs = static_cast<E>(
		      static_cast<underlying>(lhs) ^
		      static_cast<underlying>(rhs)
	      );
	return lhs;
}

} /* namespace px4 */

#endif /* __cplusplus && (__PX4_POSIX || __PX4_LINUX) */
