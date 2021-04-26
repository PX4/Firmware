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
 * @@file FlightTasks_generated.hpp
 *
 * Generated Header to list all required flight tasks
 *
 * @@author Christoph Tobler <christoph@@px4.io>
 */

 #pragma once

// include all required headers
#include "FlightModeManager.hpp"
@# loop through all requested tasks
@[if tasks]@
@[for task in tasks]@
#include "FlightTask@(task).hpp"
@[end for]@
@[end if]@

enum class FlightTaskIndex : int {
    None = -1,
@# loop through all requested tasks
@[if tasks]@
@[for task in tasks]@
    @(task),
@[end for]@
@[end if]@

    Count // number of tasks
};

union TaskUnion {
    TaskUnion() {}
    ~TaskUnion() {}

@# loop through all requested tasks
@[if tasks]@
@{
firstLowercase = lambda s: s[:1].lower() + s[1:] if s else ''
}@
@[for task in tasks]@
    FlightTask@(task) @(firstLowercase(task));
@[end for]@
@[end if]@
};
