cmake_minimum_required(VERSION 2.8.12)

# We want to test the lockstep schedule even if it is not used otherwise.
px4_add_library(lockstep_scheduler
	src/lockstep_scheduler.cpp
	src/lockstep_components.cpp
)

target_include_directories(lockstep_scheduler
	PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include
)

px4_add_functional_gtest(SRC test/src/lockstep_scheduler_test.cpp LINKLIBS lockstep_scheduler)
