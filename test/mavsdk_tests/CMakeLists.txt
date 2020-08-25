cmake_minimum_required(VERSION 3.5.1)

project(mavsdk_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(MAVSDK REQUIRED)

if(MAVSDK_FOUND)
    add_executable(mavsdk_tests
        test_main.cpp
        autopilot_tester.cpp
        test_multicopter_offboard.cpp
        test_multicopter_mission.cpp
        test_vtol_mission.cpp
    )

    target_link_libraries(mavsdk_tests
        MAVSDK::mavsdk
        MAVSDK::mavsdk_action
        MAVSDK::mavsdk_mission
        MAVSDK::mavsdk_offboard
        MAVSDK::mavsdk_telemetry
    )

    target_compile_options(mavsdk_tests
        PRIVATE -Wall -Wextra -Werror
    )
else()
    message("MAVSDK C++ not found, skipping mavsdk_tests build..")
endif()
