include_directories(
    ./include
    )

add_library(uavcan_stm32h7_driver STATIC
    ./src/uc_stm32h7_can.cpp
    ./src/uc_stm32h7_clock.cpp
    ./src/uc_stm32h7_thread.cpp
    )

add_dependencies(uavcan_stm32h7_driver uavcan)

install(DIRECTORY include/uavcan_stm32h7 DESTINATION include)
install(TARGETS uavcan_stm32h7_driver DESTINATION lib)

# vim: set et ft=cmake fenc=utf-8 ff=unix sts=4 sw=4 ts=4 :)
