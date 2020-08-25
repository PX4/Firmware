include_directories(
    ./include
    )

add_compile_options(-Wno-unused-variable)
add_library(uavcan_kinetis_driver STATIC
    ./src/uc_kinetis_flexcan.cpp
    ./src/uc_kinetis_clock.cpp
    ./src/uc_kinetis_thread.cpp
    )

add_dependencies(uavcan_kinetis_driver uavcan)

install(DIRECTORY include/uavcan_kinetis DESTINATION include)
install(TARGETS uavcan_kinetis_driver DESTINATION lib)

# vim: set et ft=cmake fenc=utf-8 ff=unix sts=4 sw=4 ts=4 :)
