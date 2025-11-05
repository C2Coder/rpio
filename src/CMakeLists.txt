# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# add optional drivers and capabilities
add_subdirectory(itf)
# add_subdirectory(drv)

# build final fw binary
add_executable(rpio
    entry.c
    misc.c
)

pico_generate_pio_header(rpio ${CMAKE_CURRENT_LIST_DIR}/entry.pio)


pico_set_program_name(rpio "rpio")
pico_set_program_version(rpio "v0")

pico_enable_stdio_uart(rpio 0)
pico_enable_stdio_usb(rpio 1)

target_link_libraries(rpio
    hardware_watchdog
    pico_unique_id
    pico_multicore
    pico_stdlib

    rpio_proto
    itf_includes
    # drv_includes
)

# link drivers with fw binary

target_link_libraries(rpio itf_spi hardware_pio)

if (RPIO_ENABLE_FB)
    target_link_libraries(rpio drv_fb)
endif()

if (RPIO_ENABLE_HUB75)
    target_link_libraries(rpio drv_hub75)
endif()

pico_add_extra_outputs(rpio)