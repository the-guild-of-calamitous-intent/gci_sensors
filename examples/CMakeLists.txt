LIST(APPEND examples
    sox_spi
    # sox_spi_core1
    # sox_i2c
    # bmp
    # lis3
    # gps
    # interrupt
    # lps22_spi
    # qmc5883_i2c
)

foreach(app ${examples})
    add_executable(${app} ${app}.c)
    target_link_libraries(${app} PUBLIC
        ${PROJECT_NAME}
        pico_multicore
    )

    # Set stdio functions to use USB rather than the serial ports
    pico_enable_stdio_usb(${app} 1)
    pico_enable_stdio_uart(${app} 0)

    # create uf2 output --------------------------------------------------
    pico_add_extra_outputs(${app})

    # fix cleanup command ------------------------------------------------
    set_target_properties(${app}
        PROPERTIES
            ADDITIONAL_CLEAN_FILES
            "${app}.bin;${app}.hex;${app}.elf;${app}.dis;${app}.elf.map;${app}.uf2")

    # Show info after project bulid ---------------------------------------
    add_custom_command(
        TARGET ${app}
        POST_BUILD
        COMMAND echo -------------------------------------
                && picotool info ${app}.uf2 -a
                && echo -------------------------------------
    )
endforeach()