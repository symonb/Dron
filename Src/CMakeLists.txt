set(MAIN_SRC

    main.c

    setup.c

    acro.c
    adc1.c
    battery.c
    blackbox/blackbox.c
    blackbox/blackbox_encoding.c
    common/encoding.c
    common/printf.c
    common/typeconversion.c
    drivers/SPI1.c
    drivers/SPI2.c
    drivers/SPI3.c
    drivers/i2c1.c
    filters.c
    flash.c
    global_functions.c
    global_variables.c
    ibus.c
    math/statistics.c 
    math/quaternions.c
    math/math_quicker.c
    motors.c
    MPU6000.c
    OSD.c
    rx.c
    scheduler.c
    stabilize.c

    startup/startup_stm32f405xx.s
    startup/system_stm32f4xx.c
    startup/stm32f4xx_it.c
    startup/stm32f4xx_hal_msp.c

    tasks.c
    telemetry.c
    usb.c

    PARENT_SCOPE
)