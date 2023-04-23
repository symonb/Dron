cmake_minimum_required(VERSION 3.16)
set(CMAKE_SYSTEM_NAME Generic)

# turn off compiler checking (compiler is working):
set(CMAKE_C_COMPILER_WORKS TRUE)
# set C standard:
set(CMAKE_C_STANDARD 17)
# set directory for binary files (variable):
set(BUILD_DIR ${CMAKE_SOURCE_DIR}/bin)
# set path for build files:
set(EXECUTABLE_OUTPUT_PATH ${BUILD_DIR})
# create targets variables:
set(TARGET "main")
set(TARGET_ELF "${TARGET}.elf")
set(TARGET_HEX "${TARGET}.hex")
set(TARGET_BIN "${TARGET}.bin")

# create project name (DRON) and define required languages for building (C and assembler - ASM should be at the end):
project(DRON C ASM)

# create main SRC variable:
set(MAIN_SRC_DIR "${CMAKE_SOURCE_DIR}/Src")

set(LINKER_DIR "${CMAKE_SOURCE_DIR}/link")

set(LD_include "-lc -lm -lnosys -L${LINKER_DIR}")

set(linker_script "${LINKER_DIR}/STM32F405RGTx_FLASH.ld")

set(MCU_flags "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb")


# C defines (defines parse with cmd line not defined in files):
set(C_DEFS "-DUSE_HAL_DRIVER -DSTM32F405xx")

set(C_flags "${MCU_flags} ${C_DEFS} -Wall -fdata-sections -ffunction-sections -DARM_MATH_CM4 -fanalyzer ")
set(AS_flags "${MCU_flags} -Wall -fdata-sections -ffunction-sections")
set(LD_flags "${MCU_flags} -specs=nano.specs -specs=nosys.specs  -T${linker_script} ${LD_include} -Wl,--print-memory-usage -u _printf_float ")

# macro for adding path for execatubles files defined in "${MAIN_SRC_DIR}/"
macro(main_sources var)
    set(${var} ${ARGN})
    list(TRANSFORM ${var} PREPEND "${MAIN_SRC_DIR}/")
endmacro()

add_subdirectory("${MAIN_SRC_DIR}")

# add paths for executables files:
main_sources(src "${MAIN_SRC}")

add_executable(${TARGET_ELF}
    "${src}"
    USB_files/USB_DEVICE/App/usb_device.c 
    USB_files/USB_DEVICE/App/usbd_desc.c 
    USB_files/USB_DEVICE/App/usbd_cdc_if.c 
    USB_files/USB_DEVICE/Target/usbd_conf.c 
    USB_files/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c 
    USB_files/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c 
    USB_files/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c 
    USB_files/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c 
    USB_files/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c 
    USB_files/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c 
    USB_files/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c 
    USB_files/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
    USB_files/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
    USB_files/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c 
    USB_files/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c 
    USB_files/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
    USB_files/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c 
    USB_files/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c 
    USB_files/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c 
    USB_files/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c 
    USB_files/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c 
    USB_files/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c 
    USB_files/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c 
    USB_files/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c 
    USB_files/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c 
    USB_files/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c 
)

target_include_directories(${TARGET_ELF} PUBLIC
    Src
    Src/startup
    Drivers/Include
    Drivers/CMSIS/DSP/Include
    Drivers/STM32F4xx/Include
    USB_files/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    USB_files/Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    USB_files/STM32F4xx_HAL_Driver/Inc
    USB_files/STM32F4xx_HAL_Driver/Inc/Legacy
    USB_files/USB_DEVICE/App
    USB_files/USB_DEVICE/Target)

set(CMAKE_C_FLAGS "${C_flags}")
set(CMAKE_ASM_FLAGS "${AS_flags}")
set(CMAKE_EXE_LINKER_FLAGS "${LD_flags}")

target_link_libraries(${TARGET_ELF} PUBLIC "${CMAKE_SOURCE_DIR}/Drivers/libarm_cortexM4lf_math.a")
target_link_libraries(${TARGET_ELF} PUBLIC c m)

set(OBJCOPY arm-none-eabi-objcopy)
add_custom_target(${TARGET_BIN} ALL COMMAND ${OBJCOPY} -O binary -S ${BUILD_DIR}/${TARGET_ELF} ${BUILD_DIR}/${TARGET_BIN})
add_custom_target(${TARGET_HEX} ALL COMMAND ${OBJCOPY} -O ihex -S ${BUILD_DIR}/${TARGET_ELF} ${BUILD_DIR}/${TARGET_HEX})

add_dependencies(${TARGET_BIN} ${TARGET_ELF} ${TARGET_HEX})
