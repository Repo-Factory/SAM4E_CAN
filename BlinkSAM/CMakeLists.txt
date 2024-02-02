cmake_minimum_required(VERSION 3.12)

project(SAM4E_CAN)

set(MCU "cortex-m4")
set(FPU "softfp") 

set(CMAKE_C_COMPILER arm-none-eabi-gcc)

# Source files
add_executable(
    ${PROJECT_NAME}
 	src/ASF/common/services/clock/sam4e/sysclk.c
 	src/ASF/common/utils/interrupt/interrupt_sam_nvic.c
 	src/ASF/sam/boards/sam4e_ek/init.c
 	src/ASF/sam/drivers/pmc/pmc.c
 	src/ASF/sam/drivers/pmc/sleep.c
 	src/ASF/sam/utils/cmsis/sam4e/source/templates/exceptions.c
 	src/ASF/sam/utils/cmsis/sam4e/source/templates/gcc/startup_sam4e.c
 	src/ASF/sam/utils/cmsis/sam4e/source/templates/system_sam4e.c
 	src/ASF/sam/utils/syscalls/gcc/syscalls.c
 	src/main.c
)

# Include directories
include_directories(
    src
    src/ASF
    src/ASF/common
    src/ASF/common/boards
    src/ASF/common/services
    src/ASF/common/services/clock
    src/ASF/common/services/clock/sam4e
    src/ASF/common/services/ioport
    src/ASF/common/services/ioport/sam
    src/ASF/common/utils
    src/ASF/common/utils/interrupt
    src/ASF/sam
    src/ASF/sam/boards
    src/ASF/sam/boards/sam4e_ek
    src/ASF/sam/drivers
    src/ASF/sam/drivers/pmc
    src/ASF/sam/utils
    src/ASF/sam/utils/cmsis
    src/ASF/sam/utils/cmsis/sam4e
    src/ASF/sam/utils/cmsis/sam4e/include
    src/ASF/sam/utils/cmsis/sam4e/include/component
    src/ASF/sam/utils/cmsis/sam4e/include/instance
    src/ASF/sam/utils/cmsis/sam4e/include/pio
    src/ASF/sam/utils/cmsis/sam4e/source
    src/ASF/sam/utils/cmsis/sam4e/source/templates
    src/ASF/sam/utils/cmsis/sam4e/source/templates/gcc
    src/ASF/sam/utils/fpu
    src/ASF/sam/utils/header_files
    src/ASF/sam/utils/linker_scripts
    src/ASF/sam/utils/linker_scripts/sam4e
    src/ASF/sam/utils/linker_scripts/sam4e/sam4e16
    src/ASF/sam/utils/linker_scripts/sam4e/sam4e16/gcc
    src/ASF/sam/utils/make
    src/ASF/sam/utils/preprocessor
    src/ASF/sam/utils/syscalls
    src/ASF/sam/utils/syscalls/gcc
    src/ASF/thirdparty
    src/ASF/thirdparty/CMSIS
    src/ASF/thirdparty/CMSIS/Include
    src/ASF/thirdparty/CMSIS/Lib
    src/ASF/thirdparty/CMSIS/Lib/GCC
    src/config
)

target_compile_options(${PROJECT_NAME} PRIVATE
    -mcpu=${MCU}
    -mthumb
    -Wall
    -O2
    -D__SAM4E16E__
    -DDEBUG
    -DBOARD=SAM4E_EK
    -Dscanf=iscanf
    -DARM_MATH_CM4=true
    -Dprintf=iprintf
    -D__SAM4E16E__
)

# Add linker options
target_link_options(${PROJECT_NAME} PRIVATE
    -mcpu=${MCU}
    -mthumb
    -T ${CMAKE_SOURCE_DIR}/src/ASF/sam/utils/linker_scripts/sam4e/sam4e16/gcc/flash.ld
    -Wl,-Map=${PROJECT_NAME}.map
    -Wl,--start-group
    -larm_cortexM4lf_math_${FPU}
    -lm
    -Wl,--end-group
    -L"${CMAKE_SOURCE_DIR}/src/ASF/thirdparty/CMSIS/Lib/GCC"
    -Wl,--gc-sections
    -Wl,--entry=Reset_Handler
    -Wl,--cref
)