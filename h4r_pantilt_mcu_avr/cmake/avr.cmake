SET(CMAKE_C_COMPILER avr-gcc)
SET(CMAKE_CXX_COMPILER avr-gcc)

SET(CMAKE_FIND_ROOT_PATH /usr/lib/gcc/avr )
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

SET(STD "-std=gnu99")
SET(OPT "-Os")
SET(MCU "-mmciu=atmega16")
SET(SPD "-DF_CPU=4000000UL")
SET(WARN "-Wall")

SET(CFLAGS ${STD} ${OPT} ${SPD} ${WARN})
SET(CXXFLAGS ${MCU} ${OPT} ${SPD}) 

SET(CMAKE_C_FLAGS ${CFLAGS})
SET(CMAKE_CXX_FLAGS ${CXXFLAGS})

include_directories(../include/pan_tilt_avr)
add_executable(pan_tilt_avr
../src/pan_tilt_avr/adc.c
../src/pan_tilt_avr/globals.c
../src/pan_tilt_avr/init.c
../src/pan_tilt_avr/main.c
../src/pan_tilt_avr/pwm.c
../src/pan_tilt_avr/uart.c
)
