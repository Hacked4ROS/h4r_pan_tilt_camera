cmake_minimum_required(VERSION 2.8.3)

#get rid of the -rdynamic flag which is not supported by avr-gcc
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")
project(h4r_pantilt_mcu_avr_mcu_code NONE)

include_directories(
    ${h4r_pantilt_proto_INCLUDE_DIRS}
    ${h4r_libsersyncproto_INCLUDE_DIRS}
    include
)

SET(CMAKE_C_COMPILER avr-gcc)
SET(CMAKE_CXX_COMPILER avr-gcc)

SET(CMAKE_FIND_ROOT_PATH /usr/lib/gcc/avr )
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

SET(AVR_WARN "-Wall")
SET(AVR_FREQ "F_CPU=16000000UL")
SET(AVR_OPT "-fpack-struct -fshort-enums -funsigned-bitfields -funsigned-char -ffunction-sections")
SET(AVR_CPU_TYPE "-mmcu=atmega328")
SET(AVR_STD "-std=gnu99")

add_definitions("-DF_CPU=16000000UL")
add_definitions("-fpack-struct")
add_definitions("-fshort-enums")
#add_definitions("-Wall")
#add_definitions("-Werror")
add_definitions("-funsigned-bitfields")
add_definitions("-funsigned-char")
add_definitions("-ffunction-sections")
add_definitions("-std=gnu99")
add_definitions("-mmcu=atmega328")

SET(AVR_SOURCES
src/adc.c
src/globals.c
src/init.c
src/main.c
src/pwm.c
src/uart.c
${h4r_libsersyncproto_SRC}
)

add_executable(pan_tilt_avr_arduino_uno.hex
${AVR_SOURCES}
)
  
macro(convert_hex name)
add_custom_command( 
TARGET ${name}
POST_BUILD 
COMMAND avr-objcopy -O ihex ${name}
COMMAND chmod -x ${name}
WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
)
endmacro(convert_hex)

convert_hex(pan_tilt_avr_arduino_uno.hex)

## Mark executables for installation
install(TARGETS pan_tilt_avr_arduino_uno.hex
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


