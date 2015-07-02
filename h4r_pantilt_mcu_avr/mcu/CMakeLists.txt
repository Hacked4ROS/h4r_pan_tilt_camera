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


SET(MCU_TYPE atmega328p)
SET(FREQUENCY_HZ 16000000)
SET(OPTIMIZATIONS 0)
SET(STANDARD gnu99)


#Compiler
SET(CMAKE_C_COMPILER avr-gcc)
SET(CMAKE_CXX_COMPILER avr-gcc)
#Linker
SET(CMAKE_LINKER avr-g++)


SET(CMAKE_FIND_ROOT_PATH /usr/lib/gcc/avr )
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)


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


  set_target_properties(
      pan_tilt_avr_arduino_uno.hex
      PROPERTIES
         COMPILE_FLAGS "-mmcu=${MCU_TYPE} -O${OPTIMIZATIONS} -DF_CPU=${FREQUENCY_HZ}UL -std=c99"
         LINK_FLAGS "-mmcu=${MCU_TYPE} -Wl,--gc-sections -mrelax -Wl,-Map,pan_tilt_avr_arduino_uno.map"
   )
  
macro(convert_hex name)
add_custom_command( 
TARGET ${name}
POST_BUILD 
COMMAND cp ${name} ${name}.elf
COMMAND avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex ${name}
COMMAND chmod -x ${name}
WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
)
endmacro(convert_hex)

convert_hex(pan_tilt_avr_arduino_uno.hex)

## Mark executables for installation
install(TARGETS pan_tilt_avr_arduino_uno.hex
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


