cmake_minimum_required(VERSION 3.20.0)

set(BOARD stm32f3_disco)

set(CONF_FILE "boards/stm32f3_disco.conf")
list(APPEND CONF_FILE "boards/stm32f3_disco.conf")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(se28cp_app)

target_sources(app PRIVATE 
    src/main.h 
    src/led.h 
    src/status.c 
    src/keyboard.h
    src/keyboard.c
    src/adc.c
    src/zbus.c
    src/main.c 
   
)


