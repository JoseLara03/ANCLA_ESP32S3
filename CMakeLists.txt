cmake_minimum_required(VERSION 3.20.0)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/modules/dw3000-decadriver)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(ANCLA_ESP32S3)

target_sources(app PRIVATE src/main.c)
