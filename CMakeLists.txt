cmake_minimum_required(VERSION 3.20.0)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/modules/dw3000-decadriver)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(ANCLA_ESP32S3)

target_sources(app PRIVATE
	src/anchor_respond.c
	src/anchor_shell.c
	src/beacon_guard.c
	src/disc_schedule.c
	src/gw_core.c
	src/main.c
	src/net_config.c
	src/pos_json.c
	src/pos_sink.c
	src/uwb_config.c
	src/uwb_dwtime.c
	src/uwb_frame_802_15_4z.c
	src/uwb_gateway.c
	src/uwb_radio.c
	src/uwb_slave.c
	src/uwb_store.c
)
