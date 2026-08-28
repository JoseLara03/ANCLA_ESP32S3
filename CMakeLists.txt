cmake_minimum_required(VERSION 3.20.0)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/modules/dw3000-decadriver)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(ANCLA_ESP32S3)

target_sources(app PRIVATE
	src/anchor_respond.c
	src/anchor_shell.c
	src/apos_frame.c
	src/apos_geom.c
	src/apos_gw.c
	src/apos_node.c
	src/apos_shell.c
	src/apos_store.c
	src/apos_table.c
	src/beacon_guard.c
	src/blink_frame.c
	src/ccp_frame.c
	src/ccp_master.c
	src/ccp_slave.c
	src/disc_schedule.c
	src/gw_core.c
	src/mac_budget.c
	src/main.c
	src/net_config.c
	src/net_shell.c
	src/net_store.c
	src/net_uplink.c
	src/pos_json.c
	src/pos_residual.c
	src/pos_sink.c
	src/pos_solver.c
	src/ss_initiator.c
	src/sync_model.c
	src/sync_shell.c
	src/tag_id.c
	src/tdoa_collect.c
	src/tdoa_solve.c
	src/uwb_config.c
	src/uwb_dwtime.c
	src/uwb_frame_802_15_4z.c
	src/uwb_gateway.c
	src/uwb_radio.c
	src/uwb_slave.c
	src/uwb_store.c
)

# Calibration image only (see cal.conf, docs/superpowers/specs/
# 2026-08-13-antenna-delay-calibration-design.md). ss_initiator.c is
# deliberately NOT here: the production image needs it for the anchor survey
# (apos_node.c), which gates it on a gateway-opened window rather than on the
# build. What must stay out of production is the `cal` shell tree and cal_run's
# unsolicited-poll loop.
target_sources_ifdef(CONFIG_ANCLA_CAL_MODE app PRIVATE
	src/cal_math.c
	src/cal_run.c
	src/cal_shell.c
	src/cal_solve.c
)
