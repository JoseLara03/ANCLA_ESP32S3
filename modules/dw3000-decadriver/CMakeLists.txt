# SPDX-License-Identifier: Apache-2.0

if(CONFIG_DW3000)

# LOCAL DELTA vs upstream: named explicitly. A bare zephyr_library() derives the
# name from the module's path relative to ZEPHYR_BASE, which for an out-of-tree
# module on a different drive path yields a ~200-char object directory and trips
# CMAKE_OBJECT_PATH_MAX (250) on Windows.
zephyr_library_named(dw3000_decadriver)

zephyr_library_sources(
    platform/deca_port.c
    platform/dw3000_hw.c
    platform/dw3000_spi.c
    platform/dw3000_spi_trace.c
    platform/deca_compat.c
    dwt_uwb_driver/deca_interface.c
    dwt_uwb_driver/deca_rsl.c
    dwt_uwb_driver/lib/qmath/src/qmath.c
)

zephyr_library_sources_ifdef(CONFIG_DW3000_CHIP_DW3000 dwt_uwb_driver/dw3000/dw3000_device.c)
zephyr_library_sources_ifdef(CONFIG_DW3000_CHIP_DW3720 dwt_uwb_driver/dw3720/dw3720_device.c)

zephyr_include_directories(platform)
zephyr_include_directories(dwt_uwb_driver)
zephyr_include_directories(dwt_uwb_driver/lib/qmath/include)

endif(CONFIG_DW3000)
