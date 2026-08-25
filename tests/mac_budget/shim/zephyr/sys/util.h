/* Host-test shim: the only thing src/uwb_mac.h needs from Zephyr is
 * BUILD_ASSERT. Same pattern as the tag's tests/uwb_radio_owner/shim.
 * Lets the budget asserts in uwb_mac.h be checked with plain gcc instead of
 * only at Zephyr build time. */
#ifndef SHIM_ZEPHYR_SYS_UTIL_H
#define SHIM_ZEPHYR_SYS_UTIL_H
#define BUILD_ASSERT(cond, msg) _Static_assert(cond, msg)
#endif
