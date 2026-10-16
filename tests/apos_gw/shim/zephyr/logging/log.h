/* Host-test shim: log lines go to stdout only when APOS_GW_TEST_VERBOSE is set
 * in the environment, and are always counted by level so a test can assert that
 * an error WAS reported. */
#ifndef SHIM_ZEPHYR_LOG_H
#define SHIM_ZEPHYR_LOG_H

void shim_log(int level, const char *fmt, ...);

#define LOG_MODULE_REGISTER(name, lvl) typedef int shim_log_dummy_##name
#define LOG_DBG(...) shim_log(0, __VA_ARGS__)
#define LOG_INF(...) shim_log(1, __VA_ARGS__)
#define LOG_WRN(...) shim_log(2, __VA_ARGS__)
#define LOG_ERR(...) shim_log(3, __VA_ARGS__)

#endif
