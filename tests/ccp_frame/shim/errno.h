/*
 * Host-test shim, NOT part of the firmware build.
 *
 * This machine's default `gcc` resolves to a 32-bit mingw.org toolchain
 * (GCC 6.3.0, `mingw32` target) whose <errno.h> only carries ~40 basic
 * POSIX.1 codes -- EMSGSIZE, EPROTO, EBADMSG, ENODATA are simply absent, no
 * feature-test macro adds them. Recorded in this project's own memory
 * (ancla-build-env-fixes.md) as a real, previously-flagged environment gap
 * affecting tests/ccp_frame/ among others, deliberately left unfixed at
 * source level then because patching every affected .c file's error codes
 * is a real decision, not a one-liner -- this is the smaller, scoped fix:
 * a test-only shim, same precedent as tests/mac_budget/shim/'s
 * zephyr/sys/util.h. `-Itests/ccp_frame/shim` ahead of the system include
 * path is what makes gcc pick this up instead of (before, via
 * #include_next) the real one.
 */
#ifndef ANCLA_CCP_FRAME_TEST_ERRNO_SHIM_H
#define ANCLA_CCP_FRAME_TEST_ERRNO_SHIM_H

#include_next <errno.h>

#ifndef EMSGSIZE
#define EMSGSIZE 90
#endif
#ifndef EPROTO
#define EPROTO 71
#endif
#ifndef EBADMSG
#define EBADMSG 74
#endif
#ifndef ENODATA
#define ENODATA 61
#endif

#endif /* ANCLA_CCP_FRAME_TEST_ERRNO_SHIM_H */
