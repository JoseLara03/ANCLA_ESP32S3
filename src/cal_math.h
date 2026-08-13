#ifndef CAL_MATH_H
#define CAL_MATH_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* Stored-record identity / layout guards. */
#define CAL_MAGIC               0xCA11B000u
#define CAL_VERSION             1u

/* Solver slope: combined (TX+RX) antenna-delay unit -> reported distance.
 * Increasing the combined delay by 1 unit decreases reported distance by
 * c * 0.5 * DWT_TIME_UNITS = ~2.34 mm. Stored x1000 for integer math. */
#define CAL_MM_PER_UNIT_X1000   2340

/* Clamp for the combined (TX+RX) antenna delay. */
#define CAL_MAX_TOTAL_DLY       65000u

/* Maximum ranging samples processed per calibration iteration. */
#define CAL_MAX_SAMPLES         128u

/* Persisted calibration record. Explicit padding keeps the layout (and thus
 * the CRC) stable across compilers. CRC is computed over every byte preceding
 * the crc32 field. */
struct cal_record {
    uint32_t magic;       /* CAL_MAGIC */
    uint8_t  version;     /* CAL_VERSION */
    uint8_t  phy_option;  /* CONFIG_OPTION value the cal was taken under */
    uint16_t tx_ant_dly;
    uint16_t rx_ant_dly;
    uint16_t _pad;        /* explicit padding for stable layout */
    uint32_t ref_mm;      /* reference distance used (traceability) */
    uint16_t residual_mm; /* achieved residual error */
    uint16_t _pad2;       /* explicit padding for stable layout */
    uint32_t crc32;       /* integrity, computed last */
};

/* Standard CRC-32 (poly 0xEDB88320, init 0xFFFFFFFF, final XOR 0xFFFFFFFF). */
uint32_t cal_crc32(const void *data, size_t len);

/* Populate magic/version and compute crc32 over the rest of the record. */
void cal_record_finalize(struct cal_record *r);

/* True if magic, version, phy_option and crc32 all check out. */
bool cal_record_valid(const struct cal_record *r, uint8_t expected_phy);

/* Given the mean measured distance and the reference (both mm) and the current
 * combined delay, return the new combined delay (clamped to [0, MAX]). */
uint16_t cal_solve_step(int32_t measured_mm, int32_t ref_mm, uint16_t cur_total_dly);

/* Split a combined delay equally into TX/RX (rx gets the odd remainder). */
void cal_split_dly(uint16_t total, uint16_t *tx, uint16_t *rx);

/* Mean of samples after median/MAD outlier rejection. Returns false if n==0
 * or all samples are rejected. out_kept = number of inliers used. */
bool cal_filtered_mean(const int32_t *samples, size_t n,
                       int32_t *out_mean, size_t *out_kept);

/* Run built-in assertion vectors. Returns the number of failed checks
 * (0 == all pass). */
int cal_math_selftest(void);

#endif /* CAL_MATH_H */
