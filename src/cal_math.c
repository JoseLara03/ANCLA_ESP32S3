#include "cal_math.h"
#include <string.h>

/* ---- CRC-32 (IEEE 802.3, reflected) ---------------------------------------- */
uint32_t cal_crc32(const void *data, size_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFu;

    for (size_t i = 0; i < len; i++) {
        crc ^= p[i];
        for (int b = 0; b < 8; b++) {
            uint32_t mask = -(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

/* Bytes covered by the CRC: everything before the crc32 field. */
#define CAL_CRC_LEN  (offsetof(struct cal_record, crc32))

void cal_record_finalize(struct cal_record *r)
{
    r->magic   = CAL_MAGIC;
    r->version = CAL_VERSION;
    r->_pad    = 0;
    r->_pad2   = 0;
    r->crc32   = cal_crc32(r, CAL_CRC_LEN);
}

bool cal_record_valid(const struct cal_record *r, uint8_t expected_phy)
{
    if (r->magic != CAL_MAGIC) {
        return false;
    }
    if (r->version != CAL_VERSION) {
        return false;
    }
    if (r->phy_option != expected_phy) {
        return false;
    }
    return r->crc32 == cal_crc32(r, CAL_CRC_LEN);
}

/* Round-to-nearest signed integer division (den must be > 0). */
static int32_t div_round_pos(int32_t num, int32_t den)
{
    if (num >= 0) {
        return (num + den / 2) / den;
    }
    return -(((-num) + den / 2) / den);
}

uint16_t cal_solve_step(int32_t measured_mm, int32_t ref_mm, uint16_t cur_total_dly)
{
    /* err > 0 => measuring too far => increase delay to pull distance down. */
    int32_t err_mm = measured_mm - ref_mm;
    int32_t delta_units = div_round_pos(err_mm * 1000, CAL_MM_PER_UNIT_X1000);

    /* Bound a single iteration's correction -- see CAL_MAX_STEP_UNITS. */
    if (delta_units > CAL_MAX_STEP_UNITS) {
        delta_units = CAL_MAX_STEP_UNITS;
    } else if (delta_units < -CAL_MAX_STEP_UNITS) {
        delta_units = -CAL_MAX_STEP_UNITS;
    }

    int32_t new_total = (int32_t)cur_total_dly + delta_units;

    if (new_total < 0) {
        new_total = 0;
    }
    if (new_total > (int32_t)CAL_MAX_TOTAL_DLY) {
        new_total = (int32_t)CAL_MAX_TOTAL_DLY;
    }
    return (uint16_t)new_total;
}

void cal_split_dly(uint16_t total, uint16_t *tx, uint16_t *rx)
{
    *tx = total / 2u;
    *rx = total - *tx;
}

static void sort_i32(int32_t *a, size_t n)
{
    /* Insertion sort: n <= CAL_MAX_SAMPLES (128), simple and allocation-free. */
    for (size_t i = 1; i < n; i++) {
        int32_t key = a[i];
        size_t j = i;
        while (j > 0 && a[j - 1] > key) {
            a[j] = a[j - 1];
            j--;
        }
        a[j] = key;
    }
}

static int32_t median_sorted(const int32_t *a, size_t n)
{
    return a[n / 2];  /* upper-middle for even n; adequate for outlier centring */
}

bool cal_filtered_mean(const int32_t *samples, size_t n,
                       int32_t *out_mean, size_t *out_kept)
{
    if (n == 0 || n > CAL_MAX_SAMPLES) {
        return false;
    }

    int32_t work[CAL_MAX_SAMPLES];
    memcpy(work, samples, n * sizeof(int32_t));
    sort_i32(work, n);
    int32_t med = median_sorted(work, n);

    /* MAD = median of absolute deviations from the median. */
    int32_t devs[CAL_MAX_SAMPLES];
    for (size_t i = 0; i < n; i++) {
        int32_t d = work[i] - med;
        devs[i] = (d < 0) ? -d : d;
    }
    sort_i32(devs, n);
    int32_t mad = median_sorted(devs, n);
    if (mad < 1) {
        mad = 1;  /* floor: avoid rejecting everything when samples are tight */
    }

    int64_t sum = 0;
    size_t kept = 0;
    int32_t limit = 6 * mad;
    for (size_t i = 0; i < n; i++) {
        int32_t d = samples[i] - med;
        if (d < 0) {
            d = -d;
        }
        if (d <= limit) {
            sum += samples[i];
            kept++;
        }
    }
    if (kept == 0) {
        return false;
    }
    *out_mean = (int32_t)(sum / (int64_t)kept);
    *out_kept = kept;
    return true;
}

int cal_math_selftest(void)
{
    int fails = 0;

    /* CRC32 known-answer: "123456789" -> 0xCBF43926. */
    if (cal_crc32("123456789", 9) != 0xCBF43926u) {
        fails++;
    }

    /* Record round-trip: finalize then validate must pass for matching phy
     * and fail for a different phy. */
    struct cal_record r = {0};
    r.phy_option = 7;
    r.tx_ant_dly = 16371;
    r.rx_ant_dly = 16371;
    r.ref_mm = 2000;
    cal_record_finalize(&r);
    if (!cal_record_valid(&r, 7)) {
        fails++;
    }
    if (cal_record_valid(&r, 9)) {
        fails++;
    }
    /* Corrupt a byte -> CRC must reject. */
    r.tx_ant_dly ^= 0x01;
    if (cal_record_valid(&r, 7)) {
        fails++;
    }

    /* Solver: 234 mm too far / 2.34 mm-per-unit = +100 units. */
    if (cal_solve_step(2234, 2000, 32742) != 32842) {
        fails++;
    }
    /* Solver: 234 mm too short = -100 units. */
    if (cal_solve_step(1766, 2000, 32742) != 32642) {
        fails++;
    }
    /* Solver clamps at zero (cannot go negative). */
    if (cal_solve_step(0, 100000, 10) != 0) {
        fails++;
    }
    /* Solver bounds a single correction to +/-CAL_MAX_STEP_UNITS: a 75000 mm
     * error (~32051 raw units) must not swing the total by more than 2000
     * units in one step, regardless of how far off the reading is. */
    if (cal_solve_step(76000, 1000, 31844) != 31844 + CAL_MAX_STEP_UNITS) {
        fails++;
    }
    if (cal_solve_step(1000, 76000, 31844) != 31844 - CAL_MAX_STEP_UNITS) {
        fails++;
    }
    /* Equal split: even and odd totals. */
    uint16_t tx, rx;
    cal_split_dly(32742, &tx, &rx);
    if (tx != 16371 || rx != 16371) {
        fails++;
    }
    cal_split_dly(33, &tx, &rx);
    if (tx != 16 || rx != 17) {
        fails++;
    }

    /* Filtered mean: one gross outlier (5000) must be rejected. */
    int32_t s[] = {100, 102, 98, 101, 99, 5000};
    int32_t mean;
    size_t kept;
    if (!cal_filtered_mean(s, 6, &mean, &kept)) {
        fails++;
    } else {
        if (kept != 5) {
            fails++;
        }
        if (mean < 98 || mean > 102) {
            fails++;
        }
    }

    return fails;
}
