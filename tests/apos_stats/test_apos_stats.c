#include "apos_stats.h"

#include <stdio.h>

static int g_fail;

#define CHECK(cond) do { \
	if (!(cond)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); g_fail++; } \
} while (0)

static void test_clean_batch(void)
{
	int32_t s[5] = {1000, 1010, 990, 1005, 995};
	int32_t mean;
	uint16_t sd;

	CHECK(apos_batch_stats(s, 5, &mean, &sd) == 5);
	CHECK(mean == 1000);
	CHECK(sd == 7); /* sqrt((0+100+100+25+25)/5) = 7.07 */
}

static void test_outlier_is_dropped_and_not_in_sd(void)
{
	int32_t s[6] = {1000, 1002, 998, 1001, 999, 2500};
	int32_t mean;
	uint16_t sd;

	CHECK(apos_batch_stats(s, 6, &mean, &sd) == 5);
	CHECK(mean == 1000);
	CHECK(sd < 5);
}

static void test_empty_and_single(void)
{
	int32_t s[1] = {1234};
	int32_t mean = 99;
	uint16_t sd = 99;

	CHECK(apos_batch_stats(s, 0, &mean, &sd) == 0 && mean == 0 && sd == 0);
	CHECK(apos_batch_stats(s, 1, &mean, &sd) == 1 && mean == 1234 && sd == 0);
}

static void test_clamps_to_max(void)
{
	int32_t s[100];
	int32_t mean;
	uint16_t sd;

	for (int i = 0; i < 100; i++) {
		s[i] = 5000;
	}
	CHECK(apos_batch_stats(s, 100, &mean, &sd) == APOS_STATS_MAX);
	CHECK(mean == 5000 && sd == 0);
}

int main(void)
{
	test_clean_batch();
	test_outlier_is_dropped_and_not_in_sd();
	test_empty_and_single();
	test_clamps_to_max();
	if (g_fail) {
		printf("%d CHECK(s) FAILED\n", g_fail);
		return 1;
	}
	printf("apos_stats: ALL TESTS PASSED\n");
	return 0;
}
