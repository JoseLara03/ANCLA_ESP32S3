#include "pos_residual.h"
#include <math.h>

float pos_residual_rms(const struct pos_meas *m, size_t n, float x, float y)
{
    if (n == 0) {
        return 0.0f;
    }

    float acc = 0.0f;

    for (size_t i = 0; i < n; i++) {
        float dx   = x - m[i].x;
        float dy   = y - m[i].y;
        float dz   = m[i].dz;
        float pred = sqrtf(dx * dx + dy * dy + dz * dz);
        float err  = pred - m[i].range_m;

        acc += err * err;
    }

    return sqrtf(acc / (float)n);
}
