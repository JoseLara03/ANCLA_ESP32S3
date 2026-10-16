/* Cross-check driver: runs the REAL firmware tdoa_solve() and pos_abg gain code
 * on cases supplied on stdin, so edge/tdoa_server/tdoa.py can be compared
 * against them number for number.  Build + run: see edge/tools/xcheck.py.
 *
 * stdin, one case per line:
 *   S <n> <hasSeed> <sx> <sy>  then n x (x y dz t_dtu)      -> "S ok x y resid" | "S fail"
 *   G <lambda>                                               -> "G alpha beta gamma"
 */
#include <stdio.h>
#include <stdlib.h>
#include "pos_abg.h"
#include "tdoa_solve.h"

int main(void)
{
	char tag;

	while (scanf(" %c", &tag) == 1) {
		if (tag == 'G') {
			double lam;
			struct pos_abg_cfg c;

			if (scanf("%lf", &lam) != 1) { return 2; }
			pos_abg_cfg_from_index(&c, (float)lam);
			printf("G %.9g %.9g %.9g\n", (double)c.alpha, (double)c.beta, (double)c.gamma);
		} else if (tag == 'S') {
			int n, has;
			double sx, sy;
			struct tdoa_meas m[POS_MAX_ANCHORS];
			struct pos_result r;
			float seed[2];

			if (scanf("%d %d %lf %lf", &n, &has, &sx, &sy) != 4 || n > POS_MAX_ANCHORS) {
				return 2;
			}
			for (int i = 0; i < n; i++) {
				double x, y, dz;
				long t;

				if (scanf("%lf %lf %lf %ld", &x, &y, &dz, &t) != 4) { return 2; }
				m[i].x = (float)x;
				m[i].y = (float)y;
				m[i].dz = (float)dz;
				m[i].t_dtu = t;
				m[i].sigma_m = 0.0f;
			}
			seed[0] = (float)sx;
			seed[1] = (float)sy;
			if (tdoa_solve(m, (size_t)n, has ? seed : NULL, &r) && r.valid) {
				printf("S ok %.9g %.9g %.9g\n", (double)r.x, (double)r.y, (double)r.residual_m);
			} else {
				printf("S fail\n");
			}
		}
	}
	return 0;
}
