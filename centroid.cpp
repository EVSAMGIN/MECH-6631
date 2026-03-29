#include <cmath>
#include "image_transfer.h"
#include "centroid.h"

void draw_marker(ibyte* p0, int width, int height, int radius, double ic, double jc)
{
	int kc = (int)ic + width * (int)jc;
	if (kc < 0 || kc >= width * height) return;

	for (int r = 0; r <= radius; r++) {
		for (int theta = 0; theta < 360; theta++) {
			int i1 = (int)(ic + r * cos(theta * PI / 180.0));
			int j1 = (int)(jc + r * sin(theta * PI / 180.0));
			if (i1 >= 0 && i1 < width && j1 >= 0 && j1 < height) {
				ibyte* pm = p0 + 3 * (i1 + width * j1);
				*pm = 180; *(pm + 1) = 105; *(pm + 2) = 255;
			}
		}
	}
}
