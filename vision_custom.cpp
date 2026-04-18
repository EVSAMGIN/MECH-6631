#include <cmath>
#include "image_transfer.h"
#include "vision_custom.h"

void colour_filter(ibyte* p0, int width, int height, ColourFilter& f, int pthresh)
{
	int i, j, k, q, up[6][3], down[6][3], product_up, product_down;
	ibyte* p, * p_U, * p_D;

	int ca = (f.primary + 1) % 3;
	int cb = (f.primary + 2) % 3;

	for (j = 0; j < height; j++) {
		for (i = 0; i < width; i++) {

			k = i + width * j;
			p = p0 + 3 * k;

			product_up = 1;
			product_down = 1;

			for (q = 1; q <= pthresh; q++) {
				if (j > pthresh && j < (height - pthresh)) {
					p_U = p + 3 * width * q;
					p_D = p - 3 * width * q;
				}
				else if (j <= pthresh) {
					p_U = p + 3 * width * q;
					p_D = p;
				}
				else {
					p_U = p;
					p_D = p - 3 * width * q;
				}

				if (!f.RGB) {
					up[q][0] = *(p_U + f.primary) > f.thresh;
					up[q][1] = *(p_U + ca) < f.ch1;
					up[q][2] = *(p_U + cb) < f.ch2;
					down[q][0] = *(p_D + f.primary) > f.thresh;
					down[q][1] = *(p_D + ca) < f.ch1;
					down[q][2] = *(p_D + cb) < f.ch2;
				}
				else {
					up[q][0] = *(p_U + 2) > f.thresh;
					up[q][1] = *(p_U + 1) > f.thresh;
					up[q][2] = *(p_U)     < f.ch1;
					down[q][0] = *(p_D + 2) > f.thresh;
					down[q][1] = *(p_D + 1) > f.thresh;
					down[q][2] = *(p_D)     < f.ch1;
				}

				product_up   *= up[q][0]   * up[q][1]   * up[q][2];
				product_down *= down[q][0] * down[q][1] * down[q][2];
			}

			int match = 0;
			if (!f.RGB)
				match = (*(p + f.primary) > f.thresh) && (*(p + ca) < f.ch1) && (*(p + cb) < f.ch2);
			else
				match = (*(p + 2) > f.thresh) && (*(p + 1) > f.thresh) && (*p < f.ch1);

			if (match && (product_up == 1 || product_down == 1)) {
				*p = f.hB; *(p + 1) = f.hG; *(p + 2) = f.hR;
			}
		}
	}
}

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

int threshold_range(image& a, image& b, int tlow1, int thigh1, int tlow2, int thigh2)
{
	ibyte* pa = (ibyte*)a.pdata;
	ibyte* pb = (ibyte*)b.pdata;
	int np = a.width * a.height;
	if (tlow1 > thigh1) std::swap(tlow1, thigh1);
	if (tlow2 > thigh2) std::swap(tlow2, thigh2);
	for (int k = 0; k < np; k++) {
		int v = pa[k];
		bool in1 = (v >= tlow1 && v <= thigh1);
		bool in2 = (v >= tlow2 && v <= thigh2);
		pb[k] = (in1 || in2) ? 255 : 0;
	}
	return 0;
}

void calculate_HSV(int R, int G, int B, double& hue, double& sat, double& value)
{
	int mx = R, mn = R;
	if (G > mx) mx = G; if (B > mx) mx = B;
	if (G < mn) mn = G; if (B < mn) mn = B;
	int delta = mx - mn;
	value = mx;
	sat = (delta == 0) ? 0.0 : (double)delta / value;
	if (delta == 0) { hue = 0; return; }
	if      (mx == R) hue = 60.0 * (double)(G - B) / delta;
	else if (mx == G) hue = 60.0 * ((double)(B - R) / delta + 2);
	else              hue = 60.0 * ((double)(R - G) / delta + 4);
	if (hue < 0) hue += 360;
}

int threshold_sat(image& a, image& b, image& rgb1, int tlow, int thigh, double sat_max)
{
	ibyte* pa = a.pdata;
	ibyte* pb = b.pdata;
	ibyte* pr = rgb1.pdata;
	int np = a.width * a.height;
	for (int k = 0; k < np; k++) {
		if (pa[k] >= tlow && pa[k] <= thigh) {
			double hue, sat, val;
			calculate_HSV(pr[3*k+2], pr[3*k+1], pr[3*k], hue, sat, val);
			pb[k] = (sat <= sat_max) ? 255 : 0;
		} else {
			pb[k] = 0;
		}
	}
	return 0;
}

int threshold_v(image& a, image& b, image& rgb, int vmin, int tlow, int thigh, double sat_min)
{
	ibyte* pb = b.pdata;
	int np = a.width * a.height;
	for (int k = 0; k < np; k++) {
		int grey = (int)(0.299*rgb.pdata[3*k+2] + 0.587*rgb.pdata[3*k+1] + 0.114*rgb.pdata[3*k]);
		double hue, sat, val;
		calculate_HSV(rgb.pdata[3*k+2], rgb.pdata[3*k+1], rgb.pdata[3*k], hue, sat, val);
		pb[k] = (val >= vmin && grey >= tlow && grey <= thigh && sat >= sat_min) ? 255 : 0;
	}
	return 0;
}

int threshold_mask(image& grey_gauss, image& mask, image& rgb, const MaskParameters& pm)
{
	ibyte* pg = grey_gauss.pdata;
	ibyte* pb = mask.pdata;
	int np = mask.width * mask.height;
	for (int k = 0; k < np; k++) {
		double hue, sat, val;
		calculate_HSV(rgb.pdata[3*k+2], rgb.pdata[3*k+1], rgb.pdata[3*k], hue, sat, val);
		bool grey_ok = (pg[k] >= pm.tlow && pg[k] <= pm.thigh);
		bool hue_ok  = (hue >= pm.hlow && hue <= pm.hhigh);
		if (pm.hlow2 >= 0) hue_ok = hue_ok || (hue >= pm.hlow2 && hue <= pm.hhigh2);
		pb[k] = (grey_ok && hue_ok && val >= pm.vmin && sat >= pm.sat_min) ? 255 : 0;
	}
	return 0;
}

int scale_skew(image& a, image& b, double rskew, double gskew, double bskew)
{
	i4byte size, i;
	ibyte* pa, * pb, min, max;

	pa = a.pdata;
	pb = b.pdata;

	if (a.height != b.height || a.width != b.width) {
		std::cout << "\nerror in scale_skew: sizes not the same!";
		return 1;
	}
	if (a.type != b.type) {
		std::cout << "\nerror in scale_skew: types not the same!";
		return 1;
	}

	min = 255; max = 0;

	if (a.type == RGB_IMAGE) {
		size = (i4byte)a.width * a.height * 3;
		for (i = 0; i < size; i += 3) {
			if (pa[i]   < min) min = pa[i];   if (pa[i]   > max) max = pa[i];
			if (pa[i+1] < min) min = pa[i+1]; if (pa[i+1] > max) max = pa[i+1];
			if (pa[i+2] < min) min = pa[i+2]; if (pa[i+2] > max) max = pa[i+2];
		}
		for (i = 0; i < size; i += 3) {
			double n, boosted, sigma = 0.15;
			n = (double)(pa[i]   - min) / (double)(max - min);
			boosted = n + bskew * exp(-((n-0.5)*(n-0.5)) / (2*sigma*sigma));
			if (boosted > 1.0) boosted = 1.0;
			pb[i] = (ibyte)(255.0 * boosted);

			n = (double)(pa[i+1] - min) / (double)(max - min);
			boosted = n + gskew * exp(-((n-0.5)*(n-0.5)) / (2*sigma*sigma));
			if (boosted > 1.0) boosted = 1.0;
			pb[i+1] = (ibyte)(255.0 * boosted);

			n = (double)(pa[i+2] - min) / (double)(max - min);
			boosted = n + rskew * exp(-((n-0.5)*(n-0.5)) / (2*sigma*sigma));
			if (boosted > 1.0) boosted = 1.0;
			pb[i+2] = (ibyte)(255.0 * boosted);
		}
	}
	else if (a.type == GREY_IMAGE) {
		size = (i4byte)a.width * a.height;
		for (i = 0; i < size; i++) {
			if (pa[i] < min) min = pa[i];
			if (pa[i] > max) max = pa[i];
		}
		for (i = 0; i < size; i++)
			pb[i] = (ibyte)(255.0 * (pa[i] - min) / (max - min));
	}
	else {
		std::cout << "\nerror in scale_skew: type not valid!";
		return 1;
	}
	return 0;
}
