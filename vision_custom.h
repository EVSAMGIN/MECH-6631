#pragma once

#include <cmath>
#include <iostream>
#include <fstream>

#define PI 3.14159265

struct ColourFilter {
	int primary;
	int thresh;
	int ch1, ch2;
	bool RGB;
	ibyte hB, hG, hR;
};

struct TargetPositions {
	double ic[6];
	double jc[6];
	bool   valid[6];
};

void colour_filter(ibyte* p0, int width, int height, ColourFilter& f, int pthresh);
void calculate_HSV(int R, int G, int B, double& hue, double& sat, double& value);
void draw_marker(ibyte* p0, int width, int height, int radius, double ic, double jc);
int threshold_range(image& a, image& b, int tlow1, int thigh1, int tlow2, int thigh2);
int threshold_sat(image& a, image& b, image& rgb1, int tlow, int thigh, double sat_max);
int threshold_v(image& a, image& b, image& rgb, int vmin, int tlow, int thigh, double sat_min);

struct MaskParameters {
	int tlow, thigh;     // greyscale range on grey_gauss
	int hlow, hhigh;     // primary hue range
	int hlow2, hhigh2;   // secondary hue range (-1 = unused)
	int vmin;            // minimum V
	double sat_min;      // minimum saturation
};

int threshold_mask(image& grey_gauss, image& mask, image& rgb, const MaskParameters& pm);
int scale_skew(image& a, image& b, double rskew, double gskew, double bskew);

// Sample pixels within a circle of radius sample_r centred at (ic, jc) in the RGB image.
// Computes mean and std of hue, greyscale, value (V), and saturation, then sets all fields:
//   pm.hlow/hhigh  = mean_hue  +/- max(2*std_hue,  10 deg)  [wraparound handled via hlow2/hhigh2]
//   pm.tlow/thigh  = mean_grey +/- max(2*std_grey, 15 counts)
//   pm.vmin        = mean_val  - max(2*std_val,  20)         [floor at 0]
//   pm.sat_min     = mean_sat  - max(2*std_sat,  0.10)       [floor at 0]
// Near-grey pixels (sat < 0.10) are excluded as their hue is unreliable.
void sample_mask_at_cursor(image& rgb, int ic, int jc, int sample_r, MaskParameters& pm);
