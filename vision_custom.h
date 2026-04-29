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
	double ic_car1_1, jc_car1_1; bool valid_car1_1;
	double ic_car1_2, jc_car1_2; bool valid_car1_2;
	double ic_car2_1, jc_car2_1; bool valid_car2_1;
	double ic_car2_2, jc_car2_2; bool valid_car2_2;
	double ic_obs1,   jc_obs1;   bool valid_obs1;
	double ic_obs2,   jc_obs2;   bool valid_obs2;
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
