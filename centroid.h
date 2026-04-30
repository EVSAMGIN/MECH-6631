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

struct CentroidData {
	double ic_1, jc_1;
	double ic_2, jc_2;
	double ic_3, jc_3;
	double ic_4, jc_4;
	int frame;
};

void colour_filter(ibyte* p0, int width, int height, ColourFilter& f, int pthresh);
void calculate_HSV(int R, int G, int B, double& hue, double& sat, double& value);
void draw_marker(ibyte* p0, int width, int height, int radius, double ic, double jc);
int threshold_range(image& a, image& b, int tlow1, int thigh1, int tlow2, int thigh2);
int threshold_sat(image& a, image& b, image& rgb1, int tlow, int thigh, double sat_max);
int threshold_v(image& a, image& b, image& rgb, int vmin, int tlow, int thigh, double sat_min);

struct HueMaskParams {
	int tlow, thigh;     // greyscale range on grey_gauss
	int hlow, hhigh;     // primary hue range
	int hlow2, hhigh2;   // secondary hue range (-1 = unused)
	int vmin;            // minimum V
	double sat_min;      // minimum saturation
};

int threshold_hue(image& grey_gauss, image& mask, image& rgb, const HueMaskParams& pm);
