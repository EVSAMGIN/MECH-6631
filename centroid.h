#pragma once

#include <cmath>

#define PI 3.14159265

// Struct for storing centroid data committed to shared memory
struct CentroidData {
	double ic_1, jc_1;
	double ic_2, jc_2;
	double ic_3, jc_3;
	double ic_4, jc_4;
	int frame;
};

// Draw circular pink marker around centroid on an RGB image
void draw_marker(ibyte* p0, int width, int height, int radius, double ic, double jc);
