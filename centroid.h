#pragma once

#include <cmath>
#include <iostream>
#include <fstream>

#include <cmath>
#include <Windows.h>

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

// Apply two inclusive threshold ranges: [tlow1, thigh1] OR [tlow2, thigh2]
int threshold_range(image& a, image& b, int tlow1, int thigh1, int tlow2, int thigh2);
