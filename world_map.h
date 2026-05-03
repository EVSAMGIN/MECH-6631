#pragma once

#include "vision_custom.h"
#include "serial_com.h"

void mapper(TargetPositions& centroid_array, HANDLE& h, int* waypoint_array, int array_size, int width, int height);
