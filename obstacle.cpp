#include "obstacle.h"

// Default Constructor places obstacle outside of world map
obstacle::obstacle() {
//	obstacle_centroid_x = -10;
//	*obstacle_centroid_y = -10;
//	diameter = 0;
//	keep_out = 1;
}

// Preferred Constructor places obstacle where it is found on the world map
obstacle::obstacle(int *centroid_x, int *centroid_y, int obstacle_diameter, int obstacle_keep_out) {
	obstacle_centroid_x = centroid_x;
	obstacle_centroid_y = centroid_y;
	diameter = obstacle_diameter;
	keep_out = obstacle_keep_out;
}

// Getters

int obstacle::get_center_x() {
	return *obstacle_centroid_x;
}

int obstacle::get_center_y() {
	return *obstacle_centroid_y;
}

int obstacle::get_diameter() {
	return diameter;
}

int obstacle::get_keep_out() {
	return keep_out;
}

int obstacle::obstacle_radius() {
	return diameter / 2 + keep_out;
}

// Setters

void obstacle::set_center_x(int* new_center_x) {
	obstacle_centroid_x = new_center_x;
}

void obstacle::set_center_y(int* new_center_y) {
	obstacle_centroid_y = new_center_y;
}

void obstacle::set_diameter(int new_diameter) {
	diameter = new_diameter;
}
void obstacle::set_keep_out(int new_keep_out) {
	keep_out = new_keep_out;
}


