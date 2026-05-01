#include <cmath>
#include <iostream>
//#include <numbers>

#include "vehicle.h"

// Class method to update the vehicle center and orientation
void vehicle::update_position_orientation() {
	vehicle_center_x = (*obstacle_centroid_x + *rear_centroid_x) / 2;
	vehicle_center_y = (*obstacle_centroid_y + *rear_centroid_y) / 2;
	std::cout << "Updated vehicle center to: (" << vehicle_center_x << ", " << vehicle_center_y << ")\n";
	orientation = atan2((*obstacle_centroid_y - *rear_centroid_y), (*obstacle_centroid_x - *rear_centroid_x));
	std::cout << "Updated vehicle orientation to: " << orientation;
};

// Update component centroid locations, recalculate vehicle center and orientation
void vehicle::update_position_orientation(double &front_centroid_x, double &front_centroid_y, double &back_centroid_x, double &back_centroid_y) {
	obstacle_centroid_x = &front_centroid_x;
	obstacle_centroid_y = &front_centroid_y;

	rear_centroid_x = &back_centroid_x;
	rear_centroid_y = &back_centroid_y;

	vehicle_center_x = (*obstacle_centroid_x + *rear_centroid_x) / 2;
	vehicle_center_y = (*obstacle_centroid_y + *rear_centroid_y) / 2;

	orientation = atan2((*obstacle_centroid_y - *rear_centroid_y), (*obstacle_centroid_x - *rear_centroid_x));
};

// Default Constructor places vehicle outside of world map
vehicle::vehicle() {
	//*obstacle_centroid_x = -10;
	//*obstacle_centroid_y = -10;
//
	//*rear_centroid_x = -15;
	//*rear_centroid_y = -15;
//
	//diameter = 0;
	//keep_out = 1;
//
	//update_position_orientation();
};

// Preferred Constructor places vehicle where it is found on the world map
vehicle::vehicle(double *front_centroid_x, double *front_centroid_y, double *back_centroid_x, double *back_centroid_y, int vehicle_diameter, int vehicle_keep_out) {
	std::cout << "Initiliazing centroids";
	obstacle_centroid_x = front_centroid_x;
	obstacle_centroid_y = front_centroid_y;

	rear_centroid_x = back_centroid_x;
	rear_centroid_y = back_centroid_y;

	diameter = vehicle_diameter;
	keep_out = vehicle_keep_out;

	std::cout << "Updating position";
	update_position_orientation();
	std::cout << "Updated position";
};

// Getters

int vehicle::get_rear_center_x() {
	return *rear_centroid_x;
}

int vehicle::get_rear_center_y() {
	return *rear_centroid_y;
}

int vehicle::get_vehicle_center_x() {
	return vehicle_center_x;
}

int vehicle::get_vehicle_center_y() {
	return vehicle_center_y;
}

double vehicle::get_orientation() {
	return orientation;
}

// Setters

void vehicle::set_rear_center_x(double* back_centroid_x) {
	rear_centroid_x = back_centroid_x;
}

void vehicle::set_rear_center_y(double* back_centroid_y) {
	rear_centroid_y = back_centroid_y;
}