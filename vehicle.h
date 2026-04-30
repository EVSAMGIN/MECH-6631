#pragma once
#include "obstacle.h"

// The Vehicle class extends Obstacle, adding another centroid position to act as the rear of a vehicle, and additional methods for center position, orientation, etc.
class vehicle : public obstacle
{
	private:
		int* rear_centroid_x;
		int* rear_centroid_y;
		int vehicle_center_x, vehicle_center_y;
		double orientation;

	public:
		// Class method to update the vehicle center and orientation
		void update_position_orientation();

		// Update component centroid locations, recalculate vehicle center and orientation
		void update_position_orientation(int &front_centroid_x, int &front_centroid_y, int &back_centroid_x, int &back_centroid_y);

		// Default Constructor places vehicle outside of world map
		vehicle();

		// Preferred Constructor places vehicle where it is found on the world map
		vehicle(int *front_centroid_x, int *front_centroid_y, int *back_centroid_x, int *back_centroid_y, int vehicle_diameter, int vehicle_keep_out);

		// Getters

		int get_rear_center_x();
		int get_rear_center_y();
		int get_vehicle_center_x();
		int get_vehicle_center_y();
		double get_orientation();

		// Setters

		void set_rear_center_x(int* rear_centroid_x);
		void set_rear_center_y(int* rear_centroid_y);
};

