#include <cmath>

#include "world_map.h"
#include "obstacle.h"
#include "vehicle.h"
#include "vision_custom.h"


#include <windows.h>
#include <tchar.h>
#include <stdio.h>
#include <iostream>

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

// Centroid index values for centroid array
static const int RED_CENTROID_INDEX = 0;
static const int GREEN_CENTROID_INDEX = 1;
static const int BLUE_CENTROID_INDEX = 2;
static const int YELLOW_CENTROID_INDEX = 3;
static const int OBSTACLE_1_CENTROID_INDEX = 4;
static const int OBSTACLE_2_CENTROID_INDEX = 5;
static const int OBSTACLE_3_CENTROID_INDEX = 6;


int new_x, new_y, position_error; //, position_threshold = 60^2, laser_threshold = 60^2;

double waypoint_direction, direction_error, obstacle_direction, obstacle_perpendicular, position_threshold = pow(30, 2), obstacle_threshold = pow(30, 2), laser_threshold = pow(150, 2);
double direction_threshold = 0.3; // radians
double fire_direction_threshold = 0.1; // tighter threshold for firing (~3 degrees)
int turn_flag = 0, vertical_flag = 0, slow_flag = 0, obstacle_1_flag = 0, obstacle_2_flag = 0, obstacle_3_flag = 0, fine_align_flag = 0;

char drive_speed_string[5] = "100\n";
char slow_drive_speed_string[5] = "100\n";
char slower_drive_speed_string[5] = "100\n";


// Centroid pointers
double* control_front_x, * control_front_y, * control_back_x, * control_back_y;
double* enemy_front_x, * enemy_front_y, * enemy_back_x, * enemy_back_y;
double default_centroid_position = -10;
double obstacle_1_distance;

double* target_x, * target_y;
double defense_target_x, defense_target_y;

obstacle obstacle_1(&default_centroid_position, &default_centroid_position, 20, 60);
obstacle obstacle_2(&default_centroid_position, &default_centroid_position, 20, 60);
obstacle obstacle_3(&default_centroid_position, &default_centroid_position, 20, 60);


void initialize_waypoints(int* waypoints, int array_size, int start_x_waypoint, int start_y_waypoint) {

	waypoints[0] = start_x_waypoint;
	waypoints[1] = start_y_waypoint;

	for (int i = 2; i < (array_size - 1); i++) {
		waypoints[i] = -10;
	}
}

void add_new_waypoint(int* waypoints, int array_size, int new_x_waypoint, int new_y_waypoint) {

	for (int i = array_size - 1; i > 1; i--) {
		waypoints[i] = waypoints[i - 2];
	}

	std::cout << "Adding waypoints: (" << new_x_waypoint << ", " << new_y_waypoint << ")\n";

	waypoints[1] = new_y_waypoint;
	waypoints[0] = new_x_waypoint;

	std::cout << "Current Waypoint Array:\n";
	for (int ii = 0; ii < (array_size - 2); ii += 2) {
		std::cout << "x(" << (int)(ii / 2) << "): " << waypoints[ii] << "\n";
		std::cout << "y(" << (int)(ii / 2) << "): " << waypoints[ii + 1] << "\n";
	}
}


void remove_current_waypoint(int* waypoints, int array_size) {

	for (int i = 0; i < (array_size - 2); i++) {
		waypoints[i] = waypoints[i + 2];
	}

	std::cout << "Removed Waypoints - Current Waypoint Array:\n";
	for (int ii = 0; ii < (array_size - 2); ii += 2) {
		std::cout << "x(" << (int)(ii / 2) << "): " << waypoints[ii] << "\n";
		std::cout << "y(" << (int)(ii / 2) << "): " << waypoints[ii + 1] << "\n";
	}

	waypoints[array_size - 2] = -10;
	waypoints[array_size - 1] = -10;
}


void mapper(TargetPositions& centroid_array, HANDLE& h, int* waypoint_array, int array_size, int width, int height) {

	// Accept centroid array
	// Create vehicle and obstacle objects
	// Develop waypoint buffer
	// Find path through waypoint buffer, update as needed
	std::cout << "Centroid Array";// << centroid_array;

	// Initial setup, only run once:

	//// Our robot

	control_front_x = centroid_array.ic + RED_CENTROID_INDEX;
	control_front_y = centroid_array.jc + RED_CENTROID_INDEX;
	control_back_x = centroid_array.ic + GREEN_CENTROID_INDEX;
	control_back_y = centroid_array.jc + GREEN_CENTROID_INDEX;

	vehicle our_vehicle(control_front_x, control_front_y, control_back_x, control_back_y, 9, 1);

	std::cout << "Created friendly robot vehicle object\n\n";

	//// Enemy robot

	enemy_front_x = centroid_array.ic + BLUE_CENTROID_INDEX;
	enemy_front_y = centroid_array.jc + BLUE_CENTROID_INDEX;
	enemy_back_x = centroid_array.ic + YELLOW_CENTROID_INDEX;
	enemy_back_y = centroid_array.jc + YELLOW_CENTROID_INDEX;

	vehicle enemy_vehicle(enemy_front_x, enemy_front_y, enemy_back_x, enemy_back_y, 9, 1);

	std::cout << "Created enemy robot vehicle object\n\n";

	//// Obstacle Initialization
	if ((centroid_array.ic[OBSTACLE_1_CENTROID_INDEX] > 0) && (centroid_array.jc[OBSTACLE_1_CENTROID_INDEX] > 0)) {
		obstacle_1.set_center_x(centroid_array.ic + OBSTACLE_1_CENTROID_INDEX);
		obstacle_1.set_center_y(centroid_array.jc + OBSTACLE_1_CENTROID_INDEX);//, centroid_array + OBSTACLE_1_CENTROID_INDEX + 1, 4, 1);

		if ((centroid_array.ic[OBSTACLE_2_CENTROID_INDEX] > 0) && (centroid_array.jc[OBSTACLE_2_CENTROID_INDEX] > 0)) {
			obstacle_2.set_center_x(centroid_array.ic + OBSTACLE_2_CENTROID_INDEX);
			obstacle_2.set_center_y(centroid_array.jc + OBSTACLE_2_CENTROID_INDEX);

			if ((centroid_array.ic[OBSTACLE_3_CENTROID_INDEX] > 0) && (centroid_array.jc[OBSTACLE_3_CENTROID_INDEX] > 0)) {
				obstacle_3.set_center_x(centroid_array.ic + OBSTACLE_3_CENTROID_INDEX);
				obstacle_3.set_center_y(centroid_array.jc + OBSTACLE_3_CENTROID_INDEX);
			}
		}
		else {
			if ((centroid_array.ic[OBSTACLE_3_CENTROID_INDEX] > 0) && (centroid_array.jc[OBSTACLE_3_CENTROID_INDEX] > 0)) {
				obstacle_3.set_center_x(centroid_array.ic + OBSTACLE_3_CENTROID_INDEX);
				obstacle_3.set_center_y(centroid_array.jc + OBSTACLE_3_CENTROID_INDEX);
			}
		}
	}
	else {
		if ((centroid_array.ic[OBSTACLE_2_CENTROID_INDEX] > 0) && (centroid_array.jc[OBSTACLE_2_CENTROID_INDEX] > 0)) {
			obstacle_2.set_center_x(centroid_array.ic + OBSTACLE_2_CENTROID_INDEX);
			obstacle_2.set_center_y(centroid_array.jc + OBSTACLE_2_CENTROID_INDEX);

			if ((centroid_array.ic[OBSTACLE_3_CENTROID_INDEX] > 0) && (centroid_array.jc[OBSTACLE_3_CENTROID_INDEX] > 0)) {
				obstacle_3.set_center_x(centroid_array.ic + OBSTACLE_3_CENTROID_INDEX);
				obstacle_3.set_center_y(centroid_array.jc + OBSTACLE_3_CENTROID_INDEX);
			}
		}
		else {
			if ((centroid_array.ic[OBSTACLE_3_CENTROID_INDEX] > 0) && (centroid_array.jc[OBSTACLE_3_CENTROID_INDEX] > 0)) {
				obstacle_3.set_center_x(centroid_array.ic + OBSTACLE_3_CENTROID_INDEX);
				obstacle_3.set_center_y(centroid_array.jc + OBSTACLE_3_CENTROID_INDEX);
			}
		}
	}

	// Load target coordinates into buffer
	initialize_waypoints(waypoint_array, array_size, -10, -10);



	// Define target pointer
	//target_x = centroid_array.ic + OBSTACLE_2_CENTROID_INDEX;
	//target_y = centroid_array.jc + OBSTACLE_2_CENTROID_INDEX;

	// Target the sandwich box at the front of enemy robot
	target_x = enemy_front_x;
	target_y = enemy_front_y;

	waypoint_direction = atan2((waypoint_array[1] - our_vehicle.get_vehicle_center_y()), (waypoint_array[0]) - our_vehicle.get_vehicle_center_x());
	//waypoint_direction = atan2((centroid_array[BLUE_CENTROID_INDEX + 1] - our_vehicle.get_vehicle_center_y()), (centroid_array[BLUE_CENTROID_INDEX] - our_vehicle.get_vehicle_center_x()));

	serial_send(drive_speed_string, 4, h);

	while (true) {
		// Update vehicles information

		our_vehicle.update_position_orientation();
		enemy_vehicle.update_position_orientation();

		// UPDATE ORIENTATION AND POSITION ERRORS
		if (waypoint_array[0] > 0) {
			// If there are active waypoints, navigate to them
			waypoint_direction = atan2((waypoint_array[1] - our_vehicle.get_vehicle_center_y()), (waypoint_array[0]) - our_vehicle.get_vehicle_center_x());
			position_error = ((waypoint_array[0] - our_vehicle.get_vehicle_center_x()) * (waypoint_array[0] - our_vehicle.get_vehicle_center_x()) + (waypoint_array[1] - our_vehicle.get_vehicle_center_y()) * (waypoint_array[1] - our_vehicle.get_vehicle_center_y()));

			position_threshold = obstacle_threshold;
		}
		else {
			// Otherwise, track the moving target
			waypoint_direction = atan2((*target_y - our_vehicle.get_vehicle_center_y()), (*target_x) - our_vehicle.get_vehicle_center_x());
			position_error = (((*target_x - our_vehicle.get_vehicle_center_x()) * (*target_x - our_vehicle.get_vehicle_center_x()) + (*target_y - our_vehicle.get_vehicle_center_y()) * (*target_y - our_vehicle.get_vehicle_center_y())));

			position_threshold = laser_threshold;
		}

		// Check direct path for obstacles
		// // Using formula from: https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line


		if ((obstacle_1.get_center_x() >= 0) && (obstacle_1_flag == 0)) {
			//std::cout << "CHECKING FOR OBSTACLE 1.";

			// IF WAYPOINTS ALREADY EXIST, USE NEXT WAYPOINT POSITION
			if (waypoint_array[0] >= 0) {
				double dist_calc_1 = abs((waypoint_array[1] - our_vehicle.get_vehicle_center_y()) * obstacle_1.get_center_x() - (waypoint_array[0] - our_vehicle.get_vehicle_center_x()) * obstacle_1.get_center_y() + waypoint_array[0] * our_vehicle.get_vehicle_center_y() - waypoint_array[1] * our_vehicle.get_vehicle_center_x());
				double dist_calc_2 = sqrt((pow(waypoint_array[1] - our_vehicle.get_vehicle_center_y(), 2) + pow(waypoint_array[0] - our_vehicle.get_vehicle_center_x(), 2)));// *obstacle_1.obstacle_radius();
				double distance = abs(dist_calc_1) / abs(dist_calc_2);

				//std::cout << "WAY_DIST_CALC_1: " << dist_calc_1 << "\n";
				//std::cout << "WAY_DIST_CALC_2: " << dist_calc_2 << "\n";
				if (distance < obstacle_1.obstacle_radius()) {
					// Path intersects with keep-out zone, need to find new waypoint
					obstacle_direction = atan2((obstacle_1.get_center_y() - our_vehicle.get_vehicle_center_y()), (obstacle_1.get_center_x()) - our_vehicle.get_vehicle_center_x());
					if (waypoint_direction > obstacle_direction) {
						std::cout << "Obstacle 1 waypoint greater than\n";

						std::cout << "Current Vehicle Position: " << our_vehicle.get_center_x() << ", " << our_vehicle.get_center_y() << ")\n";

						obstacle_perpendicular = obstacle_direction + 3.14159 / 2;
						new_x = (int)(obstacle_1.obstacle_radius() * cos(obstacle_perpendicular)) + obstacle_1.get_center_x();
						new_y = (int)(obstacle_1.obstacle_radius() * sin(obstacle_perpendicular)) + obstacle_1.get_center_y();

						add_new_waypoint(waypoint_array, array_size, new_x, new_y);

						obstacle_1_flag = 1;

						waypoint_direction = atan2((waypoint_array[1] - our_vehicle.get_vehicle_center_y()), (waypoint_array[0]) - our_vehicle.get_vehicle_center_x());
						position_error = ((waypoint_array[0] - our_vehicle.get_vehicle_center_x()) * (waypoint_array[0] - our_vehicle.get_vehicle_center_x()) + (waypoint_array[1] - our_vehicle.get_vehicle_center_y()) * (waypoint_array[1] - our_vehicle.get_vehicle_center_y()));

						position_threshold = obstacle_threshold;
					}
					else {
						std::cout << "Obstacle 1 waypoint less than\n";

						std::cout << "Current Vehicle Position: " << our_vehicle.get_center_x() << ", " << our_vehicle.get_center_y() << ")\n";

						obstacle_perpendicular = obstacle_direction - 3.14159 / 2;
						new_x = (int)(obstacle_1.obstacle_radius() * cos(obstacle_perpendicular)) + obstacle_1.get_center_x();
						new_y = (int)(obstacle_1.obstacle_radius() * sin(obstacle_perpendicular)) + obstacle_1.get_center_y();

						add_new_waypoint(waypoint_array, array_size, new_x, new_y);

						obstacle_1_flag = 1;

						waypoint_direction = atan2((waypoint_array[1] - our_vehicle.get_vehicle_center_y()), (waypoint_array[0]) - our_vehicle.get_vehicle_center_x());
						position_error = ((waypoint_array[0] - our_vehicle.get_vehicle_center_x()) * (waypoint_array[0] - our_vehicle.get_vehicle_center_x()) + (waypoint_array[1] - our_vehicle.get_vehicle_center_y()) * (waypoint_array[1] - our_vehicle.get_vehicle_center_y()));

						position_threshold = obstacle_threshold;
					}
				}
			}
			// IF THERE ARE NO EXISTING WAYPOINTS, USE TARGET LOCATION
			else {
				double dist_calc_1 = abs((*target_y - our_vehicle.get_vehicle_center_y()) * obstacle_1.get_center_x() - (*target_x - our_vehicle.get_vehicle_center_x()) * obstacle_1.get_center_y() + *target_x * our_vehicle.get_vehicle_center_y() - *target_y * our_vehicle.get_vehicle_center_x());
				double dist_calc_2 = sqrt((pow(*target_y - our_vehicle.get_vehicle_center_y(), 2) + pow(*target_x - our_vehicle.get_vehicle_center_x(), 2)));// *obstacle_1.obstacle_radius();
				double distance = abs(dist_calc_1) / abs(dist_calc_2);

				//std::cout << "TAR_DIST_CALC_1: " << dist_calc_1 << "\n";
				//std::cout << "TAR_DIST_CALC_2: " << dist_calc_2 << "\n";

				if (distance < obstacle_1.obstacle_radius()) {
					// Path intersects with keep-out zone, need to find new waypoint
					obstacle_direction = atan2((obstacle_1.get_center_y() - our_vehicle.get_vehicle_center_y()), (obstacle_1.get_center_x()) - our_vehicle.get_vehicle_center_x());
					if (waypoint_direction > obstacle_direction) {
						std::cout << "Obstacle 1 waypoint greater than\n";

						std::cout << "Current Vehicle Position: " << our_vehicle.get_center_x() << ", " << our_vehicle.get_center_y() << ")\n";

						obstacle_perpendicular = obstacle_direction + 3.14159 / 2;
						new_x = (int)(obstacle_1.obstacle_radius() * cos(obstacle_perpendicular)) + obstacle_1.get_center_x();
						new_y = (int)(obstacle_1.obstacle_radius() * sin(obstacle_perpendicular)) + obstacle_1.get_center_y();

						add_new_waypoint(waypoint_array, array_size, new_x, new_y);

						obstacle_1_flag = 1;

						waypoint_direction = atan2((waypoint_array[1] - our_vehicle.get_vehicle_center_y()), (waypoint_array[0]) - our_vehicle.get_vehicle_center_x());
						position_error = ((waypoint_array[0] - our_vehicle.get_vehicle_center_x()) * (waypoint_array[0] - our_vehicle.get_vehicle_center_x()) + (waypoint_array[1] - our_vehicle.get_vehicle_center_y()) * (waypoint_array[1] - our_vehicle.get_vehicle_center_y()));

						position_threshold = obstacle_threshold;
					}
					else {
						std::cout << "Obstacle 1 waypoint less than\n";

						std::cout << "Current Vehicle Position: " << our_vehicle.get_center_x() << ", " << our_vehicle.get_center_y() << ")\n";

						obstacle_perpendicular = obstacle_direction - 3.14159 / 2;
						new_x = (int)(obstacle_1.obstacle_radius() * cos(obstacle_perpendicular)) + obstacle_1.get_center_x();
						new_y = (int)(obstacle_1.obstacle_radius() * sin(obstacle_perpendicular)) + obstacle_1.get_center_y();

						add_new_waypoint(waypoint_array, array_size, new_x, new_y);

						obstacle_1_flag = 1;

						waypoint_direction = atan2((waypoint_array[1] - our_vehicle.get_vehicle_center_y()), (waypoint_array[0]) - our_vehicle.get_vehicle_center_x());
						position_error = ((waypoint_array[0] - our_vehicle.get_vehicle_center_x()) * (waypoint_array[0] - our_vehicle.get_vehicle_center_x()) + (waypoint_array[1] - our_vehicle.get_vehicle_center_y()) * (waypoint_array[1] - our_vehicle.get_vehicle_center_y()));

						position_threshold = obstacle_threshold;
					}
				}
			}
		}

		if ((obstacle_2.get_center_x() >= 0) && (obstacle_2_flag == 0)) {
			//std::cout << "CHECKING FOR OBSTACLE 2.";

			// IF WAYPOINTS ALREADY EXIST, USE NEXT WAYPOINT POSITION
			if (waypoint_array[0] >= 0) {
				double dist_calc_1 = abs((waypoint_array[1] - our_vehicle.get_vehicle_center_y()) * obstacle_2.get_center_x() - (waypoint_array[0] - our_vehicle.get_vehicle_center_x()) * obstacle_2.get_center_y() + waypoint_array[0] * our_vehicle.get_vehicle_center_y() - waypoint_array[1] * our_vehicle.get_vehicle_center_x());
				double dist_calc_2 = sqrt((pow(waypoint_array[1] - our_vehicle.get_vehicle_center_y(), 2) + pow(waypoint_array[0] - our_vehicle.get_vehicle_center_x(), 2)));// *obstacle_2.obstacle_radius();
				double distance = abs(dist_calc_1) / abs(dist_calc_2);

				//std::cout << "WAY_DIST_CALC_1: " << dist_calc_1 << "\n";
				//std::cout << "WAY_DIST_CALC_2: " << dist_calc_2 << "\n";
				if (distance < obstacle_2.obstacle_radius()) {
					// Path intersects with keep-out zone, need to find new waypoint
					obstacle_direction = atan2((obstacle_2.get_center_y() - our_vehicle.get_vehicle_center_y()), (obstacle_2.get_center_x()) - our_vehicle.get_vehicle_center_x());
					if (waypoint_direction > obstacle_direction) {
						std::cout << "Obstacle 2 waypoint greater than\n";

						std::cout << "Current Vehicle Position: " << our_vehicle.get_center_x() << ", " << our_vehicle.get_center_y() << ")\n";

						obstacle_perpendicular = obstacle_direction + 3.14159 / 2;
						new_x = (int)(obstacle_2.obstacle_radius() * cos(obstacle_perpendicular)) + obstacle_2.get_center_x();
						new_y = (int)(obstacle_2.obstacle_radius() * sin(obstacle_perpendicular)) + obstacle_2.get_center_y();

						add_new_waypoint(waypoint_array, array_size, new_x, new_y);

						obstacle_2_flag = 1;

						waypoint_direction = atan2((waypoint_array[1] - our_vehicle.get_vehicle_center_y()), (waypoint_array[0]) - our_vehicle.get_vehicle_center_x());
						position_error = ((waypoint_array[0] - our_vehicle.get_vehicle_center_x()) * (waypoint_array[0] - our_vehicle.get_vehicle_center_x()) + (waypoint_array[1] - our_vehicle.get_vehicle_center_y()) * (waypoint_array[1] - our_vehicle.get_vehicle_center_y()));

						position_threshold = obstacle_threshold;
					}
					else {
						std::cout << "Obstacle 2 waypoint less than\n";

						std::cout << "Current Vehicle Position: " << our_vehicle.get_center_x() << ", " << our_vehicle.get_center_y() << ")\n";

						obstacle_perpendicular = obstacle_direction - 3.14159 / 2;
						new_x = (int)(obstacle_2.obstacle_radius() * cos(obstacle_perpendicular)) + obstacle_2.get_center_x();
						new_y = (int)(obstacle_2.obstacle_radius() * sin(obstacle_perpendicular)) + obstacle_2.get_center_y();

						add_new_waypoint(waypoint_array, array_size, new_x, new_y);

						obstacle_2_flag = 1;

						waypoint_direction = atan2((waypoint_array[1] - our_vehicle.get_vehicle_center_y()), (waypoint_array[0]) - our_vehicle.get_vehicle_center_x());
						position_error = ((waypoint_array[0] - our_vehicle.get_vehicle_center_x()) * (waypoint_array[0] - our_vehicle.get_vehicle_center_x()) + (waypoint_array[1] - our_vehicle.get_vehicle_center_y()) * (waypoint_array[1] - our_vehicle.get_vehicle_center_y()));

						position_threshold = obstacle_threshold;
					}
				}
			}
			// IF THERE ARE NO EXISTING WAYPOINTS, USE TARGET LOCATION
			else {
				double dist_calc_1 = abs((*target_y - our_vehicle.get_vehicle_center_y()) * obstacle_2.get_center_x() - (*target_x - our_vehicle.get_vehicle_center_x()) * obstacle_2.get_center_y() + *target_x * our_vehicle.get_vehicle_center_y() - *target_y * our_vehicle.get_vehicle_center_x());
				double dist_calc_2 = sqrt((pow(*target_y - our_vehicle.get_vehicle_center_y(), 2) + pow(*target_x - our_vehicle.get_vehicle_center_x(), 2)));// *obstacle_1.obstacle_radius();
				double distance = abs(dist_calc_1) / abs(dist_calc_2);

				//std::cout << "TAR_DIST_CALC_1: " << dist_calc_1 << "\n";
				//std::cout << "TAR_DIST_CALC_2: " << dist_calc_2 << "\n";

				if (distance < obstacle_1.obstacle_radius()) {
					// Path intersects with keep-out zone, need to find new waypoint
					obstacle_direction = atan2((obstacle_2.get_center_y() - our_vehicle.get_vehicle_center_y()), (obstacle_2.get_center_x()) - our_vehicle.get_vehicle_center_x());
					if (waypoint_direction > obstacle_direction) {
						std::cout << "Obstacle 2 waypoint greater than\n";

						std::cout << "Current Vehicle Position: " << our_vehicle.get_center_x() << ", " << our_vehicle.get_center_y() << ")\n";

						obstacle_perpendicular = obstacle_direction + 3.14159 / 2;
						new_x = (int)(obstacle_2.obstacle_radius() * cos(obstacle_perpendicular)) + obstacle_2.get_center_x();
						new_y = (int)(obstacle_2.obstacle_radius() * sin(obstacle_perpendicular)) + obstacle_2.get_center_y();

						add_new_waypoint(waypoint_array, array_size, new_x, new_y);

						obstacle_2_flag = 1;

						waypoint_direction = atan2((waypoint_array[1] - our_vehicle.get_vehicle_center_y()), (waypoint_array[0]) - our_vehicle.get_vehicle_center_x());
						position_error = ((waypoint_array[0] - our_vehicle.get_vehicle_center_x()) * (waypoint_array[0] - our_vehicle.get_vehicle_center_x()) + (waypoint_array[1] - our_vehicle.get_vehicle_center_y()) * (waypoint_array[1] - our_vehicle.get_vehicle_center_y()));

						position_threshold = obstacle_threshold;
					}
					else {
						std::cout << "Obstacle 2 waypoint less than\n";

						std::cout << "Current Vehicle Position: " << our_vehicle.get_center_x() << ", " << our_vehicle.get_center_y() << ")\n";

						obstacle_perpendicular = obstacle_direction - 3.14159 / 2;
						new_x = (int)(obstacle_2.obstacle_radius() * cos(obstacle_perpendicular)) + obstacle_2.get_center_x();
						new_y = (int)(obstacle_2.obstacle_radius() * sin(obstacle_perpendicular)) + obstacle_2.get_center_y();

						add_new_waypoint(waypoint_array, array_size, new_x, new_y);

						obstacle_2_flag = 1;

						waypoint_direction = atan2((waypoint_array[1] - our_vehicle.get_vehicle_center_y()), (waypoint_array[0]) - our_vehicle.get_vehicle_center_x());
						position_error = ((waypoint_array[0] - our_vehicle.get_vehicle_center_x()) * (waypoint_array[0] - our_vehicle.get_vehicle_center_x()) + (waypoint_array[1] - our_vehicle.get_vehicle_center_y()) * (waypoint_array[1] - our_vehicle.get_vehicle_center_y()));

						position_threshold = obstacle_threshold;
					}
				}
			}
		}

		if ((obstacle_3.get_center_x() >= 0) && (obstacle_3_flag == 0)) {
			//std::cout << "CHECKING FOR OBSTACLE 3.";

			// IF WAYPOINTS ALREADY EXIST, USE NEXT WAYPOINT POSITION
			if (waypoint_array[0] >= 0) {
				double dist_calc_1 = abs((waypoint_array[1] - our_vehicle.get_vehicle_center_y()) * obstacle_3.get_center_x() - (waypoint_array[0] - our_vehicle.get_vehicle_center_x()) * obstacle_3.get_center_y() + waypoint_array[0] * our_vehicle.get_vehicle_center_y() - waypoint_array[1] * our_vehicle.get_vehicle_center_x());
				double dist_calc_2 = sqrt((pow(waypoint_array[1] - our_vehicle.get_vehicle_center_y(), 2) + pow(waypoint_array[0] - our_vehicle.get_vehicle_center_x(), 2)));// *obstacle_3.obstacle_radius();
				double distance = abs(dist_calc_1) / abs(dist_calc_2);

				//std::cout << "WAY_DIST_CALC_1: " << dist_calc_1 << "\n";
				//std::cout << "WAY_DIST_CALC_2: " << dist_calc_2 << "\n";
				if (distance < obstacle_3.obstacle_radius()) {
					// Path intersects with keep-out zone, need to find new waypoint
					obstacle_direction = atan2((obstacle_3.get_center_y() - our_vehicle.get_vehicle_center_y()), (obstacle_3.get_center_x()) - our_vehicle.get_vehicle_center_x());
					if (waypoint_direction > obstacle_direction) {
						std::cout << "Obstacle 3 waypoint greater than\n";

						std::cout << "Current Vehicle Position: " << our_vehicle.get_center_x() << ", " << our_vehicle.get_center_y() << ")\n";

						obstacle_perpendicular = obstacle_direction + 3.14159 / 2;
						new_x = (int)(obstacle_3.obstacle_radius() * cos(obstacle_perpendicular)) + obstacle_3.get_center_x();
						new_y = (int)(obstacle_3.obstacle_radius() * sin(obstacle_perpendicular)) + obstacle_3.get_center_y();

						add_new_waypoint(waypoint_array, array_size, new_x, new_y);

						obstacle_3_flag = 1;

						waypoint_direction = atan2((waypoint_array[1] - our_vehicle.get_vehicle_center_y()), (waypoint_array[0]) - our_vehicle.get_vehicle_center_x());
						position_error = ((waypoint_array[0] - our_vehicle.get_vehicle_center_x()) * (waypoint_array[0] - our_vehicle.get_vehicle_center_x()) + (waypoint_array[1] - our_vehicle.get_vehicle_center_y()) * (waypoint_array[1] - our_vehicle.get_vehicle_center_y()));

						position_threshold = obstacle_threshold;
					}
					else {
						std::cout << "Obstacle 3 waypoint less than\n";

						std::cout << "Current Vehicle Position: " << our_vehicle.get_center_x() << ", " << our_vehicle.get_center_y() << ")\n";

						obstacle_perpendicular = obstacle_direction - 3.14159 / 2;
						new_x = (int)(obstacle_3.obstacle_radius() * cos(obstacle_perpendicular)) + obstacle_3.get_center_x();
						new_y = (int)(obstacle_3.obstacle_radius() * sin(obstacle_perpendicular)) + obstacle_3.get_center_y();

						add_new_waypoint(waypoint_array, array_size, new_x, new_y);

						obstacle_3_flag = 1;

						waypoint_direction = atan2((waypoint_array[1] - our_vehicle.get_vehicle_center_y()), (waypoint_array[0]) - our_vehicle.get_vehicle_center_x());
						position_error = ((waypoint_array[0] - our_vehicle.get_vehicle_center_x()) * (waypoint_array[0] - our_vehicle.get_vehicle_center_x()) + (waypoint_array[1] - our_vehicle.get_vehicle_center_y()) * (waypoint_array[1] - our_vehicle.get_vehicle_center_y()));

						position_threshold = obstacle_threshold;
					}
				}
			}

			// IF THERE ARE NO EXISTING WAYPOINTS, USE TARGET LOCATION
			else {
				double dist_calc_1 = abs((*target_y - our_vehicle.get_vehicle_center_y()) * obstacle_1.get_center_x() - (*target_x - our_vehicle.get_vehicle_center_x()) * obstacle_1.get_center_y() + *target_x * our_vehicle.get_vehicle_center_y() - *target_y * our_vehicle.get_vehicle_center_x());
				double dist_calc_2 = sqrt((pow(*target_y - our_vehicle.get_vehicle_center_y(), 2) + pow(*target_x - our_vehicle.get_vehicle_center_x(), 2)));// *obstacle_1.obstacle_radius();
				double distance = abs(dist_calc_1) / abs(dist_calc_2);

				//std::cout << "TAR_DIST_CALC_1: " << dist_calc_1 << "\n";
				//std::cout << "TAR_DIST_CALC_2: " << dist_calc_2 << "\n";

				if (distance < obstacle_1.obstacle_radius()) {
					// Path intersects with keep-out zone, need to find new waypoint
					obstacle_direction = atan2((obstacle_1.get_center_y() - our_vehicle.get_vehicle_center_y()), (obstacle_1.get_center_x()) - our_vehicle.get_vehicle_center_x());
					if (waypoint_direction > obstacle_direction) {
						std::cout << "Obstacle 1 waypoint greater than\n";

						std::cout << "Current Vehicle Position: " << our_vehicle.get_center_x() << ", " << our_vehicle.get_center_y() << ")\n";

						obstacle_perpendicular = obstacle_direction + 3.14159 / 2;
						new_x = (int)(obstacle_1.obstacle_radius() * cos(obstacle_perpendicular)) + obstacle_1.get_center_x();
						new_y = (int)(obstacle_1.obstacle_radius() * sin(obstacle_perpendicular)) + obstacle_1.get_center_y();

						add_new_waypoint(waypoint_array, array_size, new_x, new_y);

						obstacle_1_flag = 1;

						waypoint_direction = atan2((waypoint_array[1] - our_vehicle.get_vehicle_center_y()), (waypoint_array[0]) - our_vehicle.get_vehicle_center_x());
						position_error = ((waypoint_array[0] - our_vehicle.get_vehicle_center_x()) * (waypoint_array[0] - our_vehicle.get_vehicle_center_x()) + (waypoint_array[1] - our_vehicle.get_vehicle_center_y()) * (waypoint_array[1] - our_vehicle.get_vehicle_center_y()));

						position_threshold = obstacle_threshold;
					}
					else {
						std::cout << "Obstacle 1 waypoint less than\n";

						std::cout << "Current Vehicle Position: " << our_vehicle.get_center_x() << ", " << our_vehicle.get_center_y() << ")\n";

						obstacle_perpendicular = obstacle_direction - 3.14159 / 2;
						new_x = (int)(obstacle_1.obstacle_radius() * cos(obstacle_perpendicular)) + obstacle_1.get_center_x();
						new_y = (int)(obstacle_1.obstacle_radius() * sin(obstacle_perpendicular)) + obstacle_1.get_center_y();

						add_new_waypoint(waypoint_array, array_size, new_x, new_y);

						obstacle_1_flag = 1;

						waypoint_direction = atan2((waypoint_array[1] - our_vehicle.get_vehicle_center_y()), (waypoint_array[0]) - our_vehicle.get_vehicle_center_x());
						position_error = ((waypoint_array[0] - our_vehicle.get_vehicle_center_x()) * (waypoint_array[0] - our_vehicle.get_vehicle_center_x()) + (waypoint_array[1] - our_vehicle.get_vehicle_center_y()) * (waypoint_array[1] - our_vehicle.get_vehicle_center_y()));

						position_threshold = obstacle_threshold;
					}
				}
			}
		}

		// Debug Prints
		if (KEY(VK_RETURN)) {
			std::cout << "Current Direction Error: " << direction_error << "\n\n";
			std::cout << "Current Position Error: " << position_error << "\n\n";
			std::cout << "Current Waypoints:\n";


			//std::cout << "Array length:" << array_size << "\n\n";

			for (int ii = 0; ii < (array_size - 2); ii += 2) {
				std::cout << "x(" << (int)(ii / 2) << "): " << waypoint_array[ii] << "\n";
				std::cout << "y(" << (int)(ii / 2) << "): " << waypoint_array[ii + 1] << "\n";
			}

			std::cout << "\n";

		}

		// Calculate position and orientation errors
		direction_error = waypoint_direction - our_vehicle.get_orientation();

		// Use tighter threshold when fine-aligning for fire
		double active_direction_threshold = fine_align_flag ? fire_direction_threshold : direction_threshold;

		// Turn towards current waypoint
		if ((abs(direction_error) > active_direction_threshold)) {
			if (direction_error > 0 && turn_flag < 1) {

				serial_send("9\n", 2, h);
				//Sleep(50);

				turn_flag = 1;
				vertical_flag = 0;
				std::cout << "GO LEFT BROTHER.\n";
			}
			else if (direction_error < 0 && turn_flag > -1) {
				serial_send("10\n", 3, h);
				//Sleep(50);

				turn_flag = -1;
				vertical_flag = 0;
				std::cout << "GO RIGHT BROTHER.\n";
			}
			else if ((abs(direction_error) < 2 * direction_threshold) && (turn_flag != 0) && (slow_flag < 2)) {
				slow_flag = 2;

				//slow_down_drive_speed_string = "100\n";

				std::cout << "SLOW DOWN MORE BROTHER: " << slower_drive_speed_string << "\n";

				serial_send(slower_drive_speed_string, 4, h);
			}
			else if ((abs(direction_error) < 3 * direction_threshold) && (turn_flag != 0) && (slow_flag < 1)) {
				slow_flag = 1;

				//slow_down_drive_speed_string = "160\n";

				std::cout << "SLOW DOWN BROTHER: " << slow_drive_speed_string << "\n";

				serial_send(slow_drive_speed_string, 4, h);
			}

		}

		// DIRECTION ERROR MINIMIZED, MOVE FORWARD
		else {

			// IF ROBOT WASN'T MOVING FORWARD, MOVE FORWARD
			if ((abs(position_error) > position_threshold) && vertical_flag < 1) {


				serial_send("2\n", 2, h);
				turn_flag = 0;
				vertical_flag = 1;
				fine_align_flag = 0; // Reset fine-align when moving forward

				// SET BACK TO ORIGINAL SPEED IF MOVING SLOW
				if (slow_flag != 0) {
					//drive_speed_string = "200\n";

					serial_send(drive_speed_string, 4, h);

					std::cout << "SPEED UP BROTHER:" << drive_speed_string << "\n";
					slow_flag = 0;


				}

//				std::cout << "Position Threshold: " << position_threshold << "\n";
//				std::cout << "Position Error: " << position_error << "\n";
//
				std::cout << "FULL STEAM AHEAD!\n";

			}

			// IF NEAR THE WAYPOINT
			else if ((abs(position_error) <= position_threshold)) {

				std::cout << "APPROACHING TARGET MENACINGLY.\n";

				vertical_flag = 0;

				obstacle_1_distance = sqrt(pow((our_vehicle.get_center_x() - obstacle_1.get_center_x()), 2) + pow(our_vehicle.get_center_y() - obstacle_1.get_center_y(), 2));

				std::cout << "Distance to Obstacle 1: " << obstacle_1_distance << "\n";
				std::cout << "Obstacle 1 Flag: " << obstacle_1_flag << "\n";

				if (obstacle_1_distance > 2 * obstacle_1.obstacle_radius()) {
					obstacle_1_flag = 0;
				}

				// CLEAR WAYPOINT
				if (waypoint_array[0] > 0) {
					remove_current_waypoint(waypoint_array, array_size);

					std::cout << "TIME EXTENDED!\n";
				}

				// FIRE MAH LAZARRR SINCE I'M NEAR THE TARGET
				else {
					// Fine-align check: must be within tight direction threshold before firing
					if (abs(direction_error) > fire_direction_threshold) {
						fine_align_flag = 1;  // Tighten turn threshold next iteration
						turn_flag = 0;        // Allow turn logic to re-engage
						vertical_flag = 0;
						std::cout << "FINE ALIGNING BEFORE FIRE. Direction error: " << direction_error << "\n";
					}
					else {
						fine_align_flag = 0;
						serial_send("13\n", 3, h);
						std::cout << "TARGET ELIMINATED. GET TO EXTRACTION 47.\n\n";
						Sleep(100);
						serial_send("0\n", 2, h);
						break;
					}
				}
			}
		}
	}
}

void defense_mapper(TargetPositions& centroid_array, HANDLE& h, int* waypoint_array, int array_size, int width, int height) {

	// Accept centroid array
	// Create vehicle and obstacle objects
	// Develop waypoint buffer
	// Find path through waypoint buffer, update as needed
	std::cout << "Centroid Array";// << centroid_array;

	// Initial setup, only run once:

	//// Our robot

	control_front_x = centroid_array.ic + RED_CENTROID_INDEX;
	control_front_y = centroid_array.jc + RED_CENTROID_INDEX;
	control_back_x = centroid_array.ic + GREEN_CENTROID_INDEX;
	control_back_y = centroid_array.jc + GREEN_CENTROID_INDEX;

	vehicle our_vehicle(control_front_x, control_front_y, control_back_x, control_back_y, 9, 1);

	std::cout << "Created friendly robot vehicle object\n\n";

	//// Enemy robot

	enemy_front_x = centroid_array.ic + BLUE_CENTROID_INDEX;
	enemy_front_y = centroid_array.jc + BLUE_CENTROID_INDEX;
	enemy_back_x = centroid_array.ic + YELLOW_CENTROID_INDEX;
	enemy_back_y = centroid_array.jc + YELLOW_CENTROID_INDEX;

	vehicle enemy_vehicle(enemy_front_x, enemy_front_y, enemy_back_x, enemy_back_y, 9, 1);

	std::cout << "Created enemy robot vehicle object\n\n";

	//// Obstacle Initialization
	if ((centroid_array.ic[OBSTACLE_1_CENTROID_INDEX] > 0) && (centroid_array.jc[OBSTACLE_1_CENTROID_INDEX] > 0)) {
		obstacle_1.set_center_x(centroid_array.ic + OBSTACLE_1_CENTROID_INDEX);
		obstacle_1.set_center_y(centroid_array.jc + OBSTACLE_1_CENTROID_INDEX);//, centroid_array + OBSTACLE_1_CENTROID_INDEX + 1, 4, 1);

		if ((centroid_array.ic[OBSTACLE_2_CENTROID_INDEX] > 0) && (centroid_array.jc[OBSTACLE_2_CENTROID_INDEX] > 0)) {
			obstacle_2.set_center_x(centroid_array.ic + OBSTACLE_2_CENTROID_INDEX);
			obstacle_2.set_center_y(centroid_array.jc + OBSTACLE_2_CENTROID_INDEX);

			if ((centroid_array.ic[OBSTACLE_3_CENTROID_INDEX] > 0) && (centroid_array.jc[OBSTACLE_3_CENTROID_INDEX] > 0)) {
				obstacle_3.set_center_x(centroid_array.ic + OBSTACLE_3_CENTROID_INDEX);
				obstacle_3.set_center_y(centroid_array.jc + OBSTACLE_3_CENTROID_INDEX);
			}
		}
		else {
			if ((centroid_array.ic[OBSTACLE_3_CENTROID_INDEX] > 0) && (centroid_array.jc[OBSTACLE_3_CENTROID_INDEX] > 0)) {
				obstacle_3.set_center_x(centroid_array.ic + OBSTACLE_3_CENTROID_INDEX);
				obstacle_3.set_center_y(centroid_array.jc + OBSTACLE_3_CENTROID_INDEX);
			}
		}
	}
	else {
		if ((centroid_array.ic[OBSTACLE_2_CENTROID_INDEX] > 0) && (centroid_array.jc[OBSTACLE_2_CENTROID_INDEX] > 0)) {
			obstacle_2.set_center_x(centroid_array.ic + OBSTACLE_2_CENTROID_INDEX);
			obstacle_2.set_center_y(centroid_array.jc + OBSTACLE_2_CENTROID_INDEX);

			if ((centroid_array.ic[OBSTACLE_3_CENTROID_INDEX] > 0) && (centroid_array.jc[OBSTACLE_3_CENTROID_INDEX] > 0)) {
				obstacle_3.set_center_x(centroid_array.ic + OBSTACLE_3_CENTROID_INDEX);
				obstacle_3.set_center_y(centroid_array.jc + OBSTACLE_3_CENTROID_INDEX);
			}
		}
		else {
			if ((centroid_array.ic[OBSTACLE_3_CENTROID_INDEX] > 0) && (centroid_array.jc[OBSTACLE_3_CENTROID_INDEX] > 0)) {
				obstacle_3.set_center_x(centroid_array.ic + OBSTACLE_3_CENTROID_INDEX);
				obstacle_3.set_center_y(centroid_array.jc + OBSTACLE_3_CENTROID_INDEX);
			}
		}
	}

	// Load target coordinates into buffer
	initialize_waypoints(waypoint_array, array_size, -10, -10);



	// Define target pointer
	//target_x = centroid_array.ic + OBSTACLE_2_CENTROID_INDEX;
	//target_y = centroid_array.jc + OBSTACLE_2_CENTROID_INDEX;

	//target_x = enemy_front_x;
	//target_y = enemy_front_y;

	waypoint_direction = atan2((waypoint_array[1] - our_vehicle.get_vehicle_center_y()), (waypoint_array[0]) - our_vehicle.get_vehicle_center_x());
	//waypoint_direction = atan2((centroid_array[BLUE_CENTROID_INDEX + 1] - our_vehicle.get_vehicle_center_y()), (centroid_array[BLUE_CENTROID_INDEX] - our_vehicle.get_vehicle_center_x()));

	serial_send(drive_speed_string, 4, h);

	while (true) {
		// Update vehicles information

		our_vehicle.update_position_orientation();
		enemy_vehicle.update_position_orientation();

		// Defensive strategy is to hide behind the enemy, so they can't turn and shoot us
		defense_target_x = enemy_vehicle.get_center_x() + cos(enemy_vehicle.get_orientation() + 3.14159) * enemy_vehicle.get_radius();
		defense_target_y = enemy_vehicle.get_center_y() + sin(enemy_vehicle.get_orientation() + 3.14159) * enemy_vehicle.get_radius();

		// UPDATE ORIENTATION AND POSITION ERRORS
		if (waypoint_array[0] > 0) {
			// If there are active waypoints, navigate to them
			waypoint_direction = atan2((waypoint_array[1] - our_vehicle.get_vehicle_center_y()), (waypoint_array[0]) - our_vehicle.get_vehicle_center_x());
			position_error = ((waypoint_array[0] - our_vehicle.get_vehicle_center_x()) * (waypoint_array[0] - our_vehicle.get_vehicle_center_x()) + (waypoint_array[1] - our_vehicle.get_vehicle_center_y()) * (waypoint_array[1] - our_vehicle.get_vehicle_center_y()));

			position_threshold = obstacle_threshold;
		}
		else {
			// Otherwise, track the moving target
			waypoint_direction = atan2((defense_target_y - our_vehicle.get_vehicle_center_y()), (defense_target_x) - our_vehicle.get_vehicle_center_x());
			position_error = (((defense_target_x - our_vehicle.get_vehicle_center_x()) * (defense_target_x - our_vehicle.get_vehicle_center_x()) + (defense_target_y - our_vehicle.get_vehicle_center_y()) * (defense_target_y - our_vehicle.get_vehicle_center_y())));

			position_threshold = laser_threshold;
		}

		// Check direct path for obstacles
		// // Using formula from: https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line


		if ((obstacle_1.get_center_x() >= 0) && (obstacle_1_flag == 0)) {
			//std::cout << "CHECKING FOR OBSTACLE 1.";

			// IF WAYPOINTS ALREADY EXIST, USE NEXT WAYPOINT POSITION
			if (waypoint_array[0] >= 0) {
				double dist_calc_1 = abs((waypoint_array[1] - our_vehicle.get_vehicle_center_y()) * obstacle_1.get_center_x() - (waypoint_array[0] - our_vehicle.get_vehicle_center_x()) * obstacle_1.get_center_y() + waypoint_array[0] * our_vehicle.get_vehicle_center_y() - waypoint_array[1] * our_vehicle.get_vehicle_center_x());
				double dist_calc_2 = sqrt((pow(waypoint_array[1] - our_vehicle.get_vehicle_center_y(), 2) + pow(waypoint_array[0] - our_vehicle.get_vehicle_center_x(), 2)));// *obstacle_1.obstacle_radius();
				double distance = abs(dist_calc_1) / abs(dist_calc_2);

				//std::cout << "WAY_DIST_CALC_1: " << dist_calc_1 << "\n";
				//std::cout << "WAY_DIST_CALC_2: " << dist_calc_2 << "\n";
				if (distance < obstacle_1.obstacle_radius()) {
					// Path intersects with keep-out zone, need to find new waypoint
					obstacle_direction = atan2((obstacle_1.get_center_y() - our_vehicle.get_vehicle_center_y()), (obstacle_1.get_center_x()) - our_vehicle.get_vehicle_center_x());
					if (waypoint_direction > obstacle_direction) {
						std::cout << "Obstacle 1 waypoint greater than\n";

						std::cout << "Current Vehicle Position: " << our_vehicle.get_center_x() << ", " << our_vehicle.get_center_y() << ")\n";

						obstacle_perpendicular = obstacle_direction + 3.14159 / 2;
						new_x = (int)(obstacle_1.obstacle_radius() * cos(obstacle_perpendicular)) + obstacle_1.get_center_x();
						new_y = (int)(obstacle_1.obstacle_radius() * sin(obstacle_perpendicular)) + obstacle_1.get_center_y();

						add_new_waypoint(waypoint_array, array_size, new_x, new_y);

						obstacle_1_flag = 1;

						waypoint_direction = atan2((waypoint_array[1] - our_vehicle.get_vehicle_center_y()), (waypoint_array[0]) - our_vehicle.get_vehicle_center_x());
						position_error = ((waypoint_array[0] - our_vehicle.get_vehicle_center_x()) * (waypoint_array[0] - our_vehicle.get_vehicle_center_x()) + (waypoint_array[1] - our_vehicle.get_vehicle_center_y()) * (waypoint_array[1] - our_vehicle.get_vehicle_center_y()));

						position_threshold = obstacle_threshold;
					}
					else {
						std::cout << "Obstacle 1 waypoint less than\n";

						std::cout << "Current Vehicle Position: " << our_vehicle.get_center_x() << ", " << our_vehicle.get_center_y() << ")\n";

						obstacle_perpendicular = obstacle_direction - 3.14159 / 2;
						new_x = (int)(obstacle_1.obstacle_radius() * cos(obstacle_perpendicular)) + obstacle_1.get_center_x();
						new_y = (int)(obstacle_1.obstacle_radius() * sin(obstacle_perpendicular)) + obstacle_1.get_center_y();

						add_new_waypoint(waypoint_array, array_size, new_x, new_y);

						obstacle_1_flag = 1;

						waypoint_direction = atan2((waypoint_array[1] - our_vehicle.get_vehicle_center_y()), (waypoint_array[0]) - our_vehicle.get_vehicle_center_x());
						position_error = ((waypoint_array[0] - our_vehicle.get_vehicle_center_x()) * (waypoint_array[0] - our_vehicle.get_vehicle_center_x()) + (waypoint_array[1] - our_vehicle.get_vehicle_center_y()) * (waypoint_array[1] - our_vehicle.get_vehicle_center_y()));

						position_threshold = obstacle_threshold;
					}
				}
			}
			// IF THERE ARE NO EXISTING WAYPOINTS, USE TARGET LOCATION
			else {
				double dist_calc_1 = abs((defense_target_y - our_vehicle.get_vehicle_center_y()) * obstacle_1.get_center_x() - (defense_target_x - our_vehicle.get_vehicle_center_x()) * obstacle_1.get_center_y() + defense_target_x * our_vehicle.get_vehicle_center_y() - defense_target_y * our_vehicle.get_vehicle_center_x());
				double dist_calc_2 = sqrt((pow(defense_target_y - our_vehicle.get_vehicle_center_y(), 2) + pow(defense_target_x - our_vehicle.get_vehicle_center_x(), 2)));// *obstacle_1.obstacle_radius();
				double distance = abs(dist_calc_1) / abs(dist_calc_2);

				//std::cout << "TAR_DIST_CALC_1: " << dist_calc_1 << "\n";
				//std::cout << "TAR_DIST_CALC_2: " << dist_calc_2 << "\n";

				if (distance < obstacle_1.obstacle_radius()) {
					// Path intersects with keep-out zone, need to find new waypoint
					obstacle_direction = atan2((obstacle_1.get_center_y() - our_vehicle.get_vehicle_center_y()), (obstacle_1.get_center_x()) - our_vehicle.get_vehicle_center_x());
					if (waypoint_direction > obstacle_direction) {
						std::cout << "Obstacle 1 waypoint greater than\n";

						std::cout << "Current Vehicle Position: " << our_vehicle.get_center_x() << ", " << our_vehicle.get_center_y() << ")\n";

						obstacle_perpendicular = obstacle_direction + 3.14159 / 2;
						new_x = (int)(obstacle_1.obstacle_radius() * cos(obstacle_perpendicular)) + obstacle_1.get_center_x();
						new_y = (int)(obstacle_1.obstacle_radius() * sin(obstacle_perpendicular)) + obstacle_1.get_center_y();

						add_new_waypoint(waypoint_array, array_size, new_x, new_y);

						obstacle_1_flag = 1;

						waypoint_direction = atan2((waypoint_array[1] - our_vehicle.get_vehicle_center_y()), (waypoint_array[0]) - our_vehicle.get_vehicle_center_x());
						position_error = ((waypoint_array[0] - our_vehicle.get_vehicle_center_x()) * (waypoint_array[0] - our_vehicle.get_vehicle_center_x()) + (waypoint_array[1] - our_vehicle.get_vehicle_center_y()) * (waypoint_array[1] - our_vehicle.get_vehicle_center_y()));

						position_threshold = obstacle_threshold;
					}
					else {
						std::cout << "Obstacle 1 waypoint less than\n";

						std::cout << "Current Vehicle Position: " << our_vehicle.get_center_x() << ", " << our_vehicle.get_center_y() << ")\n";

						obstacle_perpendicular = obstacle_direction - 3.14159 / 2;
						new_x = (int)(obstacle_1.obstacle_radius() * cos(obstacle_perpendicular)) + obstacle_1.get_center_x();
						new_y = (int)(obstacle_1.obstacle_radius() * sin(obstacle_perpendicular)) + obstacle_1.get_center_y();

						add_new_waypoint(waypoint_array, array_size, new_x, new_y);

						obstacle_1_flag = 1;

						waypoint_direction = atan2((waypoint_array[1] - our_vehicle.get_vehicle_center_y()), (waypoint_array[0]) - our_vehicle.get_vehicle_center_x());
						position_error = ((waypoint_array[0] - our_vehicle.get_vehicle_center_x()) * (waypoint_array[0] - our_vehicle.get_vehicle_center_x()) + (waypoint_array[1] - our_vehicle.get_vehicle_center_y()) * (waypoint_array[1] - our_vehicle.get_vehicle_center_y()));

						position_threshold = obstacle_threshold;
					}
				}
			}
		}

		if ((obstacle_2.get_center_x() >= 0) && (obstacle_2_flag == 0)) {
			//std::cout << "CHECKING FOR OBSTACLE 2.";

			// IF WAYPOINTS ALREADY EXIST, USE NEXT WAYPOINT POSITION
			if (waypoint_array[0] >= 0) {
				double dist_calc_1 = abs((waypoint_array[1] - our_vehicle.get_vehicle_center_y()) * obstacle_2.get_center_x() - (waypoint_array[0] - our_vehicle.get_vehicle_center_x()) * obstacle_2.get_center_y() + waypoint_array[0] * our_vehicle.get_vehicle_center_y() - waypoint_array[1] * our_vehicle.get_vehicle_center_x());
				double dist_calc_2 = sqrt((pow(waypoint_array[1] - our_vehicle.get_vehicle_center_y(), 2) + pow(waypoint_array[0] - our_vehicle.get_vehicle_center_x(), 2)));// *obstacle_2.obstacle_radius();
				double distance = abs(dist_calc_1) / abs(dist_calc_2);

				//std::cout << "WAY_DIST_CALC_1: " << dist_calc_1 << "\n";
				//std::cout << "WAY_DIST_CALC_2: " << dist_calc_2 << "\n";
				if (distance < obstacle_2.obstacle_radius()) {
					// Path intersects with keep-out zone, need to find new waypoint
					obstacle_direction = atan2((obstacle_2.get_center_y() - our_vehicle.get_vehicle_center_y()), (obstacle_2.get_center_x()) - our_vehicle.get_vehicle_center_x());
					if (waypoint_direction > obstacle_direction) {
						std::cout << "Obstacle 2 waypoint greater than\n";

						std::cout << "Current Vehicle Position: " << our_vehicle.get_center_x() << ", " << our_vehicle.get_center_y() << ")\n";

						obstacle_perpendicular = obstacle_direction + 3.14159 / 2;
						new_x = (int)(obstacle_2.obstacle_radius() * cos(obstacle_perpendicular)) + obstacle_2.get_center_x();
						new_y = (int)(obstacle_2.obstacle_radius() * sin(obstacle_perpendicular)) + obstacle_2.get_center_y();

						add_new_waypoint(waypoint_array, array_size, new_x, new_y);

						obstacle_2_flag = 1;

						waypoint_direction = atan2((waypoint_array[1] - our_vehicle.get_vehicle_center_y()), (waypoint_array[0]) - our_vehicle.get_vehicle_center_x());
						position_error = ((waypoint_array[0] - our_vehicle.get_vehicle_center_x()) * (waypoint_array[0] - our_vehicle.get_vehicle_center_x()) + (waypoint_array[1] - our_vehicle.get_vehicle_center_y()) * (waypoint_array[1] - our_vehicle.get_vehicle_center_y()));

						position_threshold = obstacle_threshold;
					}
					else {
						std::cout << "Obstacle 2 waypoint less than\n";

						std::cout << "Current Vehicle Position: " << our_vehicle.get_center_x() << ", " << our_vehicle.get_center_y() << ")\n";

						obstacle_perpendicular = obstacle_direction - 3.14159 / 2;
						new_x = (int)(obstacle_2.obstacle_radius() * cos(obstacle_perpendicular)) + obstacle_2.get_center_x();
						new_y = (int)(obstacle_2.obstacle_radius() * sin(obstacle_perpendicular)) + obstacle_2.get_center_y();

						add_new_waypoint(waypoint_array, array_size, new_x, new_y);

						obstacle_2_flag = 1;

						waypoint_direction = atan2((waypoint_array[1] - our_vehicle.get_vehicle_center_y()), (waypoint_array[0]) - our_vehicle.get_vehicle_center_x());
						position_error = ((waypoint_array[0] - our_vehicle.get_vehicle_center_x()) * (waypoint_array[0] - our_vehicle.get_vehicle_center_x()) + (waypoint_array[1] - our_vehicle.get_vehicle_center_y()) * (waypoint_array[1] - our_vehicle.get_vehicle_center_y()));

						position_threshold = obstacle_threshold;
					}
				}
			}
			// IF THERE ARE NO EXISTING WAYPOINTS, USE TARGET LOCATION
			else {
				double dist_calc_1 = abs((defense_target_y - our_vehicle.get_vehicle_center_y()) * obstacle_2.get_center_x() - (defense_target_x - our_vehicle.get_vehicle_center_x()) * obstacle_2.get_center_y() + defense_target_x * our_vehicle.get_vehicle_center_y() - defense_target_y * our_vehicle.get_vehicle_center_x());
				double dist_calc_2 = sqrt((pow(defense_target_y - our_vehicle.get_vehicle_center_y(), 2) + pow(defense_target_x - our_vehicle.get_vehicle_center_x(), 2)));// *obstacle_1.obstacle_radius();
				double distance = abs(dist_calc_1) / abs(dist_calc_2);

				//std::cout << "TAR_DIST_CALC_1: " << dist_calc_1 << "\n";
				//std::cout << "TAR_DIST_CALC_2: " << dist_calc_2 << "\n";

				if (distance < obstacle_1.obstacle_radius()) {
					// Path intersects with keep-out zone, need to find new waypoint
					obstacle_direction = atan2((obstacle_2.get_center_y() - our_vehicle.get_vehicle_center_y()), (obstacle_2.get_center_x()) - our_vehicle.get_vehicle_center_x());
					if (waypoint_direction > obstacle_direction) {
						std::cout << "Obstacle 2 waypoint greater than\n";

						std::cout << "Current Vehicle Position: " << our_vehicle.get_center_x() << ", " << our_vehicle.get_center_y() << ")\n";

						obstacle_perpendicular = obstacle_direction + 3.14159 / 2;
						new_x = (int)(obstacle_2.obstacle_radius() * cos(obstacle_perpendicular)) + obstacle_2.get_center_x();
						new_y = (int)(obstacle_2.obstacle_radius() * sin(obstacle_perpendicular)) + obstacle_2.get_center_y();

						add_new_waypoint(waypoint_array, array_size, new_x, new_y);

						obstacle_2_flag = 1;

						waypoint_direction = atan2((waypoint_array[1] - our_vehicle.get_vehicle_center_y()), (waypoint_array[0]) - our_vehicle.get_vehicle_center_x());
						position_error = ((waypoint_array[0] - our_vehicle.get_vehicle_center_x()) * (waypoint_array[0] - our_vehicle.get_vehicle_center_x()) + (waypoint_array[1] - our_vehicle.get_vehicle_center_y()) * (waypoint_array[1] - our_vehicle.get_vehicle_center_y()));

						position_threshold = obstacle_threshold;
					}
					else {
						std::cout << "Obstacle 2 waypoint less than\n";

						std::cout << "Current Vehicle Position: " << our_vehicle.get_center_x() << ", " << our_vehicle.get_center_y() << ")\n";

						obstacle_perpendicular = obstacle_direction - 3.14159 / 2;
						new_x = (int)(obstacle_2.obstacle_radius() * cos(obstacle_perpendicular)) + obstacle_2.get_center_x();
						new_y = (int)(obstacle_2.obstacle_radius() * sin(obstacle_perpendicular)) + obstacle_2.get_center_y();

						add_new_waypoint(waypoint_array, array_size, new_x, new_y);

						obstacle_2_flag = 1;

						waypoint_direction = atan2((waypoint_array[1] - our_vehicle.get_vehicle_center_y()), (waypoint_array[0]) - our_vehicle.get_vehicle_center_x());
						position_error = ((waypoint_array[0] - our_vehicle.get_vehicle_center_x()) * (waypoint_array[0] - our_vehicle.get_vehicle_center_x()) + (waypoint_array[1] - our_vehicle.get_vehicle_center_y()) * (waypoint_array[1] - our_vehicle.get_vehicle_center_y()));

						position_threshold = obstacle_threshold;
					}
				}
			}
		}

		if ((obstacle_3.get_center_x() >= 0) && (obstacle_3_flag == 0)) {
			//std::cout << "CHECKING FOR OBSTACLE 3.";

			// IF WAYPOINTS ALREADY EXIST, USE NEXT WAYPOINT POSITION
			if (waypoint_array[0] >= 0) {
				double dist_calc_1 = abs((waypoint_array[1] - our_vehicle.get_vehicle_center_y()) * obstacle_3.get_center_x() - (waypoint_array[0] - our_vehicle.get_vehicle_center_x()) * obstacle_3.get_center_y() + waypoint_array[0] * our_vehicle.get_vehicle_center_y() - waypoint_array[1] * our_vehicle.get_vehicle_center_x());
				double dist_calc_2 = sqrt((pow(waypoint_array[1] - our_vehicle.get_vehicle_center_y(), 2) + pow(waypoint_array[0] - our_vehicle.get_vehicle_center_x(), 2)));// *obstacle_3.obstacle_radius();
				double distance = abs(dist_calc_1) / abs(dist_calc_2);

				//std::cout << "WAY_DIST_CALC_1: " << dist_calc_1 << "\n";
				//std::cout << "WAY_DIST_CALC_2: " << dist_calc_2 << "\n";
				if (distance < obstacle_3.obstacle_radius()) {
					// Path intersects with keep-out zone, need to find new waypoint
					obstacle_direction = atan2((obstacle_3.get_center_y() - our_vehicle.get_vehicle_center_y()), (obstacle_3.get_center_x()) - our_vehicle.get_vehicle_center_x());
					if (waypoint_direction > obstacle_direction) {
						std::cout << "Obstacle 3 waypoint greater than\n";

						std::cout << "Current Vehicle Position: " << our_vehicle.get_center_x() << ", " << our_vehicle.get_center_y() << ")\n";

						obstacle_perpendicular = obstacle_direction + 3.14159 / 2;
						new_x = (int)(obstacle_3.obstacle_radius() * cos(obstacle_perpendicular)) + obstacle_3.get_center_x();
						new_y = (int)(obstacle_3.obstacle_radius() * sin(obstacle_perpendicular)) + obstacle_3.get_center_y();

						add_new_waypoint(waypoint_array, array_size, new_x, new_y);

						obstacle_3_flag = 1;

						waypoint_direction = atan2((waypoint_array[1] - our_vehicle.get_vehicle_center_y()), (waypoint_array[0]) - our_vehicle.get_vehicle_center_x());
						position_error = ((waypoint_array[0] - our_vehicle.get_vehicle_center_x()) * (waypoint_array[0] - our_vehicle.get_vehicle_center_x()) + (waypoint_array[1] - our_vehicle.get_vehicle_center_y()) * (waypoint_array[1] - our_vehicle.get_vehicle_center_y()));

						position_threshold = obstacle_threshold;
					}
					else {
						std::cout << "Obstacle 3 waypoint less than\n";

						std::cout << "Current Vehicle Position: " << our_vehicle.get_center_x() << ", " << our_vehicle.get_center_y() << ")\n";

						obstacle_perpendicular = obstacle_direction - 3.14159 / 2;
						new_x = (int)(obstacle_3.obstacle_radius() * cos(obstacle_perpendicular)) + obstacle_3.get_center_x();
						new_y = (int)(obstacle_3.obstacle_radius() * sin(obstacle_perpendicular)) + obstacle_3.get_center_y();

						add_new_waypoint(waypoint_array, array_size, new_x, new_y);

						obstacle_3_flag = 1;

						waypoint_direction = atan2((waypoint_array[1] - our_vehicle.get_vehicle_center_y()), (waypoint_array[0]) - our_vehicle.get_vehicle_center_x());
						position_error = ((waypoint_array[0] - our_vehicle.get_vehicle_center_x()) * (waypoint_array[0] - our_vehicle.get_vehicle_center_x()) + (waypoint_array[1] - our_vehicle.get_vehicle_center_y()) * (waypoint_array[1] - our_vehicle.get_vehicle_center_y()));

						position_threshold = obstacle_threshold;
					}
				}
			}

			// IF THERE ARE NO EXISTING WAYPOINTS, USE TARGET LOCATION
			else {
				double dist_calc_1 = abs((defense_target_y - our_vehicle.get_vehicle_center_y()) * obstacle_1.get_center_x() - (defense_target_x - our_vehicle.get_vehicle_center_x()) * obstacle_1.get_center_y() + defense_target_x * our_vehicle.get_vehicle_center_y() - defense_target_y * our_vehicle.get_vehicle_center_x());
				double dist_calc_2 = sqrt((pow(defense_target_y - our_vehicle.get_vehicle_center_y(), 2) + pow(defense_target_x - our_vehicle.get_vehicle_center_x(), 2)));// *obstacle_1.obstacle_radius();
				double distance = abs(dist_calc_1) / abs(dist_calc_2);

				//std::cout << "TAR_DIST_CALC_1: " << dist_calc_1 << "\n";
				//std::cout << "TAR_DIST_CALC_2: " << dist_calc_2 << "\n";

				if (distance < obstacle_1.obstacle_radius()) {
					// Path intersects with keep-out zone, need to find new waypoint
					obstacle_direction = atan2((obstacle_1.get_center_y() - our_vehicle.get_vehicle_center_y()), (obstacle_1.get_center_x()) - our_vehicle.get_vehicle_center_x());
					if (waypoint_direction > obstacle_direction) {
						std::cout << "Obstacle 1 waypoint greater than\n";

						std::cout << "Current Vehicle Position: " << our_vehicle.get_center_x() << ", " << our_vehicle.get_center_y() << ")\n";

						obstacle_perpendicular = obstacle_direction + 3.14159 / 2;
						new_x = (int)(obstacle_1.obstacle_radius() * cos(obstacle_perpendicular)) + obstacle_1.get_center_x();
						new_y = (int)(obstacle_1.obstacle_radius() * sin(obstacle_perpendicular)) + obstacle_1.get_center_y();

						add_new_waypoint(waypoint_array, array_size, new_x, new_y);

						obstacle_1_flag = 1;

						waypoint_direction = atan2((waypoint_array[1] - our_vehicle.get_vehicle_center_y()), (waypoint_array[0]) - our_vehicle.get_vehicle_center_x());
						position_error = ((waypoint_array[0] - our_vehicle.get_vehicle_center_x()) * (waypoint_array[0] - our_vehicle.get_vehicle_center_x()) + (waypoint_array[1] - our_vehicle.get_vehicle_center_y()) * (waypoint_array[1] - our_vehicle.get_vehicle_center_y()));

						position_threshold = obstacle_threshold;
					}
					else {
						std::cout << "Obstacle 1 waypoint less than\n";

						std::cout << "Current Vehicle Position: " << our_vehicle.get_center_x() << ", " << our_vehicle.get_center_y() << ")\n";

						obstacle_perpendicular = obstacle_direction - 3.14159 / 2;
						new_x = (int)(obstacle_1.obstacle_radius() * cos(obstacle_perpendicular)) + obstacle_1.get_center_x();
						new_y = (int)(obstacle_1.obstacle_radius() * sin(obstacle_perpendicular)) + obstacle_1.get_center_y();

						add_new_waypoint(waypoint_array, array_size, new_x, new_y);

						obstacle_1_flag = 1;

						waypoint_direction = atan2((waypoint_array[1] - our_vehicle.get_vehicle_center_y()), (waypoint_array[0]) - our_vehicle.get_vehicle_center_x());
						position_error = ((waypoint_array[0] - our_vehicle.get_vehicle_center_x()) * (waypoint_array[0] - our_vehicle.get_vehicle_center_x()) + (waypoint_array[1] - our_vehicle.get_vehicle_center_y()) * (waypoint_array[1] - our_vehicle.get_vehicle_center_y()));

						position_threshold = obstacle_threshold;
					}
				}
			}
		}

		// Debug Prints
		if (KEY(VK_RETURN)) {
			std::cout << "Current Direction Error: " << direction_error << "\n\n";
			std::cout << "Current Position Error: " << position_error << "\n\n";
			std::cout << "Current Waypoints:\n";


			//std::cout << "Array length:" << array_size << "\n\n";

			for (int ii = 0; ii < (array_size - 2); ii += 2) {
				std::cout << "x(" << (int)(ii / 2) << "): " << waypoint_array[ii] << "\n";
				std::cout << "y(" << (int)(ii / 2) << "): " << waypoint_array[ii + 1] << "\n";
			}

			std::cout << "\n";

		}

		// Calculate position and orientation errors
		direction_error = waypoint_direction - our_vehicle.get_orientation();

		// Use tighter threshold when fine-aligning for fire
		double active_direction_threshold = fine_align_flag ? fire_direction_threshold : direction_threshold;

		// Turn towards current waypoint
		if ((abs(direction_error) > active_direction_threshold)) {
			if (direction_error > 0 && turn_flag < 1) {

				serial_send("9\n", 2, h);
				//Sleep(50);

				turn_flag = 1;
				vertical_flag = 0;
				std::cout << "GO LEFT BROTHER.\n";
			}
			else if (direction_error < 0 && turn_flag > -1) {
				serial_send("10\n", 3, h);
				//Sleep(50);

				turn_flag = -1;
				vertical_flag = 0;
				std::cout << "GO RIGHT BROTHER.\n";
			}
			else if ((abs(direction_error) < 2 * direction_threshold) && (turn_flag != 0) && (slow_flag < 2)) {
				slow_flag = 2;

				//slow_down_drive_speed_string = "100\n";

				std::cout << "SLOW DOWN MORE BROTHER: " << slower_drive_speed_string << "\n";

				serial_send(slower_drive_speed_string, 4, h);
			}
			else if ((abs(direction_error) < 3 * direction_threshold) && (turn_flag != 0) && (slow_flag < 1)) {
				slow_flag = 1;

				//slow_down_drive_speed_string = "160\n";

				std::cout << "SLOW DOWN BROTHER: " << slow_drive_speed_string << "\n";

				serial_send(slow_drive_speed_string, 4, h);
			}

		}

		// DIRECTION ERROR MINIMIZED, MOVE FORWARD
		else {

			// IF ROBOT WASN'T MOVING FORWARD, MOVE FORWARD
			if ((abs(position_error) > position_threshold) && vertical_flag < 1) {


				serial_send("2\n", 2, h);
				turn_flag = 0;
				vertical_flag = 1;
				fine_align_flag = 0; // Reset fine-align when moving forward

				// SET BACK TO ORIGINAL SPEED IF MOVING SLOW
				if (slow_flag != 0) {
					//drive_speed_string = "200\n";

					serial_send(drive_speed_string, 4, h);

					std::cout << "SPEED UP BROTHER:" << drive_speed_string << "\n";
					slow_flag = 0;


				}

				//				std::cout << "Position Threshold: " << position_threshold << "\n";
				//				std::cout << "Position Error: " << position_error << "\n";
				//
				std::cout << "FULL STEAM AHEAD!\n";

			}

			// IF NEAR THE WAYPOINT
			else if ((abs(position_error) <= position_threshold)) {

				std::cout << "APPROACHING TARGET MENACINGLY.\n";

				vertical_flag = 0;

				obstacle_1_distance = sqrt(pow((our_vehicle.get_center_x() - obstacle_1.get_center_x()), 2) + pow(our_vehicle.get_center_y() - obstacle_1.get_center_y(), 2));

				std::cout << "Distance to Obstacle 1: " << obstacle_1_distance << "\n";
				std::cout << "Obstacle 1 Flag: " << obstacle_1_flag << "\n";

				if (obstacle_1_distance > 2 * obstacle_1.obstacle_radius()) {
					obstacle_1_flag = 0;
				}

				// CLEAR WAYPOINT
				if (waypoint_array[0] > 0) {
					remove_current_waypoint(waypoint_array, array_size);

					std::cout << "TIME EXTENDED!\n";
				}

				// FIRE MAH LAZARRR SINCE I'M NEAR THE TARGET
				else {
					// Fine-align check: must be within tight direction threshold before firing
					if (abs(direction_error) > fire_direction_threshold) {
						fine_align_flag = 1;  // Tighten turn threshold next iteration
						turn_flag = 0;        // Allow turn logic to re-engage
						vertical_flag = 0;
						std::cout << "FINE ALIGNING BEFORE FIRE. Direction error: " << direction_error << "\n";
					}
					else {
						fine_align_flag = 0;
						serial_send("13\n", 3, h);
						std::cout << "TARGET ELIMINATED. GET TO EXTRACTION 47.\n\n";
						Sleep(100);
						serial_send("0\n", 2, h);
						break;
					}
				}
			}
		}
	}
}