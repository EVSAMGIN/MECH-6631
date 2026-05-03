#include <cmath>

#include "world_map.h"
#include "obstacle.h"
#include "vehicle.h"
//#include "cost_map.h"
//#include "serial_com.h"
#include "vision_custom.h"

// For Serial COM Port
// From https://learn.microsoft.com/en-us/windows/win32/devio/configuring-a-communications-resource

#include <windows.h>
#include <tchar.h>
#include <stdio.h>
#include <iostream>

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

// Centroid index values for centroid array
static const int RED_CENTROID_INDEX        = 0;
static const int GREEN_CENTROID_INDEX      = 1;
static const int BLUE_CENTROID_INDEX       = 2;
static const int YELLOW_CENTROID_INDEX     = 3;
static const int OBSTACLE_1_CENTROID_INDEX = 4;
static const int OBSTACLE_2_CENTROID_INDEX = 5;
static const int OBSTACLE_3_CENTROID_INDEX = 6;

//static const int array_size = 14;
//
//
//int* waypoint_array = new int[array_size];

int new_x, new_y, position_error, position_threshold = 2, laser_threshold = 10;

double waypoint_direction, direction_error, obstacle_direction, obstacle_perpendicular;
double direction_threshold = 0.3;
int turn_flag=0;
char wheelspeed [5] = "255\n";
int backlash = 255;

// Centroid pointers
double *control_front_x, *control_front_y, *control_back_x, *control_back_y;
double   *enemy_front_x,   *enemy_front_y,   *enemy_back_x,   *enemy_back_y;
double default_centroid_position = -10;

obstacle obstacle_1(&default_centroid_position, &default_centroid_position, 5, 1);
obstacle obstacle_2(&default_centroid_position, &default_centroid_position, 5, 1);
obstacle obstacle_3(&default_centroid_position, &default_centroid_position, 5, 1);

// Serial
//HANDLE h1;
//int speed = 0;

void initialize_waypoints(int* waypoints, int array_size, int start_x_waypoint, int start_y_waypoint) {
	//int array_length = (int)(sizeof(waypoints) / sizeof(waypoints[0]));
	
	waypoints[0] = start_x_waypoint;
	waypoints[1] = start_y_waypoint;

	for (int i = 2; i < (array_size - 1); i++) {
		waypoints[i] = -10;
	}
}

void add_new_waypoint(int* waypoints, int array_size, int new_x_waypoint, int new_y_waypoint) {
	//int array_length = (int) (sizeof(waypoints) / sizeof(waypoints[0]));

	for (int i = array_size - 1; i > 2 ; i--) {
		waypoints[i] = waypoints[i - 2];
	}

	std::cout << "Adding waypoints: (" << new_x_waypoint << ", " << new_y_waypoint << ")\n";

	waypoints[1] = new_y_waypoint;
	waypoints[0] = new_x_waypoint;
}


void remove_current_waypoint(int* waypoints, int array_size) {
	//int array_length = (int) (sizeof(waypoints) / sizeof(waypoints[0]));

	for (int i = 0; i < (array_size - 2); i++) {
		waypoints[i] = waypoints[i + 2];
	}

	waypoints[array_size - 2] = -10;
	waypoints[array_size - 1] = -10;
}

// Need to include passthrough for keyboard checks?
void mapper(TargetPositions& centroid_array, HANDLE& h, int* waypoint_array, int array_size, int width, int height) {

	// Accept centroid array
	// Create vehicle and obstacle objects
	// Develop waypoint buffer
	// Find path through waypoint buffer, update as needed
	std::cout << "Centroid Array";// << centroid_array;
	
	// Initial setup, only run once:

	// Open serial port
	//open_serial("COM5", h1, speed);


	//// Our robot

	control_front_x = centroid_array.ic + RED_CENTROID_INDEX;
	control_front_y = centroid_array.jc + RED_CENTROID_INDEX;
	control_back_x = centroid_array.ic + GREEN_CENTROID_INDEX;
	control_back_y = centroid_array.jc + GREEN_CENTROID_INDEX;

	//std::cout << "Something\n\n";

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
			//static obstacle obstacle_2(centroid_array + OBSTACLE_2_CENTROID_INDEX, centroid_array + OBSTACLE_2_CENTROID_INDEX + 1, 4, 1);
			obstacle_2.set_center_x(centroid_array.ic + OBSTACLE_2_CENTROID_INDEX);
			obstacle_2.set_center_y(centroid_array.jc + OBSTACLE_2_CENTROID_INDEX);

			if ((centroid_array.ic[OBSTACLE_3_CENTROID_INDEX] > 0) && (centroid_array.jc[OBSTACLE_3_CENTROID_INDEX] > 0)) {
				//static obstacle obstacle_3(centroid_array + OBSTACLE_2_CENTROID_INDEX, centroid_array + OBSTACLE_3_CENTROID_INDEX + 1, 4, 1);
				obstacle_3.set_center_x(centroid_array.ic + OBSTACLE_3_CENTROID_INDEX);
				obstacle_3.set_center_y(centroid_array.jc + OBSTACLE_3_CENTROID_INDEX);

//				static cost_map map(width, height, &our_vehicle, &enemy_vehicle, &obstacle_1, &obstacle_2, &obstacle_3);
			}
			else {
//				static cost_map map(width, height, &our_vehicle, &enemy_vehicle, &obstacle_1, &obstacle_2);
			}
		}
		else {
			if ((centroid_array.ic[OBSTACLE_3_CENTROID_INDEX] > 0) && (centroid_array.jc[OBSTACLE_3_CENTROID_INDEX] > 0)) {
				//static obstacle obstacle_3(centroid_array + OBSTACLE_2_CENTROID_INDEX, centroid_array + OBSTACLE_3_CENTROID_INDEX + 1, 4, 1);
				obstacle_3.set_center_x(centroid_array.ic + OBSTACLE_3_CENTROID_INDEX);
				obstacle_3.set_center_y(centroid_array.jc + OBSTACLE_3_CENTROID_INDEX);

//				static cost_map map(width, height, &our_vehicle, &enemy_vehicle, &obstacle_1, &obstacle_3);
			} else {
//				static cost_map map(width, height, &our_vehicle, &enemy_vehicle, &obstacle_1);
			}
		}
	} else {
		if ((centroid_array.ic[OBSTACLE_2_CENTROID_INDEX] > 0) && (centroid_array.jc[OBSTACLE_2_CENTROID_INDEX] > 0)) {
			//static obstacle obstacle_2(centroid_array + OBSTACLE_2_CENTROID_INDEX, centroid_array + OBSTACLE_2_CENTROID_INDEX + 1, 4, 1);
			obstacle_2.set_center_x(centroid_array.ic + OBSTACLE_2_CENTROID_INDEX);
			obstacle_2.set_center_y(centroid_array.jc + OBSTACLE_2_CENTROID_INDEX);

			if ((centroid_array.ic[OBSTACLE_3_CENTROID_INDEX] > 0) && (centroid_array.jc[OBSTACLE_3_CENTROID_INDEX] > 0)) {
				//static obstacle obstacle_3(centroid_array + OBSTACLE_2_CENTROID_INDEX, centroid_array + OBSTACLE_3_CENTROID_INDEX + 1, 4, 1);
				obstacle_3.set_center_x(centroid_array.ic + OBSTACLE_3_CENTROID_INDEX);
				obstacle_3.set_center_y(centroid_array.jc + OBSTACLE_3_CENTROID_INDEX);

//				static cost_map map(width, height, &our_vehicle, &enemy_vehicle, &obstacle_2, &obstacle_3);
			}
			else {
//				static cost_map map(width, height, &our_vehicle, &enemy_vehicle, &obstacle_2);
			}
		}
		else {
			if ((centroid_array.ic[OBSTACLE_3_CENTROID_INDEX] > 0) && (centroid_array.jc[OBSTACLE_3_CENTROID_INDEX] > 0)) {
				//static obstacle obstacle_3(centroid_array + OBSTACLE_2_CENTROID_INDEX, centroid_array + OBSTACLE_3_CENTROID_INDEX + 1, 4, 1);
				obstacle_3.set_center_x(centroid_array.ic + OBSTACLE_3_CENTROID_INDEX);
				obstacle_3.set_center_y(centroid_array.jc + OBSTACLE_3_CENTROID_INDEX);

//				static cost_map map(width, height, &our_vehicle, &enemy_vehicle, &obstacle_3);
			}
			else {
//				static cost_map map(width, height, &our_vehicle, &enemy_vehicle);
			}
		}
	}
	
	// Load target coordinates into buffer
	//initialize_waypoints(waypoint_array, enemy_vehicle.get_vehicle_center_x(), enemy_vehicle.get_vehicle_center_y());
	initialize_waypoints(waypoint_array, array_size, centroid_array.ic[OBSTACLE_2_CENTROID_INDEX], centroid_array.jc[OBSTACLE_2_CENTROID_INDEX]);

	/*
	waypoint_array[0] = enemy_vehicle.get_vehicle_center_x();
	waypoint_array[1] = enemy_vehicle.get_vehicle_center_y();
	waypoint_array[2] = -10;
	waypoint_array[3] = -10;
	waypoint_array[4] = -10;
	waypoint_array[5] = -10;
	waypoint_array[6] = -10;
	waypoint_array[7] = -10;
	waypoint_array[8] = -10;
	waypoint_array[9] = -10;
	waypoint_array[10] = -10;
	waypoint_array[11] = -10;
	waypoint_array[12] = -10;
	waypoint_array[13] = -10;
	*/
	
	waypoint_direction = atan2((waypoint_array[1] - our_vehicle.get_vehicle_center_y()), (waypoint_array[0]) - our_vehicle.get_vehicle_center_x());
	//waypoint_direction = atan2((centroid_array[BLUE_CENTROID_INDEX + 1] - our_vehicle.get_vehicle_center_y()), (centroid_array[BLUE_CENTROID_INDEX] - our_vehicle.get_vehicle_center_x()));

	serial_send(wheelspeed, 4, h);

	// while loop to iterate through waypoints
	while (true) {
		
		
		// Update vehicles information

		initialize_waypoints(waypoint_array, array_size, centroid_array.ic[OBSTACLE_2_CENTROID_INDEX], centroid_array.jc[OBSTACLE_2_CENTROID_INDEX]);

		our_vehicle.update_position_orientation();
		enemy_vehicle.update_position_orientation();

		// Debug prints
		//std::cout << "Current Waypoints:\n" << waypoint_array << "\n\n";
	
		if (KEY('Q')) {
			serial_send("9", 1, h);
			Sleep(500);
			serial_send("0", 1, h);
			Sleep(100); 
			std::cout << "Command sent";
		}
		else if (KEY('W')) {
			serial_send("2", 1, h);
			Sleep(500);
			serial_send("0", 1, h);
			Sleep(100);
			std::cout << "Command sent";
		}
		else if (KEY('E')) {
			serial_send("10", 2, h);
			Sleep(500);
			serial_send("0", 1, h);
			Sleep(100);
			std::cout << "Command sent";
		}
		else if (KEY('A')) {
			serial_send("5", 1, h);
			Sleep(500);
			serial_send("0", 1, h);
			Sleep(100);
			std::cout << "Command sent";
		}
		else if (KEY('S')) {
			serial_send("7", 1, h);
			Sleep(500);
			serial_send("0", 1, h);
			Sleep(100);
			std::cout << "Command sent";
		}
		else if (KEY('D')) {
			serial_send("4", 1, h);
			Sleep(500);
			serial_send("0", 1, h);
			Sleep(100);
			std::cout << "Command sent";
		}

		//std::cout << "Cleared keyboard commands";


		// Check direct path for obstacles
		// // Using formula from: https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
		// // 
		// // waypoint_direction = atan2((waypoint_array[1] - our_vehicle.get_vehicle_center_y()), (waypoint_array[1]) - our_vehicle.get_vehicle_center_x());


		/*
		if(obstacle_1.get_center_x() >= 0) {
			std::cout << "Obstacle 1 loop";
			if ( ( (waypoint_array[1] - our_vehicle.get_vehicle_center_y() )*obstacle_1.get_center_x() - (waypoint_array[0] - our_vehicle.get_vehicle_center_x())*obstacle_1.get_center_y() + waypoint_array[0]*our_vehicle.get_vehicle_center_y() - waypoint_array[1] - our_vehicle.get_vehicle_center_x())^2 < ((waypoint_array[1] - our_vehicle.get_vehicle_center_y())^2 - (waypoint_array[0] - our_vehicle.get_vehicle_center_x())^2 )*(obstacle_1.obstacle_radius())^2 ) {
				std::cout << "Obstacle 1 intersection";
				// Path intersects with keep-out zone, need to find new waypoint
				obstacle_direction = atan2((obstacle_1.get_center_y() - our_vehicle.get_vehicle_center_y()), (obstacle_1.get_center_x()) - our_vehicle.get_vehicle_center_x());
				if(waypoint_direction > obstacle_direction) {
					std::cout << "Obstacle 1 waypoint greater than";
					obstacle_perpendicular = obstacle_direction + 3.14159/2;
					new_x = (int) obstacle_1.obstacle_radius() * cos(obstacle_perpendicular);
					new_y = (int) obstacle_1.obstacle_radius() * sin(obstacle_perpendicular);
					
					add_new_waypoint(waypoint_array, new_x, new_y);
				} else {
					std::cout << "Obstacle 1 waypoint less than";
					obstacle_perpendicular = obstacle_direction - 3.14159 / 2;
					new_x = (int)obstacle_1.obstacle_radius() * cos(obstacle_perpendicular);
					new_y = (int)obstacle_1.obstacle_radius() * sin(obstacle_perpendicular);
					
					add_new_waypoint(waypoint_array, new_x, new_y);
				}
			}
		}
		*/

		//std::cout << "Obstacle 1 cleared";

		// if(obstacle_2.get_center_x() >= 0) {
		// //	if ( ( (waypoint_array[1] - our_vehicle.get_vehicle_center_y() )*obstacle_2.get_center_x() - (waypoint_array[0] - our_vehicle.get_vehicle_center_x())*obstacle_2.get_center_y() + waypoint_array[0]*our_vehicle.get_vehicle_center_y() - waypoint_array[1] - our_vehicle.get_vehicle_center_x())^2 < ((waypoint_array[1] - our_vehicle.get_vehicle_center_y())^2 - (waypoint_array[0] - our_vehicle.get_vehicle_center_x())^2 )*(obstacle_2.obstacle_radius)^2 ) {
		// //		// Find new waypoint around obstacle
		// //		// Alter waypoint array
		// //	}
		// }
		// if(obstacle_3.get_center_x() >= 0) {
		// //	if ( ( (waypoint_array[1] - our_vehicle.get_vehicle_center_y() )*obstacle_3.get_center_x() - (waypoint_array[0] - our_vehicle.get_vehicle_center_x())*obstacle_3.get_center_y() + waypoint_array[0]*our_vehicle.get_vehicle_center_y() - waypoint_array[1] - our_vehicle.get_vehicle_center_x())^2 < ((waypoint_array[1] - our_vehicle.get_vehicle_center_y())^2 - (waypoint_array[0] - our_vehicle.get_vehicle_center_x())^2 )*(obstacle_3.obstacle_radius)^2 ) {
		// //		// Find new waypoint around obstacle
		// //		// Alter waypoint array
		// //	}
		// }


		waypoint_direction = atan2((waypoint_array[1] - our_vehicle.get_vehicle_center_y()), (waypoint_array[0]) - our_vehicle.get_vehicle_center_x());

		// Calculate position and orientation errors
		direction_error = waypoint_direction - our_vehicle.get_orientation();



		if (waypoint_array[0] > 0) {
		//	std::cout << "Entering error calcs";
			position_error = (int)((waypoint_array[0] - our_vehicle.get_center_x()) ^ 2 + (waypoint_array[1] - our_vehicle.get_center_y()) ^ 2);
		}
		else {
			std::cout << "Exiting nav";
			position_error = 0;
			break;
		}

		// Debug Prints
		if (KEY(VK_RETURN)) {
			std::cout << "Current Direction Error: " << direction_error << "\n\n";
			std::cout << "Current Position Error: " << position_error << "\n\n";
			std::cout << "Current Waypoints:\n";
			
			

			//int array_length = (int)(sizeof(waypoint_array) / sizeof(waypoint_array[0]));

			std::cout << "Array length:" << array_size << "\n\n";

			for (int ii = 0; ii < (array_size - 2); ii+=2) {
				std::cout << "x(" << (int) (ii/2) << "): " << waypoint_array[ii] << "\n";
				std::cout << "y(" << (int) (ii/2) << "): " << waypoint_array[ii+1] << "\n";
			}

			std::cout << "\n";
					
		}

		// Turn towards current waypoint
		if (abs(direction_error) > direction_threshold) {
			if (direction_error > 0 && turn_flag < 1) {

				serial_send("9\n", 2, h);
				//Sleep(50);

				turn_flag = 1;
				std::cout << "GO LEFT BROTHER.\n";
			}
			else if (direction_error < 0 && turn_flag > -1) {		
				serial_send("10\n", 3, h);
				//Sleep(50);

				turn_flag = -1;
				std::cout << "GO RIGHT BROTHER.\n";
			}
		}
		// Move towards current waypoint
		else {

			//serial_send("0\n", 2, h);
			//Sleep(50);

			if (turn_flag == 1) {
				serial_send("10\n", 3, h);
				Sleep(backlash);
				std::cout << "MAJOR LAZER!!!\n";
				serial_send("13\n", 3, h);
				Sleep(50);
			}
			else if (turn_flag == -1) {
				serial_send("9\n", 2, h);
				Sleep(backlash);
				std::cout << "MAJOR LAZER!!!\n";
				serial_send("13\n", 3, h);
				Sleep(50);
			}
		
			
			//std::cout << "MAJOR LAZER!!!\n";
			//serial_send("13\n", 3, h);
			
			//Sleep(50);
			if (waypoint_array[2] < 0) {
				position_threshold = laser_threshold;

				//std::cout << "Reached laser stage.\n";
			}

			break;

			/*
			if (abs(position_error) > position_threshold) {
				if (waypoint_array[2] > 0) {
					//serial_send("2", 1, h1);
					Sleep(10);

					//std::cout << "Sent forward command.\n\n";
				}
				else {
					if (waypoint_array[2] > 0) {
						remove_current_waypoint(waypoint_array, array_size);
					}

					if (abs(position_error) < laser_threshold) {
						//serial_send("13", 2, h1);
						Sleep(100);

						remove_current_waypoint(waypoint_array, array_size);

						//std::cout << "Sent laser command.\n\n";
					}
					else {
						//serial_send("2", 1, h1);
						Sleep(10);

						//std::cout << "Sent forward command.\n\n";
					}
				}
			}
			*/
		}

		

		/*
		while ( abs(direction_error) > 0.1 ) {
			std::cout << centroid_array.ic[1] << centroid_array.jc[1]<<centroid_array.ic[2] << centroid_array.jc[2];
			if (direction_error < 0) {
				//serial_send("9", 1, h1);
				Sleep(100);

			//	osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
			//	WriteFile(hCom, serial_instruction, 1, &dwWritten, &osWrite);

				std::cout << "Sent turn left command.\n\n";
			}
			else {
				//serial_send("10", 2, h1);
				Sleep(100);

				std::cout << "Sent turn right command.\n\n";
			}

			our_vehicle.update_position_orientation();
			enemy_vehicle.update_position_orientation();
			direction_error = waypoint_direction - our_vehicle.get_orientation();

			std::cout << "Updated vehicle position.\n\n";
		}


		// Move towards current waypoint
		if (waypoint_array[0] > 0) {
			position_error = (int) ((waypoint_array[0] - our_vehicle.get_center_x()) ^ 2 + (waypoint_array[1] - our_vehicle.get_center_y()) ^ 2);
		}
		else {
			position_error = 0;
		}
		//position_threshold = 2;

		while ( abs(position_error) > position_threshold) {
			if (waypoint_array[2] > 0) {
				serial_send("2", 1, h1);
				Sleep(100);
			}
			else {
				if (abs(position_error) < laser_threshold) {
					position_threshold = laser_threshold;
					serial_send("13", 2, h1);
					Sleep(100);
				}
				else {
					serial_send("2", 1, h1);
					Sleep(100);
				}
			}

			our_vehicle.update_position_orientation();
			enemy_vehicle.update_position_orientation();
			position_error = (int)((waypoint_array[0] - our_vehicle.get_center_x()) ^ 2 + (waypoint_array[1] - our_vehicle.get_center_y()) ^ 2);

		}
		*/

		// Remove reached waypoint
		//remove_current_waypoint(waypoint_array);
		
		if (waypoint_array[0] < 0) {
			std::cout << "Exiting Nav 2";
			break;
		}
	}
};