#include <cmath>

#include "world_map.h"
#include "obstacle.h"
#include "vehicle.h"
//#include "cost_map.h"
#include "serial_com.h"

// For Serial COM Port
// From https://learn.microsoft.com/en-us/windows/win32/devio/configuring-a-communications-resource

#include <windows.h>
#include <tchar.h>
#include <stdio.h>
#include <iostream>

// Centroid index values for centroid array

static const int RED_CENTROID_INDEX        =  0;
static const int GREEN_CENTROID_INDEX      =  2;
static const int BLUE_CENTROID_INDEX       =  4;
static const int YELLOW_CENTROID_INDEX     =  6;
static const int OBSTACLE_1_CENTROID_INDEX =  8;
static const int OBSTACLE_2_CENTROID_INDEX = 10;
static const int OBSTACLE_3_CENTROID_INDEX = 12;

int* waypoint_array = new int[14];
int new_x, new_y, position_error, position_threshold = 2, laser_threshold = 10;

double waypoint_direction, direction_error, obstacle_direction, obstacle_perpendicular;
//char *serial_instruction = new char[5];

// Centroid pointers
int *control_front_x, *control_front_y, *control_back_x, *control_back_y;
int   *enemy_front_x,   *enemy_front_y,   *enemy_back_x,   *enemy_back_y;
int default_centroid_position = -10;

obstacle obstacle_1(&default_centroid_position, &default_centroid_position, 5, 1);
obstacle obstacle_2(&default_centroid_position, &default_centroid_position, 5, 1);
obstacle obstacle_3(&default_centroid_position, &default_centroid_position, 5, 1);

// Serial
HANDLE h1;
int speed = 0;

// Need to include passthrough for keyboard checks?
void mapper(int *centroid_array, int width, int height) {

	// Accept centroid array
	// Create vehicle and obstacle objects
	// Develop waypoint buffer
	// Find path through waypoint buffer, update as needed
	std::cout << "Centroid Array" << centroid_array;
	
	// Initial setup, only run once:

	/*
	// Serial Port setup

	DCB dcb;
	HANDLE hCom;
	BOOL fSuccess;
	TCHAR* pcCommPort = TEXT("COM1");

	OVERLAPPED osWrite = { 0 };
	DWORD dwWritten;
	DWORD dwRes;
	BOOL fRes;
	*/

	//std::cout << "Created serial port configuration variables\n\n";

	//  Open a handle to the specified com port.

	/*
	hCom = CreateFile(pcCommPort,
		GENERIC_READ | GENERIC_WRITE,
		0,      //  must be opened with exclusive-access
		NULL,   //  default security attributes
		OPEN_EXISTING, //  must use OPEN_EXISTING
		0,      //  not overlapped I/O
		NULL); //  hTemplate must be NULL for comm devices

	if (hCom == INVALID_HANDLE_VALUE)
	{
		//  Handle the error.
		printf("CreateFile failed with error %d.\n", GetLastError());
		//return (1);
	}

	//  Initialize the DCB structure.
	SecureZeroMemory(&dcb, sizeof(DCB));
	dcb.DCBlength = sizeof(DCB);

	//  Build on the current configuration by first retrieving all current
	//  settings.
	fSuccess = GetCommState(hCom, &dcb);

	if (!fSuccess)
	{
		//  Handle the error.
		printf("GetCommState failed with error %d.\n", GetLastError());
		//return (2);
	}

	dcb.BaudRate = CBR_57600;     //  baud rate
	dcb.ByteSize = 8;             //  data size, xmit and rcv
	dcb.Parity = NOPARITY;      //  parity bit
	dcb.StopBits = ONESTOPBIT;    //  stop bit

	fSuccess = SetCommState(hCom, &dcb);

	if (!fSuccess)
	{
		//  Handle the error.
		printf("SetCommState failed with error %d.\n", GetLastError());
		//return (3);
	}*/

	// Open serial port
	open_serial("COM5", h1, speed);


	//// Our robot

	control_front_x = centroid_array + RED_CENTROID_INDEX;
	control_front_y = centroid_array + RED_CENTROID_INDEX + 1;
	control_back_x = centroid_array + GREEN_CENTROID_INDEX;
	control_back_y = centroid_array + GREEN_CENTROID_INDEX + 1;

	std::cout << "Something\n\n";

	vehicle our_vehicle(control_front_x, control_front_y, control_back_x, control_back_y, 9, 1);

	std::cout << "Created friendly robot vehicle object\n\n";

	//// Enemy robot

	enemy_front_x = centroid_array + BLUE_CENTROID_INDEX;
	enemy_front_y = centroid_array + BLUE_CENTROID_INDEX + 1;
	enemy_back_x = centroid_array + YELLOW_CENTROID_INDEX;
	enemy_back_y = centroid_array + YELLOW_CENTROID_INDEX + 1;

	vehicle enemy_vehicle(enemy_front_x, enemy_front_y, enemy_back_x, enemy_back_y, 9, 1);

	std::cout << "Created enemy robot vehicle object\n\n";

	//// Obstacle Initialization
	if ((centroid_array[OBSTACLE_1_CENTROID_INDEX] > 0) && (centroid_array[OBSTACLE_1_CENTROID_INDEX + 1] > 0)) {
		obstacle_1.set_center_x(centroid_array + OBSTACLE_1_CENTROID_INDEX);
		obstacle_1.set_center_y(centroid_array + OBSTACLE_1_CENTROID_INDEX + 1);//, centroid_array + OBSTACLE_1_CENTROID_INDEX + 1, 4, 1);

		if ((centroid_array[OBSTACLE_2_CENTROID_INDEX] > 0) && (centroid_array[OBSTACLE_2_CENTROID_INDEX + 1] > 0)) {
			//static obstacle obstacle_2(centroid_array + OBSTACLE_2_CENTROID_INDEX, centroid_array + OBSTACLE_2_CENTROID_INDEX + 1, 4, 1);
			obstacle_2.set_center_x(centroid_array + OBSTACLE_2_CENTROID_INDEX);
			obstacle_2.set_center_y(centroid_array + OBSTACLE_2_CENTROID_INDEX + 1);

			if ((centroid_array[OBSTACLE_3_CENTROID_INDEX] > 0) && (centroid_array[OBSTACLE_3_CENTROID_INDEX + 1] > 0)) {
				//static obstacle obstacle_3(centroid_array + OBSTACLE_2_CENTROID_INDEX, centroid_array + OBSTACLE_3_CENTROID_INDEX + 1, 4, 1);
				obstacle_3.set_center_x(centroid_array + OBSTACLE_3_CENTROID_INDEX);
				obstacle_3.set_center_y(centroid_array + OBSTACLE_3_CENTROID_INDEX + 1);

//				static cost_map map(width, height, &our_vehicle, &enemy_vehicle, &obstacle_1, &obstacle_2, &obstacle_3);
			}
			else {
//				static cost_map map(width, height, &our_vehicle, &enemy_vehicle, &obstacle_1, &obstacle_2);
			}
		}
		else {
			if ((centroid_array[OBSTACLE_3_CENTROID_INDEX] > 0) && (centroid_array[OBSTACLE_3_CENTROID_INDEX + 1] > 0)) {
				//static obstacle obstacle_3(centroid_array + OBSTACLE_2_CENTROID_INDEX, centroid_array + OBSTACLE_3_CENTROID_INDEX + 1, 4, 1);
				obstacle_3.set_center_x(centroid_array + OBSTACLE_3_CENTROID_INDEX);
				obstacle_3.set_center_y(centroid_array + OBSTACLE_3_CENTROID_INDEX + 1);

//				static cost_map map(width, height, &our_vehicle, &enemy_vehicle, &obstacle_1, &obstacle_3);
			} else {
//				static cost_map map(width, height, &our_vehicle, &enemy_vehicle, &obstacle_1);
			}
		}
	} else {
		if ((centroid_array[OBSTACLE_2_CENTROID_INDEX] > 0) && (centroid_array[OBSTACLE_2_CENTROID_INDEX + 1] > 0)) {
			//static obstacle obstacle_2(centroid_array + OBSTACLE_2_CENTROID_INDEX, centroid_array + OBSTACLE_2_CENTROID_INDEX + 1, 4, 1);
			obstacle_2.set_center_x(centroid_array + OBSTACLE_2_CENTROID_INDEX);
			obstacle_2.set_center_y(centroid_array + OBSTACLE_2_CENTROID_INDEX + 1);

			if ((centroid_array[OBSTACLE_3_CENTROID_INDEX] > 0) && (centroid_array[OBSTACLE_3_CENTROID_INDEX + 1] > 0)) {
				//static obstacle obstacle_3(centroid_array + OBSTACLE_2_CENTROID_INDEX, centroid_array + OBSTACLE_3_CENTROID_INDEX + 1, 4, 1);
				obstacle_3.set_center_x(centroid_array + OBSTACLE_3_CENTROID_INDEX);
				obstacle_3.set_center_y(centroid_array + OBSTACLE_3_CENTROID_INDEX + 1);

//				static cost_map map(width, height, &our_vehicle, &enemy_vehicle, &obstacle_2, &obstacle_3);
			}
			else {
//				static cost_map map(width, height, &our_vehicle, &enemy_vehicle, &obstacle_2);
			}
		}
		else {
			if ((centroid_array[OBSTACLE_3_CENTROID_INDEX] > 0) && (centroid_array[OBSTACLE_3_CENTROID_INDEX + 1] > 0)) {
				//static obstacle obstacle_3(centroid_array + OBSTACLE_2_CENTROID_INDEX, centroid_array + OBSTACLE_3_CENTROID_INDEX + 1, 4, 1);
				obstacle_3.set_center_x(centroid_array + OBSTACLE_3_CENTROID_INDEX);
				obstacle_3.set_center_y(centroid_array + OBSTACLE_3_CENTROID_INDEX + 1);

//				static cost_map map(width, height, &our_vehicle, &enemy_vehicle, &obstacle_3);
			}
			else {
//				static cost_map map(width, height, &our_vehicle, &enemy_vehicle);
			}
		}
	}
	
	// Load target coordinates into buffer
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
	
	waypoint_direction = atan2((waypoint_array[1] - our_vehicle.get_vehicle_center_y()), (waypoint_array[1]) - our_vehicle.get_vehicle_center_x());
	//waypoint_direction = atan2((centroid_array[BLUE_CENTROID_INDEX + 1] - our_vehicle.get_vehicle_center_y()), (centroid_array[BLUE_CENTROID_INDEX] - our_vehicle.get_vehicle_center_x()));

	// while loop to iterate through waypoints
	while (true) {
		// Check direct path for obstacles
		// // Using formula from: https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
		// // 
		// // waypoint_direction = atan2((waypoint_array[1] - our_vehicle.get_vehicle_center_y()), (waypoint_array[1]) - our_vehicle.get_vehicle_center_x());
		if(obstacle_1.get_center_x() >= 0) {
			if ( ( (waypoint_array[1] - our_vehicle.get_vehicle_center_y() )*obstacle_1.get_center_x() - (waypoint_array[0] - our_vehicle.get_vehicle_center_x())*obstacle_1.get_center_y() + waypoint_array[0]*our_vehicle.get_vehicle_center_y() - waypoint_array[1] - our_vehicle.get_vehicle_center_x())^2 < ((waypoint_array[1] - our_vehicle.get_vehicle_center_y())^2 - (waypoint_array[0] - our_vehicle.get_vehicle_center_x())^2 )*(obstacle_1.obstacle_radius)^2 ) {
				// Path intersects with keep-out zone, need to find new waypoint
				obstacle_direction = atan2((obstacle_1.get_center_y() - our_vehicle.get_vehicle_center_y()), (obstacle_1.get_center_x()) - our_vehicle.get_vehicle_center_x());
				if(waypoint_direction > obstacle_direction) {
					obstacle_perpendicular = obstacle_direction + 3.14159/2;
					new_x = (int) obstacle_1.obstacle_radius() * cos(obstacle_perpendicular);
					new_y = (int) obstacle_1.obstacle_radius() * sin(obstacle_perpendicular);
					waypoint_array[13] = waypoint_array[12];
					waypoint_array[12] = waypoint_array[11];
					waypoint_array[11] = waypoint_array[10];
					waypoint_array[10] = waypoint_array[9];
					waypoint_array[9]  = waypoint_array[8];
					waypoint_array[8]  = waypoint_array[7];
					waypoint_array[7]  = waypoint_array[6];
					waypoint_array[6]  = waypoint_array[5];
					waypoint_array[5]  = waypoint_array[4];
					waypoint_array[4]  = waypoint_array[3];
					waypoint_array[3]  = waypoint_array[2];
					waypoint_array[2]  = waypoint_array[1];
					waypoint_array[1]  = new_y;
					waypoint_array[0]  = new_x;
				} else {
					obstacle_perpendicular = obstacle_direction - 3.14159 / 2;
					new_x = (int)obstacle_1.obstacle_radius() * cos(obstacle_perpendicular);
					new_y = (int)obstacle_1.obstacle_radius() * sin(obstacle_perpendicular);
					waypoint_array[13] = waypoint_array[12];
					waypoint_array[12] = waypoint_array[11];
					waypoint_array[11] = waypoint_array[10];
					waypoint_array[10] = waypoint_array[9];
					waypoint_array[9] = waypoint_array[8];
					waypoint_array[8] = waypoint_array[7];
					waypoint_array[7] = waypoint_array[6];
					waypoint_array[6] = waypoint_array[5];
					waypoint_array[5] = waypoint_array[4];
					waypoint_array[4] = waypoint_array[3];
					waypoint_array[3] = waypoint_array[2];
					waypoint_array[2] = waypoint_array[1];
					waypoint_array[1] = new_y;
					waypoint_array[0] = new_x;
				}
			}
		}
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
		// Turn towards current waypoint
		direction_error = waypoint_direction - our_vehicle.get_orientation();
		while ( abs(direction_error) > 0.1 ) {
			if (direction_error < 0) {
				serial_send("9", 1, h1);
				Sleep(100);

			//	osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
			//	WriteFile(hCom, serial_instruction, 1, &dwWritten, &osWrite);

				std::cout << "Sent turn left command.\n\n";
			}
			else {
				serial_send("10", 2, h1);
				Sleep(100);

			//	osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
			//	WriteFile(hCom, serial_instruction, 2, &dwWritten, &osWrite);

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
			break;
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

		// Remove reached waypoint
		waypoint_array[0]  = waypoint_array[2];
		waypoint_array[1]  = waypoint_array[3];
		waypoint_array[2]  = waypoint_array[4];
		waypoint_array[3]  = waypoint_array[5];
		waypoint_array[4]  = waypoint_array[6];
		waypoint_array[5]  = waypoint_array[7];
		waypoint_array[6]  = waypoint_array[8];
		waypoint_array[7]  = waypoint_array[9];
		waypoint_array[8]  = waypoint_array[10];
		waypoint_array[9]  = waypoint_array[11];
		waypoint_array[10] = waypoint_array[12];
		waypoint_array[11] = waypoint_array[13];
		waypoint_array[12] = -10;
		waypoint_array[13] = -10;
		
		if (waypoint_array[0] < 0) {
			break;
		}

		// // If final waypoint (i.e. the target), and within range:
		// // // Fire laser
		// // If not final waypoint, and within tolerance:
		// // // Remove waypoint
	}

	//direction_error = waypoint_direction - our_vehicle.get_orientation();

	
	

	// Orient robot towards enemy if needed (center them along the robot orientation)
	// Calculate path to enemy
	// // Accept/Reject path: check for obstacle intersections
	// // // Need to check each obstacle that exists
	// // This checks if the obstacle is close enough to interfere with the robot moving in a straight line to the target

/*	if (((centroid_array[BLUE_CENTROID_INDEX + 1] - our_vehicle.get_vehicle_center_y()) * obstacle_1.get_center_x() - (centroid_array[BLUE_CENTROID_INDEX] - our_vehicle.get_vehicle_center_x()) * obstacle_1.get_center_y() + centroid_array[BLUE_CENTROID_INDEX] * our_vehicle->get_vehicle_center_y() - centroid_array[BLUE_CENTROID_INDEX + 1] - our_vehicle->get_vehicle_center_x()) ^ 2 < ((centroid_array[BLUE_CENTROID_INDEX + 1] - our_vehicle->get_vehicle_center_y()) ^ 2 - (centroid_array[BLUE_CENTROID_INDEX] - our_vehicle->get_vehicle_center_x()) ^ 2) * (obstacle_1.obstacle_radius ^ 2) ^ 2) {
		// Go to point on edge of keep-out zone
		double obstacle_direction = atan2((obstacle_1.get_center_y() - our_vehicle.get_vehicle_center_y()), (obstacle_1.get_center_x() - our_vehicle.get_vehicle_center_x()));
		if ( (waypoint_direction - obstacle_direction) < 0) {
			// Find point along obstacle radius that is perpendicular to enemy_direction
		}
		else {
			// Find other point along obstacle radius that is perpendicular to enemy_direction
		}
	} */

	// Send commands to robot over Bluetooth
};