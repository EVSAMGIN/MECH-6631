#include <cmath>

#include "world_map.h"
#include "obstacle.h"
#include "vehicle.h"
#include "cost_map.h"

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

double enemy_direction, direction_error;
char *serial_instruction = new char[5];

void mapper(int *centroid_array, int width, int height) {
	
	// Initial setup, only run once:

	// Serial Port setup

	DCB dcb;
	HANDLE hCom;
	BOOL fSuccess;
	TCHAR* pcCommPort = TEXT("COM1");

	OVERLAPPED osWrite = { 0 };
	DWORD dwWritten;
	DWORD dwRes;
	BOOL fRes;

	std::cout << "Created serial port configuration variables\n\n";

	//  Open a handle to the specified com port.
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
	}


	//// Our robot

	static int *control_front_x = centroid_array + RED_CENTROID_INDEX;
	static int *control_front_y = centroid_array + RED_CENTROID_INDEX + 1;
	static int *control_back_x = centroid_array + GREEN_CENTROID_INDEX;
	static int *control_back_y = centroid_array + GREEN_CENTROID_INDEX + 1;

	static vehicle our_vehicle(control_front_x, control_front_y, control_back_x, control_back_y, 9, 1);

	std::cout << "Created friendly robot vehicle object\n\n";

	//// Enemy robot

	static int *enemy_front_x = centroid_array + BLUE_CENTROID_INDEX;
	static int *enemy_front_y = centroid_array + BLUE_CENTROID_INDEX + 1;
	static int *enemy_back_x = centroid_array + YELLOW_CENTROID_INDEX;
	static int *enemy_back_y = centroid_array + YELLOW_CENTROID_INDEX + 1;

	static vehicle enemy_vehicle(enemy_front_x, enemy_front_y, enemy_back_x, enemy_back_y, 9, 1);

	std::cout << "Created enemy robot vehicle object\n\n";

	//// Obstacles and Cost-Map Initialization
	/*
	if ((centroid_array[OBSTACLE_1_CENTROID_INDEX] > 0) && (centroid_array[OBSTACLE_1_CENTROID_INDEX + 1] > 0)) {
		static obstacle obstacle_1(centroid_array + OBSTACLE_1_CENTROID_INDEX, centroid_array + OBSTACLE_1_CENTROID_INDEX + 1, 4, 1);

		if ((centroid_array[OBSTACLE_2_CENTROID_INDEX] > 0) && (centroid_array[OBSTACLE_2_CENTROID_INDEX + 1] > 0)) {
			static obstacle obstacle_2(centroid_array + OBSTACLE_2_CENTROID_INDEX, centroid_array + OBSTACLE_2_CENTROID_INDEX + 1, 4, 1);

			if ((centroid_array[OBSTACLE_3_CENTROID_INDEX] > 0) && (centroid_array[OBSTACLE_3_CENTROID_INDEX + 1] > 0)) {
				static obstacle obstacle_3(centroid_array + OBSTACLE_2_CENTROID_INDEX, centroid_array + OBSTACLE_3_CENTROID_INDEX + 1, 4, 1);

				static cost_map map(width, height, &our_vehicle, &enemy_vehicle, &obstacle_1, &obstacle_2, &obstacle_3);
			}
			else {
				static cost_map map(width, height, &our_vehicle, &enemy_vehicle, &obstacle_1, &obstacle_2);
			}
		}
		else {
			if ((centroid_array[OBSTACLE_3_CENTROID_INDEX] > 0) && (centroid_array[OBSTACLE_3_CENTROID_INDEX + 1] > 0)) {
				static obstacle obstacle_3(centroid_array + OBSTACLE_2_CENTROID_INDEX, centroid_array + OBSTACLE_3_CENTROID_INDEX + 1, 4, 1);

				static cost_map map(width, height, &our_vehicle, &enemy_vehicle, &obstacle_1, &obstacle_3);
			} else {
				static cost_map map(width, height, &our_vehicle, &enemy_vehicle, &obstacle_1);
			}
		}
	} else {
		if ((centroid_array[OBSTACLE_2_CENTROID_INDEX] > 0) && (centroid_array[OBSTACLE_2_CENTROID_INDEX + 1] > 0)) {
			static obstacle obstacle_2(centroid_array + OBSTACLE_2_CENTROID_INDEX, centroid_array + OBSTACLE_2_CENTROID_INDEX + 1, 4, 1);

			if ((centroid_array[OBSTACLE_3_CENTROID_INDEX] > 0) && (centroid_array[OBSTACLE_3_CENTROID_INDEX + 1] > 0)) {
				static obstacle obstacle_3(centroid_array + OBSTACLE_2_CENTROID_INDEX, centroid_array + OBSTACLE_3_CENTROID_INDEX + 1, 4, 1);

				static cost_map map(width, height, &our_vehicle, &enemy_vehicle, &obstacle_2, &obstacle_3);
			}
			else {
				static cost_map map(width, height, &our_vehicle, &enemy_vehicle, &obstacle_2);
			}
		}
		else {
			if ((centroid_array[OBSTACLE_3_CENTROID_INDEX] > 0) && (centroid_array[OBSTACLE_3_CENTROID_INDEX + 1] > 0)) {
				static obstacle obstacle_3(centroid_array + OBSTACLE_2_CENTROID_INDEX, centroid_array + OBSTACLE_3_CENTROID_INDEX + 1, 4, 1);

				static cost_map map(width, height, &our_vehicle, &enemy_vehicle, &obstacle_3);
			}
			else {
				static cost_map map(width, height, &our_vehicle, &enemy_vehicle);
			}
		}
	}
	*/
	
	//enemy_direction = atan2((enemy_vehicle.get_center_y() - our_vehicle.get_vehicle_center_y()), (enemy_vehicle.get_center_x()) - our_vehicle.get_vehicle_center_x());
	enemy_direction = atan2((centroid_array[BLUE_CENTROID_INDEX + 1] - our_vehicle.get_vehicle_center_y()), (centroid_array[BLUE_CENTROID_INDEX] - our_vehicle.get_vehicle_center_x()));

	direction_error = enemy_direction - our_vehicle.get_orientation();

	while ( abs(direction_error) > 0.1 ) {
		if (direction_error < 0) {
			*serial_instruction = '9';

			osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
			WriteFile(hCom, serial_instruction, 1, &dwWritten, &osWrite);

			std::cout << "Sent turn left command.\n\n";
		}
		else {
			*serial_instruction = '10';

			osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
			WriteFile(hCom, serial_instruction, 2, &dwWritten, &osWrite);

			std::cout << "Sent turn right command.\n\n";
		}

		our_vehicle.update_position_orientation();
		enemy_vehicle.update_position_orientation();
		direction_error = enemy_direction - our_vehicle.get_orientation();

		std::cout << "Updated vehicle position.\n\n";
	}
	

	// Orient robot towards enemy if needed (center them along the robot orientation)
	// Calculate path to enemy
	// // Accept/Reject path: check for obstacle intersections
	// // // Check half of robot diamter lengths along path for obstacle intersections
	// // // If intersection exists, break loop and shift to the side of obstacle center
	

	// Send commands to robot over Bluetooth
};