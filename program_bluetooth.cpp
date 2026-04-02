
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <cmath>
#include <Windows.h>

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )
#define PI 3.14159265

using namespace std;

#include "timer.h"
#include "image_transfer.h"
#include "vision.h"


int main()
{
	AllocConsole();
	freopen("CONOUT$", "w", stdout);

	int i, j, k, k1, k2, i1, j1, i2, j2, r, radius, theta, width, height, size, cam_number, M, kc, kc2;
	double mi, mj, mi2, mj2, m2, m, ic, jc, ic2, jc2, eps, signal;
	image rgb1;
	ibyte* p, * p0, * pc, * pr, * pg, R, G, B, R2, G2, B2;

	// ========== SERIAL COMMUNICATION SETUP ==========
	HANDLE hSerial = CreateFile("\\\\.\\COM6", GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	if (hSerial == INVALID_HANDLE_VALUE) {
		DWORD error = GetLastError();
		cout << "Error opening COM6. Code: " << error << "\n";
		if (error == 5) cout << "Access denied - close Arduino IDE Serial Monitor\n";
		return 1;
	}

	PurgeComm(hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);
	Sleep(2000);

	DCB dcb = { 0 };
	dcb.DCBlength = sizeof(dcb);
	GetCommState(hSerial, &dcb);
	dcb.BaudRate = 115200;
	dcb.ByteSize = 8;
	dcb.StopBits = ONESTOPBIT;
	dcb.Parity = NOPARITY;
	SetCommState(hSerial, &dcb);

	COMMTIMEOUTS timeouts = { 0 };
	timeouts.ReadIntervalTimeout = 50;
	timeouts.ReadTotalTimeoutConstant = 50;
	timeouts.ReadTotalTimeoutMultiplier = 10;
	SetCommTimeouts(hSerial, &timeouts);

	cout << "COM6 opened successfully\n";
	// ==========================================================

	activate_vision();

	cout << "\npress space key to acquire image\n";
	while (!KEY(VK_SPACE));

	cam_number = 0;
	width = 640;
	height = 480;

	//red circle radius
	radius = 10;

	activate_camera(cam_number, height, width);

	rgb1.width = width;
	rgb1.height = height;
	rgb1.type = RGB_IMAGE;

	allocate_image(rgb1);

	p0 = rgb1.pdata;

	size = width * height;

	while (1) {

		// Initialize moment accumulators for each frame
		m = 0;
		mi = 0;
		mj = 0;

		m2 = 0;
		mi2 = 0;
		mj2 = 0;

		acquire_image(rgb1, cam_number);
		p = rgb1.pdata;

		for (j = 0; j < height; j++) {
			for (i = 0; i < width; i++) {

				k = i + 640 * j;
				p = p0 + 3 * k;

				B = *p;
				G = *(p + 1);
				R = *(p + 2);

				//BLUE
				if ((B > 130) && (R < 100) && (G < 100)) {

					R = 0;
					G = 0;
					B = 255;

					// highlight blue pixels in the image
					*p = B;
					*(p + 1) = G;
					*(p + 2) = R;

					m += B;

					mi += i * B;
					mj += j * B;

				}
				//GREEN
				if ((B < 100) && (R < 100) && (G > 130)) {
					R2 = 0;
					G2 = 255;
					B2 = 0;

					// highlight green pixels in the image
					*p = B2;
					*(p + 1) = G2;
					*(p + 2) = R2;

					m2 += G2;

					mi2 += i * G2;
					mj2 += j * G2;

				}

			}
		}

		eps = 1.0e-10;
		//Blue centroid
		ic = mi / (m + eps);
		jc = mj / (m + eps);
		kc = (int)ic + 640 * (int)jc;

		//Green Centroid
		ic2 = mi2 / (m2 + eps);
		jc2 = mj2 / (m2 + eps);
		kc2 = (int)ic2 + 640 * (int)jc2;


		// Red Target Circle
		if (m > 0 && kc >= 0 && kc < size) {
			for (r = 0; r <= radius; r++) {
				for (theta = 0; theta < 360; theta++) {
					i1 = (int)(ic + r * cos(theta * PI / 180.0));
					j1 = (int)(jc + r * sin(theta * PI / 180.0));
					if (i1 >= 0 && i1 < width && j1 >= 0 && j1 < height) {
						k1 = i1 + 640 * j1;
						pr = p0 + 3 * k1;
						*pr = 0;
						*(pr + 1) = 0;
						*(pr + 2) = 255;
					}
				}
			}
		}
		// Red Target Circle
		if (m2 > 0 && kc2 >= 0 && kc2 < size) {
			for (r = 0; r <= radius; r++) {
				for (theta = 0; theta < 360; theta++) {
					i2 = (int)(ic2 + r * cos(theta * PI / 180.0));
					j2 = (int)(jc2 + r * sin(theta * PI / 180.0));
					if (i2 >= 0 && i2 < width && j2 >= 0 && j2 < height) {
						k2 = i2 + 640 * j2;
						pg = p0 + 3 * k2;
						*pg = 0;
						*(pg + 1) = 0;
						*(pg + 2) = 255;
					}
				}
			}
		}

		//Angle Difference
		double theta1, theta2;
		compute_relative_angle(ic, jc, ic2, jc2, theta1, theta2);
		//Theta signal
		signal = theta1 - theta2;

		cout << "\ntheta1 = " << theta1 << " , theta2 = " << theta2 << ",signal=" << signal;

		// ========== SEND SIGNAL TO ESP32 VIA BLUETOOTH ==========
		char buffer[50];
		sprintf(buffer, "%.2f\n", signal);
		DWORD bytesWritten;
		WriteFile(hSerial, buffer, strlen(buffer), &bytesWritten, NULL);
		cout << " [Sent " << bytesWritten << " bytes: " << buffer << "]";

		// ========== READ RESPONSE FROM ESP32 ==========
		char readBuffer[256];
		DWORD bytesRead;
		if (ReadFile(hSerial, readBuffer, sizeof(readBuffer) - 1, &bytesRead, NULL) && bytesRead > 0) {
			readBuffer[bytesRead] = '\0';
			cout << "ESP32: " << readBuffer;
		}
		// ===============================================

		view_rgb_image(rgb1);

		cout << "\nic = " << ic << " , jc = " << jc;
		if (KEY('X')) break;

	}

	// ========== CLOSE BLUETOOTH PORT ==========
	CloseHandle(hSerial);
	// ==========================================
	free_image(rgb1);
	deactivate_vision();

	cout << "\n \n done.\n";

	return 0;
}
