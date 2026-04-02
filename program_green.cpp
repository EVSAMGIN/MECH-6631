
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

	int i, j, k, k1,k2, i1, j1,i2,j2, r, radius, theta, width, height, size, cam_number, M, kc,kc2;
	double mi, mj,mi2,mj2,m2, m, ic, jc,ic2,jc2, eps;
	image rgb1;
	ibyte *p, * p0, * pc, *pr,*pg, R, G, B;

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

		// FIXED: Initialize moment accumulators for each frame
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

				// FIXED: Changed B > 255 to B > 50 (255 is impossible for ibyte)
				if ((B > 130) && (R < 120) && (G < 120)) {

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
				if ((B < 130) && (R < 120) && (G > 120)) {
					R = 0;
					G = 255;
					B = 0;

					// highlight green pixels in the image
					*p = B;
					*(p + 1) = G;
					*(p + 2) = R;

					m2 += G;

					mi += i * G;
					mj += j * G;

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

		double theta1, theta2;
		compute_relative_angle(ic, jc, ic2, jc2, theta1, theta2);

		cout << "\ntheta1 = " << theta1 << " , theta2 = " << theta2;

		view_rgb_image(rgb1);

		cout << "\nic = " << ic << " , jc = " << jc;
		if (KEY('X')) break;

	}

	free_image(rgb1);
	deactivate_vision();

	cout << "\n \n done.\n";

	return 0;
}





