#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <cmath>
#include <Windows.h>

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )
#define PI 3.14159265

using namespace std;

#include "timer.h"
#include "image_transfer.h"
#include "vision.h"


// color filter: thresh = primary channel lower bound, ch1/ch2 = secondary channel upper bounds
// primary = BGR index of dominant channel (0=B, 1=G, 2=R)
// RGB = true for yellow (two primary channels must exceed thresh)
// hB,hG,hR = highlight color painted on matched pixels
struct ColorFilter {
	int primary;
	int thresh;
	int ch1, ch2;
	bool RGB;
	ibyte hB, hG, hR;
};

//struct for storing centroid data. To be comitted to shared memory
struct CentroidData {
	double ic_b, jc_b;
	double ic_g, jc_g;
	double ic_r, jc_r;
	double ic_y, jc_y;
	int frame;
};

//Initial Filters (user adjustable in terminal)
ColorFilter blueFilter   = { 0, 100, 150, 100, false, 255,   0,   0 }; //BGR
ColorFilter greenFilter  = { 1,  120,  110,  110, false,   0, 255,   0 }; //GBR
ColorFilter redFilter    = { 2, 200,  150,  150, false,   0,   0, 255 }; //RGB
ColorFilter yellowFilter = { 2, 200, 150,   0,  true,   0, 255, 255 }; //YB



void calc_centroid(ibyte* p0, int width, int height, ColorFilter& f, int pthresh, double& ic, double& jc)
{

//Calculates centroid location, colours all captured pixels to full intensity 
//Uses a vertical up/down check for neighbouring pixels of the same colour to manage stray pixels

	int i, j, k, q, up[6][3], down[6][3], product_up, product_down;
	double m, mi, mj;
	ibyte* p, * p_U, * p_D;
	double eps = 1.0e-10;

	// secondary channel indices
	int ca = (f.primary + 1) % 3;
	int cb = (f.primary + 2) % 3;

	//intialize moment accumulators
	m = 0; mi = 0; mj = 0;

	// per-pixel: vertical neighborhood check + color classification + moment accumulation
	for (j = 0; j < height; j++) {
		for (i = 0; i < width; i++) {

			k = i + 640 * j; //pixel calc
			p = p0 + 3 * k; //cast pointer to pixel

			product_up = 1;
			product_down = 1;

			for (q = 1; q <= pthresh; q++) {
				if (j > pthresh && j < (height - pthresh)) {
					p_U = p + 3 * 640 * q; //Casting pointer to pixel above
					p_D = p - 3 * 640 * q; //Casting pointer to pixel below
				}
				else if (j <= pthresh) {
					p_U = p + 3 * 640 * q;
					p_D = p;
				}
				else {
					p_U = p;
					p_D = p - 3 * 640 * q;
				}

				// vertical neighbour check: R,G,B 
				// if neighbouring pixel below or above matches filtered colour return true
				//if RGB true filtering for red green or blue
				if (!f.RGB) {			
					up[q][0] = *(p_U + f.primary) > f.thresh;
					up[q][1] = *(p_U + ca) < f.ch1;
					up[q][2] = *(p_U + cb) < f.ch2;

					down[q][0] = *(p_D + f.primary) > f.thresh;
					down[q][1] = *(p_D + ca) < f.ch1;
					down[q][2] = *(p_D + cb) < f.ch2;
				}
				else {
					// vertical neighbour check for non RGB colour (eg. Yellow)
					// both R and G must exceed thresh, B must be below ch1

					up[q][0] = *(p_U + 2) > f.thresh; 
					up[q][1] = *(p_U + 1) > f.thresh; 
					up[q][2] = *(p_U)     < f.ch1;    

					down[q][0] = *(p_D + 2) > f.thresh;
					down[q][1] = *(p_D + 1) > f.thresh;
					down[q][2] = *(p_D)     < f.ch1;
				}

				//check for pixel match up/down
				product_up   *= up[q][0]   * up[q][1]   * up[q][2];
				product_down *= down[q][0] * down[q][1] * down[q][2];
			}

			int B = *p, G = *(p + 1), R = *(p + 2);
			int match = 0;

			//checking pixel against thresholds
			if (!f.RGB)
				match = (*(p + f.primary) > f.thresh) && (*(p + ca) < f.ch1) && (*(p + cb) < f.ch2);
			else
				match = (R > f.thresh) && (G > f.thresh) && (B < f.ch1);

			//accept pixel if there is a neighbouring one above or below of the same colour
			if (match && (product_up == 1 || product_down == 1)) {
				*p = f.hB; *(p + 1) = f.hG; *(p + 2) = f.hR; //paint pixel to full intensity
				m  += 255; //overall moment
				mi += i * 255; //i moment
				mj += j * 255; //j moment
			}
		}
	}

	ic = mi / (m + eps); // i centroid
	jc = mj / (m + eps); //j centroid
}

//Draw circular pink marker around centroid
void draw_marker(ibyte* p0, int width, int height, int radius, double ic, double jc)
{
	int r, theta, i1, j1;
	ibyte* pm;
	int kc = (int)ic + width * (int)jc;

	if (kc < 0 || kc >= width * height) return;

	//drawing pink marker circle
	for (r = 0; r <= radius; r++) {
		for (theta = 0; theta < 360; theta++) {
			i1 = (int)(ic + r * cos(theta * PI / 180.0));
			j1 = (int)(jc + r * sin(theta * PI / 180.0));
			if (i1 >= 0 && i1 < width && j1 >= 0 && j1 < height) {
				pm = p0 + 3 * (i1 + 640 * j1);
				*pm = 180; *(pm + 1) = 105; *(pm + 2) = 255;
			}
		}
	}
}


int main()
{
	//console output
	//AllocConsole();
	//freopen("CONOUT$", "w", stdout);

	int radius, width, height, size, cam_number, pthresh;
	double ic_b, jc_b, ic_g, jc_g, ic_r, jc_r, ic_y, jc_y, signal, ic_arr[4], jc_arr[4];
	image rgb1, rgb2, rgb3;
	ibyte* p0;

	//shared memory setup
	HANDLE hMapFile = CreateFileMapping(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, sizeof(CentroidData), "CentroidSharedMem"); //Creating mapping object CentroidSharedMem
	if (hMapFile == NULL) { cout << "Failed to create shared memory\n"; return 1; } 
	CentroidData* sharedData = (CentroidData*)MapViewOfFile(hMapFile, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(CentroidData)); //returns void pointer to memory address. Void pointer is cast to the CentroidData struct type
	if (sharedData == NULL) { CloseHandle(hMapFile); cout << "Failed to map shared memory\n"; return 1; }
	sharedData->frame = 0;

	activate_vision();

	cout << "\npress space key to acquire image\n";
	while (!KEY(VK_SPACE));

	cam_number = 0; //Adjust this for your camera
	width = 640;
	height = 480;
	radius = 3; //Centroid marker radius (pixels)

	activate_camera(cam_number, height, width);

	rgb1.width = width;  rgb1.height = height;  rgb1.type = RGB_IMAGE;
	rgb2.width = width;  rgb2.height = height;  rgb2.type = RGB_IMAGE;
	rgb3.width = width;  rgb3.height = height;  rgb3.type = RGB_IMAGE;

	allocate_image(rgb1);
	allocate_image(rgb2);
	allocate_image(rgb3);

	size = width * height;

	//pixel filtering (user adj.) 
	//sets the number of pixels for the vertical neighbouring pixel search in the centroid search function
	pthresh = 1;

	bool bHeld = false, gHeld = false, rHeld = false, yHeld = false;

	while (1) {

		double t_start = high_resolution_time(); //loop timer

		//blue threshold user adjustment
		if (KEY('B') && !bHeld) {
			bHeld = true;
			char channel; int val;
			cout << "Adjust blue filter - enter channel (b/g/r): ";
			cin >> channel;
			cout << "Enter value (0-255): ";
			cin >> val;
			if (val >= 0 && val <= 255) {
				if      (channel == 'b') blueFilter.thresh = val;
				else if (channel == 'g') blueFilter.ch1    = val;
				else if (channel == 'r') blueFilter.ch2    = val;
				else cout << "Invalid channel, rejected\n";
			}
			else cout << "Out of range, rejected\n";
		}
		if (!KEY('B')) bHeld = false;

		//green threshold user adjustment
		if (KEY('G') && !gHeld) {
			gHeld = true;
			char channel; int val;
			cout << "Adjust green filter - enter channel (b/g/r): ";
			cin >> channel;
			cout << "Enter value (0-255): ";
			cin >> val;
			if (val >= 0 && val <= 255) {
				if      (channel == 'g') greenFilter.thresh = val;
				else if (channel == 'b') greenFilter.ch1    = val;
				else if (channel == 'r') greenFilter.ch2    = val;
				else cout << "Invalid channel, rejected\n";
			}
			else cout << "Out of range, rejected\n";
		}
		if (!KEY('G')) gHeld = false;

		//red threshold user adjustment
		if (KEY('R') && !rHeld) {
			rHeld = true;
			char channel; int val;
			cout << "Adjust red filter - enter channel (r/g/b): ";
			cin >> channel;
			cout << "Enter value (0-255): ";
			cin >> val;
			if (val >= 0 && val <= 255) {
				if      (channel == 'r') redFilter.thresh = val;
				else if (channel == 'g') redFilter.ch1    = val;
				else if (channel == 'b') redFilter.ch2    = val;
				else cout << "Invalid channel, rejected\n";
			}
			else cout << "Out of range, rejected\n";
		}
		if (!KEY('R')) rHeld = false;

		//yellow threshold user adjustment
		if (KEY('Y') && !yHeld) {
			yHeld = true;
			char channel; int val;
			cout << "Adjust yellow filter - enter channel (y/b): ";
			cin >> channel;
			cout << "Enter value (0-255): ";
			cin >> val;
			if (val >= 0 && val <= 255) {
				if      (channel == 'y') yellowFilter.thresh = val;
				else if (channel == 'b') yellowFilter.ch1    = val;
				else cout << "Invalid channel, rejected\n";
			}
			else cout << "Out of range, rejected\n";
		}
		if (!KEY('Y')) yHeld = false;

		//pixel user adjustment
		if (KEY('P')) {
			int newPthresh;
			cout << "Enter pixel threshold (0-255): ";
			cin >> newPthresh;
			if (newPthresh >= 0 && newPthresh <= 255)
				pthresh = newPthresh;
			else
				cout << "Out of range, rejected\n";
		}

		acquire_image(rgb1, cam_number);
		scale(rgb1, rgb2); //scaling image output
		copy(rgb2, rgb3); //copy to rgb3 for inspection 
		p0 = rgb2.pdata; //cast pointer to rgb2

		calc_centroid(p0, width, height, blueFilter,   pthresh, ic_b, jc_b);
		calc_centroid(p0, width, height, greenFilter,  pthresh, ic_g, jc_g);
		calc_centroid(p0, width, height, redFilter,    pthresh, ic_r, jc_r);
		calc_centroid(p0, width, height, yellowFilter, pthresh, ic_y, jc_y);

		draw_marker(p0, width, height, radius, ic_b, jc_b);
		draw_marker(p0, width, height, radius, ic_g, jc_g);
		draw_marker(p0, width, height, radius, ic_r, jc_r);
		draw_marker(p0, width, height, radius, ic_y, jc_y);

		//commit centroids to shared memory
		sharedData->ic_b = ic_b; sharedData->jc_b = jc_b;
		sharedData->ic_g = ic_g; sharedData->jc_g = jc_g;
		sharedData->ic_r = ic_r; sharedData->jc_r = jc_r;
		sharedData->ic_y = ic_y; sharedData->jc_y = jc_y;
		sharedData->frame++;

		//Array storage for track.cpp
		//0-blue, 1-green,2-red,3-yellow
		ic_arr[0] = ic_b; jc_arr[0] = jc_b;
		ic_arr[1] = ic_g; jc_arr[1] = jc_g;
		ic_arr[2] = ic_r; jc_arr[2] = jc_r;
		ic_arr[3] = ic_y; jc_arr[3] = jc_y;

		double loop_time = high_resolution_time() - t_start; //timer

		view_rgb_image(rgb2);

		if (KEY(VK_RETURN))
			cout << "\n--- centroids ---"
			     << "\n  blue:   ic=" << ic_b << " jc=" << jc_b
			     << "\n  green:  ic=" << ic_g << " jc=" << jc_g
			     << "\n  red:    ic=" << ic_r << " jc=" << jc_r
			     << "\n  yellow: ic=" << ic_y << " jc=" << jc_y
			     << "\n loop time=" << loop_time
			     << "\n--- blue filter ---"
			     << "\n  b(thresh)=" << blueFilter.thresh << "  g(ch1)=" << blueFilter.ch1 << "  r(ch2)=" << blueFilter.ch2
			     << "\n--- green filter ---"
			     << "\n  g(thresh)=" << greenFilter.thresh << "  b(ch1)=" << greenFilter.ch1 << "  r(ch2)=" << greenFilter.ch2
			     << "\n--- red filter ---"
			     << "\n  r(thresh)=" << redFilter.thresh << "  g(ch1)=" << redFilter.ch1 << "  b(ch2)=" << redFilter.ch2
			     << "\n--- yellow filter ---"
			     << "\n  y(thresh)=" << yellowFilter.thresh << "  b(ch1)=" << yellowFilter.ch1
			     << "\n--- pixel threshold ---"
			     << "\n  pthresh=" << pthresh << "\n";

		if (KEY('X')) break;
	}

	save_rgb_image("rgb1.bmp", rgb1);
	save_rgb_image("rgb3.bmp", rgb3);
	free_image(rgb2);
	deactivate_vision();

	//release shared memory
	UnmapViewOfFile(sharedData);
	CloseHandle(hMapFile);

	cout << "\n\ndone.\n";
	return 0;
}
