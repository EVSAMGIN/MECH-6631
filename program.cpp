#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <cmath>
#include <Windows.h>

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

using namespace std;

#include "timer.h"
#include "image_transfer.h"
#include "vision.h"
#include "centroid.h"
#include "world_map.h"
#include <thread>

int* centroid_array_for_control = new int[14];

int main()
{
	AllocConsole();
	freopen("CONOUT$", "w", stdout);

	int radius, width, height, cam_number, nlabels, tvalue = 70, min_label_size = 200;
	bool upHeld = false, downHeld = false, leftHeld = false, rightHeld = false;
	double ic_1, jc_1, ic_2, jc_2, ic_3, jc_3, ic_4, jc_4;
	image rgb1, rgb2,rgb3, gscale1, gscale2, label;

	// Shared memory setup
	HANDLE hMapFile = CreateFileMapping(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, sizeof(CentroidData), "CentroidSharedMem");
	if (hMapFile == NULL) { cout << "Failed to create shared memory\n"; return 1; }
	CentroidData* sharedData = (CentroidData*)MapViewOfFile(hMapFile, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(CentroidData));
	if (sharedData == NULL) { CloseHandle(hMapFile); cout << "Failed to map shared memory\n"; return 1; }
	sharedData->frame = 0;

	activate_vision();

	cout << "\npress space key to acquire image\n";
	while (!KEY(VK_SPACE));

	cam_number = 0;
	width  = 640;
	height = 480;
	radius = 3;

	activate_camera(cam_number, height, width);

	rgb1.width   = width;  rgb1.height   = height;  rgb1.type   = RGB_IMAGE;
	rgb2.width   = width;  rgb2.height   = height;  rgb2.type   = RGB_IMAGE;
	rgb3.width = width;  rgb3.height = height;  rgb3.type = RGB_IMAGE;
	gscale1.width = width; gscale1.height = height; gscale1.type = GREY_IMAGE;
	gscale2.width = width; gscale2.height = height; gscale2.type = GREY_IMAGE;
	label.type   = LABEL_IMAGE; label.width = width; label.height = height;

	allocate_image(rgb1);
	allocate_image(rgb2);
	allocate_image(rgb3);
	allocate_image(gscale1);
	allocate_image(gscale2);
	allocate_image(label);

	thread control_thread( mapper, centroid_array_for_control, width, height );

	control_thread.detach();

	while (1) {

		double t_start = high_resolution_time();

		acquire_image(rgb1, cam_number);
		scale(rgb1, rgb2);
		copy(rgb2, rgb3); //For inspection
		copy(rgb2, gscale1);  // RGB -> greyscale
		lowpass_filter(gscale1, gscale2);
		highpass_filter(gscale2, gscale1);
		copy(gscale1, rgb1); //Filtered image copied for inspection later

		// Up/down arrow: adjust threshold (1-254)
		if (KEY(VK_UP) && !upHeld)   { if (tvalue < 254) tvalue++; cout << "  threshold=" << tvalue << "\n"; upHeld   = true; }
		if (KEY(VK_DOWN) && !downHeld) { if (tvalue >   1) tvalue--; cout << "  threshold=" << tvalue << "\n"; downHeld = true; }
		if (!KEY(VK_UP))   upHeld   = false;
		if (!KEY(VK_DOWN)) downHeld = false;
		if (KEY(VK_RIGHT) && !rightHeld) { if (min_label_size < 10000) min_label_size++; cout << "  min_label_size=" << min_label_size << "\n"; rightHeld = true; }
		if (KEY(VK_LEFT)  && !leftHeld)  { if (min_label_size >     1) min_label_size--; cout << "  min_label_size=" << min_label_size << "\n"; leftHeld  = true; }
		if (!KEY(VK_RIGHT)) rightHeld = false;
		if (!KEY(VK_LEFT))  leftHeld  = false;

		threshold(gscale1, gscale2, tvalue);
		invert(gscale2, gscale2);
		label_image(gscale2, label, nlabels);
		copy(gscale2, rgb2); //for inspection

		// Find first 4 labels meeting min_label_size and draw their centroids
		i2byte *pl = (i2byte *)label.pdata;
		double ic_arr[4] = {}, jc_arr[4] = {};
		int found = 0;
		for (int n = 1; n <= nlabels && found < 4; n++) {
			int c = 0;
			for (int j = 0; j < height; j++) {
				for (int i = 0; i < width; i++) {
					if (pl[i + width * j] == n) c++;
				}
			}
			if (c >= min_label_size) {
				centroid(gscale2, label, n, ic_arr[found], jc_arr[found]);
				draw_marker(rgb2.pdata, width, height, radius, ic_arr[found], jc_arr[found]);
				found++;
			}
		}
		ic_1 = ic_arr[0]; jc_1 = jc_arr[0];
		ic_2 = ic_arr[1]; jc_2 = jc_arr[1];
		ic_3 = ic_arr[2]; jc_3 = jc_arr[2];
		ic_4 = ic_arr[3]; jc_4 = jc_arr[3];

		centroid_array_for_control[0] = (int) ic_arr[0] + 0.5;
		centroid_array_for_control[1] = (int) jc_arr[0] + 0.5;
		centroid_array_for_control[2] = (int) ic_arr[1] + 0.5;
		centroid_array_for_control[3] = (int) jc_arr[1] + 0.5;
		centroid_array_for_control[4] = (int) ic_arr[2] + 0.5;
		centroid_array_for_control[5] = (int) jc_arr[2] + 0.5;
		centroid_array_for_control[6] = (int) ic_arr[3] + 0.5;
		centroid_array_for_control[7] = (int) jc_arr[3] + 0.5;

		// Commit centroids to shared memory
		sharedData->ic_1 = ic_1; sharedData->jc_1 = jc_1;
		sharedData->ic_2 = ic_2; sharedData->jc_2 = jc_2;
		sharedData->ic_3 = ic_3; sharedData->jc_3 = jc_3;
		sharedData->ic_4 = ic_4; sharedData->jc_4 = jc_4;
		sharedData->frame++;

		double loop_time = high_resolution_time() - t_start;

		view_rgb_image(rgb2);

		if (KEY(VK_RETURN))
			cout << "\n--- centroids ---"
			     << "\n  1: ic=" << ic_1 << " jc=" << jc_1
			     << "\n  2: ic=" << ic_2 << " jc=" << jc_2
			     << "\n  3: ic=" << ic_3 << " jc=" << jc_3
			     << "\n  4: ic=" << ic_4 << " jc=" << jc_4
			     << "\n  loop time=" << loop_time
			     << "\n  nlabels=" << nlabels
		     << "\n  threshold=" << tvalue << "\n";

		if (KEY('X')) break;
	}

	save_rgb_image("rgb1.bmp", rgb1);
	save_rgb_image("gscale1.bmp", rgb2);
	save_rgb_image("rgb3.bmp", rgb3);
	free_image(rgb1);
	free_image(rgb2);
	free_image(gscale1);
	free_image(gscale2);
	free_image(label);
	deactivate_vision();

	UnmapViewOfFile(sharedData);
	CloseHandle(hMapFile);

	cout << "\n\ndone.\n";
	return 0;
}
