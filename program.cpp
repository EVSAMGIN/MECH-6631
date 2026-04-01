#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <cmath>
#include <Windows.h>
#include <vector>
#include <thread>
#include <atomic>
#include <string>
#include <mutex>
#include "world_map.h"

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

using namespace std;

#include "timer.h"
#include "image_transfer.h"
#include "vision.h"
#include "centroid.h"
#include <thread>

int* centroid_array_for_control = new int[14];


int main()
{
	AllocConsole();
	freopen("CONOUT$", "w", stdout);

	int radius, width, height, cam_number, nlabels, tlow = 80, thigh = 100, tlow2 = 80, thigh2 = 100, min_label_size = 400, max_label_size = 5000;
	bool upHeld = false, downHeld = false, leftHeld = false, rightHeld = false;
	bool maxIncHeld = false, maxDecHeld = false;
	bool thighUpHeld = false, thighDownHeld = false;
	bool toggleHeld = false;
	int activeRange = 1; // 1 => range1, 2 => range2
	double ic_1, jc_1, ic_2, jc_2, ic_3, jc_3, ic_4, jc_4;
	image rgb1, rgb2, rgb3, gscale1, gscale2, label;
	// tracking state
	std::atomic<bool> trackingMode(false);
	int tracked_labels[4] = { 0,0,0,0 };
	std::mutex trackMutex;
	int selected_labels[4] = { 0,0,0,0 };

	// Shared memory setup
	HANDLE hMapFile = CreateFileMapping(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, sizeof(CentroidData), "CentroidSharedMem");
	if (hMapFile == NULL) { cout << "Failed to create shared memory\n"; return 1; }
	CentroidData* sharedData = (CentroidData*)MapViewOfFile(hMapFile, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(CentroidData));
	if (sharedData == NULL) { CloseHandle(hMapFile); cout << "Failed to map shared memory\n"; return 1; }
	sharedData->frame = 0;

	thread control_thread(mapper, centroid_array_for_control, width, height);

	control_thread.detach();

	activate_vision();

	cout << "\npress space key to acquire image\n";
	while (!KEY(VK_SPACE));

	cam_number = 1;
	width = 640;
	height = 480;
	radius = 3;

	activate_camera(cam_number, height, width);

	rgb1.width = width;  rgb1.height = height;  rgb1.type = RGB_IMAGE;
	rgb2.width = width;  rgb2.height = height;  rgb2.type = RGB_IMAGE;
	rgb3.width = width;  rgb3.height = height;  rgb3.type = RGB_IMAGE;
	gscale1.width = width; gscale1.height = height; gscale1.type = GREY_IMAGE;
	gscale2.width = width; gscale2.height = height; gscale2.type = GREY_IMAGE;
	label.type = LABEL_IMAGE; label.width = width; label.height = height;

	allocate_image(rgb1);
	allocate_image(rgb2);
	allocate_image(rgb3);
	allocate_image(gscale1);
	allocate_image(gscale2);
	allocate_image(label);

	// start command thread to accept 'track' and 'untrack' commands
	std::thread cmdThread([&]() {
		std::string cmd;
		while (std::getline(std::cin, cmd)) {
			if (cmd == "q") {
				std::lock_guard<std::mutex> lk(trackMutex);
				for (int i = 0; i < 4; ++i) tracked_labels[i] = selected_labels[i];
				trackingMode.store(true);
				cout << "Tracking labels:";
				for (int i = 0; i < 4; ++i) if (tracked_labels[i]) cout << ' ' << tracked_labels[i];
				cout << "\n";
			}
			else if (cmd == "w") {
				trackingMode.store(false);
				std::lock_guard<std::mutex> lk(trackMutex);
				for (int i = 0; i < 4; ++i) tracked_labels[i] = 0;
				cout << "Untracked.\n";
			}
			else if (cmd == "quit" || cmd == "exit") {
				break; // thread will exit; main loop continues until user presses X
			}
		}
		});
	cmdThread.detach();

	while (1) {


		double t_start = high_resolution_time();

		acquire_image(rgb1, cam_number);
		scale(rgb1, rgb2);
		scale(rgb2, rgb2);
		//scale(rgb2, rgb2);
		//dialate(rgb2, rgb2);
		copy(rgb2, rgb3); //For inspection
		copy(rgb2, gscale1);  // RGB -> greyscale
		gaussian_filter(gscale1, gscale1);
		gaussian_filter(gscale1, gscale1);
		lowpass_filter(gscale1, gscale2);
		highpass_filter(gscale2, gscale1);


		copy(gscale1, rgb1); //Filtered image copied for inspection later

		// Toggle active range with 'T' (selects which range Up/Down and PageUp/PageDown adjust)
		if (KEY('T') && !toggleHeld) { activeRange = (activeRange == 1) ? 2 : 1; cout << "Active range=" << activeRange << "\n"; toggleHeld = true; }
		if (!KEY('T')) toggleHeld = false;

		// Adjust lower threshold with Up/Down arrows (range 1-254) for active range
		if (KEY(VK_UP) && !upHeld) {
			if (activeRange == 1) { if (tlow < 254) tlow++; }
			else { if (tlow2 < 254) tlow2++; }
			cout << "  tlow=" << tlow << " thigh=" << thigh << "  tlow2=" << tlow2 << " thigh2=" << thigh2 << "  (active=" << activeRange << ")\n";
			upHeld = true;
		}
		if (KEY(VK_DOWN) && !downHeld) {
			if (activeRange == 1) { if (tlow > 1) tlow--; }
			else { if (tlow2 > 1) tlow2--; }
			cout << "  tlow=" << tlow << " thigh=" << thigh << "  tlow2=" << tlow2 << " thigh2=" << thigh2 << "  (active=" << activeRange << ")\n";
			downHeld = true;
		}
		if (!KEY(VK_UP))   upHeld = false;
		if (!KEY(VK_DOWN)) downHeld = false;

		// Adjust upper threshold with PageUp/PageDown (range 1-254) for active range
		if (KEY(VK_PRIOR) && !thighUpHeld) {
			if (activeRange == 1) { if (thigh < 254) thigh++; }
			else { if (thigh2 < 254) thigh2++; }
			cout << "  tlow=" << tlow << " thigh=" << thigh << "  tlow2=" << tlow2 << " thigh2=" << thigh2 << "  (active=" << activeRange << ")\n";
			thighUpHeld = true;
		}
		if (KEY(VK_NEXT) && !thighDownHeld) {
			if (activeRange == 1) { if (thigh > 1) thigh--; }
			else { if (thigh2 > 1) thigh2--; }
			cout << "  tlow=" << tlow << " thigh=" << thigh << "  tlow2=" << tlow2 << " thigh2=" << thigh2 << "  (active=" << activeRange << ")\n";
			thighDownHeld = true;
		}
		if (!KEY(VK_PRIOR)) thighUpHeld = false;
		if (!KEY(VK_NEXT)) thighDownHeld = false;

		// Adjust minimum label size with Left/Right arrows
		if (KEY(VK_RIGHT) && !rightHeld) { if (min_label_size < 10000) min_label_size += 100; cout << "  min_label_size=" << min_label_size << "\n"; rightHeld = true; }
		if (KEY(VK_LEFT) && !leftHeld) { if (min_label_size > 1) min_label_size -= 100; cout << "  min_label_size=" << min_label_size << "\n"; leftHeld = true; }
		if (!KEY(VK_RIGHT)) rightHeld = false;
		if (!KEY(VK_LEFT))  leftHeld = false;

		// Adjust maximum label size with '[' and ']' keys (range 1..10000)
		if (KEY(']') && !maxIncHeld) { if (max_label_size < 10000) max_label_size++; cout << "  max_label_size=" << max_label_size << "\n"; maxIncHeld = true; }
		if (KEY('[') && !maxDecHeld) { if (max_label_size > 1) max_label_size--; cout << "  max_label_size=" << max_label_size << "\n"; maxDecHeld = true; }
		if (!KEY(']')) maxIncHeld = false;
		if (!KEY('[')) maxDecHeld = false;

		threshold_range(gscale1, gscale2, tlow, thigh, tlow2, thigh2);
		//erode(gscale2, gscale2); 
		//dialate(gscale2, gscale2);
		//invert(gscale2, gscale2);
		label_image(gscale2, label, nlabels);
		copy(gscale2, rgb2); //for inspection

		// Find first 4 labels meeting min_label_size and draw their centroids
		i2byte* pl = (i2byte*)label.pdata;
		int np = width * height;

		// 1) count pixels per label in one pass
		std::vector<int> counts(nlabels + 1, 0);
		for (int k = 0; k < np; ++k) {
			int lab = pl[k];
			if (lab >= 0 && lab <= nlabels) counts[lab]++;
		}

		// 2) removed: previously small/out-of-range labels were blackened here
		 //    keep original label image so tracking/selection uses label data directly

		 // 3) select labels and compute centroids
		double ic_arr[4] = {}, jc_arr[4] = {};
		int size_arr[4] = { 0,0,0,0 };
		int found = 0;

		if (trackingMode.load()) {
			// track only the labels previously captured
			int tracked_local[4] = { 0,0,0,0 };
			{
				std::lock_guard<std::mutex> lk(trackMutex);
				for (int i = 0; i < 4; ++i) tracked_local[i] = tracked_labels[i];
			}
			for (int i = 0; i < 4 && found < 4; ++i) {
				int lab = tracked_local[i];
				if (lab <= 0) continue;
				if (lab > nlabels) {
					cout << "Tracked label " << lab << " not present (nlabels=" << nlabels << ")\n";
					continue;
				}
				if (counts[lab] < min_label_size || counts[lab] > max_label_size) {
					cout << "Tracked label " << lab << " out of size range (size=" << counts[lab] << ")\n";
					continue;
				}
				centroid(gscale2, label, lab, ic_arr[found], jc_arr[found]);
				draw_marker(rgb2.pdata, width, height, radius, ic_arr[found], jc_arr[found]);
				size_arr[found] = counts[lab];
				found++;
			}
		}
		else {
			// not tracking: pick first up to 4 labels that meet size criteria
			for (int n = 1; n <= nlabels && found < 4; ++n) {
				if (counts[n] < min_label_size || counts[n] > max_label_size) continue; // reuse computed counts
				centroid(gscale2, label, n, ic_arr[found], jc_arr[found]);
				draw_marker(rgb2.pdata, width, height, radius, ic_arr[found], jc_arr[found]);
				size_arr[found] = counts[n];
				// record currently selected labels so user can 'track' them
				{
					std::lock_guard<std::mutex> lk(trackMutex);
					selected_labels[found] = n;
				}
				found++;
			}
		}

		ic_1 = ic_arr[0]; jc_1 = jc_arr[0];
		ic_2 = ic_arr[1]; jc_2 = jc_arr[1];
		ic_3 = ic_arr[2]; jc_3 = jc_arr[2];
		ic_4 = ic_arr[3]; jc_4 = jc_arr[3];

		centroid_array_for_control[0] = (int)ic_arr[0] + 0.5;
		centroid_array_for_control[1] = (int)jc_arr[0] + 0.5;
		centroid_array_for_control[2] = (int)ic_arr[1] + 0.5;
		centroid_array_for_control[3] = (int)jc_arr[1] + 0.5;
		centroid_array_for_control[4] = (int)ic_arr[2] + 0.5;
		centroid_array_for_control[5] = (int)jc_arr[2] + 0.5;
		centroid_array_for_control[6] = (int)ic_arr[3] + 0.5;
		centroid_array_for_control[7] = (int)jc_arr[3] + 0.5;

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
			<< "\n  1: ic=" << ic_1 << " jc=" << jc_1 << " size=" << size_arr[0]
			<< "\n  2: ic=" << ic_2 << " jc=" << jc_2 << " size=" << size_arr[1]
			<< "\n  3: ic=" << ic_3 << " jc=" << jc_3 << " size=" << size_arr[2]
			<< "\n  4: ic=" << ic_4 << " jc=" << jc_4 << " size=" << size_arr[3]
			<< "\n  loop time=" << loop_time
			<< "\n  nlabels=" << nlabels;
		//<< "\n  threshold=" << tvalue << "\n";

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
