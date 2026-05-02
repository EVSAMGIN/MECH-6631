#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <Windows.h>
#include <cmath>

using namespace std;

//#include "image_transfer.h"
#include "vision.h"
#include "timer.h"
#include "vision_custom.h"
//#include "serial_com.h"
#include "world_map.h"
#include "thread"


#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

#define NTARGETS 6

/*TO DO:
		Selection thread:
		-Replace cursor with sobel edge detection
		-If matching colours between cars then determine which car based on relative distance from other target

		Tracking thread:
		-Optimize search for predicted motion of label instead of spiral (perhaps based on last recorded heading direction)
		-Implement Kalman filter?
		-Match based on other label info(colour?)
		-Requires a label_info function

		Additional functions:
		Add label info function to calculate label size and average colour (roll existing label area function into this one)

		*/

int activate(image& a, image& b, image& grey_gauss, image& rgb, image& rgb0, image& rgb1, image mask[], image label[]);
int deactivate(image& a, image& b, image& grey_gauss, image& rgb, image& rgb0, image& rgb1, image mask[], image label[]);
int label_filtered_objects(image& rgb0, image& rgb1, image& b, image& grey_gauss, image& a, image mask[], image label[], MaskParameters pm[]);
int find_objects(image& rgb0, image& a, image& b, image& grey_gauss, image& rgb, image& rgb1, image mask[], image label[], MaskParameters pm[], double ic[], double jc[], int ref_areas[]);
int select_object(image& rgb0, image& a, image& b, image& grey_gauss, image& rgb, image& rgb1, image mask[], image label[], MaskParameters pm[], double& ic, double& jc, int target_index);
i2byte search_object_circular(image& label_t, int is, int js, int min_area, int max_area);
int track_objects(image& rgb0, image& a, image& b, image& grey_gauss, image& rgb, image& rgb1, image mask[], image label[], MaskParameters pm[], double ic[], double jc[], int ref_areas[], TargetPositions& tp);
int label_area(image& label, i2byte nlabel);
i2byte largest_label(image& label);

const int IMAGE_WIDTH  = 640;
const int IMAGE_HEIGHT = 480;
int cam_number = 1;

// Radius in pixels for the sample circle within cursor
// Tune this 
const int SAMPLE_RADIUS = 5;

// Object indices
// 0=C1F  1=C1R  2=C2F  3=C2R  4=OBS1  5=OBS2
const char* TARGET_NAME[NTARGETS] = { "C1F", "C1R", "C2F", "C2R", "OBS1", "OBS2" };

// Marker colours per target (BGR order) - distinct colours so cursor is always visible
const int MARKER_R[NTARGETS] = { 255,   0,   0, 255, 255,   0 };
const int MARKER_G[NTARGETS] = {   0, 255,   0, 255,   0, 255 };
const int MARKER_B[NTARGETS] = {   0,   0, 255,   0, 255, 255 };

image a, b, rgb, rgb0, rgb1;
image grey_gauss;
image mask[NTARGETS];
image label[NTARGETS];

// viewMode: 0=raw  1..NTARGETS=individual mask  NTARGETS+1=combined binary
int viewMode = 0;

// All mask parameters start zeroed - sample_mask_at_cursor fills every field at selection time
MaskParameters pm[NTARGETS] = {};

TargetPositions target_positions = {}; // Centroid Positions stored here!!!!! 

// Serial
HANDLE h1;
int speed = 0;

int main()
{
	// Open serial port
	open_serial("COM5", h1, speed);

	activate_vision();
	cam_number = 1;
	activate_camera(cam_number, IMAGE_HEIGHT, IMAGE_WIDTH);

	cout << "\npress space to begin.";
	pause();

	double ic_arr[NTARGETS] = {}, jc_arr[NTARGETS] = {};
	int    ref_areas[NTARGETS] = {};

	activate(a, b, grey_gauss, rgb, rgb0, rgb1, mask, label);

	find_objects(rgb0, a, b, grey_gauss, rgb, rgb1, mask, label, pm, ic_arr, jc_arr, ref_areas);

	int i = 0;
	while (1) {
		

		
		track_objects(rgb0, a, b, grey_gauss, rgb, rgb1, mask, label, pm, ic_arr, jc_arr, ref_areas, target_positions);
		if (i < 1) {
			thread control_thread(mapper, ref(target_positions), ref(h1), IMAGE_WIDTH, IMAGE_HEIGHT);
			control_thread.detach();
		}

		
		if (KEY('T')) {
				cout << target_positions.ic[i] << " " << target_positions.jc[i] << "\n";
				cout << target_positions.jc[i] << " " << target_positions.jc[i] << "\n";
		}

		if (KEY('X')) {
			close_serial(h1);
			break;
		}


		//cout << "\n\nloop.\n";

		if(i<1) i++;
			}
	deactivate(a, b, grey_gauss, rgb, rgb0, rgb1, mask, label);
	deactivate_vision();
}

int activate(image& a, image& b, image& grey_gauss, image& rgb, image& rgb0, image& rgb1, image mask[], image label[])
{
	a.type = b.type = grey_gauss.type = GREY_IMAGE;
	a.width = b.width = grey_gauss.width = IMAGE_WIDTH;
	a.height = b.height = grey_gauss.height = IMAGE_HEIGHT;

	rgb.type = rgb0.type = rgb1.type = RGB_IMAGE;
	rgb.width = rgb0.width = rgb1.width = IMAGE_WIDTH;
	rgb.height = rgb0.height = rgb1.height = IMAGE_HEIGHT;

	for (int t = 0; t < NTARGETS; t++) {
		mask[t].type  = GREY_IMAGE;  mask[t].width  = IMAGE_WIDTH;  mask[t].height  = IMAGE_HEIGHT;
		label[t].type = LABEL_IMAGE; label[t].width = IMAGE_WIDTH;  label[t].height = IMAGE_HEIGHT;
	}

	allocate_image(a); allocate_image(b); allocate_image(grey_gauss);
	allocate_image(rgb); allocate_image(rgb0); allocate_image(rgb1);
	for (int t = 0; t < NTARGETS; t++) {
		allocate_image(mask[t]);
		allocate_image(label[t]);
	}

	return 0;
}

int deactivate(image& a, image& b, image& grey_gauss, image& rgb, image& rgb0, image& rgb1, image mask[], image label[])
{
	free_image(a); free_image(b); free_image(grey_gauss);
	free_image(rgb); free_image(rgb0); free_image(rgb1);
	for (int t = 0; t < NTARGETS; t++) {
		free_image(mask[t]);
		free_image(label[t]);
	}
	return 0;
}

//===SELECTION THREAD===
int find_objects(image& rgb0, image& a, image& b, image& grey_gauss, image& rgb, image& rgb1, image mask[], image label[], MaskParameters pm[], double ic[], double jc[], int ref_areas[])
{
	cout << "\npress space to get an image";
	pause();
	acquire_image(rgb0, cam_number);
	label_filtered_objects(rgb0, rgb1, b, grey_gauss, a, mask, label, pm);

	for (int t = 0; t < NTARGETS; t++)
		select_object(rgb0, a, b, grey_gauss, rgb, rgb1, mask, label, pm, ic[t], jc[t], t);

	for (int t = 0; t < NTARGETS; t++) {
		i2byte nlabel = largest_label(label[t]);
		ref_areas[t] = label_area(label[t], nlabel);
		cout << "\ntarget " << TARGET_NAME[t] << "  ic=" << ic[t] << "  jc=" << jc[t] << "  area=" << ref_areas[t];
	}

	return 0;
}

//===SELECTION THREAD LVL2===
int select_object(image& rgb0, image& a, image& b, image& grey_gauss, image& rgb, image& rgb1, image mask[], image label[], MaskParameters pm[], double& ic, double& jc, int target_index)
{
	int i = 200, j = 300;
	i2byte* pl, nlabel;

	// number of view modes: 0=raw, 1..NTARGETS=individual masks, NTARGETS+1=combined
	int n_view_modes = NTARGETS + 2;

	cout << "\nmove cursor to target " << TARGET_NAME[target_index] << " and press C";
	cout << "\n  V=cycle view  C=confirm";

	while (1) {
		acquire_image(rgb0, cam_number);
		label_filtered_objects(rgb0, rgb1, b, grey_gauss, a, mask, label, pm);

		if (KEY('V')) {
			viewMode = (viewMode + 1) % n_view_modes;
			Sleep(200);
			if      (viewMode == 0)            cout << "\nview: raw";
			else if (viewMode <= NTARGETS)     cout << "\nview: mask " << TARGET_NAME[viewMode - 1];
			else                               cout << "\nview: combined";
		}

		if      (viewMode == 0)            copy(rgb0, rgb);
		else if (viewMode <= NTARGETS)     copy(mask[viewMode - 1], rgb);
		else                               copy(a, rgb);

		draw_point_rgb(rgb, i, j, MARKER_R[target_index], MARKER_G[target_index], MARKER_B[target_index]);
		draw_point_rgb(rgb, 320, 240, 0, 255, 0);
		view_rgb_image(rgb);

		if (KEY(VK_UP))    j -= 6;
		if (KEY(VK_DOWN))  j += 6;
		if (KEY(VK_LEFT))  i -= 6;
		if (KEY(VK_RIGHT)) i += 6;
		if (i < 0) i = 0; if (i > IMAGE_WIDTH  - 1) i = IMAGE_WIDTH  - 1;
		if (j < 0) j = 0; if (j > IMAGE_HEIGHT - 1) j = IMAGE_HEIGHT - 1;

		if (KEY('C')) { Sleep(200); break; }
	}

	// sample all mask parameters from the RGB image at the confirmed cursor position
	sample_mask_at_cursor(rgb0, i, j, SAMPLE_RADIUS, pm[target_index]);

	// re-run filtering now that this target's mask parameters are populated,
	// then find the label under the cursor
	label_filtered_objects(rgb0, rgb1, b, grey_gauss, a, mask, label, pm);

	pl     = (i2byte*)label[target_index].pdata;
	nlabel = *(pl + j * label[target_index].width + i);
	if (nlabel == 0) nlabel = largest_label(label[target_index]);
	if (nlabel != 0) centroid(a, label[target_index], nlabel, ic, jc);
	return 0;
}

//===TRACKING THREAD===
int track_objects(image& rgb0, image& a, image& b, image& grey_gauss, image& rgb, image& rgb1, image mask[], image label[], MaskParameters pm[], double ic[], double jc[], int ref_areas[], TargetPositions& tp)
{
	bool   valid[NTARGETS];
	double last_elapsed = 0.0;
	int    cursor_i = 320, cursor_j = 240;
	int    cursor_target = -1;

	int n_view_modes = NTARGETS + 2;

	cout << "\n\ntracking " << NTARGETS << " targets.";
	cout << "\n  V=cycle view  1-6=place cursor for target  Enter=print centroids+timing  X=exit";

	
		double t0 = high_resolution_time();

		acquire_image(rgb0, cam_number);
		label_filtered_objects(rgb0, rgb1, b, grey_gauss, a, mask, label, pm);
		last_elapsed = high_resolution_time() - t0;

		for (int t = 0; t < NTARGETS; t++) {
			int tol = ref_areas[t] / 2;
			i2byte nl = search_object_circular(label[t], (int)ic[t], (int)jc[t], ref_areas[t] - tol, ref_areas[t] + tol);
			valid[t] = (nl != 0);
			if (valid[t]) centroid(a, label[t], nl, ic[t], jc[t]);
		}

		for (int t = 0; t < NTARGETS; t++) {
			tp.ic[t] = ic[t]; tp.jc[t] = jc[t]; tp.valid[t] = valid[t];
		}

		// cycle view
		if (KEY('V')) {
			viewMode = (viewMode + 1) % n_view_modes;
			Sleep(200);
			if (viewMode == 0)        cout << "\nview: raw";
			else if (viewMode <= NTARGETS) cout << "\nview: mask " << TARGET_NAME[viewMode - 1];
			else                           cout << "\nview: combined";
		}

		if (viewMode == 0)        copy(rgb0, rgb);
		else if (viewMode <= NTARGETS) copy(mask[viewMode - 1], rgb);
		else                           copy(a, rgb);

		for (int t = 0; t < NTARGETS; t++)
			draw_point_rgb(rgb, (int)ic[t], (int)jc[t], MARKER_R[t], MARKER_G[t], MARKER_B[t]);
		draw_point_rgb(rgb, 320, 240, 0, 255, 0);

		// cursor placement: keys 1-6 snap cursor to that target
		for (int t = 0; t < NTARGETS; t++) {
			if (KEY('1' + t)) {
				cursor_target = t; cursor_i = (int)ic[t]; cursor_j = (int)jc[t]; Sleep(200);
				cout << "\ncursor on " << TARGET_NAME[t] << " -- arrow keys to move, C to confirm";
			}
		}
		if (cursor_target >= 0) {
			if (KEY(VK_UP)) { cursor_j -= 6; Sleep(50); }
			if (KEY(VK_DOWN)) { cursor_j += 6; Sleep(50); }
			if (KEY(VK_LEFT)) { cursor_i -= 6; Sleep(50); }
			if (KEY(VK_RIGHT)) { cursor_i += 6; Sleep(50); }
			if (cursor_i < 0) cursor_i = 0; if (cursor_i > IMAGE_WIDTH - 1) cursor_i = IMAGE_WIDTH - 1;
			if (cursor_j < 0) cursor_j = 0; if (cursor_j > IMAGE_HEIGHT - 1) cursor_j = IMAGE_HEIGHT - 1;
			draw_point_rgb(rgb, cursor_i, cursor_j,
				MARKER_R[cursor_target], MARKER_G[cursor_target], MARKER_B[cursor_target]);
			if (KEY('C')) {
				ic[cursor_target] = cursor_i;
				jc[cursor_target] = cursor_j;
				cursor_target = -1;
				Sleep(200);
				cout << "\ncursor confirmed";
			}
		}

		view_rgb_image(rgb);

		// print centroids + frame time on Enter
		if (KEY(VK_RETURN)) {
			cout << "\n--- centroids  frame=" << (int)(last_elapsed * 1e6) << " us  (" << (int)(last_elapsed * 1000) << " ms) ---";
			for (int t = 0; t < NTARGETS; t++)
				cout << "\n  " << TARGET_NAME[t] << "  ic=" << ic[t] << "  jc=" << jc[t] << "  valid=" << valid[t];
			Sleep(300);
		}

		//	if (KEY('X')) break;
		//}

		// save masks on exit
		if (KEY('X')) {
			save_rgb_image("rgb0.bmp", rgb0);
			save_rgb_image("rgb1.bmp", rgb1);
			for (int t = 0; t < NTARGETS; t++) {
				char fname[32];
				sprintf(fname, "mask_%s.bmp", TARGET_NAME[t]);
				copy(mask[t], rgb);
				save_rgb_image(fname, rgb);
			}
			copy(a, rgb); save_rgb_image("combined.bmp", rgb);
		}

		return 0;
	}



//===TRACKING THREAD LVL2===
i2byte search_object_circular(image& label_t, int is, int js, int min_area, int max_area)
{
	i2byte* pl = (i2byte*)label_t.pdata;
	i2byte nlabel;

	nlabel = *(pl + js * label_t.width + is);
	if (nlabel != 0 && label_area(label_t, nlabel) >= min_area && label_area(label_t, nlabel) <= max_area) return nlabel;

	double rmax = 60.0, dr = 3.0, ds = 3.0;
	for (double r = 1.0; r <= rmax; r += dr) {
		double smax = 2 * 3.1416 * r;
		for (double s = 0; s <= smax; s += ds) {
			double theta = s / r;
			int i = (int)(is + r * cos(theta));
			int j = (int)(js + r * sin(theta));
			if (i < 0) i = 0; if (i > label_t.width  - 1) i = label_t.width  - 1;
			if (j < 0) j = 0; if (j > label_t.height - 1) j = label_t.height - 1;
			nlabel = *(pl + j * label_t.width + i);
			if (nlabel != 0) {
				int area = label_area(label_t, nlabel);
				if (area >= min_area && area <= max_area) return nlabel;
			}
		}
	}

	return largest_label(label_t);
}

//===LEVEL 3 FUNCTION===
i2byte largest_label(image& lbl)
{
	i2byte* pl = (i2byte*)lbl.pdata;
	int np = lbl.width * lbl.height;
	int best_area = 0;
	i2byte best_nlabel = 0;
	int counted[65536] = {};
	for (int k = 0; k < np; k++) {
		i2byte nlabel = pl[k];
		if (nlabel == 0 || counted[nlabel]) continue;
		counted[nlabel] = 1;
		int area = label_area(lbl, nlabel);
		if (area > best_area) { best_area = area; best_nlabel = nlabel; }
	}
	return best_nlabel;
}

//===LEVEL 3 FUNCTION===
int label_area(image& lbl, i2byte nlabel)
{
	i2byte* pl = (i2byte*)lbl.pdata;
	int area = 0;
	for (int k = 0; k < lbl.width * lbl.height; k++)
		if (pl[k] == nlabel) area++;
	return area;
}

//===LEVEL 3 FUNCTION===
int label_filtered_objects(image& rgb0, image& rgb1, image& b, image& grey_gauss, image& a, image mask[], image label[], MaskParameters pm[])
{
	int nlabels;
	int np = IMAGE_WIDTH * IMAGE_HEIGHT;

	copy(rgb0, rgb1);
	scale(rgb1, rgb1);
	copy(rgb1, b);
	gaussian_filter(b, grey_gauss);

	for (int t = 0; t < NTARGETS; t++) {
		threshold_mask(grey_gauss, mask[t], rgb1, pm[t]);
		dialate(mask[t], b); copy(b, mask[t]);
		label_image(mask[t], label[t], nlabels);
	}

	// combine all masks into image a for the combined view
	ibyte* pa = a.pdata;
	for (int k = 0; k < np; k++) {
		ibyte combined = 0;
		for (int t = 0; t < NTARGETS; t++)
			if (mask[t].pdata[k]) combined = 255;
		pa[k] = combined;
	}

	return 0;
}
