
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <Windows.h>
#include <cmath>

using namespace std;

#include "image_transfer.h"
#include "vision.h"
#include "timer.h"
#include "vision_custom.h"

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

#define NTARGETS 4

/*TO DO:
		Selection thread:
		-Replace cursor with sobel edge detection
		-Prompt user for colour of each target and obstacle
		-If matching colours between cars then determine which car based on relative distance from other target
		
		Tracking thread:
		-Optimize search for predicted motion of label instead of spiral (perhaps based on last recorded heading direction)
		-Implement Kalman filter?
		-Match based on other label info(colour?)
		-Requires a label_info function

		Additional functions:
		Add label info function to calculate label size and average colour (roll existing label area function into this one)
		
		*/

int activate();
int deactivate();
int find_objects(i2byte nlabels_out[NTARGETS], int areas_out[NTARGETS]);
int select_object(i2byte &nlabel, image &label, image &a, image &b, int target_index);
int search_object_circular(i2byte &nlabel, image &label, int is, int js, int min_area, int max_area);
int track_objects(i2byte nlabels[NTARGETS], int areas[NTARGETS]);
int label_area(image &label, i2byte nlabel);
int label_filtered_objects();

const int IMAGE_WIDTH  = 640;
const int IMAGE_HEIGHT = 480;
int cam_number = 0;

image a, b, rgb, rgb0, rgb1, label;
image grey_gauss;               // greyscale of rgb1 with gaussian applied
image mask_r, mask_g, mask_b, mask_y; // per-colour binary masks

// viewMode: 0=raw  1=red  2=green  3=blue  4=yellow  5=combined binary
int viewMode = 0;
const char* VIEW_NAME[] = { "raw", "red mask", "green mask", "blue mask", "yellow mask", "combined" };

/*


struct MaskParameters {
	int tlow, thigh;     // greyscale range on grey_gauss
	int hlow, hhigh;     // primary hue range
	int hlow2, hhigh2;   // secondary hue range (-1 = unused)
	int vmin;            // minimum V
	double sat_min;      // minimum saturation
}

*/

//Tune these parameters for each mask depeding on target colour and background
//vmin and sat_min tuned to reject shadow and floor

MaskParameters pm_r = { 120, 175,   0,  25, 340, 360, 130, 0.45 };
MaskParameters pm_g = { 100, 140,  80, 150,  -1,  -1, 120, 0.35 };
MaskParameters pm_b = {  75, 135, 180, 230,  -1,  -1, 120, 0.35 };
MaskParameters pm_y = { 185, 230,  26,  65,  -1,  -1, 150, 0.40 };

// marker colours per target BGR: red, green, blue, yellow
const int MARKER_B[NTARGETS] = {   0,   0, 255,   0 };
const int MARKER_G[NTARGETS] = {   0, 255,   0, 255 };
const int MARKER_R[NTARGETS] = { 255,   0,   0, 255 };
const char* TARGET_NAME[NTARGETS] = { "1:red", "2:green", "3:blue", "4:yellow" };


int main()
{
	i2byte nlabels[NTARGETS] = {};
	int    areas[NTARGETS]   = {};

	activate_vision();
	cam_number = 0;
	activate_camera(cam_number, IMAGE_HEIGHT, IMAGE_WIDTH);

	cout << "\npress space to begin.";
	pause();

	activate();

	find_objects(nlabels, areas);
	track_objects(nlabels, areas);

	deactivate();
	deactivate_vision();

	cout << "\n\ndone.\n";
	pause();
	return 0;
}

int activate()
{
	a.type = b.type = grey_gauss.type = GREY_IMAGE;
	a.width = b.width = grey_gauss.width = IMAGE_WIDTH;
	a.height = b.height = grey_gauss.height = IMAGE_HEIGHT;

	rgb.type = rgb0.type = rgb1.type = RGB_IMAGE;
	rgb.width = rgb0.width = rgb1.width = IMAGE_WIDTH;
	rgb.height = rgb0.height = rgb1.height = IMAGE_HEIGHT;

	label.type = LABEL_IMAGE;
	label.width = IMAGE_WIDTH;
	label.height = IMAGE_HEIGHT;

	mask_r.type = mask_g.type = mask_b.type = mask_y.type = GREY_IMAGE;
	mask_r.width = mask_g.width = mask_b.width = mask_y.width = IMAGE_WIDTH;
	mask_r.height = mask_g.height = mask_b.height = mask_y.height = IMAGE_HEIGHT;

	allocate_image(a); allocate_image(b); allocate_image(grey_gauss);
	allocate_image(rgb); allocate_image(rgb0); allocate_image(rgb1);
	allocate_image(label);
	allocate_image(mask_r); allocate_image(mask_g);
	allocate_image(mask_b); allocate_image(mask_y);

	return 0;
}


int deactivate()
{
	free_image(a); free_image(b); free_image(grey_gauss);
	free_image(rgb); free_image(rgb0); free_image(rgb1);
	free_image(label);
	free_image(mask_r); free_image(mask_g);
	free_image(mask_b); free_image(mask_y);
	return 0;
}

//===SELECTION THREAD===
int find_objects(i2byte nlabels_out[NTARGETS], int areas_out[NTARGETS])
{
	cout << "\npress space to get an image";
	pause();
	acquire_image(rgb0, cam_number);

	for (int t = 0; t < NTARGETS; t++) {
		cout << "\n--- select target " << TARGET_NAME[t] << " ---";
		select_object(nlabels_out[t], label, a, b, t);
		areas_out[t] = label_area(label, nlabels_out[t]);
		cout << "\ntarget " << TARGET_NAME[t] << "  label=" << nlabels_out[t] << "  area=" << areas_out[t];
	}
	return 0;
}

//===SELECTION THREAD LVL2===
int select_object(i2byte &nlabel, image &label, image &a, image &b, int target_index)
{
	i2byte *pl;
	int i = 200, j = 300;

	cout << "\nmove cursor to target " << TARGET_NAME[target_index] << " and press C";
	cout << "\n  V=cycle view  C=confirm";

	while (1) {
		acquire_image(rgb0, cam_number);
		label_filtered_objects();

		if (KEY('V')) { viewMode = (viewMode + 1) % 6; Sleep(200); cout << "\nview: " << VIEW_NAME[viewMode]; }

		switch (viewMode) {
			case 1: copy(mask_r, rgb); break;
			case 2: copy(mask_g, rgb); break;
			case 3: copy(mask_b, rgb); break;
			case 4: copy(mask_y, rgb); break;
			case 5: copy(a,      rgb); break;
			default: copy(rgb0,  rgb); break;
		}

		draw_point_rgb(rgb, i, j, MARKER_R[target_index], MARKER_G[target_index], MARKER_B[target_index]);
		draw_point_rgb(rgb, 320, 240, 0, 255, 0);
		view_rgb_image(rgb);

		if (KEY(VK_UP))    j += 3;
		if (KEY(VK_DOWN))  j -= 3;
		if (KEY(VK_LEFT))  i -= 3;
		if (KEY(VK_RIGHT)) i += 3;
		if (i < 0) i = 0; if (i > b.width-1)  i = b.width-1;
		if (j < 0) j = 0; if (j > b.height-1) j = b.height-1;

		if (KEY('C')) { Sleep(200); break; }
	}

	pl = (i2byte *)label.pdata;
	nlabel = *(pl + j * label.width + i);
	return 0;
}

//===TRACKING THREAD===
int track_objects(i2byte nlabels[NTARGETS], int areas[NTARGETS])
{
	double ic[NTARGETS], jc[NTARGETS];
	double last_elapsed = 0.0;
	int cursor_i = 320, cursor_j = 240;
	int cursor_target = -1; // -1 = no cursor active

	cout << "\n\ntracking " << NTARGETS << " targets.";
	cout << "\n  V=cycle view  1-4=place cursor for target  Enter=print centroids+timing  X=exit";

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <Windows.h>
#include <cmath>

	using namespace std;

#include "image_transfer.h"
#include "vision.h"
#include "timer.h"
#include "vision_custom.h"

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

#define NTARGETS 6

	/*TO DO:
			Selection thread:
			-Replace cursor with sobel edge detection
			-Prompt user for colour of each target and obstacle
			-If matching colours between cars then determine which car based on relative distance from other target

			Tracking thread:
			-Optimize search for predicted motion of label instead of spiral (perhaps based on last recorded heading direction)
			-Implement Kalman filter?
			-Match based on other label info(colour?)
			-Requires a label_info function

			Additional functions:
			Add label info function to calculate label size and average colour (roll existing label area function into this one)

			*/

	int activate();
	int deactivate();
	int find_objects(double ic[], double jc[], int ref_areas[], image * target_labels[]);
	int select_object(image * &active_label, double& ic, double& jc, int target_index);
	i2byte search_object_circular(image & label_t, int is, int js, int min_area, int max_area);
	int track_objects(double ic[], double jc[], int ref_areas[], image * target_labels[], TargetPositions & tp);
	int label_area(image & label, i2byte nlabel);
	i2byte largest_label(image & label);
	int label_filtered_objects();

	const int IMAGE_WIDTH = 640;
	const int IMAGE_HEIGHT = 480;
	int cam_number = 0;

	image a, b, rgb, rgb0, rgb1;
	image grey_gauss;
	image mask_r, mask_g, mask_b, mask_y;
	image label_r, label_g, label_b, label_y;

	// viewMode: 0=raw  1=red  2=green  3=blue  4=yellow  5=combined binary
	int viewMode = 0;
	const char* VIEW_NAME[] = { "raw", "red mask", "green mask", "blue mask", "yellow mask", "combined" };

	/*


	struct MaskParameters {
		int tlow, thigh;     // greyscale range on grey_gauss
		int hlow, hhigh;     // primary hue range
		int hlow2, hhigh2;   // secondary hue range (-1 = unused)
		int vmin;            // minimum V
		double sat_min;      // minimum saturation
	}

	*/

	//Tune these parameters for each mask depeding on target colour and background
	//vmin and sat_min tuned to reject shadow and floor

	MaskParameters pm_r = { 120, 175,   0,  25, 340, 360, 130, 0.45 };
	MaskParameters pm_g = { 100, 140,  80, 150,  -1,  -1, 120, 0.35 };
	MaskParameters pm_b = { 75, 135, 180, 230,  -1,  -1, 120, 0.35 };
	MaskParameters pm_y = { 185, 230,  26,  65,  -1,  -1, 150, 0.40 };

	TargetPositions target_positions = {};

	// marker colours per target BGR: car1_1, car1_2, car2_1, car2_2, obs1, obs2
	const int MARKER_B[NTARGETS] = { 0,   0,   0,   0, 255, 255 };
	const int MARKER_G[NTARGETS] = { 0,   0, 255, 255,   0,   0 };
	const int MARKER_R[NTARGETS] = { 255, 255,   0,   0,   0,   0 };
	const char* TARGET_NAME[NTARGETS] = { "1:car1_1", "2:car1_2", "3:car2_1", "4:car2_2", "5:obs1", "6:obs2" };


	int main()
	{
		activate_vision();
		cam_number = 0;
		activate_camera(cam_number, IMAGE_HEIGHT, IMAGE_WIDTH);

		cout << "\npress space to begin.";
		pause();

		double ic[NTARGETS] = {}, jc[NTARGETS] = {};
		int    ref_areas[NTARGETS] = {};
		image* target_labels[NTARGETS] = {};

		activate();

		find_objects(ic, jc, ref_areas, target_labels);
		track_objects(ic, jc, ref_areas, target_labels, target_positions);

		deactivate();
		deactivate_vision();

		cout << "\n\ndone.\n";
		pause();
		return 0;
	}

	int activate()
	{
		a.type = b.type = grey_gauss.type = GREY_IMAGE;
		a.width = b.width = grey_gauss.width = IMAGE_WIDTH;
		a.height = b.height = grey_gauss.height = IMAGE_HEIGHT;

		rgb.type = rgb0.type = rgb1.type = RGB_IMAGE;
		rgb.width = rgb0.width = rgb1.width = IMAGE_WIDTH;
		rgb.height = rgb0.height = rgb1.height = IMAGE_HEIGHT;

		mask_r.type = mask_g.type = mask_b.type = mask_y.type = GREY_IMAGE;
		mask_r.width = mask_g.width = mask_b.width = mask_y.width = IMAGE_WIDTH;
		mask_r.height = mask_g.height = mask_b.height = mask_y.height = IMAGE_HEIGHT;

		label_r.type = label_g.type = label_b.type = label_y.type = LABEL_IMAGE;
		label_r.width = label_g.width = label_b.width = label_y.width = IMAGE_WIDTH;
		label_r.height = label_g.height = label_b.height = label_y.height = IMAGE_HEIGHT;

		allocate_image(a); allocate_image(b); allocate_image(grey_gauss);
		allocate_image(rgb); allocate_image(rgb0); allocate_image(rgb1);
		allocate_image(mask_r); allocate_image(mask_g);
		allocate_image(mask_b); allocate_image(mask_y);
		allocate_image(label_r); allocate_image(label_g);
		allocate_image(label_b); allocate_image(label_y);

		return 0;
	}


	int deactivate()
	{
		free_image(a); free_image(b); free_image(grey_gauss);
		free_image(rgb); free_image(rgb0); free_image(rgb1);
		free_image(mask_r); free_image(mask_g);
		free_image(mask_b); free_image(mask_y);
		free_image(label_r); free_image(label_g);
		free_image(label_b); free_image(label_y);
		return 0;
	}

	//===SELECTION THREAD===
	int find_objects(double ic[], double jc[], int ref_areas[], image * target_labels[])
	{
		i2byte nlabel;

		cout << "\npress space to get an image";
		pause();
		acquire_image(rgb0, cam_number);
		label_filtered_objects();

		// user places cursor on each target in order: car1_1, car1_2, car2_1, car2_2, obs1, obs2
		for (int t = 0; t < NTARGETS; t++)
			select_object(target_labels[t], ic[t], jc[t], t);

		// record reference blob area for each target from its assigned label image
		for (int t = 0; t < NTARGETS; t++) {
			if (target_labels[t] != NULL) {
				nlabel = largest_label(*target_labels[t]);
				ref_areas[t] = label_area(*target_labels[t], nlabel);
			}
		}

		for (int t = 0; t < NTARGETS; t++)
			cout << "\n" << TARGET_NAME[t] << "  ic=" << ic[t] << " jc=" << jc[t] << " area=" << ref_areas[t];
		return 0;
	}

	//===SELECTION THREAD LVL2===
	// determines active label image from cursor position at confirm time
	int select_object(image * &active_label, double& ic, double& jc, int target_index)
	{
		int i = 200, j = 300;
		i2byte* pl, nlabel;

		cout << "\nmove cursor to target " << TARGET_NAME[target_index] << " and press C";
		cout << "\n  V=cycle view  C=confirm";

		while (1) {
			acquire_image(rgb0, cam_number);
			label_filtered_objects();

			if (KEY('V')) { viewMode = (viewMode + 1) % 6; Sleep(200); cout << "\nview: " << VIEW_NAME[viewMode]; }

			switch (viewMode) {
			case 1: copy(mask_r, rgb); break;
			case 2: copy(mask_g, rgb); break;
			case 3: copy(mask_b, rgb); break;
			case 4: copy(mask_y, rgb); break;
			case 5: copy(a, rgb); break;
			default: copy(rgb0, rgb); break;
			}

			draw_point_rgb(rgb, i, j, MARKER_R[target_index], MARKER_G[target_index], MARKER_B[target_index]);
			draw_point_rgb(rgb, 320, 240, 0, 255, 0);
			view_rgb_image(rgb);

			if (KEY(VK_UP))    j -= 3;
			if (KEY(VK_DOWN))  j += 3;
			if (KEY(VK_LEFT))  i -= 3;
			if (KEY(VK_RIGHT)) i += 3;
			if (i < 0) i = 0; if (i > IMAGE_WIDTH - 1) i = IMAGE_WIDTH - 1;
			if (j < 0) j = 0; if (j > IMAGE_HEIGHT - 1) j = IMAGE_HEIGHT - 1;

			if (KEY('C')) { Sleep(200); break; }
		}

		// determine which label image the cursor is on at confirm time
		active_label = NULL;
		if (*(i2byte*)(label_r.pdata + j * label_r.width + i) != 0) active_label = &label_r;
		else if (*(i2byte*)(label_g.pdata + j * label_g.width + i) != 0) active_label = &label_g;
		else if (*(i2byte*)(label_b.pdata + j * label_b.width + i) != 0) active_label = &label_b;
		else if (*(i2byte*)(label_y.pdata + j * label_y.width + i) != 0) active_label = &label_y;

		if (active_label == NULL) { cout << "\nwarning: cursor missed all blobs, using largest in view"; active_label = &label_r; }

		pl = (i2byte*)active_label->pdata;
		nlabel = *(pl + j * active_label->width + i);
		if (nlabel == 0) nlabel = largest_label(*active_label);
		if (nlabel != 0) centroid(a, *active_label, nlabel, ic, jc); //ic,jc values passed to tracking thread
		return 0;
	}

	//===TRACKING THREAD===
	int track_objects(double ic[], double jc[], int ref_areas[], image * target_labels[], TargetPositions & tp)
	{
		bool   valid[NTARGETS];
		double last_elapsed = 0.0;
		int cursor_i = 320, cursor_j = 240;
		int cursor_target = -1;

		cout << "\n\ntracking " << NTARGETS << " targets.";
		cout << "\n  V=cycle view  1-6=place cursor for target  Enter=print centroids+timing  X=exit";

		while (1) {
			double t0 = high_resolution_time();

			acquire_image(rgb0, cam_number);
			label_filtered_objects();
			last_elapsed = high_resolution_time() - t0;

			{
				i2byte nlabel;
				int tol;
				for (int t = 0; t < NTARGETS; t++) {
					if (target_labels[t] == NULL) { valid[t] = false; continue; }
					tol = ref_areas[t] / 2;
					nlabel = search_object_circular(*target_labels[t], (int)ic[t], (int)jc[t], ref_areas[t] - tol, ref_areas[t] + tol);
					valid[t] = (nlabel != 0);
					if (valid[t]) centroid(a, *target_labels[t], nlabel, ic[t], jc[t]);
				}
			}

			tp.ic_car1_1 = ic[0]; tp.jc_car1_1 = jc[0]; tp.valid_car1_1 = valid[0];
			tp.ic_car1_2 = ic[1]; tp.jc_car1_2 = jc[1]; tp.valid_car1_2 = valid[1];
			tp.ic_car2_1 = ic[2]; tp.jc_car2_1 = jc[2]; tp.valid_car2_1 = valid[2];
			tp.ic_car2_2 = ic[3]; tp.jc_car2_2 = jc[3]; tp.valid_car2_2 = valid[3];
			tp.ic_obs1 = ic[4]; tp.jc_obs1 = jc[4]; tp.valid_obs1 = valid[4];
			tp.ic_obs2 = ic[5]; tp.jc_obs2 = jc[5]; tp.valid_obs2 = valid[5];

			// cycle view
			if (KEY('V')) { viewMode = (viewMode + 1) % 6; Sleep(200); cout << "\nview: " << VIEW_NAME[viewMode]; }

			switch (viewMode) {
			case 1: copy(mask_r, rgb); break;
			case 2: copy(mask_g, rgb); break;
			case 3: copy(mask_b, rgb); break;
			case 4: copy(mask_y, rgb); break;
			case 5: copy(a, rgb); break;
			default: copy(rgb0, rgb); break;
			}

			for (int t = 0; t < NTARGETS; t++)
				draw_point_rgb(rgb, (int)ic[t], (int)jc[t], MARKER_R[t], MARKER_G[t], MARKER_B[t]);
			draw_point_rgb(rgb, 320, 240, 0, 255, 0);

			/*TO DO:
			-Replace cursor with sobel edge detection
			-Prompt user for colour of each target and obstacle
			-If matching colours between cars then determine which car based on relative distance from other target
			*/

			for (int t = 0; t < NTARGETS; t++) {
				if (KEY('1' + t)) {
					cursor_target = t; cursor_i = (int)ic[t]; cursor_j = (int)jc[t]; Sleep(200);
					cout << "\ncursor on " << TARGET_NAME[t] << " -- arrow keys to move, C to confirm";
				}
			}
			if (cursor_target >= 0) {
				if (KEY(VK_UP)) { cursor_j -= 3; Sleep(50); }
				if (KEY(VK_DOWN)) { cursor_j += 3; Sleep(50); }
				if (KEY(VK_LEFT)) { cursor_i -= 3; Sleep(50); }
				if (KEY(VK_RIGHT)) { cursor_i += 3; Sleep(50); }
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
					cout << "\n  " << TARGET_NAME[t] << "  ic=" << ic[t] << "  jc=" << jc[t];
				Sleep(300);
			}

			if (KEY('X')) break;
		}

		// save masks on exit
		save_rgb_image("rgb0.bmp", rgb0);
		save_rgb_image("rgb1.bmp", rgb1);
		copy(mask_r, rgb); save_rgb_image("mask_r.bmp", rgb);
		copy(mask_g, rgb); save_rgb_image("mask_g.bmp", rgb);
		copy(mask_b, rgb); save_rgb_image("mask_b.bmp", rgb);
		copy(mask_y, rgb); save_rgb_image("mask_y.bmp", rgb);
		copy(a, rgb); save_rgb_image("combined.bmp", rgb);

		return 0;
	}

	//===TRACKING THREAD LVL2===
	i2byte search_object_circular(image & label_t, int is, int js, int min_area, int max_area)
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
				if (i < 0) i = 0; if (i > label_t.width - 1) i = label_t.width - 1;
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
	i2byte largest_label(image & label)
	{
		i2byte* pl = (i2byte*)label.pdata;
		int np = label.width * label.height;
		int best_area = 0;
		i2byte best_nlabel = 0;
		int counted[65536] = {};
		for (int k = 0; k < np; k++) {
			i2byte nlabel = pl[k];
			if (nlabel == 0 || counted[nlabel]) continue;
			counted[nlabel] = 1;
			int area = label_area(label, nlabel);
			if (area > best_area) { best_area = area; best_nlabel = nlabel; }
		}
		return best_nlabel;
	}

	//===LEVEL 3 FUNCTION===
	int label_area(image & label, i2byte nlabel)
	{
		i2byte* pl = (i2byte*)label.pdata;
		int area = 0;
		for (int k = 0; k < label.width * label.height; k++)
			if (pl[k] == nlabel) area++;
		return area;
	}

	//===LEVEL 3 FUNCTION===
	int label_filtered_objects()
	{
		int nlabels;
		int np = IMAGE_WIDTH * IMAGE_HEIGHT;
		ibyte* pr, * pg, * pb, * py, * pa;

		copy(rgb0, rgb1);
		scale(rgb1, rgb1);
		copy(rgb1, b);
		gaussian_filter(b, grey_gauss);

		threshold_mask(grey_gauss, mask_r, rgb1, pm_r);
		threshold_mask(grey_gauss, mask_g, rgb1, pm_g);
		threshold_mask(grey_gauss, mask_b, rgb1, pm_b);
		threshold_mask(grey_gauss, mask_y, rgb1, pm_y);

		erode(mask_r, b); dialate(b, mask_r); label_image(mask_r, label_r, nlabels);
		erode(mask_g, b); dialate(b, mask_g); label_image(mask_g, label_g, nlabels);
		erode(mask_b, b); dialate(b, mask_b); label_image(mask_b, label_b, nlabels);
		erode(mask_y, b); dialate(b, mask_y); label_image(mask_y, label_y, nlabels);

		pr = mask_r.pdata, pg = mask_g.pdata, pb = mask_b.pdata, py = mask_y.pdata, pa = a.pdata;
		for (int k = 0; k < np; k++)
			pa[k] = (pr[k] || pg[k] || pb[k] || py[k]) ? 255 : 0;

		return 0;
	}


	label_filtered_objects();
	for (int t = 0; t < NTARGETS; t++) {
		ic[t] = 200.0; jc[t] = 300.0;
		centroid(a, label, nlabels[t], ic[t], jc[t]);
	}

	while (1) {
		double t0 = high_resolution_time();

		acquire_image(rgb0, cam_number);
		label_filtered_objects();

		last_elapsed = high_resolution_time() - t0;

		for (int t = 0; t < NTARGETS; t++) {
			int tol = areas[t] / 2;
			search_object_circular(nlabels[t], label, (int)ic[t], (int)jc[t], areas[t] - tol, areas[t] + tol);
			centroid(a, label, nlabels[t], ic[t], jc[t]);
		}

		// cycle view
		if (KEY('V')) { viewMode = (viewMode + 1) % 6; Sleep(200); cout << "\nview: " << VIEW_NAME[viewMode]; }

		switch (viewMode) {
			case 1: copy(mask_r, rgb); break;
			case 2: copy(mask_g, rgb); break;
			case 3: copy(mask_b, rgb); break;
			case 4: copy(mask_y, rgb); break;
			case 5: copy(a,      rgb); break;
			default: copy(rgb0,  rgb); break;
		}

		for (int t = 0; t < NTARGETS; t++)
			draw_point_rgb(rgb, (int)ic[t], (int)jc[t], MARKER_R[t], MARKER_G[t], MARKER_B[t]);
		draw_point_rgb(rgb, 320, 240, 0, 255, 0);

		// cursor placement: press 1-4 to select a target, arrow keys to move, C to confirm

		/*TO DO: 
		-Replace cursor with sobel edge detection 
		-Prompt user for colour of each target and obstacle 
		-If matching colours between cars then determine which car based on relative distance from other target
		*/

		for (int t = 0; t < NTARGETS; t++) {
			if (KEY('1' + t)) { cursor_target = t; cursor_i = (int)ic[t]; cursor_j = (int)jc[t]; Sleep(200);
				cout << "\ncursor on " << TARGET_NAME[t] << " -- arrow keys to move, C to confirm"; }
		}
		if (cursor_target >= 0) {
			if (KEY(VK_UP))    { cursor_j -= 3; Sleep(50); }
			if (KEY(VK_DOWN))  { cursor_j += 3; Sleep(50); }
			if (KEY(VK_LEFT))  { cursor_i -= 3; Sleep(50); }
			if (KEY(VK_RIGHT)) { cursor_i += 3; Sleep(50); }
			if (cursor_i < 0) cursor_i = 0; if (cursor_i > IMAGE_WIDTH-1)  cursor_i = IMAGE_WIDTH-1;
			if (cursor_j < 0) cursor_j = 0; if (cursor_j > IMAGE_HEIGHT-1) cursor_j = IMAGE_HEIGHT-1;
			// draw cursor in target colour
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
			cout << "\n--- centroids  frame=" << (int)(last_elapsed*1e6) << " us  (" << (int)(last_elapsed*1000) << " ms) ---";
			for (int t = 0; t < NTARGETS; t++)
				cout << "\n  " << TARGET_NAME[t] << "  ic=" << ic[t] << "  jc=" << jc[t];
			Sleep(300);
		}

		if (KEY('X')) break;
	}

	// save masks on exit
	save_rgb_image("rgb0.bmp",  rgb0);
	save_rgb_image("rgb1.bmp",  rgb1);
	copy(mask_r, rgb); save_rgb_image("mask_r.bmp", rgb);
	copy(mask_g, rgb); save_rgb_image("mask_g.bmp", rgb);
	copy(mask_b, rgb); save_rgb_image("mask_b.bmp", rgb);
	copy(mask_y, rgb); save_rgb_image("mask_y.bmp", rgb);
	copy(a,      rgb); save_rgb_image("combined.bmp", rgb);

	return 0;
}

//===TRACKING THREAD LVL2===
int search_object_circular(i2byte &nlabel, image &label, int is, int js, int min_area, int max_area)
{
	i2byte *pl = (i2byte *)label.pdata;

	nlabel = *(pl + js * label.width + is);
	if (nlabel != 0) return 0; // if label persists from frame to frame then exit (ideal case - target found)

	//Search Loop - Currently spirals outward in search for a label of similar size (label size+- tolerance)

	/*
	TO DO: 
	-Optimize this for predicted motion of label (perhaps based on last recorded heading direction)
	-Implement Kalman filter?
	-Match based on other label info(colour?)
	-Requires a label_info function
	*/


	double rmax = 60.0, dr = 3.0, ds = 3.0;

	for (double r = 1.0; r <= rmax; r += dr) {
		double smax = 2 * 3.1416 * r;
		for (double s = 0; s <= smax; s += ds) {
			double theta = s / r;
			int i = (int)(is + r * cos(theta));
			int j = (int)(js + r * sin(theta));

			if (i < 0) i = 0; if (i > label.width-1)  i = label.width-1;
			if (j < 0) j = 0; if (j > label.height-1) j = label.height-1;

			nlabel = *(pl + j * label.width + i);
			
			//Area check
			if (nlabel != 0) {
				int area = label_area(label, nlabel);
				if (area >= min_area && area <= max_area) return 0;
				nlabel = 0;
			}
		}
	}
	return 0;
}

//===LEVEL 3 FUNCTION===
int label_area(image &label, i2byte nlabel)
{
	i2byte *pl = (i2byte *)label.pdata;
	int area = 0;
	for (int k = 0; k < label.width * label.height; k++)
		if (pl[k] == nlabel) area++;
	return area;
}

//===LEVEL 3 FUNCTION===
int label_filtered_objects()
{
	int nlabels;
	int np = IMAGE_WIDTH * IMAGE_HEIGHT;

	// scale rgb0 -> rgb1
	copy(rgb0, rgb1);
	scale(rgb1, rgb1);

	// greyscale of rgb1 -> b, then gaussian -> grey_gauss
	copy(rgb1, b);
	gaussian_filter(b, grey_gauss);

	// build per-colour hue masks from grey_gauss + rgb1
	threshold_mask(grey_gauss, mask_r, rgb1, pm_r);
	threshold_mask(grey_gauss, mask_g, rgb1, pm_g);
	threshold_mask(grey_gauss, mask_b, rgb1, pm_b);
	threshold_mask(grey_gauss, mask_y, rgb1, pm_y);

	// combine into ax
	ibyte* pr = mask_r.pdata; ibyte* pg = mask_g.pdata;
	ibyte* pb = mask_b.pdata; ibyte* py = mask_y.pdata;
	ibyte* pa = a.pdata;
	for (int k = 0; k < np; k++)
		pa[k] = (pr[k] || pg[k] || pb[k] || py[k]) ? 255 : 0;

	//cleanup
	erode(a, b);
	dialate(b, a);

	label_image(a, label, nlabels);
	return 0;
}

