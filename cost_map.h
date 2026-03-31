#pragma once

#include "vehicle.h"
#include "obstacle.h"

class cost_map
{
private:
	int map_width, map_height;
	int* start_x, start_y, goal_x, goal_y, costs;

public:
	void update_path();

	cost_map();

	cost_map(int width, int height, vehicle* controlled, vehicle* enemy);

	cost_map(int width, int height, vehicle* controlled, vehicle* enemy, obstacle* obstacle1);

	cost_map(int width, int height, vehicle* controlled, vehicle* enemy, obstacle* obstacle1, obstacle* obstacle2);

	cost_map(int width, int height, vehicle* controlled, vehicle* enemy, obstacle* obstacle1, obstacle* obstacle2, obstacle* obstacle3);

};

