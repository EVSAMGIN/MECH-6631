#include "cost_map.h"

void cost_map::update_path()
{


	if (start_x < goal_x && start_y < goal_y) {
		start_x + 1;
		start_y + 1;
	} else if (start_x < goal_x && start_y > goal_y) {

	} else if (start_x > goal_x && start_y < goal_y) {

	} else if (start_x > goal_x && start_y > goal_y) {

	}
}

cost_map::cost_map()
{
	map_width = 200;
	map_height = 300;

	//start_x = -30;
	//start_y = -30;

	//goal_x = -29;
	//goal_y = -29;

	//costs = new int[map_width * map_height];

	//update_cost_map();
}

cost_map::cost_map(int width, int height, vehicle* controlled, vehicle* enemy)
{
	map_width = width;
	map_height = height;

	start_x = (int) (*controlled->get_vehicle_center_x() + 0.5);
	start_y = (int) (*controlled->get_vehicle_center_y() + 0.5);

	goal_x = (int) (*enemy->get_vehicle_center_x() + 0.5);
	goal_y = (int) (*enemy->get_vehicle_center_y() + 0.5);

	costs = new int[map_width * map_height];

	update_path();
}

cost_map::cost_map(int width, int height, vehicle* controlled, vehicle* enemy, obstacle* obstacle1)
{
}

cost_map::cost_map(int width, int height, vehicle* controlled, vehicle* enemy, obstacle* obstacle1, obstacle* obstacle2)
{
}

cost_map::cost_map(int width, int height, vehicle* controlled, vehicle* enemy, obstacle* obstacle1, obstacle* obstacle2, obstacle* obstacle3)
{
}
