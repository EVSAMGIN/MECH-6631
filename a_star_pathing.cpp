// Inspired by: https://github.com/JDSherbert/A-Star-Pathfinding/blob/main/Pathfinder.cpp

#include <queue>
#include "a_star_pathing.h"

Path_Node::Path_Node(double _x = 0, double _y = 0)
	: x(_x)
	, y(_y)
	, f(0)
	, g(0)
	, h(0)
{
}

bool Path_Node::operator>(const Path_Node& other) const
{
	return f > other.f;
}

bool Path_Node::operator==(const Path_Node& other) const
{
	return x == other.x && y == other.y;
}

void a_star_path(double& body_ic, double& body_jc, double& goal_ic, double& goal_jc, double& body_next_ic, double& body_next_jc)
{
	// Create a min-heap for A* algorithm
	//
	// Format will be x then y
	std::priority_queue<Path_Node, std::vector<Path_Node>, std::greater<Path_Node> > path_storage;


}