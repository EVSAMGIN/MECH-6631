#pragma once

// Inspired by https://github.com/JDSherbert/A-Star-Pathfinding/blob/main/Pathfinder.h

#include <vector>

struct Path_Node {
	double x, y; // Coordinates for path node
	double f, g, h; // Cost values held by the node

	Path_Node(double _x = 0, double _y = 0);

	// Overload comparison operators for priority queue:
	bool operator>(const Path_Node& other) const;
	bool operator==(const Path_Node& other) const;
};

void a_star_path(double& body_ic, double& body_jc, double& goal_ic, double& goal_jc, double& body_next_ic, double& body_next_jc);