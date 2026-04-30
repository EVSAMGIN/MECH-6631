#pragma once

// The Obstacle class defines all objects that may be on the field.
class obstacle
{
	protected:
		int *obstacle_centroid_x;
		int *obstacle_centroid_y;
		int diameter, keep_out;

	public:

		// Default Constructor places obstacle outside of world map
		obstacle();

		// Preferred Constructor places obstacle where it is found on the world map
		obstacle(int *centroid_x, int *centroid_y, int obstacle_diameter, int obstacle_keep_out);

		// Getters

		int get_center_x();
		int get_center_y();
		int get_diameter();
		int get_keep_out();
		int obstacle_radius();

		// Setters

		void set_center_x(int* new_center_x);
		void set_center_y(int* new_center_y);
		void set_diameter(int new_diameter);
		void set_keep_out(int new_keep_out);

};

