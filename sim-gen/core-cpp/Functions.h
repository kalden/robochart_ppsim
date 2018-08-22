
#ifndef ROBOCALC_FUNCTIONS_H_
#define ROBOCALC_FUNCTIONS_H_

#include "DataTypes.h"

int calculate_high_grid_square(TreeMap chemomap);
double calculate_angle_from_direction(int direction);
int collision_check(Double2D cell_loc, Tracking_Stats tracking, int cell_id, std::string cellType);
bool detect_cell_bind(Double2D loc, std::string cellType);
double measure_chemokine_expression(Double2D loc, TreeMap chemomap);
Double2D calculateNewPosition(Double2D cell_loc, double angle_to_move, double movement_interval);
int set_ltin_contact_time();
double probability_responds_to_chemokine(Double2D cell_loc, double totalchemolevels, TreeMap chemomap);
double calculate_probability_adhesion(int contactedCell_ID);
int set_lti_contact_time();
double set_cell_speed(double random_num, double lower_bound, double upper_bound);
double avoidCellCollision(Double2D cell_loc, double distanceToMove, int contactedCell_ID);
bool check_valid_location_on_grid(Double2D cell_loc);

#endif
