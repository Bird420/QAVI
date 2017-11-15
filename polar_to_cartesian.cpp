#include <cmath>
#include <iostream>

/*
 * This function will translate polar coordinates
 * into Cartesian. 
 *
 */
int polarToCartesian(float sensor_data, int sensor_points) 
{
  int cartesian_coordinates[720][2];
  int i, j;
  float angle = 1*M_PI/540;
  for(i = 0; i < 720; i++) {
    angle *= i;
    cartesian_coordinates[i][1] = width_matrix / 2 + sensor_data[i] * cos(angle)
    cartesian_coordinates[i][1] = sensor_data[i] * sin(angle)
  }
  return cartesian_coordinates;
}