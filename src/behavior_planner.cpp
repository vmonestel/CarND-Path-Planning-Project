#include "behavior_planner.hpp"
#include <algorithm>

#define VEHICLE_GAP 30
#define VEHICLE_GAP_BEHIND 30
#define MAX_SPEED 49.5
#define SPEED_CHANGE 0.25

vector<bool> BehaviorPlanner::sensor_fusion_data_analize(
		double s,
		uint8_t lane,
		vector<vector<double>> sensor_fusion)
{
  double vehicle_s;
  double vehicle_d;
  double vehicle_speed;
  uint8_t vehicle_lane;

  bool car_ahead = false;
  bool car_on_the_right = false;
  bool car_on_the_left = false;


  // Check each vehicle reported in sensor fusion data
  for (int vehicle = 0; vehicle < sensor_fusion.size(); vehicle++) {
    vehicle_speed = sqrt(sensor_fusion[vehicle][3] * sensor_fusion[vehicle][3] +
    		sensor_fusion[vehicle][4] * sensor_fusion[vehicle][4]);
    vehicle_s = sensor_fusion[vehicle][5];
    vehicle_d = sensor_fusion[vehicle][6];

    if (vehicle_d < 4) {
    	vehicle_lane = 0;
    } else if (vehicle_d < 8) {
    	vehicle_lane = 1;
    } else {
    	vehicle_lane = 2;
    }

    // Check if lanes are the same
    if (vehicle_lane == lane) {
      // Check if there is a car ahead (GAP is less than 30m)
      if ((vehicle_s > s) && ((vehicle_s - s) < VEHICLE_GAP)) {
    	  car_ahead = true;
      }
    } else if (vehicle_lane > lane) { // Check if a car is on the right lane
      // Check the gap (30m/20m ahead/behind) on the right
      if ((vehicle_s > (s - VEHICLE_GAP_BEHIND)) && ((vehicle_s - s) < VEHICLE_GAP)) {
    	  car_on_the_right = true;
      }
    } else if (vehicle_lane < lane) { // Check if a car is on the left lane
      // Check the gap (30m/20m ahead/behind) on the left
      if ((vehicle_s > (s -VEHICLE_GAP_BEHIND)) && ((vehicle_s - s) < VEHICLE_GAP)) {
    	  car_on_the_left = true;
      }
    }
  }
  return {car_ahead, car_on_the_right, car_on_the_left};
}

void BehaviorPlanner::decision_take(
		vector<bool> car_detection,
		uint8_t *lane,
		double *reference_velocity)
{
    bool car_ahead = car_detection[0];
    bool car_on_the_right = car_detection[1];
    bool car_on_the_left = car_detection[2];

    if (car_ahead) {
    	/*
    	 * There is a car close on current lane so the velocity should be reduced
    	 * to avoid a collision
    	 */
    	*reference_velocity -= SPEED_CHANGE;

    	// Change lane if possible when there a car in front of us
    	if ((*lane == 0) && !car_on_the_right) {
    		// We are on the left lane and there is a chance to move right
    		*lane = 1;
    	} else if ((*lane == 2) && !car_on_the_left) {
    		// We are on the right lane and there is a chance to move left
    		*lane = 1;
    	} else if (*lane == 1) {
    		// we are in the middle lane
    		if (!car_on_the_left) {
    			// There is a chance to move to the right
    			*lane = 0;
    		} else if (!car_on_the_right) {
    			// There is a chance to move to the left
    			*lane = 2;
    		}
    	}
    } else if (*reference_velocity < MAX_SPEED) {
    	// If we are not at max speed and there is not a car in front, then speed up
    	*reference_velocity += SPEED_CHANGE;
    }
}
