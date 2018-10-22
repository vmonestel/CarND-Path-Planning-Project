#ifndef behavior_planner_hpp
#define behavior_planner_hpp

#include <math.h>
#include <vector>
#include <string>

using namespace std;

class BehaviorPlanner {
  public:
    /*
     * Analyze the sensor fusion data to see calculate other cars
     * position.
     */
    vector<bool> sensor_fusion_data_analize(
    		double s,
    		uint8_t lane,
    		vector<vector<double>> sensor_fusion);

    /*
     * Take a decision based on sensor data analysis.
     */
    void decision_take(
    		vector<bool> car_detection,
			uint8_t *lane,
    		double *reference_velocity);
};

#endif /* behavior_planner_hpp */
