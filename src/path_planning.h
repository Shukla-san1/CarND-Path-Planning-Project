
#include <iostream>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <fstream>
#include <math.h>

using namespace std;

//using std::vector;

class PathPlanner
{
	private:
		double lane_width = 4.0;
		double speed_limit = 49.5;
		double side_dist = 20;
		double closest_dist = 20;


	public:
		double long_dist = 20;

		void GetCurrentReferenceVelocity(int lane, double ego_car_s, double car_d, double car_s,
				double car_vx, double car_vy, int prev_size, double& ref_vel, bool& too_close,
				double& closest);

		void KeepLane(const vector<double>& map_waypoints_x, const vector<double>& map_waypoints_y,
				const vector<double>& map_waypoints_s, const vector<double>& previous_path_x,
				const vector<double>& previous_path_y, double car_x, double car_y, double car_yaw,
				double car_speed, double car_s, double ref_vel, int lane, vector<double>& next_x_vals,
				vector<double>& next_y_vals);

		bool IsChangeLaneSafe(int keep_lane, double min_left_dist, double min_right_dist, double min_long_dist, int& lane);

		void ChangeLane(bool too_close, double min_left_dist, double min_right_dist, double min_long_dist, int &lane, int& keep_lane);

		void GetReferenceVelocity(int lane, double ego_car_s, double ego_car_d,
				const vector<vector<double>>& sensor_fusion, int& prev_size, double& ref_vel,
				double& min_left_dist, double& min_right_dist, double& min_long_dist, bool& too_close, double& closest);

		void Drive(double ego_car_x, double ego_car_y, double ego_car_yaw, double ego_car_speed, double ego_car_s, double ego_car_d, double end_path_s, const vector<vector<double>>& sensor_fusion, const vector<double>& map_waypoints_x, const vector<double>& map_waypoints_y, const vector<double>& map_waypoints_s, const vector<double>& previous_path_x, const vector<double>& previous_path_y, vector<double>& next_x_vals, vector<double>& next_y_vals, int& lane, int& keep_lane);

};
