#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Calculates Euclidean distance
double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Map of waypoints around highway is given, we find closest here.
// Beware! Closest can also be behind
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

// Find next based on angle, so you move forward not backward if there is closest point behind you.
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
    if (closestWaypoint == maps_x.size())
    {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
    }

    int ego_lane = 1; // start in lane 1
    double ref_v = 0.0; // to give speed limit

  h.onMessage([&ref_v, &ego_lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];


          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];
            int prev_size = previous_path_x.size();

            if (prev_size > 0) {
              car_s = end_path_s; // we start from the last value
            }


            /*****************************************************************************
              *  Prediction based on Sensor Fusion
            ****************************************************************************/

            // Lane markers for other cars around the ego car as reported by sensor fusion
            bool too_close = false;
            bool car_left = false;
            bool car_right = false;

            // check data from sensor fusion
            for (int i=0; i<sensor_fusion.size(); i++) {
              // check if car is our lane
              float d = sensor_fusion[i][6];

              // Find the lane of the car
              int car_lane;
              if (d >= 0 && d < 4) {
                  car_lane = 0;
              } else if (d >= 4 && d < 8) {
                  car_lane = 1;
              } else if (d >= 8 && d <= 12) {
                  car_lane = 2;
              } else {
                  continue;
              }


              // Check the details of the car in the other lane
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double other_car_speed = sqrt(vx*vx+vy*vy);
              double other_car_s = sensor_fusion[i][5];

              // where the car is in future needs to be derived
              // We have to check the s-distance from our car.
              double other_car_s_future = other_car_s + ((double)prev_size * 0.02 * other_car_speed);

              int ego_gap = 30; // gap we want to keep is 30 meters

              // Check the lane details of the car.
              if (car_lane == ego_lane) {
                // Other car is ahead of ego
                too_close |= (other_car_s_future > car_s) && ((other_car_s_future - car_s) < ego_gap);
              } else if (car_lane - ego_lane == 1) {
                // Other car is to the right
                car_right |= ((car_s - ego_gap) < other_car_s_future) && ((car_s + ego_gap) > other_car_s_future);
              } else if (ego_lane - car_lane == 1) {
                // Other car is to the left
                car_left |= ((car_s - ego_gap) < other_car_s_future) && ((car_s + ego_gap) > other_car_s_future);
              }

              }

            /*****************************************************************************
              *  Behaviour Planning
            ****************************************************************************/
            // Behaviour planning state machine, work out safe speed and lane to move.
            // Increase or decrease speed in steps, to avoid jerks
            // Keep up the speed limit but not overflow.
            // Switch lanes when safe to do so
            double acc = 0.224;
            double max_speed = 49.5;
            if (too_close) {
                // Other car is infront, possibility of collission or being slowed down
                // Decide to shift lanes or slow down if necessary to avoid collission
                if (!car_right && ego_lane < 2) {
                    // Checks there is no car in the right, safe to move to the right lane.
                    // Shift right
                    ego_lane++;
                } else if (!car_left && ego_lane > 0) {
                  // Checks there is no car in the left, safe to move to the left lane.
                  // Shift left
                    ego_lane--;
                } else {
                    // Not safe to shift left or right as there are cars.
                    // Slow down to avoid collission
                    ref_v -= acc;
                }
            } else {
                if (ego_lane != 1) {
                    // Currently not in the center lane, as we want to position the car.
                    // Check if it is safe to move back to lane 1 or else keep the current lane.
                    if ((ego_lane == 2 && !car_left) || (ego_lane == 0 && !car_right)) {
                        // Move back to the center lane
                        ego_lane = 1;
                    }
                }

                if (ref_v < max_speed) {
                    // There is no car ahead and we are not yet at the maximum speed limit.
                    // Accelerate smoothly to increase speed slowly.
                    ref_v += acc;
                }
            }

            /*****************************************************************************
              *  Trajectory Generation
            ****************************************************************************/
            // create a list of waypoints spaced at 30m and interpolate using spline
            // Idea is to take two points from previous path and predict three points
            // Use spine to then create path points distanced by 0.5 m
            vector<double> waypts_x;
            vector<double> waypts_y;

            // reference states for x, y and yaw for creating way_points
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

            if (prev_size < 2) { // previous size is empty
              // two points that is tangential to the position of the car
              double prev_x = car_x - cos(car_yaw);
              double prev_y = car_y - sin(car_yaw);

              waypts_x.push_back(prev_x);
              waypts_x.push_back(ref_x);

              waypts_y.push_back(prev_y);
              waypts_y.push_back(ref_y);

            }
            else {
              // take last point from previous path as reference
              ref_x = previous_path_x[prev_size - 1];
              ref_y = previous_path_y[prev_size - 1];


              // take second last point as well
              double prev_ref_x = previous_path_x[prev_size - 2];
              double prev_ref_y = previous_path_y[prev_size - 2];

              ref_yaw = atan2(ref_y-prev_ref_y, ref_x-prev_ref_x);


              // update the last two elements to the waypoint that we are creating
              waypts_x.push_back(prev_ref_x);
              waypts_x.push_back(ref_x);

              waypts_y.push_back(prev_ref_y);
              waypts_y.push_back(ref_y);
            }

            // In Frenet coordinates, find and add 30m spaced waypoints ahead
            // Just predicting straught path to car's current position values.

            vector<double> wpts_next_0 = getXY(car_s+30, (2+4*ego_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> wpts_next_1 = getXY(car_s+60, (2+4*ego_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> wpts_next_2 = getXY(car_s+90, (2+4*ego_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

            // update the predicted future 3 elements to the waypoint that we are creating
            waypts_x.push_back(wpts_next_0[0]);
            waypts_x.push_back(wpts_next_1[0]);
            waypts_x.push_back(wpts_next_2[0]);

            waypts_y.push_back(wpts_next_0[1]);
            waypts_y.push_back(wpts_next_1[1]);
            waypts_y.push_back(wpts_next_2[1]);

            // shift way points to car coordinates
            for (int i = 0; i < waypts_x.size(); i++) {
                double shift_x = waypts_x[i] - ref_x;
                double shift_y = waypts_y[i] - ref_y;

                waypts_x[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
                waypts_y[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
            }

            // Create spline to get points for path between the 30m points we predicted
            tk::spline s;

            // set (x,y) points to the spline
            s.set_points(waypts_x, waypts_y);


          	json msgJson;

            // Path planner should provide these, as this is the path car would follow.
          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

            // We will start with points from previous Path Planner
            for (int i=0; i<prev_size; i++) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            // calculate how to break the spline points to make sure we choose
            // points that allow us to travel in our reference velocity

            double target_x = 30.0; // Total target distance we want to move ahead
            double target_y = s(target_x); // splie can give equivalent y point
            double target_dist = sqrt((target_x * target_x) + (target_y * target_y));

            double x_add_on = 0;
            // Fill rest of path planner after filling with previous set_points
            // We are looking for 50 points. There can be points that car didnt cover yet
            // from previous points. Depends in simulator speed etc.
            // We just decided we want 50, we can have more as well.

            for (int i = 1; i <= 50 - prev_size; i++) {
              double N = target_dist/(0.02*ref_v/2.24); // getting number of points we need, m/s
              double x_point = x_add_on + target_x/N;
              double y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              // gte back global coordinates
              x_point = x_ref * cos(ref_yaw)-y_ref*sin(ref_yaw);
              y_point = x_ref * sin(ref_yaw)+y_ref*cos(ref_yaw);

              // Add to the reference x, which is last point from previous
              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);

            }

            msgJson["next_x"] = next_x_vals;
            msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
