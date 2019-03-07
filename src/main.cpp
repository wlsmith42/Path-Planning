#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  //The vehicle's current lane. Starting position is the middle lane, lane 1
  int curr_lane = 1;

  //The vehicle's reference velocity, used for acceleration when traveling below the desired speed (mph)
  double ref_vel = 0.0;

  h.onMessage([&curr_lane, &ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];


          int prev_path_size = previous_path_x.size();

          //If a previous path exists, sync the s-coordinate to prevent lateral collisions
          if(prev_path_size > 0) {
            car_s = end_path_s;
          }

          //Analyze surrounding vehicles' positions
          bool car_ahead = false;
          bool car_left = false;
          bool car_right = false;

          //For each vehicle detected by the sensor fusion layer:
          for(int i=0; i<sensor_fusion.size(); i++) {
              float d = sensor_fusion[i][6];
              int car_lane = -1;

              //A lane is roughly 4 units wide on the d axis
              //This can be used to find the lane based on the d value
              if(d > 0 && d < 4) {
                car_lane = 0;
              } else if (d > 4 && d < 8) {
                car_lane = 1;
              } else if (d > 8 && d < 12) {
                car_lane = 2;
              }
              if(car_lane < 0) {
                continue;
              }

              //Find vehicle's speed
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx + vy*vy);
              double check_car_s = sensor_fusion[i][5];

              //Estimate the vehicle's position given previous trajectory
              check_car_s += ((double)prev_path_size*0.02*check_speed);


              //controls the distance along the s axis that the vehicle will get to other vehicles in front of it
              int front_pad_dist = 30;
              int rear_pad_dist = 10;

              //See if the surrounding lanes are clear within the padding distance
              if(car_lane == curr_lane) {
                car_ahead |= check_car_s > car_s && check_car_s - car_s < front_pad_dist;
              } else if(car_lane - curr_lane == -1) {
                car_left |= car_s - rear_pad_dist < check_car_s && car_s + front_pad_dist > check_car_s;
              } else if(car_lane - curr_lane == 1) {
                car_right |= car_s - rear_pad_dist < check_car_s && car_s + front_pad_dist > check_car_s;
              }
          }

          //Create our vehicle's path plan
          double speed_change = 0.0;
          const double MAX_SPEED = 49.8;
          const double MAX_ACCEL = 0.3;

            //Determine if lane change is needed
            if(car_ahead) {
              if(!car_left && curr_lane > 0) {
                //Pass to the left
                curr_lane--;
              } else if(!car_right && curr_lane != 2) {
                //Pass to the right
                curr_lane++;
              } else {
                //Can't pass, slow down
                speed_change -= MAX_ACCEL;
              }
            }
            else {
              if (curr_lane !=1) {
                //Vehicle is not in the center center lane
                if((curr_lane == 0 && !car_right) || (curr_lane == 2 && !car_left)) {
                  //If there is no other vehicle in the way, move back to center.
                  curr_lane = 1;
                }
              }
              if(ref_vel < MAX_SPEED) {
                //Car is traveling below optimial speed, accelerate!
                speed_change += MAX_ACCEL;
              }
            }

            //Create vector of points to travel
            vector<double> x_pts;
            vector<double> y_pts;

            //Save the car's position as reference values
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

            //Check if previous points exist
            if(prev_path_size < 2) {
              //Not enough points to continue path or new path started
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              x_pts.push_back(prev_car_x);
              x_pts.push_back(car_x);

              y_pts.push_back(prev_car_y);
              y_pts.push_back(car_y);
            }
            else {
              //Continue with last two points of previous path
              ref_x = previous_path_x[prev_path_size - 1];
              ref_y = previous_path_y[prev_path_size - 1];

              double ref_x_prev = previous_path_x[prev_path_size - 2];
              double ref_y_prev = previous_path_y[prev_path_size - 2];
              ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

              x_pts.push_back(ref_x_prev);
              x_pts.push_back(ref_x);

              y_pts.push_back(ref_y_prev);
              y_pts.push_back(ref_y);
            }

            //Set 3 target waypoints
            vector<double> waypt0 = getXY(car_s + 30, 2 + 4*curr_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            x_pts.push_back(waypt0[0]);
            y_pts.push_back(waypt0[1]);

            vector<double> waypt1 = getXY(car_s + 60, 2 + 4*curr_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            x_pts.push_back(waypt1[0]);
            y_pts.push_back(waypt1[1]);

            vector<double> waypt2 = getXY(car_s + 90, 2 + 4*curr_lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            x_pts.push_back(waypt2[0]);
            y_pts.push_back(waypt2[1]);


            //Translate coordinates to car coordinates
            for(int i=0; i<x_pts.size(); i++) {
              double x_shift = x_pts[i] - ref_x;
              double y_shift = y_pts[i] - ref_y;

              x_pts[i] = x_shift * cos(0 - ref_yaw) - y_shift * sin(0 - ref_yaw);
              y_pts[i] = x_shift * sin(0 - ref_yaw) + y_shift * cos(0 - ref_yaw);
            }

            //Create spline
            tk::spline s;
            s.set_points(x_pts, y_pts);

            //Output points from prevous path to smooth transitions
            vector<double> next_x_vals;
            vector<double> next_y_vals;

            for(int i=0; i<prev_path_size; i++) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            //Calcualte distance to target 30m ahead
            double x_target = 30.0;   //Distance incremented each cycle
            double y_target = s(x_target);
            double target_dist = sqrt(x_target*x_target + y_target*y_target);

            double x_add_on = 0;

            //Loop through each point in the plan
            for(int i=1; i<50 - prev_path_size; i++) {

              //Update speed
              ref_vel += speed_change;
              if(ref_vel > MAX_SPEED) {
                ref_vel = MAX_SPEED;
              }
              else if(ref_vel < MAX_ACCEL) {
                ref_vel = MAX_ACCEL;
              }

              double N = target_dist/(0.02*ref_vel/2.24);
              double x_pt = x_add_on + x_target/N;
              double y_pt = s(x_pt);

              x_add_on = x_pt;

              double x_ref = x_pt;
              double y_ref = y_pt;

              x_pt = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
              y_pt = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

              x_pt += ref_x;
              y_pt += ref_y;

              next_x_vals.push_back(x_pt);
              next_y_vals.push_back(y_pt);
            }

          json msgJson;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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
