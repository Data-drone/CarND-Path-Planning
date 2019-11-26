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

int getLane(double d_value) {

  // return the lane of the vehicle
  if (d_value > 0 && d_value < 4) {
    return 0;
  } else if (d_value > 4 && d_value < 8) {
    return 1;
  }

  return 2;
}

// checks which lanes are viable to switch into
vector<int> laneOptions(int cur_lane) 
{
  vector<int> options;
  if (cur_lane == 0) 
  {
    options.push_back(1);
  } else if (cur_lane == 1) {
    options.push_back(0);
    options.push_back(2);
  } else if (cur_lane == 2) {
    options.push_back(1);
  }

  return options;
}

// checks if there are cars in front that are too close
bool tooFrontClose(vector<double> sensor_fusion, int size, double my_car_s) 
{
  double vx = sensor_fusion[3];
  double vy = sensor_fusion[4];
  double check_speed = sqrt(vx*vx+vy*vy);
  double check_car_s = sensor_fusion[5];

  check_car_s += ((double)size*.02*check_speed);
              
  if((check_car_s>my_car_s) && (check_car_s-my_car_s) < 50)
  {
  // do some logic to prevent crash
    return true;
  }
              
  return false;
}

// checks if there are cars behind that are too cloes for a lane change
bool tooCloseEitherWay(vector<double> sensor_fusion, int size, double my_car_s) 
{
  double vx = sensor_fusion[3];
  double vy = sensor_fusion[4];
  double check_speed = sqrt(vx*vx+vy*vy);
  double check_car_s = sensor_fusion[5];

  check_car_s += ((double)size*.02*check_speed);
              
  if( std::abs(check_car_s-my_car_s) < 30)
  {
  // do some logic to prevent crash
    return true;
  }
              
  return false;
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

  // code from tutorial
  // add lane numbers and reference speeds
  // we need to move this
  int lane = 1;
  double ref_vel = 0.0; //mph

  h.onMessage([&lane, &ref_vel, &map_waypoints_x, &map_waypoints_y,&map_waypoints_s,
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

          int prev_size = previous_path_x.size();

          // stop cars coliding
          if(prev_size > 0)
          {
            car_s = end_path_s;
          }

          bool too_close = false;
          bool too_close_for_change = false;

          // work out current lane and see what we can do:
          int car_current_lane = getLane(car_d);
          //std::cout << car_current_lane << std::endl;

          vector<int> lane_options = laneOptions(car_current_lane);

          for(int i = 0; i < sensor_fusion.size(); i++) 
          {
            float d = sensor_fusion[i][6];
            // d determines the lane - > for each car find the lane
            // then work out whether it is in front of behind our car

            // get car lane of detected car
            int other_car_lane = getLane(d);

            // in the other car is in our lane
            if (other_car_lane == car_current_lane)
            {
              too_close = tooFrontClose(sensor_fusion[i], prev_size, car_s);
            } 

            if (std::find(lane_options.begin(), lane_options.end(), other_car_lane) != lane_options.end() ) 
            {
              too_close_for_change = tooCloseEitherWay(sensor_fusion[i], prev_size, car_s);

              if (too_close_for_change) {
                // add it to lane_options
               lane_options.erase(std::remove(lane_options.begin(), lane_options.end(), other_car_lane), lane_options.end());
              } 
            }
          }

          // std::cout << "too_close: " << too_close << " too_close_change: " << too_close_for_change << std::endl;


          // we need to spit out options here after the detect routine
          // we should know if cars to left or right options to go left or right
          // maybe a cost function result

          // uses too close flag to increase or decrease the speed
          if (too_close) 
          {
            ref_vel -= .224;
          }
          else if (ref_vel < 49.5)
          {
            ref_vel += .224;
          }

          if (too_close && !lane_options.empty()) {
            lane = lane_options[0];
          }

          //to finish

          vector<double> ptsx;
          vector<double> ptsy;

          // current car pos
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // previous trajectory
          // integrate previous trajectory to stop jerk
          if(prev_size < 2) 
          {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);

          } else 
          {

            // previous reference state
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);

          }

          // in Frenet add evenly 30m spaced points ahead of the starting ref
          // lane chooses lane
          vector<double> next_wp0 = getXY(car_s + 30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for (int i = 0; i < ptsx.size(); i++)
          {
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;

            ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
          }

          // spline logic to smooth out waypoints
          tk::spline s;

          s.set_points(ptsx,ptsy);

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for(int i=0; i<prev_size;i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

          double x_add_on = 0;

          for (int i = 1; i <= 50 - previous_path_x.size(); i++) 
          {

            double N = (target_dist/(.02*ref_vel/2.24)); // ref_vel better to plan accel decell here
            double x_point = x_add_on+(target_x)/N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            // generate the next waypoints to guide car
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          // end of custom code

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