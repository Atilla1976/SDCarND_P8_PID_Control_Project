#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

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
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid;
  /**
   * TODO: Initialize the pid variable.
   */
  // pid.Init(0.2, 0.000, 0.0); .... case 1: coefficient kp first try 0,2, ki and kd = 0; best case until now
  // pid.Init(0.5, 0.000, 0.0); .... case 2: coefficient kp higher; ... to much oszillation
  // pid.Init(1.0, 0.000, 0.0); .... case 3: coefficient kp even higher; ... as expected even worse oszillation
  // pid.Init(0.1, 0.000, 0.0); .... case 4: coefficient kp lower than case 1; ... better result
  // pid.Init(0.05, 0.000, 0.0); ... case 5: coefficient kp lower than case 4; ... better result
  // pid.Init(0.01, 0.000, 0.0); ... case 6: coefficient kp lower than case 5; ... worse result
  // pid.Init(0.03, 0.000, 0.0); ... case 7: coefficient kp higher than case 6; ... farthest distance traveled so far, but too soft correction
  // pid.Init(0.08, 0.000, 0.0); ... case 8: coefficient kp higher than case 5; ... kp range between 0.5 an 1.0 gets the best results
  // pid.Init(0.08, 0.000, 3.0); ... case 9: next coefficient -> kd; first try kd = 3.0 ... car could pass the complete track but with to much oszillation
  // pid.Init(0.08, 0.000, 2.0); ... case 10: coefficient kd lower than case 9 .... better result
  // pid.Init(0.08, 0.000, 1.0); ... case 11: coefficient kd lower than case 10 ... partly dangerously close the the lane lines
  // pid.Init(0.08, 0.000, 1.5); ... case 12: coefficient kd higher than case 11;  ... slightly better in lane line area
  // pid.Init(0.08, 0.000, 1.7); ... case 13: coefficient kd higher than case 12;  ... again slightly better in lane line area but not really good
  // pid.Init(0.08, 0.004, 1.7); ... case 14: next coefficient -> ki; first try ki = 0.004 ... car could not keep the lane 
  // pid.Init(0.08, 0.001, 1.7); ... case 15: coefficient ki lower than case 14;  ... car could not keep the lane but with strong oszillation
  // pid.Init(0.09, 0.001, 1.7); ... case 15: coefficient ki lower than case 14;  ... car could not keep the lane but with strong oszillation
  // pid.Init(0.12, 0.0005, 1.8); // .. case 16: coefficient ki lower than case 15;  ...best result so far
  // pid.Init(0.12, 0.0007, 1.9);
  // pid.Init(0.13, 0.00095, 1.9);  car inside lane lines
  // pid.Init(0.13, 0.00095, 1.7); car inside lane lines
  // pid.Init(0.2, 0.00095, 1.68);
  pid.Init(0.12, 0.0001, 1.62);
  
  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          pid.UpdateError(cte);
          steer_value = pid.TotalError();
          if(steer_value > 1.0) steer_value = 1.0;
          if(steer_value < -1.0) steer_value = -1.0;
          
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
                    << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3; //(1 - 3 * steer_value)*0.35;   // Formula switches to between [0, 1], larger steering angle means less throttle. Multiplied by 0.35 for safety reasons.
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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
