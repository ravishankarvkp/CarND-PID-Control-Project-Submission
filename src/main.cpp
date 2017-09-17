#define _USE_MATH_DEFINES 

#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  // TODO: Initialize the pid variable.
  // steering and throttle PID controllers
  PID pid_steering;
  double throttle_spd = 0.5;

  // Trial & Error - PID coefficients
  ////pid_steering.Init(0.35, 0.02, 0.005); ==> works at 10 mph
  ////pid_steering.Init(0.2, 0.004, 3.0); ==> works at 40 mph (taken from sample lesson)
  pid_steering.Init(0.202931, -1.05288e-05, 2.99888);
  
  h.onMessage([&pid_steering, &throttle_spd](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
		  double steer_value, throttle_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

		  // update error and calculate steer_value at each step
		  pid_steering.UpdateError(cte);
		  steer_value = pid_steering.TotalError();

		  //Apply range limit to -1 ... 1.
		  if (steer_value < -1.0)
		  {
			  steer_value = -1.0;
		  }
		  else if (steer_value > 1.0)
		  {
			  steer_value = 1.0;
		  }

		  // Regulate the throttle, depending on the CTE, when speed is above 30 mph.
		  // if CTE is less than 0.6 meters, then increase the speed by 6%, 
		  // else reduce speed by 12%
		  if (speed > 30)
		  {
			  if (fabs(cte) < 0.6)
			  {
				  throttle_spd += throttle_spd * 0.06;
			  }
			  else
			  {
				  throttle_spd -= throttle_spd * 0.12;
			  }
		  }
		  else
		  {
			  throttle_spd = 0.5;
		  }

		  //Apply range limit to -1 ... 1.
		  if (throttle_spd < 0)
		  {
			  throttle_spd = 0.0;
		  }
		  else if (throttle_spd > 1)
		  {
			  throttle_spd = 1;
		  }
		  
          json msgJson;
          msgJson["steering_angle"] = steer_value;
		  msgJson["throttle"] = throttle_spd;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

		  // Evaluate accumulated CTE every some (50) iterations and adjust parameters accordingly.
		  if (pid_steering.counter_ % pid_steering.epochLength_ == 0)
		  {
			  pid_steering.backProp();
			  
			  if (pid_steering.counter_ % (pid_steering.epochLength_ * 20) == 0)
			  {
				  std::cout << "Kp: " << pid_steering.getKp() << " Ki: " << pid_steering.getKi() << " Kd: " << pid_steering.getKd() << " | CTE: " << cte << " Steering Value: " << steer_value << " Throttle Value: " << throttle_spd << " | rmse: " << pid_steering.currentEpochError_ << std::endl;
			  }

			  pid_steering.resetEpochError();
		  }

        }

      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
