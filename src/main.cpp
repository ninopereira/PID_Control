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

  PID pid_st;
  PID pid_th;

  // TODO: Initialize the pid variables.
//  pid_st.Init(0.2, 0.01, 2); //original
  pid_st.Init(0.159815, 0.001, 2.1);  //after auto tuning using twidle algorithm

  // PID for throttle: not used at the moment
  pid_th.Init(0.1, 0.0, 0.0);

  // Variables used for twidle algorithm
  double abs_cte_sum = 0;
  size_t evaluation_period = 400;
  size_t iterations = 0;
  size_t twidle_iter = 0;
  double twidle_params[3] = {0.01,0.001,0.1};
  double twidle_pid[3] = {pid_st._Kp,pid_st._Ki,pid_st._Kd};
  double twidle_tolerance=0.01;
  size_t twidle_state = 0;
  double best_error=1000;
  bool initialized = false;

  // flag to enable tuning the PID controller with twidle algorithm on the fly (online mode)
  bool twidle_enabled = false;

  // fixed throttle value
  double throttle_value = 0.4;

  h.onMessage([&pid_st, &pid_th, &iterations, &abs_cte_sum, &evaluation_period,
              &twidle_params, &twidle_pid, &twidle_iter, &twidle_tolerance,
              &twidle_state, &best_error, &initialized, &throttle_value, &twidle_enabled]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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

          /// Twidle algorithm for online calibration of PID params
          if ((iterations>evaluation_period) && (iterations%evaluation_period == 0) && twidle_enabled)
          {
              if (!initialized)
              {
                  double average_cte = abs_cte_sum/(double)evaluation_period;
                  best_error = average_cte;
                  initialized = true;
                  twidle_pid[twidle_iter]+=twidle_params[twidle_iter];
                  pid_st.Init(twidle_pid[0],twidle_pid[1], twidle_pid[2]);
                  std::cout << "Initial PID params: " << "Kp = " << pid_st._Kp << " Ki = " << pid_st._Ki << " Kd = " << pid_st._Kd <<std::endl;
              }
              else
              {
                  double average_cte = abs_cte_sum/(double)evaluation_period;

                  switch (twidle_state)
                  {
                    case 0:
                      if(average_cte<best_error)
                      {
                          best_error = average_cte;
                          twidle_params[twidle_iter]*=1.1;
                          twidle_iter = (twidle_iter+1)%3;
                          twidle_pid[twidle_iter]+=twidle_params[twidle_iter];
                          pid_st.Init(twidle_pid[0],twidle_pid[1], twidle_pid[2]);
                          // state doesn't change
                      }
                      else
                      {
                          twidle_pid[twidle_iter]-=2*twidle_params[twidle_iter];
                          pid_st.Init(twidle_pid[0],twidle_pid[1], twidle_pid[2]);
                          twidle_state=1;
                      }
                      break;
                    case 1:
                      if(average_cte<best_error)
                      {
                          best_error = average_cte;
                          twidle_params[twidle_iter]*=1.1;
                      }
                      else
                      {
                          twidle_pid[twidle_iter]+=twidle_params[twidle_iter];
                          twidle_params[twidle_iter]*=0.9;
                      }
                      twidle_iter = (twidle_iter+1)%3;
                      twidle_pid[twidle_iter]+=twidle_params[twidle_iter];
                      pid_st.Init(twidle_pid[0],twidle_pid[1], twidle_pid[2]);
                      twidle_state=0;
                      break;
                  }

                  if(twidle_params[0]+twidle_params[1]+twidle_params[2]<twidle_tolerance)
                  {
                    std::cout << "==> PID Params Complete! average_cte= " << average_cte << " sum = "<< twidle_params[0]+twidle_params[1]+twidle_params[2] << " iteration = " << iterations << std::endl;
                    std::cout << "Final PID params: " << "Kp = " << pid_st._Kp << " Ki = " << pid_st._Ki << " Kd = " << pid_st._Kd <<std::endl;
                    throttle_value = -1;
                  }
                  else
                  {
                    std::cout << "==> Adjusting PID Params! average_cte= " <<  average_cte << " sum = "<< twidle_params[0]+twidle_params[1]+twidle_params[2] << " iteration = " << iterations << std::endl;
                    std::cout << "Kp = " << pid_st._Kp << " Ki = " << pid_st._Ki << " Kd = " << pid_st._Kd <<std::endl;
                    abs_cte_sum = 0;
                  }
              }
          }

          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          pid_st.UpdateError(cte);
          double steer_value = pid_st.TotalError();

          if (steer_value > 1) steer_value = 1;
          else if (steer_value < -1) steer_value = -1;

          // DEBUG
          if(speed>30 && twidle_enabled) // for Twidle algorithm only
          {
            std::cout << "iteration = " << iterations << " CTE: " << cte << " Steering Value: " << steer_value << std::endl;
            abs_cte_sum += fabs(cte);
            iterations ++;
          }
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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
