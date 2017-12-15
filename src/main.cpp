#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// Set the latency of the Simulator
int latency_ms = 100;

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
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}


int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
   // cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double delta = j[1]["steering_angle"];

          // predict state due to latency!
          double latency = latency_ms/1000.0;
          px = px + v* 0.44704 * cos(psi)*latency;  // 0.44704 --> convert from mph to m/s
          py = py + v* 0.44704 * sin(psi)*latency;
          psi = psi - v* 0.44704 *delta/Lf*latency; // steering angle negative
         // v = v + acceleration*latency;  // not used in model


          // Convert to car space
          Eigen::VectorXd   waypoints_x_car_space = Eigen::VectorXd( ptsx.size() ) ;
          Eigen::VectorXd   waypoints_y_car_space = Eigen::VectorXd( ptsx.size() ) ;

          for (int i = 0;   i < ptsx.size() ;   i++) {
            waypoints_x_car_space(i) = (ptsx[i] - px) * cos(-psi) - (ptsy[i] - py) * sin(-psi);
            waypoints_y_car_space(i) = (ptsy[i] - py) * cos(-psi) + (ptsx[i] - px) * sin(-psi);
          }

          // In car-space those values are zero
          double car_x_car_space = 0.0;
          double car_y_car_space = 0.0;
          double car_psi_car_space = 0.0;


          // Fit 3rd order polynomial to waypoints
          auto coeffs = polyfit(waypoints_x_car_space, waypoints_y_car_space, 3);

          // Calculate cte: f(x) - y
          double cte = polyeval(coeffs, car_x_car_space) - car_y_car_space;

          // Calculate epsi = psi - atan(f'(x))
          double epsi = car_psi_car_space
                        - atan(coeffs[1]
                               + 2*coeffs[2]*car_x_car_space
                               + 3*coeffs[3]*car_x_car_space*car_x_car_space);



          // Calculate steering angle and throttle using MPC.
          Eigen::VectorXd state(6);
          state << car_x_car_space, car_y_car_space, 0, v, cte, epsi;
          MPCData solution = mpc.Solve(state, coeffs);


          json msgJson;

          // Send MPC sttering and throttle value
          double steer_value = -solution.delta / deg2rad(25); // steer value has to be negated for simulator
          double throttle_value = solution.a;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;


          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          mpc_x_vals = solution.mpc_x;
          mpc_y_vals = solution.mpc_y;

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (int i = 0;   i < waypoints_x_car_space.size();   i++) {
            next_x_vals.push_back(waypoints_x_car_space(i));
            next_y_vals.push_back(waypoints_y_car_space(i));
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          this_thread::sleep_for(chrono::milliseconds(latency_ms));
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
