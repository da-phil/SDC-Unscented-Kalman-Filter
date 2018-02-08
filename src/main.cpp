#include <uWS/uWS.h>
#include <iostream>
#include <cstdlib>
#include "json.hpp"
#include <math.h>
#include "ukf.h"
#include "tools.h"
#include <getopt.h>

using namespace std;

// for convenience
using json = nlohmann::json;

bool verbose = false;
bool use_laser = true;
bool use_radar = true;
double std_a = 2.0;
double std_yawdd = 0.5;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}


void PrintHelp() {
    std::cout <<
            "Help:"
            "--verbose   <0|1>:       Turn on verbose output, default: "<< verbose<<"\n"
            "--use_laser <0|1>:       Turn on or off laser measurements, default: "<<use_laser<<"\n"
            "--use_radar <0|1>:       Turn on or off radar measurements, default: "<<use_radar<<"\n"
            "--std_a     <num>:       Standard deviation for linear acceleration noise, default: "<<std_a<<"\n"
            "--std_yawdd <num>:       Standard deviation for angular acceleration noise, default: "<<std_yawdd<<"\n"
            "--help:                  Show help\n";
    exit(1);
}


void ParseArgs(int argc, char *argv[]) {
  char c;
  const char* const short_opts = "v:l:r:a:y:h";
  const option long_opts[] = {
          {"verbose",     1, nullptr, 'v'},
          {"use_laser",   1, nullptr, 'l'},
          {"use_radar",   1, nullptr, 'r'},
          {"std_a",       1, nullptr, 'a'},
          {"std_yawdd",   1, nullptr, 'y'},
          {"help",        0, nullptr, 'y'},
          {nullptr,       0, nullptr, 0}
  };

  while (true)
  {
    const auto opt = getopt_long(argc, argv, short_opts, long_opts, nullptr);

    if (opt == -1)
        break;

    switch (opt)
    {
      case 'v':
        verbose = (stoi(optarg) > 0) ? true : false;
        break;
      case 'l':
        use_laser = (stoi(optarg) > 0) ? true : false;
        break;
      case 'r':
        use_radar = (stoi(optarg) > 0) ? true : false;
        break;
      case 'a':
        std_a = atof(optarg);
        break;
      case 'y':
        std_yawdd = atof(optarg);
        break;
      case 'h':
      default:
        PrintHelp();
        break;
    }
  }
}


int main(int argc, char *argv[])
{
  // Parse args
  ParseArgs(argc, argv);

  uWS::Hub h;

  cout << "UKF config: use_laser="<<use_laser<< ", use_radar="<<use_radar <<
          ", verbose="<<verbose << ", std_a="<<std_a << ", std_yawdd="<<std_yawdd << endl;

  // Create a parameterized Unscented Kalman Filter instance
  UKF ukf(verbose, use_laser, use_radar, std_a, std_yawdd);

  // used to compute the RMSE later
  Tools tools;
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  h.onMessage([&ukf,&tools,&estimations,&ground_truth](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = hasData(std::string(data));
      if (s != "") {
      	
        auto j = json::parse(s);

        std::string event = j[0].get<std::string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          string sensor_measurment = j[1]["sensor_measurement"];
          
          MeasurementPackage meas_package;
          istringstream iss(sensor_measurment);
    	  long long timestamp;

    	  // reads first element from the current line
    	  string sensor_type;
    	  iss >> sensor_type;

    	  if (sensor_type.compare("L") == 0) {
      	  		meas_package.sensor_type_ = MeasurementPackage::LASER;
          		meas_package.raw_measurements_ = VectorXd(2);
          		float px;
      	  		float py;
          		iss >> px;
          		iss >> py;
          		meas_package.raw_measurements_ << px, py;
          		iss >> timestamp;
          		meas_package.timestamp_ = timestamp;
          } else if (sensor_type.compare("R") == 0) {

      	  		meas_package.sensor_type_ = MeasurementPackage::RADAR;
          		meas_package.raw_measurements_ = VectorXd(3);
          		float ro;
      	  		float theta;
      	  		float ro_dot;
          		iss >> ro;
          		iss >> theta;
          		iss >> ro_dot;
          		meas_package.raw_measurements_ << ro,theta, ro_dot;
          		iss >> timestamp;
          		meas_package.timestamp_ = timestamp;
          }
          float x_gt;
    	  float y_gt;
    	  float vx_gt;
    	  float vy_gt;
    	  iss >> x_gt;
    	  iss >> y_gt;
    	  iss >> vx_gt;
    	  iss >> vy_gt;
    	  VectorXd gt_values(4);
    	  gt_values(0) = x_gt;
    	  gt_values(1) = y_gt; 
    	  gt_values(2) = vx_gt;
    	  gt_values(3) = vy_gt;
    	  ground_truth.push_back(gt_values);
          
          //Call ProcessMeasurment(meas_package) for Kalman filter
    	  ukf.ProcessMeasurement(meas_package);    	  

    	  //Push the current estimated x,y positon from the Kalman filter's state vector

    	  VectorXd estimate(4);

    	  double p_x = ukf.x_(0);
    	  double p_y = ukf.x_(1);
    	  double v  = ukf.x_(2);
    	  double yaw = ukf.x_(3);

    	  double v1 = cos(yaw)*v;
    	  double v2 = sin(yaw)*v;

    	  estimate(0) = p_x;
    	  estimate(1) = p_y;
    	  estimate(2) = v1;
    	  estimate(3) = v2;
    	  
    	  estimations.push_back(estimate);

    	  VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);

          json msgJson;
          msgJson["estimate_x"] = p_x;
          msgJson["estimate_y"] = p_y;
          msgJson["rmse_x"] =  RMSE(0);
          msgJson["rmse_y"] =  RMSE(1);
          msgJson["rmse_vx"] = RMSE(2);
          msgJson["rmse_vy"] = RMSE(3);
          auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
	  
        }
      } else {
        
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























































































