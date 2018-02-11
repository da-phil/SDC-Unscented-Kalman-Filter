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
bool use_simulator = true;
string inputDataFile  = "../data/obj_pose-laser-radar-synthetic-input.txt";
string outputDataFile = "../data/obj_pose-fused-output.txt";

// The following UKF process noise values achieve an RMSE of 
// [0.0739, 0.0871, 0.3532, 0.2467] in px, py, vx, vy.
double std_a = 0.5;
double std_yawdd = 0.2;

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
            "Help:\n"
            "  --verbose        <0|1>:    Turn on verbose output, default: "<< verbose<<"\n"
            "  --use_laser      <0|1>:    Turn on or off laser measurements, default: "<<use_laser<<"\n"
            "  --use_radar      <0|1>:    Turn on or off radar measurements, default: "<<use_radar<<"\n"
            "  --std_a          <num>:    Standard deviation for linear acceleration noise, default: "<<std_a<<"\n"
            "  --std_yawdd      <num>:    Standard deviation for angular acceleration noise, default: "<<std_yawdd<<"\n"
            "  --use_simulator  <0|1>:    Use simulator for input and output or instead an input output csv file, default: "<< use_simulator<<"\n"
            "  --input_file     <path>:   Path to input csv file (only possible when simulator mode is not set), default: "<<inputDataFile<<"\n" 
            "  --output_file    <path>:   Path to output csv file (only possible when simulator mode is not set), default: "<<outputDataFile<<"\n" 
            "  --help:                    Show help\n";
    exit(1);
}


void ParseArgs(int argc, char *argv[])
{
  char c;
  const char* const short_opts = "v:l:r:a:y:s:i:o:h";
  const option long_opts[] = {
          {"verbose",       1, nullptr, 'v'},
          {"use_laser",     1, nullptr, 'l'},
          {"use_radar",     1, nullptr, 'r'},
          {"std_a",         1, nullptr, 'a'},
          {"std_yawdd",     1, nullptr, 'y'},
          {"use_simulator", 1, nullptr, 's'},          
          {"input_file",    1, nullptr, 'i'},
          {"output_file",   1, nullptr, 'o'},          
          {"help",          0, nullptr, 'h'},
          {nullptr,         0, nullptr, 0}
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
      case 's':
        use_simulator = (stoi(optarg) > 0) ? true : false;
        break;
      case 'i':
        inputDataFile = string(optarg);
        break;
      case 'o':
        outputDataFile = string(optarg);
        break;
      case 'h':
      default:
        PrintHelp();
        break;
    }
  }
}


MeasurementPackage getMeasurement(string &sensor_measurement)
{
  MeasurementPackage meas_package;
  istringstream iss(sensor_measurement);
  long long timestamp;
  float px, py, ro, theta, ro_dot, x_gt, y_gt, vx_gt, vy_gt, yaw_gt, yawrate_gt;

  // reads first element from the current line
  string sensor_type;
  iss >> sensor_type;
  meas_package.ground_truth_ = VectorXd(6);

  if (sensor_type.compare("L") == 0) {
        meas_package.sensor_type_ = MeasurementPackage::LASER;
        meas_package.raw_measurements_ = VectorXd(2);
        iss >> px;
        iss >> py;
        meas_package.raw_measurements_ << px, py;
        iss >> timestamp;
        meas_package.timestamp_ = timestamp;
  } else if (sensor_type.compare("R") == 0) {
        meas_package.sensor_type_ = MeasurementPackage::RADAR;
        meas_package.raw_measurements_ = VectorXd(3);
        iss >> ro;
        iss >> theta;
        iss >> ro_dot;
        meas_package.raw_measurements_ << ro,theta, ro_dot;
        iss >> timestamp;
        meas_package.timestamp_ = timestamp;
  }

  iss >> x_gt;
  iss >> y_gt;
  iss >> vx_gt;
  iss >> vy_gt;
  iss >> yaw_gt;
  iss >> yawrate_gt;
  meas_package.ground_truth_ << x_gt, y_gt, vx_gt, vy_gt, yaw_gt, yawrate_gt;
  return meas_package;
}



int main(int argc, char *argv[])
{
  // Parse cmd args
  ParseArgs(argc, argv);

  cout << "UKF config: use_laser="<<use_laser<< ", use_radar="<<use_radar <<
          ", verbose="<<verbose << ", std_a="<<std_a << ", std_yawdd="<<std_yawdd << endl;

  // Create a parameterized Unscented Kalman Filter instance
  UKF ukf(verbose, use_laser, use_radar, std_a, std_yawdd);

  // used to compute the RMSE later
  Tools tools;
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;
  ifstream in_file(inputDataFile.c_str(), ifstream::in);
  ofstream out_file(outputDataFile.c_str(), ofstream::out);

  if (!in_file.is_open()) {
    cerr << "Cannot open input file: " << inputDataFile << endl;
    exit(EXIT_FAILURE);
  }
  if (!out_file.is_open()) {
    cerr << "Cannot open output file: " << outputDataFile << endl;
    exit(EXIT_FAILURE);
  }


  if (use_simulator)
  {
    uWS::Hub h;
    h.onMessage([&ukf,&tools,&estimations,&ground_truth](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
    {
      // "42" at the start of the message means there's a websocket message event.
      // The 4 signifies a websocket message
      // The 2 signifies a websocket event
      if (length && length > 2 && data[0] == '4' && data[1] == '2')
      {
        auto s = hasData(std::string(data));
        if (s != "") {
          auto j = json::parse(s);
          std::string event = j[0].get<std::string>();
          
          if (event == "telemetry")
          {
            // j[1] is the data JSON object
            string sensor_measurement = j[1]["sensor_measurement"];
            MeasurementPackage meas_package = getMeasurement(sensor_measurement);

            //Call ProcessMeasurment(meas_package) for Kalman filter
            ukf.ProcessMeasurement(meas_package);    	  

            //Push the current estimated x,y positon from the Kalman filter's state vector
            VectorXd estimate(4);
            double p_x = ukf.x_(0);
            double p_y = ukf.x_(1);
            double v   = ukf.x_(2);
            double yaw = ukf.x_(3);
            double v1  = cos(yaw)*v;
            double v2  = sin(yaw)*v;
            estimate << p_x, p_y, v1, v2;
            estimations.push_back(estimate);
            ground_truth.push_back(meas_package.ground_truth_.head(4));

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
  else
  {
    string seperator = ", ";
    Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", seperator, "", "", "", "");
    string line = "# px,  py,  v,  yaw,  yawrate,  nis_laser,  nis_radar,  " 
                  "px_true,  py_true,  vx_true,  vy_true,  yaw_true,  yawrate_true,  "
                  "rmse_px,  rmse_py,  rmse_vx,  rmse_vy";
    out_file << line << endl;

    while (getline(in_file, line)) {
      string sensor_type;
      MeasurementPackage meas_package = getMeasurement(line);

      //Call ProcessMeasurment(meas_package) for Kalman filter
      ukf.ProcessMeasurement(meas_package);       

      //Push the current estimated x,y positon from the Kalman filter's state vector
      VectorXd estimate(4);
      double p_x = ukf.x_(0);
      double p_y = ukf.x_(1);
      double v   = ukf.x_(2);
      double yaw = ukf.x_(3);
      double v1  = cos(yaw)*v;
      double v2  = sin(yaw)*v;
      estimate << p_x, p_y, v1, v2;
      estimations.push_back(estimate);
      ground_truth.push_back(meas_package.ground_truth_.head(4));
      VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);
      out_file << ukf.x_.format(CSVFormat) << seperator << ukf.nis_laser_  << seperator << ukf.nis_radar_ << seperator
              << meas_package.ground_truth_.format(CSVFormat) << seperator << RMSE.format(CSVFormat) << endl;
    }

    if (out_file.is_open())
      out_file.close();
    if (in_file.is_open())
      in_file.close();
  }
}
