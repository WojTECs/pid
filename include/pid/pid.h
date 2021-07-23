#ifndef PID_H
#define PID_H

#include <dynamic_reconfigure/server.h>
#include <pid/DEMA.h>
#include <pid/pidConfig.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

class PID
{
public:
  PID(ros::NodeHandle &nh, std::string name);

  void update(double _data);
  void setPoint(double _point);
  double getPoint() const;
  double getControll() const;
  operator double() const;

private:
  double Kp_, Ki_, Kd_, Ti_, Td_;
  double upperLimit_, lowerLimit_;
  double windupLimit_;
  double integral_, derivative_, error_, sum_, prevError_, prevDerivative_, control_, point_, prevPoint_, alfa_;
  bool upLimitOn, downLimitOn, windupOn;
  DEMAFilter derivFilter_;
  // time
  std::chrono::steady_clock::duration dt_;
  std::chrono::steady_clock::time_point prevTime_;
  std::chrono::duration<double> timeout_;
  void reconfigCallback(pid::pidConfig &cfg, uint32_t level);
  std::unique_ptr<dynamic_reconfigure::Server<pid::pidConfig>> pidCfgServer_;
  dynamic_reconfigure::Server<pid::pidConfig>::CallbackType f;

  // double maxHz_, minHz_;
};

#endif