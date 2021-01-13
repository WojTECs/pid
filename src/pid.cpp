#include "pid/pid.h"

PID::PID(ros::NodeHandle &nh, std::string name)
    : integral_(0)
    , derivative_(0)
    , error_(0)
    , prevError_(0)
    , control_(0)
    , point_(0)
    , prevTime_(std::chrono::steady_clock::now())
    , derivFilter_(0.001) {
  ros::NodeHandle priv_nh(nh, name.c_str());
  pidCfgServer_.reset(new dynamic_reconfigure::Server<pid::pidConfig>(priv_nh));
  f = boost::bind(&PID::reconfigCallback, this, _1, _2);
  pidCfgServer_->setCallback(f);
  pid::pidConfig::DEFAULT def;
  Kp_ = def.Kp;
  Ki_ = def.Ki;
  Kd_ = def.Kd;
  upperLimit_ = def.upperLimit;
  lowerLimit_ = def.lowerLimit;
  windupLimit_ = def.windup;
  timeout_ = std::chrono::duration<double>(2);
  derivFilter_.setAlpha(def.alfa);
}
void PID::reconfigCallback(pid::pidConfig &cfg, uint32_t level) {
  Kp_ = cfg.Kp;
  Ki_ = cfg.Ki;
  Kd_ = cfg.Kd;
  upperLimit_ = cfg.upperLimit;
  lowerLimit_ = cfg.lowerLimit;
  windupLimit_ = cfg.windup;
  timeout_ = std::chrono::duration<double>(cfg.timeout);
  upLimitOn = cfg.upLimitOn;
  downLimitOn = cfg.downLimitOn;
  windupOn = cfg.windupOn;
  derivFilter_.setAlpha(cfg.alfa);
}
void PID::update(double _data) {
  dt_ = std::chrono::steady_clock::now() - prevTime_;
  // std::cout << "DT: " << dt_.count() / 1000000000.0 << std::endl;
  prevTime_ = std::chrono::steady_clock::now();

  if (dt_ > timeout_) {
    std::cout << "timeout" << std::endl;
    return;
  }
  error_ = point_ - _data;
  // dt.count() / 1000000000.0 = dt in seconds
  integral_ += error_ * dt_.count() / 1000000000.0;

  if (windupOn) {
    if (integral_ > abs(windupLimit_)) {
      integral_ = abs(windupLimit_);
    }
    if (integral_ < -abs(windupLimit_)) {
      integral_ = -abs(windupLimit_);
    }
  }
  derivative_ = (error_ - prevError_) / (dt_.count() / 1000000000.0);
  prevError_ = error_;
  derivFilter_.filter(derivative_);
  // std::cout << "derivative: " << derivative_ << std::endl;
  control_ = Kp_ * error_ + Ki_ * integral_ + Kd_ * derivFilter_.getOutput();

  if (control_ > upperLimit_ && upLimitOn) {
    control_ = upperLimit_;
  }
  if (control_ < lowerLimit_ && downLimitOn) {
    control_ = lowerLimit_;
  }
}
void PID::setPoint(double _point) { point_ = _point; }
double PID::getControll() const { return control_; }
PID::operator double() const { return control_; }
double PID::getPoint() const { return point_; }