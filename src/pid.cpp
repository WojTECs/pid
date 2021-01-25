#include "pid/pid.h"

PID::PID(ros::NodeHandle &nh, std::string name)
    : integral_(0), derivative_(0), error_(0), prevError_(0), control_(0), point_(0), prevTime_(std::chrono::steady_clock::now()), derivFilter_(0.001)
{
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
  upLimitOn = def.upLimitOn;
  downLimitOn = def.downLimitOn;
  windupOn = def.windupOn;
  alfa_ = def.alfa;
  timeout_ = std::chrono::duration<double>(2);

  if (!priv_nh.getParam("Kp", Kp_)) {
    ROS_WARN_STREAM("Can not find Kp, setting default: " << Kp_);
  }
  if (!priv_nh.getParam("Ki", Ki_)) {
    ROS_WARN_STREAM("Can not find Ki, setting default: " << Ki_);
  }
  if (!priv_nh.getParam("Kd", Kd_)) {
    ROS_WARN_STREAM("Can not find Kp, setting default: " << Kd_);
  }
    if (!priv_nh.getParam("upperLimit", upperLimit_)) {
    ROS_WARN_STREAM("Can not find upperLimit, setting default: " << upperLimit_);
  }

  if (!priv_nh.getParam("lowerLimit", lowerLimit_)) {
    ROS_WARN_STREAM("Can not find lowerLimit, setting default: " << lowerLimit_);
  }

  if (!priv_nh.getParam("windup", windupLimit_)) {
    ROS_WARN_STREAM("Can not find windup, setting default: " << windupLimit_);
  }

  if (!priv_nh.getParam("upLimitOn", upLimitOn)) {
    ROS_WARN_STREAM("Can not find upLimitOn, setting default: " << upLimitOn);
  }

  if (!priv_nh.getParam("downLimitOn", downLimitOn)) {
    ROS_WARN_STREAM("Can not find downLimitOn, setting default: " << downLimitOn);
  }

  if (!priv_nh.getParam("windupOn", windupOn)) {
    ROS_WARN_STREAM("Can not find windupOn, setting default: " << windupOn);
  }

  if (!priv_nh.getParam("alfa", alfa_)) {
    ROS_WARN_STREAM("Can not find alfa, setting default: " << alfa_);
  }

  double timeoutVal = 2; 
  if (!priv_nh.getParam("timeout", timeoutVal)) {
    ROS_WARN_STREAM("Can not find timeout, setting default: " << timeoutVal);
  }
  timeout_ = std::chrono::duration<double>(timeoutVal);

  prevDerivative_ = 0.0;
  derivFilter_.setAlpha(def.alfa);

}
void PID::reconfigCallback(pid::pidConfig &cfg, uint32_t level)
{
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
  alfa_ = cfg.alfa;
  derivFilter_.setAlpha(cfg.alfa);
}
void PID::update(double _data)
{
  dt_ = std::chrono::steady_clock::now() - prevTime_;
  // std::cout << "DT: " << dt_.count() / 1000000000.0 << std::endl;
  prevTime_ = std::chrono::steady_clock::now();

  if (dt_ > timeout_)
  {
    std::cout << "timeout" << std::endl;
    return;
  }
  
  if ((prevPoint_ > 0 && point_ < 0) || (prevPoint_ < 0 && point_ > 0)) {
    sum_ = 0;
  }

  if (abs(point_) < 1){
    point_ = 0;
    sum_ = 0;
  }
  prevPoint_ = point_;
  error_ = point_ - _data;

  
  // dt.count() / 1000000000.0 = dt in seconds
  sum_ += error_ * dt_.count() / 1000000000.0;

  if (windupOn)
  {
    if (sum_ > abs(windupLimit_))
    {
      sum_ = abs(windupLimit_);
    }
    if (sum_ < -abs(windupLimit_))
    {
      sum_ = -abs(windupLimit_);
    }
  }
  // Ti_ = Kp_ / Ki_;
  integral_ = Ki_ * sum_ ;

  if (Kd_ > 0.00000001 && alfa_ > 0.00000001)
  {
    Td_ = Kd_ / Kp_;
    derivative_ = Kd_ * ((error_ - prevError_) + (alfa_ * Td_ * prevDerivative_)) / (Td_ * alfa_ + (dt_.count() / 1000000000.0));
    if (derivative_ > upperLimit_)
    {
      derivative_ = upperLimit_;
    }
    if (derivative_ < lowerLimit_)
    {
      derivative_ = lowerLimit_;
    } 
  }
  else  
  {
    derivative_ = 0.0;
  }
  prevDerivative_ = derivative_;
  prevError_ = error_;

  // std::cout << "e - pe: " << (error_ - prevError_) << "\t a*Td*pD: " << alfa_ * Td_ * prevDerivative_ << 
  //   "\t Td*a: " << Td_ * alfa_ << "\t Td*a*dt: " << (Td_ * alfa_ + (dt_.count() / 1000000000.0)) << "\t der " << derivative_ << "\t KD " << Kd_ << "\n";

  // Ki = Kp/Ti;
  // Kd = Td*Kp;
  control_ = Kp_ * error_ + integral_ + derivative_;

  //std::cout << "Control: " << control_ << "\t Data: " << _data << "\t error: " << error_ << "\t integral: " << integral_ << "\t derivative " << derivative_ << "\n";

  if (control_ > upperLimit_ && upLimitOn)
  {
    sum_ -= abs(error_);
    control_ = upperLimit_;
  }
  if (control_ < lowerLimit_ && downLimitOn)
  {
    sum_ += abs(error_);
    control_ = lowerLimit_;
  }
}
void PID::setPoint(double _point) { point_ = _point; }
double PID::getControll() const { return control_; }
PID::operator double() const { return control_; }
double PID::getPoint() const { return point_; }