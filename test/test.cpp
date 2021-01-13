#include <geometry_msgs/Twist.h>
#include <pid/pid.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <iostream>

using namespace std;
double accPos = 0;
double goal = 0;
void posCb(const geometry_msgs::Twist::ConstPtr &msg) { accPos = msg->linear.x; }
void setP(const std_msgs::Float64ConstPtr &msg) { goal = msg->data; }
int main(int argc, char **argv) {
  ros::init(argc, argv, "pid");
  ros::NodeHandle nh("~");
  ros::Subscriber pos = nh.subscribe("/pos", 1, posCb);
  ros::Subscriber setpoint = nh.subscribe("/set", 1, setP);
  ros::Publisher ctrl = nh.advertise<std_msgs::Float64>("/ctrl", 1);
  std_msgs::Float64 msg;
  PID pid(nh, "left");
  PID pid2(nh, "right");

  ros::Rate r(100);
  while (ros::ok()) {
    pid.setPoint(goal);
    pid.update(accPos);
    msg.data = pid.getControll();
    ctrl.publish(msg);

    ros::spinOnce();
    r.sleep();
  }

  // ros::spin();
  // return 0;
}
