#include <unistd.h>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#define PI 3.141592653

int t = 500000;
int T = 1000000;

void Forward(const double& v, ros::Publisher pub);
void ForwardStart(const double& v_cur, const double& v_goal, ros::Publisher pub);
void ForwardBrake(const double& v_cur, ros::Publisher pub);
void Rotary(const double& a, ros::Publisher pub);
void RotaryStart(const double& a_cur, const double& a_goal, ros::Publisher pub);
void RotaryBrake(const double& a_cur, ros::Publisher pub);
void WalkingDemo(ros::Publisher pub);

int main(int argc, char** argv) {
  ros::init(argc, argv, "walking_demo");
  ros::NodeHandle nh;

  std::cout << "ready to start !! " << std::endl; 
  usleep(2*T);
  ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("cmd_base_joint", 100);
  WalkingDemo(pub);
  ros::spinOnce();

  return 0;
}

void Forward(const double& v, ros::Publisher pub) {

  if (v*22.5 > 1000) {
    ROS_WARN("velocity is too fast!!");
    return ;
  }

  sensor_msgs::JointState js;
  js.header.frame_id = "motor";
  js.header.stamp = ros::Time::now();
  js.name.resize(8);
  js.name = {"front_left_walking",   "front_right_walking",
             "rear_left_walking",    "rear_right_walking",
             "front_right_steering", "front_left_steering",
             "rear_right_steering",  "rear_left_steering"};
  js.velocity.resize(8);
  js.velocity = {v, v, v, v, 0, 0, 0, 0};

  js.position.resize(8);
  js.position = {0, 0, 0, 0, 0, 0, 0, 0};

  pub.publish(js);
}

void ForwardStart(const double& v_cur, const double& v_goal, ros::Publisher pub) {
  double acc = 4.0;
  double vel = v_cur;

  if (vel < v_goal) {
    while (vel < v_goal) {
      Forward(vel, pub);
      usleep(t);
      vel += acc;
    }
  } else {
    while (vel > v_goal) {
      Forward(vel, pub);
      usleep(t);
      vel -= acc;
    }
  }
  Forward(v_goal, pub);
  usleep(T);
}

void ForwardBrake(const double& v_cur, ros::Publisher pub) {
  double acc = 4.0;
  double vel = v_cur;

  if (vel > 0) {
    while (vel > 0) {
      Forward(vel, pub);
      usleep(t);
      vel -= acc;
    }
  } else {
    while (vel < 0) {
      Forward(vel, pub);
      usleep(t);
      vel += acc;
    }
  }
  Forward(0, pub);
  usleep(T);
}


void Rotary(const double& a, ros::Publisher pub) {

  sensor_msgs::JointState js;
  js.header.frame_id = "motor";
  js.header.stamp = ros::Time::now();
  js.name.resize(8);
  js.name = {
    "front_left_walking",
    "front_right_walking",
    "rear_left_walking",
    "rear_right_walking",
    "front_right_steering",
    "front_left_steering",
    "rear_right_steering",
    "rear_left_steering"
  };

  double wheel_dis_len, wheel_dis_wid;
  wheel_dis_len = 0.5;
  wheel_dis_wid = 0.395;
  double hypotenuse = sqrt(pow(wheel_dis_len, 2) + pow(wheel_dis_wid, 2));
  double r = 0.5 * hypotenuse;

  float alpha = fabs(atan(wheel_dis_len / wheel_dis_wid));
  float v_linear = a * r/(0.15/2);
  float n = v_linear * 60 / (2 * PI);
  js.velocity.resize(8);
  js.position.resize(8);

  js.position = {0, 0, 0, 0, -alpha, alpha, alpha, -alpha};
  js.velocity = {-n, n, -n, n, 0, 0, 0, 0};

  pub.publish(js);
}

void RotaryStart(const double& a_cur, const double& a_goal, ros::Publisher pub) {
  double acc = 0.4;
  double ang = a_cur;

  if (ang < a_goal) {
    while (ang < a_goal) {
      Rotary(ang, pub);
      ang += acc;
      usleep(t);
    }
  } else {
    while (ang > a_goal) {
      Rotary(ang, pub);
      ang -= acc;
      usleep(t);
    }
  }
  Rotary(a_goal, pub);
  usleep(T);
}

void RotaryBrake(const double& a_cur, ros::Publisher pub) {
  double acc = 0.04;
  double ang = a_cur;

  if (ang > 0) {
    while (ang > 0) {
      Rotary(ang, pub);
      ang -= acc;
      usleep(t);
    }
  } else {
    while (ang < 0) {
      Rotary(ang, pub);
      ang += acc;
      usleep(t);
    }
  }
  Rotary(0, pub);
  usleep(T);
}


void WalkingDemo(ros::Publisher pub) {

  int t_tmp = 1000000;
  double angular = 0.5424;

  ForwardStart(0, 20, pub);

  Forward(20, pub);
  usleep(t_tmp);

  ForwardBrake(20, pub);



  RotaryStart(0, angular, pub);

  Rotary(angular, pub);
  usleep(t_tmp);

  RotaryBrake(angular, pub);

  RotaryStart(0, -angular, pub);

  Rotary(-angular, pub);
  usleep(t_tmp);

  RotaryBrake(-angular, pub);



  ForwardStart(0, -20, pub);

  Forward(-20, pub);
  usleep(t_tmp);

  ForwardBrake(-20, pub);


}