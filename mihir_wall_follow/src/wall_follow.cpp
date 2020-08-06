#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <cmath>

using namespace std;

class WallFollow {

private:
  ros::NodeHandle n;
  ros::Publisher drive_pub;
  ros::Subscriber scan_sub;
  ackermann_msgs::AckermannDriveStamped drive_msg;
  double error, prev_error;
  double alpha, L, a, b, c, t, prev_t, dt;
  int index0, index_theta, index_theta_comp;
  double desired_left;
  double theta;
  double actual_dist;
  double kp, ki, kd;
  double steering_angle, speed, integral_prev, integral, derivative, proportional;
public:
  
  WallFollow(){
    n = ros::NodeHandle();
    error = prev_error = 0.0;
    alpha = a = b = c = t = prev_t = dt = 0.0;
    index0 = 809; //270: right
    index_theta = 629; //449: 60
    index_theta_comp = 270;
    desired_left = 1.0;
    theta = 1.0472;
    actual_dist = 0.0;
    kp = 3; //5,0,0.1
    ki = 0.0;
    kd = 0.1; //0.1
    L = 0.1;
    steering_angle = speed = integral_prev = integral = derivative = proportional = 0.0;
    scan_sub = n.subscribe("/scan",1,&WallFollow::scan_callback,this);
    drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("drive",1);
  }
  
  double compute_actual_dist(double a, double b){
    alpha = atan((a*cos(theta) - b)/(a * sin(theta)));
    actual_dist = (b * cos(alpha)) + (L * sin(alpha));
    return actual_dist;
  }

  double compute_steering_angle(double error, double dt){
    proportional = kp * error;
    integral = ki * (integral_prev + ((prev_error + error))/2);
    derivative = kd * ((error - prev_error)); 
    steering_angle = -1.0 * (proportional + integral + derivative);
    prev_error = error;
    integral_prev = integral;
    return steering_angle;
  }

  void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg){
    b = scan_msg->ranges[index0];
    a = scan_msg->ranges[index_theta];
    c = scan_msg->ranges[index_theta_comp];
    t = (scan_msg->header.stamp.sec) + (scan_msg->header.stamp.nsec/1000000000);
    dt = t - prev_t;
    prev_t = t;
    actual_dist = compute_actual_dist(a,b);
    error = desired_left - actual_dist;
    if ( (b+c >= 3.2 && b+c <=3.6) && (a <= (b + 0.2))){
      steering_angle = 0.0;
    }else{
      steering_angle = compute_steering_angle(error, dt);
    } 
    if ((abs(steering_angle) >= 0) && (abs(steering_angle) < 0.174533)){
      speed = 2.5; //2.5, 3.5, 4.5
    } else if ((abs(steering_angle) >= 0.174533) && (abs(steering_angle) < 0.349066)){
      speed = 1.5; //1.5, 2.5, 3.5
    } else{
      speed = 0.5; //0.5, 1.5, 2.0
    }
    drive_msg.drive.steering_angle = steering_angle; //steering_angle
    drive_msg.drive.speed = speed; //speed
    drive_pub.publish(drive_msg);
  }

  
};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "wall_follow");
    WallFollow wf;
    ros::spin();
    return 0;
}
