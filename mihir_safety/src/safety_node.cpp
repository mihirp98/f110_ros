#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <cmath>
#include <vector>
#include <array>
// TODO: include ROS msg type headers and libraries

using namespace std;

class Safety {
// The class that handles emergency braking
private:
    ros::NodeHandle n;
    double speed;

    ros::Publisher brake_bool_pub;
    ros::Publisher acker_brake_pub;
    ros::Subscriber scan_sub;
    ros::Subscriber odom_sub;

public:
    double num;
    double den;
    double ttc_thres;
    std_msgs::Bool bool_msg;
    ackermann_msgs::AckermannDriveStamped brake_msg;
    double start_angle;
    double increment;
    double r_dot;
    double min_ttc;
    double angle;
    std::vector<double> ttc{vector<double>(1080,0)};
    Safety() {
        n = ros::NodeHandle();
        speed = 0.0;
        ttc_thres = 0.35;
        start_angle = -3.14159274101;
        increment = 0.00582315586507;
        
        odom_sub = n.subscribe("/odom",1,&Safety::odom_callback,this);
	    scan_sub = n.subscribe("/scan",1,&Safety::scan_callback,this);
        brake_bool_pub = n.advertise<std_msgs::Bool>("brake_bool",1);
        acker_brake_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("brake",1);
        
    }
    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
        speed = odom_msg->twist.twist.linear.x;
        
    }

    std::vector<double> LinearSpacedArray(double a, double b, std::size_t N)
    {
        double h = (b - a) / static_cast<double>(N-1);
        std::vector<double> xs(N);
        std::vector<double>::iterator x;
        double val;
        for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h) {
            *x = val;
        }
        return xs;
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {

        if(speed <= 0){
            ttc_thres = 0.38;
        }else{
            ttc_thres = 0.5;
        }
        for(int i=0; i<=1079; i++){
            angle = start_angle + (i * increment);
            r_dot = -1.0 * (-1.0 * speed * cos(angle));
            den = std::max(0.00001,r_dot);
            num = scan_msg->ranges[i];
            ttc[i] = num/den;
        }
        min_ttc = *min_element(ttc.begin(), ttc.end());
        if (min_ttc < ttc_thres){
            bool_msg.data = true;
            brake_msg.drive.speed = 0.0;
            brake_bool_pub.publish(bool_msg);
            acker_brake_pub.publish(brake_msg);
        }else {
            bool_msg.data = false;
            brake_bool_pub.publish(bool_msg);
        }       
    
        ROS_INFO("Safety Node Running");
    }

};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "safety_node");
    Safety sn;
    
    ros::spin();
    return 0;
}
