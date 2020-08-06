#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <cmath>
#include <math.h>

using namespace std;

class ReactiveGapFollow{

private:
  ros::NodeHandle n;
  ros::Publisher nav_pub;
  ros::Subscriber lidar_sub;
  ackermann_msgs::AckermannDriveStamped drive_msg;
  double steering_angle, speed;
  double rb , increment, rad_angle;
  int min_range_index, start_i, end_i, angle_min, angle_max;
  vector<float> ranges, proc_ranges;
  double gap_thres;

public:
  ReactiveGapFollow()
  {
    n = ros::NodeHandle();
    steering_angle = 0;
    rb = 0.4;
    rad_angle = 0;
    min_range_index = start_i = end_i = 0;
    increment = 0.00582315586507;
    // ranges = vector<float>(1080);
    speed = 1.0;
    gap_thres = 1.2;
    angle_min = 0;
    angle_max = 0;
    lidar_sub = n.subscribe("/scan", 1, &ReactiveGapFollow::lidar_callback, this);
    nav_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive",1);
  }

  void zero_bubble(vector<float>& proc_ranges, int radius_index, int min_range_index)
  {
    int last = min(min_range_index + radius_index, static_cast<int>(proc_ranges.size() - 1));
    for (int i = max(0, min_range_index - radius_index); i <= last; i++ )
    {
      proc_ranges[i] = 0;
    }
  }

  int bubble_radius_index(int min_range_index, vector<float> proc_ranges)
  {
    rad_angle = atan(rb/proc_ranges[min_range_index]);
    int bubble_radius_index = round(rad_angle/increment);
    return bubble_radius_index;
  }

  float clip(float n, float lower, float upper) 
  {
    return std::max(lower, std::min(n, upper));
  }

  vector<float> ranges_process(vector<float> ranges, int angle_min, int angle_max)
  {
    vector<float> proc_ranges;
    for(int i = 0; i <= ranges.size() - 1; i++ )
    {
      if( i >= angle_min && i <= angle_max)
      {
        if(ranges[i] == std::numeric_limits<float>::infinity() || ranges[i] > 4.0) {ranges[i] = 4.0;}
        if(isnan(ranges[i])) {ranges[i] = 0;}
        proc_ranges.push_back(ranges[i]);
      }  
    }
    return proc_ranges;
  }

  void max_gap_length(vector<float> proc_ranges, int& start_i, int& end_i)
  {
    int count =  0;
    int prev_max_count = 0;
    int result = 0;
//    int start_i = 0;
//    int end_i = 0;

    for(int i=0; i < proc_ranges.size(); i++)
    {
      if (proc_ranges[i] <= gap_thres || i == proc_ranges.size() - 1)
      {
        if (count > prev_max_count)
        {
          end_i = i - 1;
          start_i = i - count;
          prev_max_count = count;
        }
        count = 0;
      } else 
      {
        count += 1;
      }
    }
    //return make_tuple(start_i, end_i);
  }

  // int best_point(vector<float> proc_ranges, int start_i, int end_i)
  // { 
  //   float max_range = 0;
  //   int prev_dist = 1000;
  //   int max_index = 0; 
  //   for(int i = start_i; i <= end_i; i++)
  //   {      
  //     if(proc_ranges[i] == 3.0 && abs(i - 270) < prev_dist)
  //     { 
  //         max_range = proc_ranges[i];
  //         prev_dist = abs(i - 270);
  //         max_index = i;
  //     } else if(proc_ranges[i] > max_range){
  //       max_range = proc_ranges[i];
  //       max_index = i;
  //       prev_dist = abs(i - 270);
  //     }
  //   }
  //   return max_index;
  // }

  int best_point(vector<float> proc_ranges, int start_i, int end_i)
  { 
    // if ((start_i < 270 && end_i > 270) && proc_ranges[270] == 4.0)
    // {
    //   return 270;
    // } else 
      {return round((start_i + end_i)/2);}
  }

  // int best_point(vector<float> proc_ranges, int start_i, int end_i)
  // { 
  //   double max_range = 0.0;
  //   int best_index = 0;
  //   for (int i = start_i; i < end_i; i++)
  //   {
  //     if(proc_ranges[i] > max_range)
  //     {
  //       best_index = i;
  //       max_range = proc_ranges[i];
  //     }
  //   }
  //   return best_index;
  // }

  double get_steering_angle(int best_index, vector<float> proc_ranges)
  { 
    double final_steering_angle;
    double steering_angle = (best_index * increment) - (M_PI_2);
    if (best_index == 270)
    {
      final_steering_angle = 0.0;
    } else if (proc_ranges[best_index] >= 2.0)
    {
      final_steering_angle = steering_angle/2;
    } else 
    {
      final_steering_angle = steering_angle;
    }
    
    return clip(final_steering_angle, -0.40, 0.40);
  }

  double get_speed(float steering_angle)
  { 
    //steering_angle = clip(steering_angle, -0.44, 0.44);
    if ((abs(steering_angle) >= 0) && (abs(steering_angle) < 0.174533)){
      speed = 1.0; //2.5, 3.5, 4.5
    } else if ((abs(steering_angle) >= 0.174533) && (abs(steering_angle) < 0.4)){ //0.349066
      speed = 1.0; //1.5, 2.5, 3.5
    } else{
      speed = 1.0; //0.5, 1.5, 2.0
    }
    return speed;
  }

  void lidar_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
  {
    ranges = scan_msg->ranges;
    angle_min = ranges.size()/2 - ((90 *(M_PI/180))/increment);
    angle_max = ranges.size()/2 + ((90 *(M_PI/180))/increment);
    //ROS_INFO("%d %d", angle_min, angle_max);
    proc_ranges = ranges_process(ranges, angle_min, angle_max);
    min_range_index = min_element(proc_ranges.begin(), proc_ranges.end()) - proc_ranges.begin();
    int radius_index = bubble_radius_index(min_range_index, proc_ranges);
    zero_bubble(proc_ranges, radius_index, min_range_index);
    max_gap_length(proc_ranges, start_i, end_i); 
    int best_index = best_point(proc_ranges, start_i, end_i);
    double steering_angle = get_steering_angle(best_index, proc_ranges);
    drive_msg.drive.steering_angle = steering_angle; //steering_angle
    drive_msg.drive.speed = get_speed(steering_angle); //speed
    nav_pub.publish(drive_msg);
    ROS_INFO("start_i = %d, end_i = %d, best_index_range = %f", start_i, end_i, proc_ranges[best_index]);
    ROS_INFO("zero_range = %f, best_index = %d, s_angle = %f", proc_ranges[270], best_index, steering_angle);
  }
  

};

int main(int argc, char ** argv){
  ros::init(argc, argv, "reactive_gap_follow");
  ReactiveGapFollow rgf;
  ros::spin();
  return 0;
}
