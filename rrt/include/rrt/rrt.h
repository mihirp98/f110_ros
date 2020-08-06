// ESE 680
// RRT assignment
// Author: Hongrui Zheng

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf

// ros
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <visualization_msgs/Marker.h>

// standard
#include <math.h>
#include <vector>
#include <array>
#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <random>

// Struct defining the Node object in the RRT tree.
// More fields could be added to thiis struct if more info needed.
// You can choose to use this or not
typedef struct Node {
    double x, y;
    double cost = 0; // only used for RRT*
    int parent; // index of parent node in the tree vector
    bool is_root = false;
} Node;


class RRT {
public:
    RRT(ros::NodeHandle &nh);
    virtual ~RRT();
private:
    ros::NodeHandle nh_;

    // ros pub/sub
    // TODO: add the publishers and subscribers you need

    ros::Subscriber pf_sub_;
    ros::Subscriber scan_sub_;
    ros::Publisher vis_pub;
    ros::Publisher occu_map_pub;
    ros::Publisher nav_pub;

    ros::Publisher tree_viz_pub_;
    ros::Publisher path_viz_pub_;

    ackermann_msgs::AckermannDriveStamped drive_msg;

    // tf stuff
    tf2_ros::TransformListener listener;
    tf2_ros::Buffer tfBuffer;
    // tf2_ros::TransformListener listener(tfBuffer);
    geometry_msgs::TransformStamped laser_to_map;
    geometry_msgs::TransformStamped map_to_laser;

    // TODO: create RRT params
    boost::shared_ptr<nav_msgs::OccupancyGrid const> occu_map_ptr;
    nav_msgs::OccupancyGrid occu_map;
    visualization_msgs::Marker marker;
    // std::string file_path;
    double angle_ahead, lookahead_distance, max_expansion_distance, pure_pursuit_lookahead;
    double search_radius;
    int expand_radius;
    int obstacles_count;
    int collision_discretization;
    std::vector<int> obstacles_indices;
    double goal_close_enough;
    double steering_angle;
    double speed;
    int RRT_iters;
    bool rrt_star;
    std::string file_path;
    std::vector<std::vector<double>> pure_waypoints;
    std::vector<std::vector<double>> transformed_waypoints;
    std::vector<double> best_global_waypoint;

    //first Node = current_pose
    Node first_node;

    // random generator, use this
    std::mt19937 gen;
    std::uniform_real_distribution<> x_dist;
    std::uniform_real_distribution<> y_dist;
    

    // callbacks
    // where rrt actually happens
    void pf_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
    // updates occupancy grid
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

    // RRT methods
    std::vector<double> sample();
    int nearest(std::vector<Node> &tree, std::vector<double> &sampled_point);
    Node steer(Node &nearest_node, std::vector<double> &sampled_point, int nearest_node_index);
    bool check_collision(Node &nearest_node, Node &new_node);
    bool is_goal(Node &latest_added_node, double goal_x, double goal_y);
    std::vector<Node> find_path(std::vector<Node> &tree, Node &latest_added_node);
    // RRT* methods
    double cost(std::vector<Node> &tree, Node &node);
    double line_cost(Node &n1, Node &n2);
    std::vector<int> near(std::vector<Node> &tree, Node &node);

    //getting marker point to publish
    void get_marker(visualization_msgs::Marker &marker, double &waypoint_x, double &waypoint_y, std::string frame_name);

    //converting the occupancy indices for occu_map.data
    std::vector<int> get_occupancy_indices(const double x_map, const double y_map);
    int get_occu_data_index(double &x_map, double &y_map);

    std::vector<double> get_node_to_go(std::vector<Node> &found_path, const double &pose_x, const double &pose_y);
    void get_data(std::vector<std::vector<double>> &waypoints);
    std::vector<std::vector<double>> get_transformed_waypoints(const std::vector<std::vector<double>> &waypoints);
    std::vector<double> get_best_global_waypoint(double &current_pose_x, double current_pose_y);
    double get_steering_angle(const double waypoint_x, const double waypoint_y, const double L);
    double get_speed(double steering_angle);

    void visualize_trackpoints(double x, double y, double global_x, double global_y);
    void visualize_trackpoints(std::vector<Node> &tree, std::vector<double> color);
    void visualize_pathpoints(std::vector<Node> &tree, std::vector<double> color);

};

