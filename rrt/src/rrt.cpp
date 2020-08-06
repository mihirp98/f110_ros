// ESE 680
// RRT assignment
// Author: Hongrui Zheng

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
// Make sure you have read through the header file as well

#include "rrt/rrt.h"

// Destructor of the RRT class
RRT::~RRT() {
    // Do something in here, free up used memory, print message, etc.
    ROS_INFO("RRT shutting down");
}

// Constructor of the RRT class
RRT::RRT(ros::NodeHandle &nh): nh_(nh), gen((std::random_device())()), listener(tfBuffer) {

    // TODO: Load parameters from yaml file, you could add your own parameters to the rrt_params.yaml file
    std::string pose_topic, scan_topic, marker_topic, map_topic;
    
    nh_.getParam("pose_topic", pose_topic);
    nh_.getParam("scan_topic", scan_topic);
    nh_.getParam("marker_topic", marker_topic);
    nh_.getParam("angle_ahead", angle_ahead);
    nh_.getParam("expand_radius", expand_radius);
    nh_.getParam("map_topic", map_topic);
    nh_.getParam("lookahead_distance", lookahead_distance);
    nh_.getParam("pure_pursuit_lookahead", pure_pursuit_lookahead);
    nh_.getParam("max_expansion_distance", max_expansion_distance);
    nh_.getParam("collision_discretization", collision_discretization);
    nh_.getParam("goal_close_enough", goal_close_enough);
    nh_.getParam("RRT_iters", RRT_iters);
    nh_.getParam("file_path", file_path);
    nh_.getParam("search_radius", search_radius);
    nh_.getParam("rrt_star", rrt_star);

    obstacles_count = 0;

    // ROS publishers
    // TODO: create publishers for the the drive topic, and other topics you might need

    // ROS subscribers
    // TODO: create subscribers as you need
    occu_map_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(map_topic, ros::Duration(2));
    if(occu_map_ptr != NULL){
        occu_map = *occu_map_ptr;
    } else { ROS_INFO("NULL POINTER RECEIVED!");}

    get_data(pure_waypoints);

    pf_sub_ = nh_.subscribe(pose_topic, 10, &RRT::pf_callback, this);
    scan_sub_ = nh_.subscribe(scan_topic, 10, &RRT::scan_callback, this);
    sleep(1);
    vis_pub = nh_.advertise<visualization_msgs::Marker>( marker_topic, 1);
    occu_map_pub = nh_.advertise<nav_msgs::OccupancyGrid>( "new_map", 1);
    nav_pub = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/nav",1);
    tree_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("tree_viz_marker", 1);
    path_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("path_viz_marker", 1);

    // TODO: create a occupancy grid
    

    double waypoint_x, waypoint_y;
    waypoint_x = waypoint_y = 1.0;
    // get_marker(marker, occu_map.info.origin.position.x, occu_map.info.origin.position.y, "map");
    // get_marker(marker, waypoint_x, waypoint_y, "laser");

    // ROS_INFO("size = %f, %f",occu_map.info.origin.position.x, occu_map.info.origin.position.y);
    ROS_INFO("Created new RRT Object.");
}

void RRT::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
    // The scan callback, update your occupancy grid here
    // Args:
    //    scan_msg (*LaserScan): pointer to the incoming scan message
    // Returns:
    //
    // vis_pub.publish(marker);

    //transformation from laser frame to base frame
    try
    {
        laser_to_map = tfBuffer.lookupTransform("map", "laser", ros::Time(0));
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(0.2).sleep();
    }

    const auto laser_to_map_translation = laser_to_map.transform.translation;
    const auto laser_to_map_yaw = tf::getYaw(laser_to_map.transform.rotation);

    // TODO: update your occupancy grid
    const double angle_increment = scan_msg->angle_increment; 
    const int start_ind = (3.14 - (angle_ahead/2))/angle_increment;
    const int end_ind = start_ind + (angle_ahead/angle_increment);
    // const auto range = scan_msg->ranges;
    double theta = scan_msg->angle_min + (start_ind*angle_increment);
    // std::cout << start_ind << " " << end_ind << std::endl; //449,628
    // std::cout << theta << std::endl;

    for(int i = start_ind; i <= end_ind; i++){
        theta += angle_increment;
        // std::cout << theta << " " << i << std::endl;
        const double ray_length = scan_msg->ranges[i];
        if(std::isinf(ray_length) || std::isnan(ray_length)) continue;

        double x_laser = ray_length*cos(theta);
        double y_laser = ray_length*sin(theta);

        if(x_laser > lookahead_distance || y_laser > lookahead_distance) continue;

        double x_map = (x_laser*cos(laser_to_map_yaw)) - (y_laser*sin(laser_to_map_yaw)) + laser_to_map_translation.x;
        double y_map = (x_laser*sin(laser_to_map_yaw)) + (y_laser*cos(laser_to_map_yaw)) + laser_to_map_translation.y;

        std::vector<int> occu_map_indices = get_occupancy_indices(x_map,y_map);


        for(int j = 0; j < occu_map_indices.size(); j++){
            if( occu_map.data[occu_map_indices[j]] != 100){
                occu_map.data[occu_map_indices[j]] = 100;
                obstacles_indices.push_back(occu_map_indices[j]);
            }
        }

    }

    obstacles_count += 1;
    if (obstacles_count > 40){
        for(int i = 0; i < obstacles_indices.size(); i++){
            occu_map.data[obstacles_indices[i]] = 0;
        }
        obstacles_indices.clear();
        obstacles_count = 0;
        // ROS_INFO("Obstacles Cleared!");
    }

    occu_map_pub.publish(occu_map);
    // ROS_INFO("OccupancyGrid Updated!");
    // ROS_INFO("RRT Running!");

}

void RRT::pf_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {
    // The pose callback when subscribed to particle filter's inferred pose
    // The RRT main loop happens here
    // Args:
    //    pose_msg (*PoseStamped): pointer to the incoming pose message
    // Returns:
    //

    // tree as std::vector
    std::vector<Node> tree;

    // TODO: fill in the RRT main loop
    // Node first_node;
    
    first_node.x = pose_msg->pose.position.x;
    first_node.y = pose_msg->pose.position.y;
    first_node.parent = -1;
    first_node.is_root = true;
    tree.push_back(first_node);

    // get_marker(marker, first_node.x, first_node.y, "map");
    // vis_pub.publish(marker);

    transformed_waypoints = get_transformed_waypoints(pure_waypoints);
    best_global_waypoint = get_best_global_waypoint(first_node.x, first_node.y);

    int iter_count = 0;
    while(iter_count < RRT_iters){
        //iteration number
        iter_count += 1;
        ROS_INFO("Entered while!");

        //sample a point from space
        std::vector<double> sampled_point = sample();
        int sample_occu_index = get_occu_data_index(sampled_point[0], sampled_point[1]);
        if(occu_map.data[sample_occu_index] == 100) continue;

        //from the current tree find the node closest to the sampled point 
        int nearest_node_index = nearest(tree, sampled_point);
        Node nearest_node = tree[nearest_node_index];

        //find x_new along x_nearest and x_rand
        Node new_node = steer(nearest_node, sampled_point, nearest_node_index);

        //check if the path joining x_nearest and x_new has an obstacle. If true then skip to next iteration
        if(check_collision(nearest_node, new_node)){
            // ROS_INFO("Collision Detected");
            continue; 
        } else if (rrt_star){
            new_node.cost = cost(tree, new_node);
            std::vector<int> neighborhood = near(tree, new_node);
            int best_neighbor = nearest_node_index;

            std::vector<bool> neighbor_collision;

            //find the node which will cause new_node to be of least cost and make that as the parent of new_node
            for(int i = 0; i < neighborhood.size(); i++){
                if(check_collision(tree[neighborhood[i]], new_node)){
                    neighbor_collision.push_back(true);
                    continue;
                }
                neighbor_collision.push_back(false);

                double this_neighbor_cost = tree[neighborhood[i]].cost + line_cost(tree[neighborhood[i]], new_node);

                if(this_neighbor_cost < new_node.cost){
                    new_node.cost = this_neighbor_cost;
                    new_node.parent = neighborhood[i];
                    best_neighbor = neighborhood[i];
                }
            }

            //rewiring process
            for(int i=0; i < neighborhood.size(); i++){
                if(neighbor_collision[i] == true || neighborhood[i] == best_neighbor){
                    continue;
                }
                double cost_from_new_node = new_node.cost + line_cost(new_node, tree[neighborhood[i]]);
                if (tree[neighborhood[i]].cost > cost_from_new_node){
                    tree[neighborhood[i]].cost = cost_from_new_node;
                    tree[neighborhood[i]].parent = tree.size();
                }
            }

        } 

        //if not colliding add that to the tree
        tree.push_back(new_node);

        if(is_goal(new_node, best_global_waypoint[0], best_global_waypoint[1])){

            std::vector<double> color1{0.0, 0.0, 1.0};
            // visualize_trackpoints(tree, color1);
            std::vector<Node> found_path = find_path(tree, new_node);
            // std::cout << tree.size() << "   " << found_path.size() << std::endl;
            std::vector<double> color2{1.0, 0.0, 0.0};
            // visualize_pathpoints(found_path, color2);

            std::vector<double> node_to_go = get_node_to_go(found_path, pose_msg->pose.position.x, pose_msg->pose.position.y);
            double L = node_to_go[2];

            // get_marker(marker, node_to_go[0], node_to_go[1], "map");
            // vis_pub.publish(marker);

            geometry_msgs::PoseStamped way_point;
            way_point.header.frame_id = "map";
            way_point.pose.position.x = node_to_go[0];
            way_point.pose.position.y = node_to_go[1];
            way_point.pose.orientation.x = 0.0;
            way_point.pose.orientation.y = 0.0;
            way_point.pose.orientation.z = 0.0;
            way_point.pose.orientation.w = 1.0;

            tf2::doTransform(way_point, way_point, map_to_laser);

            steering_angle = get_steering_angle(way_point.pose.position.x, way_point.pose.position.y, L);
            drive_msg.drive.steering_angle = steering_angle; //steering_angle
            drive_msg.drive.speed = get_speed(steering_angle); //speed
            nav_pub.publish(drive_msg);

            // visualize_trackpoints(way_point.pose.position.x, way_point.pose.position.y,
            //                       best_global_waypoint[0], best_global_waypoint[1]);
            visualize_trackpoints(node_to_go[0], node_to_go[1],
                                  best_global_waypoint[0], best_global_waypoint[1]);

            ROS_INFO("Path found");

            break;
        }

        // ROS_INFO("Path not found!");

    }



    // path found as Path message

}

std::vector<double> RRT::sample() {
    // This method returns a sampled point from the free space
    // You should restrict so that it only samples a small region
    // of interest around the car's current position
    // Args:
    // Returns:
    //     sampled_point (std::vector<double>): the sampled point in free space

    std::vector<double> sampled_point;
    // TODO: fill in this method
    // look up the documentation on how to use std::mt19937 devices with a distribution
    // the generator and the distribution is created for you (check the header file)

    std::uniform_real_distribution<>::param_type parameters_x(0, lookahead_distance);
    std::uniform_real_distribution<>::param_type parameters_y(-lookahead_distance, lookahead_distance);
    x_dist.param(parameters_x);
    y_dist.param(parameters_y);

    geometry_msgs::Pose sample;
    sample.position.x = x_dist(gen);
    sample.position.y = y_dist(gen);
    sample.position.z = 0;
    sample.orientation.x = 0;
    sample.orientation.y = 0;
    sample.orientation.z = 0;
    sample.orientation.w = 1;
    tf2::doTransform(sample, sample, laser_to_map);

    sampled_point.push_back(sample.position.x);
    sampled_point.push_back(sample.position.y);
    
    return sampled_point;
}


int RRT::nearest(std::vector<Node> &tree, std::vector<double> &sampled_point) {
    // This method returns the nearest node on the tree to the sampled point
    // Args:
    //     tree (std::vector<Node>): the current RRT tree
    //     sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //     nearest_node (int): index of nearest node on the tree

    int nearest_node = 0;
    double currest_min_dist = std::numeric_limits<double>::max();
    // TODO: fill in this method
    for(int i=0; i < tree.size(); i++){
        double dist = pow(tree[i].x - sampled_point[0], 2) + pow(tree[i].y - sampled_point[1], 2);
        if (dist < currest_min_dist){
            nearest_node = i;
            currest_min_dist = dist;
        }
    }


    return nearest_node;
}

Node RRT::steer(Node &nearest_node, std::vector<double> &sampled_point, int nearest_node_index) {
    // The function steer:(x,y)->z returns a point such that z is “closer” 
    // to y than x is. The point z returned by the function steer will be 
    // such that z minimizes ||z−y|| while at the same time maintaining 
    //||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0

    // basically, expand the tree towards the sample point (within a max dist)

    // Args:
    //    nearest_node (Node): nearest node on the tree to the sampled point
    //    sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //    new_node (Node): new node created from steering

    Node new_node;
    const double x_coord = sampled_point[0] - nearest_node.x;
    const double y_coord = sampled_point[1] - nearest_node.y;
    const double distance = sqrt(pow(x_coord,2) + pow(y_coord,2));
    // TODO: fill in this method
    if(distance > max_expansion_distance){
        const double theta = atan2(y_coord,x_coord);
        new_node.x = nearest_node.x + cos(theta)*max_expansion_distance;
        new_node.y = nearest_node.y + sin(theta)*max_expansion_distance;
    } else {
        new_node.x = sampled_point[0];
        new_node.y = sampled_point[1];
    }

    new_node.parent = nearest_node_index;

    return new_node;
}

bool RRT::check_collision(Node &nearest_node, Node &new_node) {
    // This method returns a boolean indicating if the path between the 
    // nearest node and the new node created from steering is collision free
    // Args:
    //    nearest_node (Node): nearest node on the tree to the sampled point
    //    new_node (Node): new node created from steering
    // Returns:
    //    collision (bool): true if in collision, false otherwise

    bool collision = false;
    // TODO: fill in this method
    double x_discrete = (new_node.x - nearest_node.x)/collision_discretization;
    double y_discrete = (new_node.y - nearest_node.y)/collision_discretization;

    double x_current = nearest_node.x;
    double y_current = nearest_node.y;

    int map_columns = occu_map.info.width;

    for(int i = 0; i < collision_discretization; i++){
        x_current += x_discrete;
        y_current += y_discrete;
        int occu_map_index = get_occu_data_index(x_current,y_current);
        if(occu_map.data[occu_map_index] == 100){
            collision = true;
        }
    }

    return collision;
}

int RRT::get_occu_data_index(double &x_map, double &y_map){

    const int x_occu_map = static_cast<int>((x_map - occu_map.info.origin.position.x)/occu_map.info.resolution);
    const int y_occu_map = static_cast<int>((y_map - occu_map.info.origin.position.y)/occu_map.info.resolution);

    return ((occu_map.info.width*y_occu_map) + x_occu_map);
}

bool RRT::is_goal(Node &latest_added_node, double goal_x, double goal_y) {
    // This method checks if the latest node added to the tree is close
    // enough (defined by goal_threshold) to the goal so we can terminate
    // the search and find a path
    // Args:
    //   latest_added_node (Node): latest addition to the tree
    //   goal_x (double): x coordinate of the current goal
    //   goal_y (double): y coordinate of the current goal
    // Returns:
    //   close_enough (bool): true if node close enough to the goal

    bool close_enough = false;
    // TODO: fill in this method
    double x_diff = goal_x - latest_added_node.x;
    double y_diff = goal_y - latest_added_node.y;

    double closeness = sqrt(pow(x_diff,2) + pow(y_diff,2));

    if (closeness <= goal_close_enough) close_enough = true;

    return close_enough;
}

std::vector<Node> RRT::find_path(std::vector<Node> &tree, Node &latest_added_node) {
    // This method traverses the tree from the node that has been determined
    // as goal
    // Args:
    //   latest_added_node (Node): latest addition to the tree that has been
    //      determined to be close enough to the goal
    // Returns:
    //   path (std::vector<Node>): the vector that represents the order of
    //      of the nodes traversed as the found path
    
    std::vector<Node> found_path;
    // TODO: fill in this method
    Node current_node = latest_added_node;

    while(current_node.is_root != true){
        found_path.push_back(current_node);
        current_node = tree[current_node.parent];
    }

    return found_path;
}

// RRT* methods
double RRT::cost(std::vector<Node> &tree, Node &node) {
    // This method returns the cost associated with a node
    // Args:
    //    tree (std::vector<Node>): the current tree
    //    node (Node): the node the cost is calculated for
    // Returns:
    //    cost (double): the cost value associated with the node

    double cost = 0;
    // TODO: fill in this method
    cost = tree[node.parent].cost + line_cost(node, tree[node.parent]);

    return cost;
}

double RRT::line_cost(Node &n1, Node &n2) {
    // This method returns the cost of the straight line path between two nodes
    // Args:
    //    n1 (Node): the Node at one end of the path
    //    n2 (Node): the Node at the other end of the path
    // Returns:
    //    cost (double): the cost value associated with the path

    double cost = 0;
    // TODO: fill in this method
    cost = sqrt(pow(n1.x - n2.x,2) + pow(n1.y - n2.y,2));

    return cost;
}

std::vector<int> RRT::near(std::vector<Node> &tree, Node &node) {
    // This method returns the set of Nodes in the neighborhood of a 
    // node.
    // Args:
    //   tree (std::vector<Node>): the current tree
    //   node (Node): the node to find the neighborhood for
    // Returns:
    //   neighborhood (std::vector<int>): the index of the nodes in the neighborhood

    std::vector<int> neighborhood;
    // TODO:: fill in this method
    for(int i=0; i < tree.size(); i++){
        double distance = sqrt(pow(node.x - tree[i].x,2) + pow(node.y - tree[i].y,2));
        if (distance < search_radius){
            neighborhood.push_back(i);
        }
    }

    return neighborhood;
}

//my methods
std::vector<int> RRT::get_occupancy_indices(const double x_map, const double y_map){
    //convert the coordinates in map frame into discretized map cells
    std::vector<int> occupancy_indices;
    const int x_occu_map = static_cast<int>((x_map - occu_map.info.origin.position.x)/occu_map.info.resolution);
    const int y_occu_map = static_cast<int>((y_map - occu_map.info.origin.position.y)/occu_map.info.resolution);

    for(int i = x_occu_map - expand_radius; i <= x_occu_map + expand_radius; i++){
        for(int j = y_occu_map - expand_radius; j <= y_occu_map + expand_radius; j++){
            occupancy_indices.push_back((occu_map.info.width*j) + i);
        }
    }

    return occupancy_indices;
}

void RRT::get_marker(visualization_msgs::Marker &marker, double &waypoint_x, double &waypoint_y, std::string frame_name){

    marker.header.frame_id = frame_name;
    marker.header.stamp = ros::Time();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = waypoint_x; //7.64029 
    marker.pose.position.y = waypoint_y; //9.04719 
    marker.pose.position.z = 0.0;
    marker.scale.x = 0.15;
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

}

std::vector<double> RRT::get_node_to_go(std::vector<Node> &found_path,const double &pose_x,const double &pose_y){

    std::vector<double> node_to_go;

    double current_min_dist = std::numeric_limits<double>::max();
    double current_dist_to_node = std::numeric_limits<double>::max();
    double current_x = 0;
    double current_y = 0;
    int node_to_go_index = -1;
    for(int i=0; i < found_path.size(); i++){
        double diff_x = found_path[i].x - pose_x;
        double diff_y = found_path[i].y - pose_y;
        double distance = sqrt(pow(diff_x,2) + pow(diff_y,2));

        if(std::abs(distance - pure_pursuit_lookahead) < current_min_dist){
            current_x = found_path[i].x;
            current_y = found_path[i].y;
            current_dist_to_node = distance;
            current_min_dist = std::abs(distance - pure_pursuit_lookahead);
        }

    }
    node_to_go.push_back(current_x);
    node_to_go.push_back(current_y);
    node_to_go.push_back(current_dist_to_node);

    return node_to_go;
}

void RRT::get_data(std::vector<std::vector<double>> &waypoints){
    std::ifstream file(file_path);
    if (!file)
    {
        std::cout << "Invalid path" << "\n";
    }
    std::string line = "";
    while (getline(file, line))
    {
        std::vector<std::string> vec;
        boost::algorithm::split(vec, line, boost::is_any_of(","));
        std::vector<double> waypoint{};
        waypoint.push_back(stod(vec[0]));
        waypoint.push_back(stod(vec[1]));
        waypoint.push_back(stod(vec[2]));
        waypoints.push_back(waypoint);
    }
}

std::vector<double> RRT::get_best_global_waypoint(double &current_pose_x, double current_pose_y){
    int best_waypoint_index = -1;
    double best_waypoint_dist = std::numeric_limits<double>::max();

    for(int i=0; i<transformed_waypoints.size(); i++){
        if(transformed_waypoints[i][0] < 0) continue;

        double current_waypoint_dist = std::abs(lookahead_distance - sqrt(pow(transformed_waypoints[i][0],2) + pow(transformed_waypoints[i][1],2)));

        if(current_waypoint_dist < best_waypoint_dist){
            int occu_map_index = get_occu_data_index(pure_waypoints[i][0], pure_waypoints[i][1]);
            if(occu_map.data[occu_map_index] == 100) continue;
            best_waypoint_dist = current_waypoint_dist;
            best_waypoint_index = i;
        }
    }

    return pure_waypoints[best_waypoint_index];
}

std::vector<std::vector<double>> RRT::get_transformed_waypoints(const std::vector<std::vector<double>> &waypoints){
    try
    {
        map_to_laser = tfBuffer.lookupTransform("laser", "map", ros::Time(0));
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(0.1).sleep();
    }

    std::vector<std::vector<double>> tranformed_waypoints;
    for(int i = 0; i < waypoints.size(); i++){
        std::vector<double> v;
        geometry_msgs::PoseStamped way_point;
        way_point.header.frame_id = "map";
        way_point.pose.position.x = waypoints[i][0];
        way_point.pose.position.y = waypoints[i][1];
        way_point.pose.orientation.x = 0.0;
        way_point.pose.orientation.y = 0.0;
        way_point.pose.orientation.z = 0.0;
        way_point.pose.orientation.w = 1.0;

        tf2::doTransform(way_point, way_point, map_to_laser);
        // listener.transformPose("base_link", way_point, base_point);
        v.push_back(way_point.pose.position.x);
        v.push_back(way_point.pose.position.y);

        tranformed_waypoints.push_back(v);
    }

    return tranformed_waypoints;
}

double RRT::get_steering_angle(const double waypoint_x, const double waypoint_y, const double L){
    double steering_angle;

    steering_angle = 0.5*(2*waypoint_y)/(L*L);
    if (abs(steering_angle) >= 0.41){
        steering_angle = steering_angle > 0 ? 0.41 : -0.41;
    }

    return steering_angle;
}

double RRT::get_speed(double steering_angle){ 
    //steering_angle = clip(steering_angle, -0.44, 0.44);
    if ((abs(steering_angle) >= 0) && (abs(steering_angle) < 0.174533)){
      speed = 4.0; //2.5, 3.5, 4.5
    } else if ((abs(steering_angle) >= 0.174533) && (abs(steering_angle) < 0.349066)){ //0.349066
      speed = 3.5; //1.5, 2.5, 3.5
    } else{
      speed = 2.5; //0.5, 1.5, 2.0
    }
    return speed;
}

void RRT::visualize_trackpoints(double x_local, double y_local, double x_global, double y_global)
{
    visualization_msgs::Marker line;

    line.header.frame_id    = "/map";
    line.header.stamp       = ros::Time::now();
    line.lifetime           = ros::Duration(0.1);
    line.id                 = 1;
    line.ns                 = "rrt";
    line.type               = visualization_msgs::Marker::LINE_STRIP;
    line.action             = visualization_msgs::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x            = 0.05;
    line.color.r            = 0.0f;
    line.color.g            = 0.0f;
    line.color.b            = 1.0f;
    line.color.a            = 1.0f;

    geometry_msgs::Point current;
    geometry_msgs::Point local;
    geometry_msgs::Point global;

    current.x = first_node.x;
    current.y = first_node.y;
    current.z =  0;

    local.x = x_local;
    local.y = y_local;
    local.z = 0;

    global.x = x_global;
    global.y = y_global;
    global.z = 0;

    line.points.push_back(current);
    line.points.push_back(local);
    line.points.push_back(global);

    tree_viz_pub_.publish(line);
    // unique_id_++;
}

void RRT::visualize_trackpoints(std::vector<Node> &tree, std::vector<double> color)
{   
    visualization_msgs::Marker line;

    line.header.frame_id    = "/map";
    line.header.stamp       = ros::Time::now();
    line.lifetime           = ros::Duration(0.1);
    line.id                 = 1;
    line.ns                 = "rrt";
    line.type               = visualization_msgs::Marker::LINE_LIST;
    line.action             = visualization_msgs::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x            = 0.05;
    line.color.a            = 1.0f;
    line.color.r            = color[0];
    line.color.g            = color[1];
    line.color.b            = color[2];


    for(int i = 0; i < tree.size(); i++){

        geometry_msgs::Point current_node;
        current_node.x = tree[i].x;
        current_node.y = tree[i].y;
        current_node.z = 0;
        int current_node_index = i;



        for(int j = 0; j < tree.size(); j++){
            if( tree[j].parent == current_node_index){
                geometry_msgs::Point child_node;
                child_node.x = tree[j].x;
                child_node.y = tree[j].y;
                child_node.z = 0;

                line.points.push_back(current_node);
                line.points.push_back(child_node);
            }
        }

    }

    tree_viz_pub_.publish(line);
    // unique_id_++;
}

// void RRT::visualize_pathpoints(std::vector<Node> &tree, std::vector<double> color)
// {   
//     visualization_msgs::Marker line;

//     line.header.frame_id    = "/map";
//     line.header.stamp       = ros::Time::now();
//     line.lifetime           = ros::Duration(0.1);
//     line.id                 = 1;
//     line.ns                 = "rrt";
//     line.type               = visualization_msgs::Marker::LINE_LIST;
//     line.action             = visualization_msgs::Marker::ADD;
//     line.pose.orientation.w = 1.0;
//     line.scale.x            = 0.05;
//     line.color.a            = 1.0f;
//     line.color.r            = color[0];
//     line.color.g            = color[1];
//     line.color.b            = color[2];


//     for(int i = 0; i < tree.size(); i++){

//         geometry_msgs::Point current_node;
//         current_node.x = tree[i].x;
//         current_node.y = tree[i].y;
//         current_node.z = 0;
//         int current_node_index = i;



//         for(int j = 0; j < tree.size(); j++){
//             if( tree[j].parent == current_node_index){
//                 geometry_msgs::Point child_node;
//                 child_node.x = tree[j].x;
//                 child_node.y = tree[j].y;
//                 child_node.z = 0;

//                 line.points.push_back(current_node);
//                 line.points.push_back(child_node);
//             }
//         }

//     }

//     std::cout << line.points.size() << std::endl;

//     path_viz_pub_.publish(line);
//     // unique_id_++;
// }

void RRT::visualize_pathpoints(std::vector<Node> &tree, std::vector<double> color)
{   
    visualization_msgs::Marker line;

    line.header.frame_id    = "/map";
    line.header.stamp       = ros::Time::now();
    line.lifetime           = ros::Duration(0.1);
    line.id                 = 1;
    line.ns                 = "rrt";
    line.type               = visualization_msgs::Marker::LINE_STRIP;
    line.action             = visualization_msgs::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x            = 0.05;
    line.color.a            = 1.0f;
    line.color.r            = color[0];
    line.color.g            = color[1];
    line.color.b            = color[2];


    for(int i = 0; i < tree.size(); i++){

        geometry_msgs::Point current_node;
        current_node.x = tree[i].x;
        current_node.y = tree[i].y;
        current_node.z = 0;
        line.points.push_back(current_node);

    }

    // std::cout << line.points.size() << std::endl;

    path_viz_pub_.publish(line);
    // unique_id_++;
}




