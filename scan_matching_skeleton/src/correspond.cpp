#include "scan_matching_skeleton/correspond.h"
#include "cmath"
#include "ros/ros.h"
#include <limits>

using namespace std;

const int UP_SMALL = 0;
const int UP_BIG = 1;
const int DOWN_SMALL = 2;
const int DOWN_BIG = 3;

void getNaiveCorrespondence(vector<Point>& old_points, vector<Point>& trans_points, vector<Point>& points,
                        vector< vector<int> >& jump_table, vector<Correspondence>& c, float prob){

      c.clear();
      int last_best = -1;
      const int n = trans_points.size();
      const int m = old_points.size();
      float min_dist = 100000.00;
      int min_index = 0;
      int second_min_index = 0;

      //Do for each point
      for(int i = 0; i<n; ++i){
        for(int j = 0; j<m; ++j){
          float dist = old_points[i].distToPoint2(&trans_points[j]);
          if(dist<min_dist){
            min_dist = dist;
            min_index = j;
            second_min_index = j-1;
          }
        }
        c.push_back(Correspondence(&trans_points[i], &points[i], &old_points[min_index], &old_points[second_min_index]));
      }


}

void getCorrespondence(vector<Point>& old_points, vector<Point>& trans_points, vector<Point>& points,
                        vector< vector<int> >& jump_table, vector<Correspondence>& c, float prob){

  // Written with inspiration from: https://github.com/AndreaCensi/gpc/blob/master/c/gpc.c
  // use helper functions and structs in transform.h and correspond.h
  // input : old_points : vector of struct points containing the old points (points of the previous frame)
  // input : trans_points : vector of struct points containing the new points transformed to the previous frame using the current estimated transform
  // input : points : vector of struct points containing the new points
  // input : jump_table : jump table computed using the helper functions from the transformed and old points
  // input : c: vector of struct correspondences . This is a refernece which needs to be updated in place and return the new correspondences to calculate the transforms.
  // output : c; update the correspondence vector in place which is provided as a reference. you need to find the index of the best and the second best point. 
  //Initializecorrespondences
  c.clear();
  int last_best = 0;
  const int n = trans_points.size();
  const int m = old_points.size();
  const double increment = 0.00582315586507;

  //Do for each point
  for(int i = 0; i<n; ++i){
  	// ROS_INFO("i = %d", i);
  	int best = last_best; //best variable will be updated as when a better correspondence is found
  	double best_dist = old_points[best].distToPoint2(&trans_points[i]); //distance of the current trans_point under consideration from the old_point at index = last_best
  	int start_index = i; //will start from the next point
  	int up = start_index + 1; //for up direction - start from one higher index for search
  	int down = last_best; //for down direction start from last_best itself
  	double last_dist_up =  std::numeric_limits<double>::infinity(); //initialise dist of current trans_point under consideration from the old_point at last_best index
  	double last_dist_down = std::numeric_limits<double>::infinity();
  	bool up_stopped = false; //start with both bool conditions false
  	bool down_stopped = false;

  	while(!(up_stopped && down_stopped)){  //will stop executing when both up search and down search is done
  		// ROS_INFO("still in while, up_stopped = %d", last_dist_up < last_dist_down);
  		bool now_up = !(up_stopped);// && (last_dist_up < last_dist_down)); //now_up = true if up_stopped is false i.e. up direction search is still not done, otherwise it will go to down direction
  		ROS_INFO("up: %d %d", up_stopped, down_stopped);
  		if (now_up){
  			if(up >= m) {up_stopped = true; continue;} //stop up search when it exceeds the number of points in the scan
  			last_dist_up = old_points[up].distToPoint2(&trans_points[i]); //distance of the currect up search index from the trans point under consideration
  			if (last_dist_up < best_dist){ //if current up search is better than previous best one - update the previous best one with this current one
  				best = up;
  				best_dist = last_dist_up;
  			}
  			if (up > start_index){ //now have to decide whether to move forward in up direction based on angle condition
  				double phi = (up - best)*increment; //compute angle between the best so far and the current up search index
  				double min_dist_up = sin(phi)*trans_points[i].r; //compute the distance trans point under consideration to the line joining the origin of ray to the current up search point
  				if (min_dist_up*min_dist_up > best_dist){ //if the min_dist is already exceeded - stop up search
  					up_stopped = true;
  					continue;
  				} 
  				up = (old_points[up].r < trans_points[i].r)? jump_table[up][UP_BIG] : jump_table[up][UP_SMALL]; //if not exceeded move to the next point based on jump table and it will again enter loop for up search
  			} else {
  				up++;
  			}
  		}
  		if (!now_up){
  			if (down <= 0) {down_stopped = true; continue;}
  			last_dist_down = old_points[down].distToPoint2(&trans_points[i]);
  			if (last_dist_down < best_dist){
  				best = down;
  				best_dist = last_dist_down;
  			}
  			if (down < start_index){
  				double phi = (best - down)*increment;
  				double min_dist_down = sin(phi)*trans_points[i].r;
  				if (min_dist_down*min_dist_down > best_dist){
  					down_stopped = true;
  					continue;
  				}
  				down = (old_points[down].r < trans_points[i].r)? jump_table[down][DOWN_BIG] : jump_table[down][DOWN_SMALL];

  			} else {
  				down--;
  			}
  		}
  	}

    

    // int best = 0;
    int second_best = best - 1;

      c.push_back(Correspondence(&trans_points[i], &points[i], &old_points[best], &old_points[second_best]));
    }
  }


void computeJump(vector< vector<int> >& table, vector<Point>& points){
  table.clear();
  int n = points.size();
  for(int i = 0; i<n; ++i){
    vector<int> v = {n,n,-1,-1};
    for(int j = i+1; j<n; ++j){
      if(points[j].r<points[i].r){
        v[UP_SMALL] = j;
        break;
      }
    }
    for(int j = i+1; j<n; ++j){
      if(points[j].r>points[i].r){
        v[UP_BIG] = j;
        break;
      }
    }
    for(int j = i-1; j>=0; --j){
      if(points[j].r<points[i].r){
        v[DOWN_SMALL] = j;
        break;
      }
    }
    for(int j = i-1; j>=0; --j){
      if(points[j].r>points[i].r){
        v[DOWN_BIG] = j;
        break;
      }
    }
    table.push_back(v);
  }
}
