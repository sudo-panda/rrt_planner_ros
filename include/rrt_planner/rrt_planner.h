//
// Created by sudo-panda on 4/4/20.
//

#ifndef RRT_PLANNER_INCLUDE_RRT_PLANNER_RRT_PLANNER_H_
#define RRT_PLANNER_INCLUDE_RRT_PLANNER_RRT_PLANNER_H_

#include <random>
#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <opencv2/opencv.hpp>

#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/geometries.hpp>

namespace rrt_planner {

namespace bg = boost::geometry;
typedef bg::model::d2::point_xy<int> point_t;
typedef bg::model::multi_point<point_t> multip_t;
typedef std::pair<point_t, int> value;

class RRTPlanner {

 public:
  explicit RRTPlanner(ros::NodeHandle *);

  ~RRTPlanner() = default;

  void plan();

  void map_callback(const nav_msgs::OccupancyGrid::Ptr &);
  void init_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &);
  void goal_callback(const geometry_msgs::PoseStamped::ConstPtr &);

 private:
  void build_map();
  void plot_goal_init_pose();
  void view_map(int delay = 1);

  void init();
  void rand_conf(point_t &);
  int nearest_vertex(const point_t &);
  bool new_vertex(point_t &, const point_t &);
  void add(point_t &, int);
  void get_path();

  void plot_circle(point_t &, int, cv::Scalar);
  void plot_line(point_t &, point_t &, cv::Scalar, int = 1);

  geometry_msgs::PoseStamped point_to_pose(const point_t &);
  void pose_to_point(point_t &p, const geometry_msgs::Pose &);
  int to_index(int, int);

  bool is_line_unoccupied(const point_t &, const point_t &);
  bool is_point_unoccupied(const point_t &);
  bool is_point_valid(const point_t &);

  int make_valid_x(int);
  int make_valid_y(int);

  static inline int make_valid(const int n, const int lower, const int upper) {
    return std::min(upper - 1, std::max(lower, n));
  }

  ros::NodeHandle *nh_;
  ros::NodeHandle private_nh_;

  bool visualize_;

  std::mt19937 generator_;
  std::unique_ptr<std::uniform_int_distribution<int> > dist_x_;
  std::unique_ptr<std::uniform_int_distribution<int> > dist_y_;
  std::unique_ptr<std::uniform_int_distribution<int> > dist_goal_;

  bool map_received_;
  std::unique_ptr<cv::Mat> map_;
  nav_msgs::OccupancyGrid::Ptr map_grid_;

  bool init_pose_received_;
  point_t init_pose_;

  bool goal_received_;
  point_t goal_;

  int max_vertices_;
  int sample_distance_;
  bg::index::rtree<value, bg::index::quadratic<16> > rtree_;
  multip_t vertices_;
  std::vector<int> edges_;

  ros::Subscriber map_sub_;
  ros::Subscriber init_pose_sub_;
  ros::Subscriber goal_sub_;
  ros::Publisher path_pub_;
};

}

#endif //RRT_PLANNER_INCLUDE_RRT_PLANNER_RRT_PLANNER_H_
