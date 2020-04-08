//
// Created by sudo-panda on 4/4/20.
//

#include "rrt_planner/rrt_planner.h"

namespace rrt_planner {

RRTPlanner::RRTPlanner(ros::NodeHandle *node)
    :
    nh_(node),
    private_nh_("~"),
    map_received_(false),
    init_pose_received_(false),
    goal_received_(false) {
  ROS_INFO("Looking for map ..... ");
  ros::Duration duration(1);
  duration.sleep();

  std::string map_topic, path_topic;
  int goal_bias;

  private_nh_.param<std::string>("map_topic", map_topic, "/map");
  private_nh_.param<std::string>("path_topic", path_topic, "/path");
  private_nh_.param<int>("max_vertices", max_vertices_, 5000);
  private_nh_.param<int>("sample_distance", sample_distance_, 10);
  private_nh_.param<int>("goal_bias_percentage", goal_bias, 10);
  private_nh_.param<bool>("visualization_on", visualize_, true);

  dist_goal_ = std::unique_ptr<std::uniform_int_distribution<int> >(
      new std::uniform_int_distribution<int>(0, 100 / goal_bias));

  map_sub_ = nh_->subscribe
      <const nav_msgs::OccupancyGrid::Ptr &>(map_topic, 1, &RRTPlanner::map_callback, this);
  init_pose_sub_ = nh_->subscribe
      <const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &>("/initialpose",
                                                                   1,
                                                                   &RRTPlanner::init_pose_callback,
                                                                   this);
  goal_sub_ = nh_->subscribe
      <const geometry_msgs::PoseStamped::ConstPtr &>("/move_base_simple/goal",
                                                     1,
                                                     &RRTPlanner::goal_callback,
                                                     this);

  path_pub_ = nh_->advertise<nav_msgs::Path>(path_topic, 1, true);

  std::random_device device;
  generator_ = std::mt19937(device());
  generator_.discard(700000);

  while (ros::ok()) {
    if (map_received_ && init_pose_received_ && goal_received_) {
      build_map();
      plot_goal_init_pose();
      plan();
    } else {
      if (map_received_)
        view_map();
      ros::Duration duration(0.1);
      duration.sleep();
      ros::spinOnce();
    }
  }
}

void RRTPlanner::map_callback(const nav_msgs::OccupancyGrid::Ptr &msg) {
  map_grid_ = msg;

  dist_x_ = std::unique_ptr<std::uniform_int_distribution<int> >(
      new std::uniform_int_distribution<int>(0, map_grid_->info.height));

  dist_y_ = std::unique_ptr<std::uniform_int_distribution<int> >(
      new std::uniform_int_distribution<int>(0, map_grid_->info.width));

  build_map();
  view_map();

  map_received_ = true;
  init_pose_received_ = false;
  goal_received_ = false;
  ROS_INFO("Map obtained successfully. Please provide initial pose and goal through RViz.");
}

void RRTPlanner::init_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
  if (init_pose_received_) {
    build_map();
  }

  pose_to_point(init_pose_, msg->pose.pose);

  if (!is_point_unoccupied(init_pose_)) {
    init_pose_received_ = false;
    ROS_WARN("The initial pose specified is on or too close to an obstacle please specify another point");
  } else {
    init_pose_received_ = true;
    plot_goal_init_pose();
    ROS_INFO("Initial pose obtained successfully.");
  }
  view_map();
}

void RRTPlanner::goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  if (goal_received_) {
    build_map();
  }

  pose_to_point(goal_, msg->pose);

  if (!is_point_unoccupied(goal_)) {
    goal_received_ = false;
    ROS_WARN("The goal specified is on or too close to an obstacle please specify another point");
  } else {
    goal_received_ = true;
    plot_goal_init_pose();
    ROS_INFO("Goal obtained successfully.");
  }
  view_map();
}

void RRTPlanner::plot_goal_init_pose() {
  if (goal_received_) {
    plot_circle(goal_, 3, cv::Scalar(12, 255, 43));
  }
  if (init_pose_received_) {
    plot_circle(init_pose_, 3, cv::Scalar(255, 200, 0));
  }
}

void RRTPlanner::plan() {
  goal_received_ = false;
  init_pose_received_ = false;

  init();

  for (unsigned k = 0; k < max_vertices_ && ros::ok() && !goal_received_ && !init_pose_received_; k++) {
    view_map();
    if (k % 100 == 0)
      ROS_INFO("%d points added", k);

    point_t p_new;
    bool found_valid_vertex;
    int p_near_index;

    do {
      rand_conf(p_new);
      p_near_index = nearest_vertex(p_new);
      found_valid_vertex = new_vertex(p_new, vertices_.at(p_near_index));
    } while (!found_valid_vertex && ros::ok());
    add(p_new, p_near_index);
    if (bg::comparable_distance(p_new, goal_) <= sample_distance_ * sample_distance_) {
      add(goal_, vertices_.size() - 1);
      get_path();
      ROS_INFO("Path found");
      return;
    }
  }
  ROS_ERROR("Path not found");
}

void RRTPlanner::init() {
  rtree_.clear();
  vertices_.clear();
  edges_.clear();
  vertices_.push_back(init_pose_);
  edges_.push_back(0);
  rtree_.insert(std::make_pair(init_pose_, 0));
}

void RRTPlanner::rand_conf(point_t &p_rand) {
  if ((*dist_goal_)(generator_) == 0) {
    bg::assign_point(p_rand, goal_);
  } else {
    bg::assign_values(p_rand, (*dist_x_)(generator_), (*dist_y_)(generator_));
  }
}

int RRTPlanner::nearest_vertex(const point_t &p_rand) {
  std::vector<value> result;
  rtree_.query(bg::index::nearest(p_rand, 1), std::back_inserter(result));
  return result.at(0).second;
}

bool RRTPlanner::new_vertex(point_t &p_new, const point_t &p_near) {

  bg::subtract_point(p_new, p_near);

  if (bg::equals(p_new, point_t(0, 0))) {
    return false;
  }

  int dist = bg::distance(p_new, point_t(0, 0));
  bg::multiply_value(p_new, sample_distance_);
  bg::divide_value(p_new, dist);
  bg::add_point(p_new, p_near);

  if (!is_point_valid(p_new)) {
    return false;
  }
  if (bg::equals(vertices_.at(nearest_vertex(p_new)), p_near)) {
    return is_line_unoccupied(p_new, p_near);
  } else {
    return false;
  }
}

void RRTPlanner::add(point_t &p_new, const int near_index) {
  vertices_.push_back(p_new);
  rtree_.insert(std::make_pair(p_new, vertices_.size() - 1));
  edges_.push_back(near_index);

  plot_line(p_new, vertices_.at(near_index), cv::Scalar(65, 172, 255));
  plot_circle(p_new, 2, cv::Scalar(86, 30, 255));
}

void RRTPlanner::get_path() {
  int end;
  nav_msgs::Path path;

  path.header.frame_id = map_grid_->header.frame_id;
  path.header.stamp = ros::Time::now();

  end = vertices_.size() - 1;
  path.poses.push_back(point_to_pose(vertices_.at(end)));

  do {
    int begin;
    begin = edges_.at(end);
    plot_line(vertices_.at(end), vertices_.at(begin), cv::Scalar(69, 3, 97), 2);
    end = begin;
    path.poses.push_back(point_to_pose(vertices_.at(end)));
  } while (end != 0);

  std::reverse(path.poses.begin(), path.poses.end());

  path_pub_.publish(path);

  view_map();
}

bool RRTPlanner::is_line_unoccupied(const point_t &p1, const point_t &p2) {
  if (!(is_point_valid(p1) && is_point_valid(p2))) {
    return false;
  }
  int del_x = p1.x() - p2.x();
  int del_y = p1.y() - p2.y();

  if (std::abs(del_x) < std::abs(del_y)) {
    double m = static_cast<double>(del_x) / del_y;
    for (int y = make_valid_y(std::min(p1.y(), p2.y()) - 1);
         y <= make_valid_y(std::max(p1.y(), p2.y()) + 1);
         y++) {
      int x = static_cast<int>((y - p2.y()) * m + p2.x());

      if (x < 0 || x >= map_grid_->info.height)
        continue;

      if (map_grid_->data[to_index(x, y)] != 0 ||
          map_grid_->data[to_index(make_valid_x(x - 1), y)] != 0 ||
          map_grid_->data[to_index(make_valid_x(x + 1), y)] != 0) {
        return false;
      }
    }
  } else {
    double m = static_cast<double>(del_y) / del_x;
    for (int x = make_valid_x(std::min(p1.x(), p2.x()) - 1);
         x <= make_valid_x(std::max(p1.x(), p2.x()) + 1);
         x++) {
      int y = static_cast<int>((x - p2.x()) * m + p2.y());

      if (y < 0 || y >= map_grid_->info.width)
        continue;

      if (map_grid_->data[to_index(x, y)] != 0 ||
          map_grid_->data[to_index(x, make_valid_y(y - 1))] != 0 ||
          map_grid_->data[to_index(x, make_valid_y(y + 1))] != 0) {
        return false;
      }
    }

  }
  return true;
}

bool RRTPlanner::is_point_unoccupied(const point_t &p) {
  if (!is_point_valid(p)) {
    return false;
  }
  for (int y = make_valid_y(p.y() - 1);
       y <= make_valid_y(p.y() + 1);
       y++) {
    if (map_grid_->data[to_index(p.x(), y)] != 0 ||
        map_grid_->data[to_index(make_valid_x(p.x() - 1), y)] != 0 ||
        map_grid_->data[to_index(make_valid_x(p.x() + 1), y)] != 0) {
      return false;
    }
  }
  return true;
}

bool RRTPlanner::is_point_valid(const point_t &p) {
  return (p.x() >= 0 && p.x() < map_grid_->info.height && p.y() >= 0 && p.y() < map_grid_->info.width);
}

int RRTPlanner::make_valid_x(const int n) {
  return make_valid(n, 0, map_grid_->info.height);
}

int RRTPlanner::make_valid_y(const int n) {
  return make_valid(n, 0, map_grid_->info.width);
}

void RRTPlanner::build_map() {
  if (!visualize_) {
    return;
  }
  map_ = std::unique_ptr<cv::Mat>(new cv::Mat(map_grid_->info.height,
                                              map_grid_->info.width,
                                              CV_8UC3,
                                              cv::Scalar::all(255)));

  for (int i = 0; i < map_grid_->info.height; i++) {
    for (int j = 0; j < map_grid_->info.width; j++) {
      if (map_grid_->data[to_index(i, j)]) {
        map_->at<cv::Vec3b>(map_grid_->info.height - i - 1, j) = cv::Vec3b(0, 0, 0);
      } else {
        map_->at<cv::Vec3b>(map_grid_->info.height - i - 1, j) = cv::Vec3b(255, 255, 255);
      }
    }
  }
}

void RRTPlanner::view_map(int delay) {
  if (!visualize_) {
    return;
  }
  cv::imshow("Output", *map_);
  cv::waitKey(delay);
}

void RRTPlanner::plot_circle(point_t &p, int radius, const cv::Scalar color) {
  if (!visualize_) {
    return;
  }
  cv::circle(
      *map_,
      cv::Point(p.y(), map_grid_->info.height - p.x() - 1),
      radius,
      color,
      -1);
}

void RRTPlanner::plot_line(point_t &p1, point_t &p2, const cv::Scalar color, int thickness) {
  if (!visualize_) {
    return;
  }
  cv::line(
      *map_,
      cv::Point(p2.y(), map_grid_->info.height - p2.x() - 1),
      cv::Point(p1.y(), map_grid_->info.height - p1.x() - 1),
      color,
      thickness);
}

geometry_msgs::PoseStamped RRTPlanner::point_to_pose(const point_t &p) {
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = p.y() * map_grid_->info.resolution;
  pose.pose.position.y = p.x() * map_grid_->info.resolution;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = map_grid_->header.frame_id;
  return pose;
}

void RRTPlanner::pose_to_point(point_t &p, const geometry_msgs::Pose &pose) {
  p.x(pose.position.y / map_grid_->info.resolution);
  p.y(pose.position.x / map_grid_->info.resolution);
}

int RRTPlanner::to_index(int x, int y) {
  return x * map_grid_->info.width + y;
}

} //namespace rrt_planner