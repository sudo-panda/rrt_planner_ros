//
// Created by sudo-panda on 4/4/20.
//

#include "rrt_planner/rrt_planner.h"

int main(int argv, char** argc) {
  ros::init(argv, argc, "rrt_planner");

  ros::NodeHandle node;
  new rrt_planner::RRTPlanner(&node);
}