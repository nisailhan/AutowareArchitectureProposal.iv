#include <planning_error_monitor/planning_error_monitor.h>
#include <ros/ros.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "planning_error_monitor");
  PlanningErrorMonitor node;
  ros::spin();
  return 0;
}
