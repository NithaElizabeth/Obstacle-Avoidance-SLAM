#ifndef NAV_EXPLORE_H_
#define NAV_EXPLORE_H_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <costmap_client.h>
#include <frontier_search.h>

#include <exp_assignment3/Explore.h>

/** 
 * @brief Explore algorithm namespace
 */

namespace explore
{
/**
 * @class Explore
 * @brief A class adhering to the robot_actions::Action interface that moves the
 * robot base to explore its environment.
 * 
 * Here I added the two service server declarations. The remaining changes I made are
 * all in explore.cpp
 */
class Explore
{
public:
  Explore();
  ~Explore();

  /**
   * @brief Method starting algorithm execution
   */
  void start();
    /**
   * @brief Method stopping algorithm execution
   */
  void stop();

  /**
   * @brief Exploration algorithm start service server declaration
   */
  bool srv_start(exp_assignment3::Explore::Request& req,
          exp_assignment3::Explore::Response& res);
  /**
   * @brief Exploration algorithm stop service server declaration
   */
  bool srv_stop(exp_assignment3::Explore::Request& req,
          exp_assignment3::Explore::Response& res);
  
  /**
   * @brief Variable used to prevent the algorithm from running at launch; it
   * will instead wait for a service client call
   */
  int first_run {1};

private:
  /**
   * @brief Make a global plan
   */
  void makePlan();

  /**
   * @brief Publish a frontiers as markers
   */
  void visualizeFrontiers(
      const std::vector<frontier_exploration::Frontier>& frontiers);

  void reachedGoal(const actionlib::SimpleClientGoalState& status,
                   const move_base_msgs::MoveBaseResultConstPtr& result,
                   const geometry_msgs::Point& frontier_goal);

  bool goalOnBlacklist(const geometry_msgs::Point& goal);

  ros::NodeHandle private_nh_;
  ros::NodeHandle relative_nh_;
  ros::Publisher marker_array_publisher_;
  tf::TransformListener tf_listener_;

  Costmap2DClient costmap_client_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
      move_base_client_;
  frontier_exploration::FrontierSearch search_;
  ros::Timer exploring_timer_;
  ros::Timer oneshot_;

  std::vector<geometry_msgs::Point> frontier_blacklist_;
  geometry_msgs::Point prev_goal_;
  double prev_distance_;
  ros::Time last_progress_;
  size_t last_markers_count_;

  // parameters
  double planner_frequency_;
  double potential_scale_, orientation_scale_, gain_scale_;
  ros::Duration progress_timeout_;
  bool visualize_;
};
}

#endif
