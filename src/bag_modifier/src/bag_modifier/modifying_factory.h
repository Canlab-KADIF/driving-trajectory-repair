#include <string>
#include <vector>
#include <unordered_map>
#include <map>
#include <algorithm>
#include "cyber_perception_msgs/PerceptionObstacle.h"
#include "cyber_perception_msgs/PerceptionObstacles.h"
#include "spline_solver.h"
#include "math/box2d.h"

#define SPEED_THRESHOLD 0.83 // 3km/h
#define MAX_SPEED_ERROR 13.9 // 50km/h
#define MAX_HEADING_ERROR M_PI_2
#define OBSTACLE_HISTORY_SIZE 10
#define TRACKING_TIME 2.0

using namespace keti::common::math;

struct DisappearedObsData{
  DisappearedObsData() = default;
  DisappearedObsData(cyber_perception_msgs::PerceptionObstacle obs_msg, double time, int index)
    : obs_msg(obs_msg), time(time), index(index) {}
  cyber_perception_msgs::PerceptionObstacle obs_msg;
  double time = 0.0;
  int index = -1;
};

struct ObsMatchingData{
  ObsMatchingData() = default;
  ObsMatchingData(int matched_id, int start_index, int end_index)
    : matched_id(matched_id), start_index(start_index), end_index(end_index) {}
  int matched_id = -1;
  int start_index = -1;
  int end_index = -1;
};

class ModifyingFactory{
public:
  ModifyingFactory() = default;
  bool Update(cyber_perception_msgs::PerceptionObstacles obstacles, int obs_msg_index);
  void AddObstaclesMsg(ros::Time ros_time, cyber_perception_msgs::PerceptionObstacles obstacles);
  bool InterpolateTrajectory();
  void ApplyIdMatching();
  void PrintTable();
  std::vector<std::pair<ros::Time,cyber_perception_msgs::PerceptionObstacles>> GetObsMsgs() { return obstacles_topics_with_time_; }
  std::vector<std::pair<ros::Time,cyber_perception_msgs::PerceptionObstacles>> GetModifiedOnlyObsMsgs() { return modified_only_obstacles_topics_with_time_; }

private:
  bool ApplyMatching(std::vector<cyber_perception_msgs::PerceptionObstacle> new_obs, double time, int new_obs_index);
  bool EstimateBoxPose(double time, DisappearedObsData dis_obs, Box2d* box_ptr, double* estimated_speed);
  void MatchID(cyber_perception_msgs::PerceptionObstacles *obstacles);

private:
  std::vector<cyber_perception_msgs::PerceptionObstacle> tracking_obs_; // 현재 tracking 중인 obstacles
  std::unordered_map<int, DisappearedObsData> disappeared_obs_table_; // 이전에 사라진 obstacles
  // std::unordered_map<int,ObsMatchingData> id_matching_table;
  std::map<int,ObsMatchingData> id_matching_table_;  // id_matching_table_[new_obs_id] = matched dis_obs data
  std::unordered_map<int, std::vector<cyber_perception_msgs::PerceptionObstacle>> obstacle_hitory_;

  std::vector<std::pair<ros::Time,cyber_perception_msgs::PerceptionObstacles>> obstacles_topics_with_time_;
  std::vector<std::pair<ros::Time,cyber_perception_msgs::PerceptionObstacles>> modified_only_obstacles_topics_with_time_;

  SplineSolver spline_solver_;
};

