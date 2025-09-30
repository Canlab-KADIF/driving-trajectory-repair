#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include "cyber_perception_msgs/PerceptionObstacle.h"
#include "cyber_perception_msgs/PerceptionObstacles.h"
#include "math/math_utils.h"
#include "math/box2d.h"
#include "jsk_recognition_msgs/BoundingBox.h"
#include "jsk_recognition_msgs/BoundingBoxArray.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <tf/tf.h>

#define SPEED_THRESHOLD 0.83
#define OBSTACLE_HISTORY_SIZE 30
#define TRACKING_TIME 2.0

using namespace keti::common::math;

ros::Publisher pub_viz_marker, pub_viz_jsk_bbox;
double x_offset, y_offset;
size_t obs_msg_index = 0;

std_msgs::ColorRGBA colorCategory20(int i) {
  std_msgs::ColorRGBA c;
  c.a = 1.0;
  switch (i % 20) {
    case 0: {
      c.r = 0.121569;
      c.g = 0.466667;
      c.b = 0.705882;
    } break;
    case 1: {
      c.r = 0.682353;
      c.g = 0.780392;
      c.b = 0.909804;
    } break;
    case 2: {
      c.r = 1.000000;
      c.g = 0.498039;
      c.b = 0.054902;
    } break;
    case 3: {
      c.r = 1.000000;
      c.g = 0.733333;
      c.b = 0.470588;
    } break;
    case 4: {
      c.r = 0.172549;
      c.g = 0.627451;
      c.b = 0.172549;
    } break;
    case 5: {
      c.r = 0.596078;
      c.g = 0.874510;
      c.b = 0.541176;
    } break;
    case 6: {
      c.r = 0.839216;
      c.g = 0.152941;
      c.b = 0.156863;
    } break;
    case 7: {
      c.r = 1.000000;
      c.g = 0.596078;
      c.b = 0.588235;
    } break;
    case 8: {
      c.r = 0.580392;
      c.g = 0.403922;
      c.b = 0.741176;
    } break;
    case 9: {
      c.r = 0.772549;
      c.g = 0.690196;
      c.b = 0.835294;
    } break;
    case 10: {
      c.r = 0.549020;
      c.g = 0.337255;
      c.b = 0.294118;
    } break;
    case 11: {
      c.r = 0.768627;
      c.g = 0.611765;
      c.b = 0.580392;
    } break;
    case 12: {
      c.r = 0.890196;
      c.g = 0.466667;
      c.b = 0.760784;
    } break;
    case 13: {
      c.r = 0.968627;
      c.g = 0.713725;
      c.b = 0.823529;
    } break;
    case 14: {
      c.r = 0.498039;
      c.g = 0.498039;
      c.b = 0.498039;
    } break;
    case 15: {
      c.r = 0.780392;
      c.g = 0.780392;
      c.b = 0.780392;
    } break;
    case 16: {
      c.r = 0.737255;
      c.g = 0.741176;
      c.b = 0.133333;
    } break;
    case 17: {
      c.r = 0.858824;
      c.g = 0.858824;
      c.b = 0.552941;
    } break;
    case 18: {
      c.r = 0.090196;
      c.g = 0.745098;
      c.b = 0.811765;
    } break;
    case 19: {
      c.r = 0.619608;
      c.g = 0.854902;
      c.b = 0.898039;
    } break;
  }
  return c;
}

std::string labelToText(int i) {
  std::string label;

  switch(i) {
    case 0: {
      label = "UNKNOWN";
    } break;
    case 1: {
      label = "UNKNOWN_MOVABLE";
    } break;
    case 2: {
      label = "UNKNOWN_UNMOVABLE";
    } break;
    case 3: {
      label = "PEDESTRIAN";
    } break;
    case 4: {
      label = "BICYCLE";
    } break;    
    case 5: {
      label = "VEHICLE";
    } break;    
  }
  return label;
}

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

std::vector<cyber_perception_msgs::PerceptionObstacle> tracking_obs; // 현재 tracking 중인 obstacles
std::unordered_map<int, DisappearedObsData> disappeared_obs_table; // 이전에 사라진 obstacles
std::unordered_map<int,ObsMatchingData> id_matching_table;
std::unordered_map<int, std::vector<cyber_perception_msgs::PerceptionObstacle>> obstacle_hitory;
std::unordered_map<int, std::vector<Vec2d>> obstacle_estimation;

bool EstimateBoxPose(double time, DisappearedObsData dis_obs, Box2d* box_ptr){
  int obs_id = dis_obs.obs_msg.id;
  if(obstacle_hitory.count(obs_id) == 0){
    std::cout << "obstacle id : " << obs_id << " has no hitory!" << std::endl;
    return false;
  }

  // calculate average speed
  double avg_speed = 0.0;
  for(auto obs_msg : obstacle_hitory[obs_id]){
    avg_speed += std::hypot(obs_msg.velocity.x, obs_msg.velocity.y);
  }

  avg_speed = avg_speed / obstacle_hitory[obs_id].size();

  if(avg_speed < SPEED_THRESHOLD){
    Vec2d box_center(dis_obs.obs_msg.position.x, dis_obs.obs_msg.position.y);
    Box2d estimated_obs_box(box_center, dis_obs.obs_msg.theta, dis_obs.obs_msg.length, dis_obs.obs_msg.width);

    *box_ptr = estimated_obs_box;
    return true;
  }

  // if obstacle_hitory.size is 1, then apply linear prediction
  if(obstacle_hitory[obs_id].size() == 1 || 
     dis_obs.obs_msg.type.type == cyber_perception_msgs::ObstacleType::PEDESTRIAN){
    double dt = time - dis_obs.time;

    Vec2d box_center(dis_obs.obs_msg.position.x + dis_obs.obs_msg.velocity.x * dt, 
                     dis_obs.obs_msg.position.y + dis_obs.obs_msg.velocity.y * dt);

    Box2d estimated_obs_box(box_center, dis_obs.obs_msg.theta, dis_obs.obs_msg.length, dis_obs.obs_msg.width);

    *box_ptr = estimated_obs_box;
    return true;
  }

  // std::cout.precision(5);
  // std::cout << std::fixed << "id : " << dis_obs.obs_msg.id << ", speed : " << std::hypot(dis_obs.obs_msg.velocity.x, dis_obs.obs_msg.velocity.y)
  //           << ", avg_speed : " << avg_speed << std::endl;

  // calculate yaw rate
  double yaw_rate = AngleDiff(obstacle_hitory[obs_id].front().theta, obstacle_hitory[obs_id].back().theta)
                    / (obstacle_hitory[obs_id].back().timestamp - obstacle_hitory[obs_id].front().timestamp);

  double accel = 0.0;
  double dt = time - dis_obs.time;

  // initial version
  // double x_dot = avg_speed * cosf(obstacle_hitory[obs_id].back().theta + yaw_rate * dt);
  // double y_dot = avg_speed * sinf(obstacle_hitory[obs_id].back().theta + yaw_rate * dt);
  // double box_theta = NormalizeAngle(obstacle_hitory[obs_id].back().theta + yaw_rate * dt);
  // Vec2d box_center(obstacle_hitory[obs_id].back().position.x + x_dot * dt, obstacle_hitory[obs_id].back().position.y + y_dot * dt);

  // Box2d estimated_obs_box(box_center, box_theta, dis_obs.obs_msg.length, dis_obs.obs_msg.width);

  // calculate heading by prev pt & curr pt
  // double x_dot = avg_speed * cosf(obstacle_hitory[obs_id].back().theta + yaw_rate * dt);
  // double y_dot = avg_speed * sinf(obstacle_hitory[obs_id].back().theta + yaw_rate * dt);
  // double box_theta;
  // Vec2d box_center(obstacle_hitory[obs_id].back().position.x + x_dot * dt, obstacle_hitory[obs_id].back().position.y + y_dot * dt);

  // if(!obstacle_estimation[obs_id].empty()){
  //   box_theta = (box_center - obstacle_estimation[obs_id].back()).Angle();
  // }
  // else{
  //   box_theta = obstacle_hitory[obs_id].back().theta + yaw_rate * dt;
  // }

  // Box2d estimated_obs_box(box_center, box_theta, dis_obs.obs_msg.length, dis_obs.obs_msg.width);
  // obstacle_estimation[obs_id].push_back(box_center);

  // ctrv model
  if(std::abs(yaw_rate) < 1e-5){
    yaw_rate = yaw_rate < 0.0 ? -1e-5 : 1e-5;
  }

  double dx = avg_speed / yaw_rate * (sinf(obstacle_hitory[obs_id].back().theta + yaw_rate * dt) - sinf(obstacle_hitory[obs_id].back().theta));
  double dy = avg_speed / yaw_rate * (-cosf(obstacle_hitory[obs_id].back().theta + yaw_rate * dt) + cosf(obstacle_hitory[obs_id].back().theta));
  double box_theta = NormalizeAngle(obstacle_hitory[obs_id].back().theta + yaw_rate * dt);

  Vec2d box_center(obstacle_hitory[obs_id].back().position.x + dx, obstacle_hitory[obs_id].back().position.y + dy);

  Box2d estimated_obs_box(box_center, box_theta, dis_obs.obs_msg.length, dis_obs.obs_msg.width);

  *box_ptr = estimated_obs_box;
  return true;
}

bool ApplyMatching(std::vector<cyber_perception_msgs::PerceptionObstacle> new_obs, double time, int new_obs_index){
  std::vector<std::pair<int, DisappearedObsData>> sorted_dis_obs_table(disappeared_obs_table.begin(), disappeared_obs_table.end());
  
  std::sort(sorted_dis_obs_table.begin(), sorted_dis_obs_table.end(),[](std::pair<int, DisappearedObsData>& a, std::pair<int, DisappearedObsData>& b){
            return a.second.time > b.second.time;
           });
  
  for(const auto& obs : new_obs){
    Vec2d center(obs.position.x, obs.position.y);
    Box2d obs_box(center, obs.theta, obs.length, obs.width);
    // std::cout << "new_obs position.x : " << obs.position.x << ", y : " << obs.position.y << ", theta : " << obs.theta
    //           << ", length : " << obs.length << ", width : " << obs.width << std::endl;

    for(const auto& dis_obs : sorted_dis_obs_table){
      if(std::abs(time - dis_obs.second.time) > 2.0) continue;
      if(obs.type != dis_obs.second.obs_msg.type) continue;

      // std::cout << "time : " << dis_obs.second.time << std::endl;

      Vec2d dis_center(dis_obs.second.obs_msg.position.x, dis_obs.second.obs_msg.position.y);
      Box2d dis_obs_box(dis_center, dis_obs.second.obs_msg.theta, dis_obs.second.obs_msg.length, dis_obs.second.obs_msg.width);
    
      // std::cout << "dis_obs id : " << dis_obs.second.obs_msg.id 
      //           << ", position.x : " << dis_obs.second.obs_msg.position.x << ", position.y : " << dis_obs.second.obs_msg.position.y 
      //           << ", distance : " << center.DistanceTo(dis_center)
      //           << ", theta : " << dis_obs.second.obs_msg.theta << ", length : " << dis_obs.second.obs_msg.length 
      //           << ", width : " << dis_obs.second.obs_msg.width << std::endl;

      if(dis_obs_box.HasOverlap(obs_box)){
        // std::cout << "matching table updated" << std::endl;
        id_matching_table[obs.id] = ObsMatchingData(dis_obs.second.obs_msg.id, dis_obs.second.index - 1, new_obs_index);
        disappeared_obs_table.erase(dis_obs.second.obs_msg.id);
        break;
      }
    }
  }
  return true;
}

void CallbackObstacles(const cyber_perception_msgs::PerceptionObstacles& obstacles){
  std::vector<cyber_perception_msgs::PerceptionObstacle> new_obs;
  double time = obstacles.cyber_header.timestamp_sec;

  // jsk header
  jsk_recognition_msgs::BoundingBoxArray jsk_ros;
  jsk_ros.header.seq = obstacles.cyber_header.sequence_num;  
  jsk_ros.header.stamp = ros::Time(obstacles.cyber_header.timestamp_sec);  
  // jsk_ros.header.frame_id = obstacles.header.frame_id;
  jsk_ros.header.frame_id = "map";  

  visualization_msgs::MarkerArray marker_ros;
  visualization_msgs::Marker marker_delete_all;
  // marker_delete_all.header.frame_id = obstacles.header.frame_id;
  marker_delete_all.header.frame_id = "map";
  marker_delete_all.id = 0;
  marker_delete_all.action = visualization_msgs::Marker::DELETEALL;
  marker_ros.markers.push_back(marker_delete_all);

  for(auto it = disappeared_obs_table.begin() ; it != disappeared_obs_table.end() ;){
    if(std::abs(time - it->second.time) > TRACKING_TIME){
      it = disappeared_obs_table.erase(it);
      continue;
    }

    jsk_recognition_msgs::BoundingBox bbox;
    Box2d box2d;
    
    if(!EstimateBoxPose(time, it->second, &box2d)){
      std::cout << "fail to EstimateBoxPose!" << std::endl;
      it++;
      continue;
    }
    
    bbox.header.frame_id = "map";
    bbox.header.stamp = ros::Time(time);

    bbox.dimensions.x = box2d.length();
    bbox.dimensions.y = box2d.width();
    bbox.dimensions.z = it->second.obs_msg.height;

    double obstacle_x = box2d.center_x();
    double obstacle_y = box2d.center_y();
    double obstacle_z = it->second.obs_msg.position.z;

    bbox.pose.position.x = obstacle_x - x_offset;// + 0.6 * std::cos(perception_obstacle.theta);
    bbox.pose.position.y = obstacle_y - y_offset;// + 0.6 * std::sin(perception_obstacle.theta);
    bbox.pose.position.z = obstacle_z;
    // std::cout.precision(7);
    // std::cout << std::fixed << "x_offset : " << x_offset << ", y_offset : " << y_offset << std::endl;

    bbox.label = 1;

    tf::Quaternion quat =
        tf::createQuaternionFromRPY(0.0, 0.0, box2d.heading());

    tf::quaternionTFToMsg(quat, bbox.pose.orientation);

    jsk_ros.boxes.push_back(bbox);

    // For class label
    visualization_msgs::Marker label;

    label.header.seq = obstacles.cyber_header.sequence_num;
    label.header.stamp = ros::Time(obstacles.cyber_header.timestamp_sec);
    label.header.frame_id = "map";

    label.pose.position.x = obstacle_x - x_offset;
    label.pose.position.y = obstacle_y - y_offset;
    label.pose.position.z = obstacle_z + 3;

    label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    label.id = 3 * it->second.obs_msg.id + 1;
    label.action = visualization_msgs::Marker::ADD;
    label.text = labelToText(static_cast<int>(it->second.obs_msg.type.type));
    label.color = colorCategory20(static_cast<int>(it->second.obs_msg.type.type));
    label.scale.z = 1;

    marker_ros.markers.push_back(label);

    // For track id
    visualization_msgs::Marker track_id;

    track_id.header.seq = obstacles.cyber_header.sequence_num;
    track_id.header.stamp = ros::Time(obstacles.cyber_header.timestamp_sec);
    track_id.header.frame_id = "map";

    track_id.pose.position.x = obstacle_x - x_offset;
    track_id.pose.position.y = obstacle_y - y_offset;
    track_id.pose.position.z = obstacle_z + 1;

    track_id.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    track_id.id = 3 * it->second.obs_msg.id + 2;
    track_id.action = visualization_msgs::Marker::ADD;
    track_id.text = std::to_string(it->second.obs_msg.id);
    track_id.color = colorCategory20(static_cast<int>(it->second.obs_msg.type.type));
    track_id.scale.z = 1;

    marker_ros.markers.push_back(track_id);
    it++;
  }

  for(auto it = tracking_obs.begin() ; it != tracking_obs.end() ;){
    auto found_obs_iter = std::find_if(obstacles.perception_obstacle.begin(), obstacles.perception_obstacle.end(), 
                                        [&](const cyber_perception_msgs::PerceptionObstacle& obs_now) { return obs_now.id == it->id; });
    
    // 사라진 obstacle tracking_obs에서 삭제 & disappeared_obs에 추가
    if(found_obs_iter == obstacles.perception_obstacle.end()){
      if(disappeared_obs_table.count(it->id) != 0){
        std::cout << "obstacle already exists! id : " << it->id << ", prev index : " << disappeared_obs_table[it->id].index 
                  << ", prev time : " << disappeared_obs_table[it->id].time << ", curr index : " << obs_msg_index << ", curr time : " << time << std::endl;
      }
      disappeared_obs_table[it->id] = DisappearedObsData(*it, time, obs_msg_index);
      it = tracking_obs.erase(it);
      continue;
    }

    // update tracking obstacles state
    *it = *found_obs_iter;

    // update obstacle hitory
    obstacle_hitory[it->id].push_back(*it);
    if(obstacle_hitory[it->id].size() > OBSTACLE_HISTORY_SIZE){
      obstacle_hitory[it->id].erase(obstacle_hitory[it->id].begin());
    }

    it++;
  }

  // find new obstacles
  for(const auto& obs : obstacles.perception_obstacle){
    int found_obs = std::count_if(tracking_obs.begin(), tracking_obs.end(), 
                                  [&](const cyber_perception_msgs::PerceptionObstacle& track_obs) { return track_obs.id == obs.id; });

    if(found_obs == 0){
      // std::cout << "found new obs : " << obs.id << std::endl;
      new_obs.push_back(obs);
      // update obstacle hitory
      obstacle_hitory[obs.id].push_back(obs);
    }
  }

  // obstacle matching
  // ApplyMatching(new_obs, time, obs_msg_index);

  // add new obstacles to tracking obstacles
  if(!new_obs.empty()){
    tracking_obs.insert(tracking_obs.begin(), new_obs.begin(), new_obs.end());
  }
  pub_viz_jsk_bbox.publish(jsk_ros);
  pub_viz_marker.publish(marker_ros);
  obs_msg_index++;
  // std::cout << "publish topic" << std::endl;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "estimation_visualizer");
  ros::NodeHandle nh;
  nh.getParam("x_offset", x_offset);
  nh.getParam("y_offset", y_offset);

  ros::Subscriber sub_obstacles = nh.subscribe("/obstacles", 10, CallbackObstacles);

  pub_viz_jsk_bbox = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/obstacles_estimation_vis", 1);
  pub_viz_marker = nh.advertise<visualization_msgs::MarkerArray>("/obstacles_estimation_vis_vel",1);

  ros::spin();

  return 0;
}