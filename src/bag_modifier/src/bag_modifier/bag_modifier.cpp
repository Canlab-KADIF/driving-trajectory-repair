#include "bag_modifier.h"
#include "control_msgs/VehicleState.h"
#include "control_msgs/CanFrame.h"
#include "novatel_oem7_msgs/INSPVA.h"
#include "novatel_oem7_msgs/INSSTDEV.h"
// #include <geometry_msgs/PoseStamped.h>
// #include <tf2_msgs/TFMessage.h>
// #include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Imu.h>

BagModifier::BagModifier(std::string raw_file_name, std::string modified_bag_file_name, std::string modified_only_bag_file_name){
  std::cout << raw_file_name << ", " << modified_bag_file_name << ", " << modified_only_bag_file_name << std::endl;

  try
  {
    std::cerr << ">> Read " << raw_file_name << std::endl;
    raw_bag_.open(raw_file_name, rosbag::bagmode::Read);
  }
  catch (const rosbag::BagException&)
  {
    std::cerr << ">> Error opening file " << raw_file_name << std::endl;
  }

  modified_bag_.open(modified_bag_file_name, rosbag::bagmode::Write);
  modified_only_bag_.open(modified_only_bag_file_name, rosbag::bagmode::Write);
}

bool BagModifier::Modify(){
  if(!ScanRawBag()){
    std::cout << "ScanRawBag() failed!" << std::endl;
    return false;
  }

  if(!modifying_factory_.InterpolateTrajectory()){
    std::cout << "modifying_factory_.InterpolateTrajectory() failed!" << std::endl;
    return false;
  }

  modifying_factory_.ApplyIdMatching();

  auto obs_msg = modifying_factory_.GetObsMsgs();
  auto modified_only_msg = modifying_factory_.GetModifiedOnlyObsMsgs();

  for(int i = 0 ; i < obs_msg.size() ; ++i){
    modified_bag_.write("/obstacles", obs_msg[i].first, obs_msg[i].second);
    modified_only_bag_.write("/obstacles_modified", modified_only_msg[i].first, modified_only_msg[i].second);
  }

  modifying_factory_.PrintTable();

  return true;
}

bool BagModifier::ScanRawBag(){
  rosbag::View view(raw_bag_);
  std::vector<std::string> datatype, topic_names;
  for(rosbag::ConnectionInfo const *ci : view.getConnections() )
  {
    datatype.emplace_back(ci->datatype);
    topic_names.emplace_back(ci->topic);
  }
  
  for(const auto name : topic_names){
    std::cout << name << std::endl;
  }

  rosbag::View::iterator view_it = view.begin();
  double prev_timestamp = 0.0;
  int obs_msg_index = 0;

  while (view_it != view.end() && ros::ok()) {
    if((*view_it).getTopic() == "/obstacles"){
      cyber_perception_msgs::PerceptionObstacles::Ptr obstacles_ptr = (*view_it).instantiate<cyber_perception_msgs::PerceptionObstacles> ();
      cyber_perception_msgs::PerceptionObstacles obstacles = *obstacles_ptr;
      cyber_perception_msgs::PerceptionObstacles empty_obstacles;
      double time = obstacles.cyber_header.timestamp_sec;

      if(time - prev_timestamp > 1e-5){
        prev_timestamp = time;
      }
      else{
        view_it++;
        continue;
      }

      // update tracking_obs & disappeared_obs & matching_table
      if(!modifying_factory_.Update(obstacles, obs_msg_index)){
        std::cout << "modifying_factory_.Update failed!" << std::endl;
        return false;
      }

      // add obstacles topic & time
      modifying_factory_.AddObstaclesMsg(view_it->getTime(), obstacles);

      obs_msg_index++;
    }
    // else if((*view_it).getTopic() == "/received_messages"){
    //   control_msgs::CanFrame::Ptr can_ptr = (*view_it).instantiate<control_msgs::CanFrame> ();
    //   modified_bag_.write("/received_messages", can_ptr->header.stamp, *can_ptr);
    // }
    // else if((*view_it).getTopic() == "/novatel/insstdev"){
    //   novatel_oem7_msgs::INSSTDEV::Ptr insstdev_ptr = (*view_it).instantiate<novatel_oem7_msgs::INSSTDEV> ();
    //   modified_bag_.write("/novatel/insstdev", insstdev_ptr->header.stamp, *insstdev_ptr);
    // }
    // else if((*view_it).getTopic() == "/imu"){
    //   sensor_msgs::Imu::Ptr imu_ptr = (*view_it).instantiate<sensor_msgs::Imu> ();
    //   modified_bag_.write("/imu", imu_ptr->header.stamp, *imu_ptr);
    // }
    // else if((*view_it).getTopic() == "/inspva"){
    //   novatel_oem7_msgs::INSPVA::Ptr inspva_ptr = (*view_it).instantiate<novatel_oem7_msgs::INSPVA> ();
    //   modified_bag_.write("/inspva", inspva_ptr->header.stamp, *inspva_ptr);
    // }
    // // else if((*view_it).getTopic().find("/tf") != std::string::npos){
    // //   // std::cout << "tf" << std::endl;
    // //   tf2_msgs::TFMessage::Ptr tf_ptr = (*view_it).instantiate<tf2_msgs::TFMessage> ();
    // //   modified_bag_.write("/tf", view_it->getTime(), *tf_ptr);
    // // }
    
    else{
      modified_bag_.write(view_it->getTopic(), view_it->getTime(), *view_it);
    }

    ++view_it;
  }

  return true;
}