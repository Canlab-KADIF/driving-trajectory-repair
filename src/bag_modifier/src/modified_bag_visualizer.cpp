#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/Bool.h>

#include <cmath>

#include "control_msgs/VehicleState.h"
#include "cyber_perception_msgs/PerceptionObstacles.h"
#include "cyber_prediction_msgs/PredictionObstacles.h"
#include "jsk_recognition_msgs/BoundingBox.h"
#include "jsk_recognition_msgs/BoundingBoxArray.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "math/math_utils.h"

using namespace keti::common;

ros::Publisher pub_viz_marker, pub_viz_jsk_bbox, pub_obstacles, pub_viz_converter_vel, pub_viz_converter_accel, pub_viz_jsk_bbox_temp;

control_msgs::VehicleState vehicle_state;
double x_offset, y_offset;
bool is_changed_params = true;

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


void CallbackObstacles(const cyber_perception_msgs::PerceptionObstaclesConstPtr& perc_apollo) {  
  
  // jsk header
  jsk_recognition_msgs::BoundingBoxArray jsk_ros;
  jsk_ros.header.seq = perc_apollo->cyber_header.sequence_num;  
  jsk_ros.header.stamp = ros::Time(perc_apollo->cyber_header.timestamp_sec);  
  // jsk_ros.header.frame_id = perc_apollo->header.frame_id;
  jsk_ros.header.frame_id = "map";  

  visualization_msgs::MarkerArray marker_ros;
  visualization_msgs::Marker marker_delete_all;
  // marker_delete_all.header.frame_id = perc_apollo->header.frame_id;
  marker_delete_all.header.frame_id = "map";
  marker_delete_all.id = 0;
  marker_delete_all.action = visualization_msgs::Marker::DELETEALL;
  marker_ros.markers.push_back(marker_delete_all);

  visualization_msgs::MarkerArray arrows_vel;
  arrows_vel.markers.push_back(marker_delete_all);

  visualization_msgs::MarkerArray arrows_accel;
  arrows_accel.markers.push_back(marker_delete_all);

  for (const auto &perception_obstacle : perc_apollo->perception_obstacle) {

    // change frame to lidar sensor
    double obstacle_x = perception_obstacle.position.x;
    double obstacle_y = perception_obstacle.position.y;
    double obstacle_z = perception_obstacle.height / 2;

    // jsk bbox
    jsk_recognition_msgs::BoundingBox bbox;

    // bbox.header.frame_id = perc_apollo->header.frame_id;
    bbox.header.frame_id = "map";
    bbox.header.stamp = ros::Time(perception_obstacle.timestamp);

    bbox.dimensions.x = perception_obstacle.length;
    bbox.dimensions.y = perception_obstacle.width;
    bbox.dimensions.z = perception_obstacle.height;

    bbox.pose.position.x = obstacle_x - x_offset;// + 0.6 * std::cos(perception_obstacle.theta);
    bbox.pose.position.y = obstacle_y - y_offset;// + 0.6 * std::sin(perception_obstacle.theta);
    bbox.pose.position.z = obstacle_z;
    // std::cout.precision(7);
    // std::cout << std::fixed << "x_offset : " << x_offset << ", y_offset : " << y_offset << std::endl;

    bbox.label = 2;

    tf::Quaternion quat =
        tf::createQuaternionFromRPY(0.0, 0.0, perception_obstacle.theta);

    tf::quaternionTFToMsg(quat, bbox.pose.orientation);

    jsk_ros.boxes.push_back(bbox);
    
    // For class label
    visualization_msgs::Marker label;

    label.header.seq = perc_apollo->cyber_header.sequence_num;
    label.header.stamp = ros::Time(perc_apollo->cyber_header.timestamp_sec);
    label.header.frame_id = "map";

    label.pose.position.x = obstacle_x - x_offset;
    label.pose.position.y = obstacle_y - y_offset;
    label.pose.position.z = obstacle_z + 3;

    label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    label.id = 3 * perception_obstacle.id + 1;
    label.action = visualization_msgs::Marker::ADD;
    label.text = labelToText(static_cast<int>(perception_obstacle.type.type));
    label.color = colorCategory20(static_cast<int>(perception_obstacle.type.type));
    label.scale.z = 1;

    marker_ros.markers.push_back(label);

    // For track id
    visualization_msgs::Marker track_id;

    track_id.header.seq = perc_apollo->cyber_header.sequence_num;
    track_id.header.stamp = ros::Time(perc_apollo->cyber_header.timestamp_sec);
    track_id.header.frame_id = "map";

    track_id.pose.position.x = obstacle_x - x_offset;
    track_id.pose.position.y = obstacle_y - y_offset;
    track_id.pose.position.z = obstacle_z + 1;

    track_id.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    track_id.id = 3 * perception_obstacle.id + 2;
    track_id.action = visualization_msgs::Marker::ADD;
    track_id.text = std::to_string(perception_obstacle.id);
    track_id.color = colorCategory20(static_cast<int>(perception_obstacle.type.type));
    track_id.scale.z = 1;

    marker_ros.markers.push_back(track_id);

    visualization_msgs::Marker arrow;

    arrow.header.seq = perc_apollo->cyber_header.sequence_num;
    arrow.header.stamp = ros::Time(perc_apollo->cyber_header.timestamp_sec);
    arrow.header.frame_id = "map";

    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.id = 3 * perception_obstacle.id + 3;
    arrow.action = visualization_msgs::Marker::ADD;

    arrow.pose.position.x = obstacle_x - x_offset;
    arrow.pose.position.y = obstacle_y - y_offset;
    arrow.pose.position.z = 0;

    tf::quaternionTFToMsg(quat, arrow.pose.orientation);
    arrow.color = colorCategory20(static_cast<int>(perception_obstacle.type.type));

    arrow.scale.x = 5.0;
    arrow.scale.y = 0.2;
    arrow.scale.z = 0.2;

    marker_ros.markers.push_back(arrow);

    double vel_arrow_size = sqrt(perception_obstacle.velocity.x * perception_obstacle.velocity.x + \
                              perception_obstacle.velocity.y * perception_obstacle.velocity.y + \
                              perception_obstacle.velocity.z * perception_obstacle.velocity.z);

    if (vel_arrow_size != 0) {
      visualization_msgs::Marker arrow_vel;
      arrow_vel.header.seq = perc_apollo->cyber_header.sequence_num;
      arrow_vel.header.stamp = ros::Time(perc_apollo->cyber_header.timestamp_sec);
      arrow_vel.header.frame_id = "map";

      arrow_vel.type = visualization_msgs::Marker::ARROW;
      arrow_vel.id = perception_obstacle.id + 1;
      arrow_vel.action = visualization_msgs::Marker::ADD;

      arrow_vel.pose.position.x = obstacle_x - x_offset;
      arrow_vel.pose.position.y = obstacle_y - y_offset;
      arrow_vel.pose.position.z = 0;

      math::Vec2d vel(perception_obstacle.velocity.x, perception_obstacle.velocity.y);
      tf::Quaternion quat_vel = tf::createQuaternionFromRPY(0.0, 0.0, vel.Angle());
      tf::quaternionTFToMsg(quat_vel, arrow_vel.pose.orientation);

      arrow_vel.color.r = 1.0;
      arrow_vel.color.g = 1.0;
      arrow_vel.color.b = 0.0;
      arrow_vel.color.a = 1.0;

      arrow_vel.scale.x = vel_arrow_size;
      arrow_vel.scale.y = 0.2;
      arrow_vel.scale.z = 0.2;

      arrows_vel.markers.push_back(arrow_vel);
    }

    double accel_arrow_size = sqrt(perception_obstacle.acceleration.x * perception_obstacle.acceleration.x + \
                              perception_obstacle.acceleration.y * perception_obstacle.acceleration.y + \
                              perception_obstacle.acceleration.z * perception_obstacle.acceleration.z);

    if (accel_arrow_size != 0) {
      visualization_msgs::Marker arrow_accel;
      arrow_accel.header.seq = perc_apollo->cyber_header.sequence_num;
      arrow_accel.header.stamp = ros::Time(perc_apollo->cyber_header.timestamp_sec);
      arrow_accel.header.frame_id = "map";

      arrow_accel.type = visualization_msgs::Marker::ARROW;
      arrow_accel.id = perception_obstacle.id + 1;
      arrow_accel.action = visualization_msgs::Marker::ADD;

      arrow_accel.pose.position.x = obstacle_x - x_offset;
      arrow_accel.pose.position.y = obstacle_y - y_offset;
      arrow_accel.pose.position.z = 0;

      math::Vec2d accel(perception_obstacle.acceleration.x, perception_obstacle.acceleration.y);
      tf::Quaternion quat_accel = tf::createQuaternionFromRPY(0.0, 0.0, accel.Angle());
      tf::quaternionTFToMsg(quat_accel, arrow_accel.pose.orientation);

      arrow_accel.color.r = 1.0;
      arrow_accel.color.g = 0.0;
      arrow_accel.color.b = 0.0;
      arrow_accel.color.a = 1.0;

      arrow_accel.scale.x = accel_arrow_size * 3.0;
      arrow_accel.scale.y = 0.2;
      arrow_accel.scale.z = 0.2;

      arrows_accel.markers.push_back(arrow_accel);
    }
  }

  pub_viz_jsk_bbox.publish(jsk_ros);
  pub_viz_marker.publish(marker_ros);
}

// void CallbackObstaclesTemp(const cyber_perception_msgs::PerceptionObstaclesConstPtr& perc_apollo) {  
  
//   // jsk header
//   jsk_recognition_msgs::BoundingBoxArray jsk_ros;
//   jsk_ros.header.seq = perc_apollo->cyber_header.sequence_num;  
//   jsk_ros.header.stamp = ros::Time(perc_apollo->cyber_header.timestamp_sec);  
//   // jsk_ros.header.frame_id = perc_apollo->header.frame_id;
//   jsk_ros.header.frame_id = "map";  

//   visualization_msgs::MarkerArray marker_ros;
//   visualization_msgs::Marker marker_delete_all;
//   // marker_delete_all.header.frame_id = perc_apollo->header.frame_id;
//   marker_delete_all.header.frame_id = "map";
//   marker_delete_all.id = 0;
//   marker_delete_all.action = visualization_msgs::Marker::DELETEALL;
//   marker_ros.markers.push_back(marker_delete_all);

//   visualization_msgs::MarkerArray arrows_vel;
//   arrows_vel.markers.push_back(marker_delete_all);

//   visualization_msgs::MarkerArray arrows_accel;
//   arrows_accel.markers.push_back(marker_delete_all);

//   for (const auto &perception_obstacle : perc_apollo->perception_obstacle) {

//     // change frame to lidar sensor
//     double obstacle_x = perception_obstacle.position.x;
//     double obstacle_y = perception_obstacle.position.y;
//     double obstacle_z = perception_obstacle.height / 2;

//     // jsk bbox
//     jsk_recognition_msgs::BoundingBox bbox;

//     // bbox.header.frame_id = perc_apollo->header.frame_id;
//     bbox.header.frame_id = "map";
//     bbox.header.stamp = ros::Time(perception_obstacle.timestamp);

//     bbox.dimensions.x = perception_obstacle.length;
//     bbox.dimensions.y = perception_obstacle.width;
//     bbox.dimensions.z = perception_obstacle.height;

//     bbox.pose.position.x = obstacle_x - x_offset;// + 0.6 * std::cos(perception_obstacle.theta);
//     bbox.pose.position.y = obstacle_y - y_offset;// + 0.6 * std::sin(perception_obstacle.theta);
//     bbox.pose.position.z = obstacle_z;
//     // std::cout.precision(7);
//     // std::cout << std::fixed << "x_offset : " << x_offset << ", y_offset : " << y_offset << std::endl;

//     bbox.label = 1;

//     tf::Quaternion quat =
//         tf::createQuaternionFromRPY(0.0, 0.0, perception_obstacle.theta);

//     tf::quaternionTFToMsg(quat, bbox.pose.orientation);

//     jsk_ros.boxes.push_back(bbox);
    
//     // For class label
//     visualization_msgs::Marker label;

//     label.header.seq = perc_apollo->cyber_header.sequence_num;
//     label.header.stamp = ros::Time(perc_apollo->cyber_header.timestamp_sec);
//     label.header.frame_id = "map";

//     label.pose.position.x = obstacle_x - x_offset;
//     label.pose.position.y = obstacle_y - y_offset;
//     label.pose.position.z = obstacle_z + 3;

//     label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
//     label.id = 3 * perception_obstacle.id + 1;
//     label.action = visualization_msgs::Marker::ADD;
//     label.text = labelToText(static_cast<int>(perception_obstacle.type.type));
//     label.color = colorCategory20(static_cast<int>(perception_obstacle.type.type));
//     label.scale.z = 1;

//     marker_ros.markers.push_back(label);

//     // For track id
//     visualization_msgs::Marker track_id;

//     track_id.header.seq = perc_apollo->cyber_header.sequence_num;
//     track_id.header.stamp = ros::Time(perc_apollo->cyber_header.timestamp_sec);
//     track_id.header.frame_id = "map";

//     track_id.pose.position.x = obstacle_x - x_offset;
//     track_id.pose.position.y = obstacle_y - y_offset;
//     track_id.pose.position.z = obstacle_z + 1;

//     track_id.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
//     track_id.id = 3 * perception_obstacle.id + 2;
//     track_id.action = visualization_msgs::Marker::ADD;
//     track_id.text = std::to_string(perception_obstacle.id);
//     track_id.color = colorCategory20(static_cast<int>(perception_obstacle.type.type));
//     track_id.scale.z = 1;

//     marker_ros.markers.push_back(track_id);

//     visualization_msgs::Marker arrow;

//     arrow.header.seq = perc_apollo->cyber_header.sequence_num;
//     arrow.header.stamp = ros::Time(perc_apollo->cyber_header.timestamp_sec);
//     arrow.header.frame_id = "map";

//     arrow.type = visualization_msgs::Marker::ARROW;
//     arrow.id = 3 * perception_obstacle.id + 3;
//     arrow.action = visualization_msgs::Marker::ADD;

//     arrow.pose.position.x = obstacle_x - x_offset;
//     arrow.pose.position.y = obstacle_y - y_offset;
//     arrow.pose.position.z = 0;

//     tf::quaternionTFToMsg(quat, arrow.pose.orientation);
//     arrow.color = colorCategory20(static_cast<int>(perception_obstacle.type.type));

//     arrow.scale.x = 5.0;
//     arrow.scale.y = 0.2;
//     arrow.scale.z = 0.2;

//     marker_ros.markers.push_back(arrow);

//     double vel_arrow_size = sqrt(perception_obstacle.velocity.x * perception_obstacle.velocity.x + \
//                               perception_obstacle.velocity.y * perception_obstacle.velocity.y + \
//                               perception_obstacle.velocity.z * perception_obstacle.velocity.z);

//     if (vel_arrow_size != 0) {
//       visualization_msgs::Marker arrow_vel;
//       arrow_vel.header.seq = perc_apollo->cyber_header.sequence_num;
//       arrow_vel.header.stamp = ros::Time(perc_apollo->cyber_header.timestamp_sec);
//       arrow_vel.header.frame_id = "map";

//       arrow_vel.type = visualization_msgs::Marker::ARROW;
//       arrow_vel.id = perception_obstacle.id + 1;
//       arrow_vel.action = visualization_msgs::Marker::ADD;

//       arrow_vel.pose.position.x = obstacle_x - x_offset;
//       arrow_vel.pose.position.y = obstacle_y - y_offset;
//       arrow_vel.pose.position.z = 0;

//       math::Vec2d vel(perception_obstacle.velocity.x, perception_obstacle.velocity.y);
//       tf::Quaternion quat_vel = tf::createQuaternionFromRPY(0.0, 0.0, vel.Angle());
//       tf::quaternionTFToMsg(quat_vel, arrow_vel.pose.orientation);

//       arrow_vel.color.r = 1.0;
//       arrow_vel.color.g = 1.0;
//       arrow_vel.color.b = 0.0;
//       arrow_vel.color.a = 1.0;

//       arrow_vel.scale.x = vel_arrow_size;
//       arrow_vel.scale.y = 0.2;
//       arrow_vel.scale.z = 0.2;

//       arrows_vel.markers.push_back(arrow_vel);
//     }

//     double accel_arrow_size = sqrt(perception_obstacle.acceleration.x * perception_obstacle.acceleration.x + \
//                               perception_obstacle.acceleration.y * perception_obstacle.acceleration.y + \
//                               perception_obstacle.acceleration.z * perception_obstacle.acceleration.z);

//     if (accel_arrow_size != 0) {
//       visualization_msgs::Marker arrow_accel;
//       arrow_accel.header.seq = perc_apollo->cyber_header.sequence_num;
//       arrow_accel.header.stamp = ros::Time(perc_apollo->cyber_header.timestamp_sec);
//       arrow_accel.header.frame_id = "map";

//       arrow_accel.type = visualization_msgs::Marker::ARROW;
//       arrow_accel.id = perception_obstacle.id + 1;
//       arrow_accel.action = visualization_msgs::Marker::ADD;

//       arrow_accel.pose.position.x = obstacle_x - x_offset;
//       arrow_accel.pose.position.y = obstacle_y - y_offset;
//       arrow_accel.pose.position.z = 0;

//       math::Vec2d accel(perception_obstacle.acceleration.x, perception_obstacle.acceleration.y);
//       tf::Quaternion quat_accel = tf::createQuaternionFromRPY(0.0, 0.0, accel.Angle());
//       tf::quaternionTFToMsg(quat_accel, arrow_accel.pose.orientation);

//       arrow_accel.color.r = 1.0;
//       arrow_accel.color.g = 0.0;
//       arrow_accel.color.b = 0.0;
//       arrow_accel.color.a = 1.0;

//       arrow_accel.scale.x = accel_arrow_size * 3.0;
//       arrow_accel.scale.y = 0.2;
//       arrow_accel.scale.z = 0.2;

//       arrows_accel.markers.push_back(arrow_accel);
//     }
//   }

//   pub_viz_jsk_bbox_temp.publish(jsk_ros);
// }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "modified_bag_visualizer");
  ros::NodeHandle nh;
  nh.getParam("x_offset", x_offset);
  nh.getParam("y_offset", y_offset);

  ros::Subscriber sub_obstacles = nh.subscribe("/obstacles_modified", 10, CallbackObstacles);
  // ros::Subscriber sub_obstacles_temp = nh.subscribe("/obstacles_modified_temp", 10, CallbackObstaclesTemp);

  pub_viz_marker = nh.advertise<visualization_msgs::MarkerArray>("/obstacles_vis_vel_modified",1);
  pub_viz_converter_vel = nh.advertise<visualization_msgs::MarkerArray>("/converter_vis_vel",1);
  pub_viz_converter_accel = nh.advertise<visualization_msgs::MarkerArray>("/converter_vis_accel",1);
  pub_viz_jsk_bbox = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/obstacles_vis_modified", 1);
  // pub_viz_jsk_bbox_temp = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/obstacles_vis_modified_temp", 1);
  // pub_obstacles = nh.advertise<cyber_perception_msgs::PerceptionObstacles>("/obstacles_local", 1);
  // pub_obstacles = nh.advertise<cyber_prediction_msgs::PredictionObstacles>("/obstacles", 1);
  
  ros::spin();

  return 0;
}