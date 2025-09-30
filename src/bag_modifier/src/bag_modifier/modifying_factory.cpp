#include "modifying_factory.h"
#include <Eigen/Dense>

using Kernel = CGAL::Exact_predicates_exact_constructions_kernel;
using Polygon = CGAL::Polygon_2<Kernel>;
using Point = CGAL::Point_2<Kernel>;
using PolygonWithHoles = CGAL::Polygon_with_holes_2<Kernel>;

void ModifyingFactory::AddObstaclesMsg(ros::Time ros_time, cyber_perception_msgs::PerceptionObstacles obstacles){
  cyber_perception_msgs::PerceptionObstacles empty_obstacles;
  obstacles_topics_with_time_.emplace_back(ros_time, obstacles);
  modified_only_obstacles_topics_with_time_.emplace_back(ros_time, empty_obstacles);
}

void ModifyingFactory::ApplyIdMatching(){
  for(int i = 0 ; i < obstacles_topics_with_time_.size() ; i++){
    MatchID(&(obstacles_topics_with_time_[i].second));
    MatchID(&(modified_only_obstacles_topics_with_time_[i].second));
  }
}

void ModifyingFactory::MatchID(cyber_perception_msgs::PerceptionObstacles *obstacles){
  for(auto it = obstacles->perception_obstacle.begin() ; it != obstacles->perception_obstacle.end() ; it++){
    while(id_matching_table_.count(it->id) != 0){
      it->id = id_matching_table_[it->id].matched_id;
    }
  }
}

bool ModifyingFactory::EstimateBoxPose(double time, DisappearedObsData dis_obs, Box2d* box_ptr, double* estimated_speed){
  int obs_id = dis_obs.obs_msg.id;
  if(obstacle_hitory_.count(obs_id) == 0){
    std::cout << "obstacle id : " << obs_id << " has no hitory!" << std::endl;
    return false;
  }

  // calculate average speed
  double avg_speed = 0.0;
  for(auto obs_msg : obstacle_hitory_[obs_id]){
    avg_speed += std::hypot(obs_msg.velocity.x, obs_msg.velocity.y);
  }

  avg_speed = avg_speed / obstacle_hitory_[obs_id].size();

  if(avg_speed < SPEED_THRESHOLD){
    Vec2d box_center(dis_obs.obs_msg.position.x, dis_obs.obs_msg.position.y);
    Box2d estimated_obs_box(box_center, dis_obs.obs_msg.theta, dis_obs.obs_msg.length, dis_obs.obs_msg.width);

    *box_ptr = estimated_obs_box;
    *estimated_speed = avg_speed;
    return true;
  }

  // calculate yaw rate
  double yaw_rate = AngleDiff(obstacle_hitory_[obs_id].front().theta, obstacle_hitory_[obs_id].back().theta)
                    / (obstacle_hitory_[obs_id].back().timestamp - obstacle_hitory_[obs_id].front().timestamp);

  // if obstacle_hitory_.size is 1 or obstacle type is pedestrian or yaw_rate is close to 0, then apply linear prediction
  if(obstacle_hitory_[obs_id].size() == 1 || 
     dis_obs.obs_msg.type.type == cyber_perception_msgs::ObstacleType::PEDESTRIAN ||
     std::abs(yaw_rate) < 1e-5){
    double dt = time - dis_obs.time;

    Vec2d box_center(dis_obs.obs_msg.position.x + dis_obs.obs_msg.velocity.x * dt, 
                     dis_obs.obs_msg.position.y + dis_obs.obs_msg.velocity.y * dt);

    Box2d estimated_obs_box(box_center, dis_obs.obs_msg.theta, dis_obs.obs_msg.length, dis_obs.obs_msg.width);

    *box_ptr = estimated_obs_box;
    *estimated_speed = avg_speed;
    return true;
  }

  // std::cout.precision(5);
  // std::cout << std::fixed << "id : " << dis_obs.obs_msg.id << ", speed : " << std::hypot(dis_obs.obs_msg.velocity.x, dis_obs.obs_msg.velocity.y)
  //           << ", avg_speed : " << avg_speed << std::endl;

  double accel = 0.0;
  double dt = time - dis_obs.time;

  // ctrv model
  double dx = avg_speed / yaw_rate * (sinf(obstacle_hitory_[obs_id].back().theta + yaw_rate * dt) - sinf(obstacle_hitory_[obs_id].back().theta));
  double dy = avg_speed / yaw_rate * (-cosf(obstacle_hitory_[obs_id].back().theta + yaw_rate * dt) + cosf(obstacle_hitory_[obs_id].back().theta));
  double box_theta = NormalizeAngle(obstacle_hitory_[obs_id].back().theta + yaw_rate * dt);

  Vec2d box_center(obstacle_hitory_[obs_id].back().position.x + dx, obstacle_hitory_[obs_id].back().position.y + dy);

  Box2d estimated_obs_box(box_center, box_theta, dis_obs.obs_msg.length, dis_obs.obs_msg.width);

  *box_ptr = estimated_obs_box;
  *estimated_speed = avg_speed;
  return true;
}

bool ModifyingFactory::ApplyMatching(std::vector<cyber_perception_msgs::PerceptionObstacle> new_obs, double time, int new_obs_index){
  if(new_obs.empty() || disappeared_obs_table_.empty()){
    return true;
  }

  Eigen::MatrixXd mat(new_obs.size(),disappeared_obs_table_.size());
  mat.array().setConstant(-1.f);

  std::pair<int, ObsMatchingData> candidate_table[new_obs.size()][disappeared_obs_table_.size()];

  for(int i = 0 ; i < new_obs.size() ; i++){
    auto obs = new_obs[i];
    Vec2d center(obs.position.x, obs.position.y);
    Box2d obs_box(center, obs.theta, obs.length, obs.width);
    // std::cout << "new_obs position.x : " << obs.position.x << ", y : " << obs.position.y << ", theta : " << obs.theta
    //           << ", length : " << obs.length << ", width : " << obs.width << std::endl;
    int j = 0;

    for(const auto& dis_obs : disappeared_obs_table_){
      
      if(obs.type != dis_obs.second.obs_msg.type) {
        j++;
        continue;
      }

      Box2d estimated_box;
      double estimated_speed;
      double real_speed = std::hypot(obs.velocity.x, obs.velocity.y);
      if(!EstimateBoxPose(time, dis_obs.second, &estimated_box, &estimated_speed)){
        std::cout << "failed to estimate pose! id : " << dis_obs.second.obs_msg.id << std::endl;
      }

      if(estimated_box.HasOverlap(obs_box) &&
         std::fabs(AngleDiff(obs.theta, estimated_box.heading())) < MAX_HEADING_ERROR &&
         std::fabs(estimated_speed - real_speed) < MAX_SPEED_ERROR){
        auto estimated_box_corners = estimated_box.GetAllCorners();
        auto obs_box_corners = obs_box.GetAllCorners();

        const std::vector<Point> points1{Point(estimated_box_corners[0].x(), estimated_box_corners[0].y()), 
                                           Point(estimated_box_corners[1].x(), estimated_box_corners[1].y()), 
                                           Point(estimated_box_corners[2].x(), estimated_box_corners[2].y()), 
                                           Point(estimated_box_corners[3].x(), estimated_box_corners[3].y())};
        const Polygon polygon1(points1.cbegin(), points1.cend());

        const std::vector<Point> points2{Point(obs_box_corners[0].x(), obs_box_corners[0].y()), 
                                           Point(obs_box_corners[1].x(), obs_box_corners[1].y()), 
                                           Point(obs_box_corners[2].x(), obs_box_corners[2].y()), 
                                           Point(obs_box_corners[3].x(), obs_box_corners[3].y())};
        const Polygon polygon2(points2.cbegin(), points2.cend());

        // std::cout << estimated_box_corners[0].x() << ", " << estimated_box_corners[0].y() << ", " 
        //           << estimated_box_corners[1].x() << ", " << estimated_box_corners[1].y() << ", "  
        //           << estimated_box_corners[2].x() << ", " << estimated_box_corners[2].y() << ", "  
        //           << estimated_box_corners[3].x() << ", " << estimated_box_corners[3].y() << std::endl;

        // std::cout << obs_box_corners[0].x() << ", " << obs_box_corners[0].y() << ", " 
        //           << obs_box_corners[1].x() << ", " << obs_box_corners[1].y() << ", "  
        //           << obs_box_corners[2].x() << ", " << obs_box_corners[2].y() << ", "  
        //           << obs_box_corners[3].x() << ", " << obs_box_corners[3].y() << std::endl;

        std::vector<PolygonWithHoles> intersections;
        PolygonWithHoles union_polygon;
        CGAL::intersection(polygon1, polygon2, std::back_inserter(intersections));
        if(!CGAL::join(polygon1, polygon2, union_polygon)){
          std::cout << "failed to compute union area!" << std::endl;
        }
        if(intersections.size() > 1){
          std::cout << "something is wrong when conputing intersection area" << std::endl;
        }
        double intersection_area = CGAL::to_double(intersections[0].outer_boundary().area());
        double union_area = CGAL::to_double(union_polygon.outer_boundary().area());
        // std::cout << "intersection area : " << intersection_area << ", union_area : " << union_area << std::endl;
        if(union_area == 0 || intersection_area > union_area){
          std::cout << "something is wrong when conputing intersection & union area" << std::endl;
        }
        double iou = std::abs(intersection_area / union_area);
        double heading_similarity = std::cos(AngleDiff(obs.theta, estimated_box.heading()));
        double speed_similarity = std::sqrt(1 - std::pow(std::fabs(estimated_speed - real_speed) / MAX_SPEED_ERROR, 2.0));
        if(std::fabs(heading_similarity) > 1.0 || std::fabs(speed_similarity) > 1.0 || iou > 1.0){
          std::cout << "something is wrong when computing similarity" << std::endl;
        }
        
        double total_similarity = iou + 0.5 * heading_similarity + 0.5 * speed_similarity;

        candidate_table[i][j] = std::make_pair(obs.id, ObsMatchingData(dis_obs.second.obs_msg.id, dis_obs.second.index - 1, new_obs_index));
        mat(i,j) = total_similarity;
      }
      j++;
    }
  }

  // for debug
  auto const rows = mat.rows();
  auto const cols = mat.cols();

  std::unordered_map<int,std::vector<int>> new_to_dis_mat;
  std::unordered_map<int,std::vector<int>> dis_to_new_mat;

  for (Eigen::Index i{0}; i < rows; ++i) {
    for (Eigen::Index j{0}; j < cols; ++j) {
      if(mat(i, j) > 0.0){
        new_to_dis_mat[candidate_table[i][j].first].push_back(candidate_table[i][j].second.matched_id);
        dis_to_new_mat[candidate_table[i][j].second.matched_id].push_back(candidate_table[i][j].first);
      }
    }
  }

  for(auto temp : new_to_dis_mat){
    if(temp.second.size() > 1){
      std::cout << "id : " << temp.first << " has " << temp.second.size() << " matched dis_obs id : ";
      for(auto t : temp.second){
        std::cout << t << ", ";
      }
      std::cout << "\n";
    }
  }

  for(auto temp : dis_to_new_mat){
    if(temp.second.size() > 1){
      std::cout << "id : " << temp.first << " has " << temp.second.size() << " matched new_obs id : ";
      for(auto t : temp.second){
        std::cout << t << ", ";
      }
      std::cout << "\n";
    }
  }

  while(true){
    int max_row = -1;
    int max_col = -1;

    if(mat.maxCoeff(&max_row, &max_col) < 0){
      break;
    }

    if(max_row < 0 || max_col < 0){
      std::cout << "maxCoeff Error !" << std::endl;
    }

    auto data = candidate_table[max_row][max_col];

    id_matching_table_[data.first] = data.second;
    disappeared_obs_table_.erase(data.second.matched_id);

    mat.row(max_row).setConstant(-1.f);
    mat.col(max_col).setConstant(-1.f);
  }
  return true;
}

bool ModifyingFactory::Update(cyber_perception_msgs::PerceptionObstacles obstacles, int obs_msg_index){
  double time = obstacles.cyber_header.timestamp_sec;
  std::vector<cyber_perception_msgs::PerceptionObstacle> new_obs;
  
  // update tracking_obs_ & disappeared_obs
  for(auto it = tracking_obs_.begin() ; it != tracking_obs_.end() ;){
    auto found_obs_iter = std::find_if(obstacles.perception_obstacle.begin(), obstacles.perception_obstacle.end(), 
                                        [&](const cyber_perception_msgs::PerceptionObstacle& obs_now) { return obs_now.id == it->id; });
    
    // 사라진 obstacle tracking_obs_에서 삭제 & disappeared_obs에 추가
    if(found_obs_iter == obstacles.perception_obstacle.end()){
      if(disappeared_obs_table_.count(it->id) != 0){
        std::cout << "obstacle already exists! id : " << it->id << ", prev index : " << disappeared_obs_table_[it->id].index 
                  << ", prev time : " << disappeared_obs_table_[it->id].time << ", curr index : " << obs_msg_index << ", curr time : " << time << std::endl;
      }
      disappeared_obs_table_[it->id] = DisappearedObsData(*it, time, obs_msg_index);
      it = tracking_obs_.erase(it);
      continue;
    }

    // update tracking obstacles state
    *it = *found_obs_iter;

    // update obstacle hitory
    obstacle_hitory_[it->id].push_back(*it);
    if(obstacle_hitory_[it->id].size() > OBSTACLE_HISTORY_SIZE){
      obstacle_hitory_[it->id].erase(obstacle_hitory_[it->id].begin());
    }

    it++;
  }

  // find new obstacles
  for(const auto& obs : obstacles.perception_obstacle){
    int found_obs = std::count_if(tracking_obs_.begin(), tracking_obs_.end(), 
                                  [&](const cyber_perception_msgs::PerceptionObstacle& track_obs) { return track_obs.id == obs.id; });

    if(found_obs == 0){
      // std::cout << "found new obs : " << obs.id << std::endl;
      new_obs.push_back(obs);
      tracking_obs_.push_back(obs);
      // update obstacle hitory
      obstacle_hitory_[obs.id].push_back(obs);
    }
  }

  // Delete Disappeared Obstacle
  for(auto it = disappeared_obs_table_.begin() ; it != disappeared_obs_table_.end() ;){
    if(std::abs(time - it->second.time) > TRACKING_TIME){
      it = disappeared_obs_table_.erase(it);
    }
    else it++;
  }

  // obstacle matching
  ApplyMatching(new_obs, time, obs_msg_index);
  return true;
}

bool ModifyingFactory::InterpolateTrajectory(){
  // std::ofstream control_points_file;
  // std::string cp_file_path {__FILE__};
  // cp_file_path = cp_file_path.substr(0, cp_file_path.rfind("src"));
  // cp_file_path += "src/heading_log/control_points_1394.csv";
  // std::cout << cp_file_path << std::endl;
  // control_points_file.open(cp_file_path, std::ios::trunc);

  // std::ofstream spline_points_file;
  // std::string spline_file_path {__FILE__};
  // spline_file_path = spline_file_path.substr(0, spline_file_path.rfind("src"));
  // spline_file_path += "src/heading_log/spline_1394_0.7.csv";
  // std::cout << spline_file_path << std::endl;
  // spline_points_file.open(spline_file_path, std::ios::trunc);

  for(const auto& obs : id_matching_table_){
    // std::cout << "obs id : " << obs.first << std::endl;
    // std::cout << "num of points : " << obs.second.end_index - obs.second.start_index + 1 << std::endl;
    std::vector<SplineControlPoint> spline_control_points;
    auto start_obs_list = obstacles_topics_with_time_[obs.second.start_index].second;
    auto end_obs_list = obstacles_topics_with_time_[obs.second.end_index].second;

    auto start_obs = std::find_if(start_obs_list.perception_obstacle.begin(), start_obs_list.perception_obstacle.end(),
                                  [&](const cyber_perception_msgs::PerceptionObstacle& obs_msg) { return obs_msg.id == obs.second.matched_id; });
    auto end_obs = std::find_if(end_obs_list.perception_obstacle.begin(), end_obs_list.perception_obstacle.end(),
                                [&](const cyber_perception_msgs::PerceptionObstacle& obs_msg) { return obs_msg.id == obs.first; });

    if(start_obs == start_obs_list.perception_obstacle.end() || end_obs == end_obs_list.perception_obstacle.end()){
      std::cout << "obstacle matching failed" << std::endl;
    }

    // compute based on ctrv model (후진하는 case 고려?)
    Vec2d start_pt(start_obs->position.x, start_obs->position.y);
    Vec2d end_pt(end_obs->position.x, end_obs->position.y);
    double total_time = end_obs_list.cyber_header.timestamp_sec - start_obs_list.cyber_header.timestamp_sec;
    double start_angle = (start_obs->theta);
    double end_angle = (end_obs->theta);
    double heading_diff = AngleDiff(start_angle, end_angle);

    bool is_linear = false;
    double angular_vel = 0.0;
    double radius = -1.0;
    double speed = 0.0;

    bool is_reverse = false;

    if((heading_diff > -1e-6 && heading_diff <= 0) ||
       (heading_diff >= 0 && heading_diff <= 1e-6) ||
        start_obs->type.type == cyber_perception_msgs::ObstacleType::PEDESTRIAN){
      is_linear = true;
    }

    Vec2d seg(end_pt.x() - start_pt.x(), end_pt.y() - start_pt.y());

    if(is_linear){
      speed = seg.Length() / total_time;
    }
    else {
      angular_vel = heading_diff / total_time;
      radius = seg.Length() / 2 / abs(sinf(heading_diff / 2));
      speed = abs(radius * angular_vel);
    }

    if(!is_linear && radius < 0.0){
      std::cout << "something is wrong with computing radius obs id : " << obs.first << std::endl;
    }

    if(std::fabs(AngleDiff(seg.Angle(), start_angle)) > M_PI * 5.0 / 6.0){
      speed = -speed;
      is_reverse = true;
      // std::cout << "-speed id : " << obs.first << std::endl;
    }

    // spline_control_points.emplace_back(Vec2d(0.0,0.0),0.0,start_angle);
    if(!is_reverse){
      spline_control_points.emplace_back(Vec2d(0.0,0.0),0.0,start_angle);
    }
    else{
      spline_control_points.emplace_back(Vec2d(0.0,0.0),0.0,NormalizeAngle(start_angle + M_PI));
    }
    for(int i = obs.second.start_index + 1 ; i <= obs.second.end_index - 1 ; i++){
      auto obs_list_msg = obstacles_topics_with_time_[i].second;
      double dt = obs_list_msg.cyber_header.timestamp_sec - start_obs_list.cyber_header.timestamp_sec;
      double dx = 0.0;
      double dy = 0.0;

      if(is_linear){
        dx = speed * cosf(start_angle) * dt;
        dy = speed * sinf(start_angle) * dt;
      }
      else{
        dx = speed / angular_vel * (sinf(start_angle + angular_vel * dt) - sinf(start_angle));
        dy = speed / angular_vel * (-cosf(start_angle + angular_vel * dt) + cosf(start_angle));
      }

      Vec2d control_pt(dx, dy);
      Vec2d spline_seg = control_pt - spline_control_points.back().control_point;
      double s = spline_control_points.back().s + spline_seg.Length();
      double angle = NormalizeAngle(start_obs->theta + angular_vel * dt);
      if(is_reverse) {
        angle = NormalizeAngle(angle + M_PI);
      }
      spline_control_points.emplace_back(control_pt,s,angle);
    }
    Vec2d spline_seg = (end_pt - start_pt) - spline_control_points.back().control_point;
    double end_seg_s = spline_control_points.back().s + spline_seg.Length();

    if(!is_reverse){
      spline_control_points.emplace_back(end_pt - start_pt, end_seg_s, end_angle);
    }
    else{
      spline_control_points.emplace_back(end_pt - start_pt, end_seg_s, NormalizeAngle(end_angle + M_PI));
    }

    // spline_control_points.emplace_back(end_pt - start_pt, end_seg_s, end_angle);

    spline_solver_.SetSplineControlPoints(spline_control_points);
    if(!spline_solver_.Solve()){
      std::cout << "failed to solve spline! obs : " << obs.first << ", matched obs : " << obs.second.matched_id << std::endl;
      continue;
    }

    const double scale = spline_control_points.back().s;
    
    for(int i = obs.second.start_index + 1 ; i <= obs.second.end_index - 1 ; i++){
      auto obs_list_msg = obstacles_topics_with_time_[i].second;
      double dt = obs_list_msg.cyber_header.timestamp_sec - start_obs_list.cyber_header.timestamp_sec;
      cyber_perception_msgs::PerceptionObstacle interpolated_obs;

      double t = spline_control_points[i - obs.second.start_index].s / scale;
      if(t < 0.0 || t > 1.0){
        std::cout << "something is woring with computing t : " << t << std::endl;
      }

      auto spline_pt = spline_solver_(t);

      // Vec2d heading_seg(spline_solver.spline().DerivativeX(t), spline_solver.spline().DerivativeY(t));
      // interpolated_obs.theta = heading_seg.Angle();

      interpolated_obs.theta = NormalizeAngle(start_obs->theta + angular_vel * dt);
      interpolated_obs.velocity.x = speed * cosf(start_angle + angular_vel * dt);
      interpolated_obs.velocity.y = speed * sinf(start_angle + angular_vel * dt);

      interpolated_obs.id = obs.first;
      interpolated_obs.position.x = spline_pt.x() + start_pt.x();
      interpolated_obs.position.y = spline_pt.y() + start_pt.y();
      interpolated_obs.type = start_obs->type;
      interpolated_obs.length = start_obs->length;
      interpolated_obs.width = start_obs->width;
      interpolated_obs.height = start_obs->height;

      obstacles_topics_with_time_[i].second.perception_obstacle.push_back(interpolated_obs);
      modified_only_obstacles_topics_with_time_[i].second.perception_obstacle.push_back(interpolated_obs);
    }

    // if(obs.first == 1394){
    //   std::cout << "obs found" << std::endl;
    //   std::cout.precision(13);
    //   std::cout << "start pt : " << start_pt.x() << ", " << start_pt.y() << ", " << start_angle << std::endl;
    //   std::cout << "end pt : " << end_pt.x() << ", " << end_pt.y() << ", " << end_angle << std::endl;
    //   std::cout << "=====================================" << std::endl;
    //   std::cout << "control_pt.front() : " << spline_control_points.front().control_point.x() + start_pt.x()
    //             << ", " << spline_control_points.front().control_point.y() + start_pt.y() << ", " << spline_control_points.front().angle << std::endl;
    //   std::cout << "control_pt.back() : " << spline_control_points.back().control_point.x() + start_pt.x()
    //             << ", " << spline_control_points.back().control_point.y() + start_pt.y() << ", " << spline_control_points.back().angle << std::endl;

    //   spline_points_file << "x,y\n";
    //   control_points_file << "x,y\n";
    //   for(double t = 0.0 ; t <= 1.0 ; t += 0.01){
    //     auto spline_pt = spline_solver_(t);
    //     spline_points_file << std::fixed << spline_pt.x() + start_pt.x() << "," << spline_pt.y() + start_pt.y() << "\n";
    //     std::cout << "spline x : " << spline_pt.x() + start_pt.x() << " y : " << spline_pt.y() + start_pt.y() << std::endl;
    //   }
    //   for(auto pt : spline_control_points){
    //     control_points_file << std::fixed << pt.control_point.x() + start_pt.x() << "," << pt.control_point.y() + start_pt.y() << "\n";
    //   }
    //   // for(auto l : lateral_bound){
    //   //   std::cout << "l : " << l << std::endl;
    //   // }
    // }
  }

  // std::cout << "Apply Matching" << std::endl;
  // for(auto matching_data : id_matching_table_){
  //   std::cout << "id : " << matching_data.first << ", matched id : " << matching_data.second.matched_id 
  //             << ", start_idx : " << matching_data.second.start_index << ", end_idx : " << matching_data.second.end_index
  //             << std::endl;
  //   if(matching_data.first == 4736){
  //     for(int i = matching_data.second.start_index ; i <= matching_data.second.end_index ; ++i){
  //       for(auto tl : obstacles_topics_with_time_[i].second.perception_obstacle){
  //         std::cout << tl.id << ", ";
  //       }
  //       std::cout << "\n";
  //     }
  //     std::cout << "\n";
  //   }
  // }

  return true;
}

void ModifyingFactory::PrintTable(){
  std::cout << "Debug Table" << std::endl;
  for(auto matching_data : id_matching_table_){
    std::cout << "id : " << matching_data.first << ", matched id : " << matching_data.second.matched_id 
              << ", start_idx : " << matching_data.second.start_index << ", end_idx : " << matching_data.second.end_index
              << std::endl;
    if(matching_data.first == 4736){
      for(int i = matching_data.second.start_index ; i <= matching_data.second.end_index ; ++i){
        for(auto obs : obstacles_topics_with_time_[i].second.perception_obstacle){
          // std::cout << obs.id << ", ";
          if(obs.id == 4708){
            std::cout.precision(7);
            std::cout << std::fixed << "x : " << obs.position.x << ", y : " << obs.position.y << ", z : " << obs.position.z << std::endl;
          }
        }
        std::cout << "\n";
      }
      std::cout << "\n";
    }
  }
}