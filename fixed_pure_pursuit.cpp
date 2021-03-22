void CourseCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr &course_msg) {
  //cur_speed_ = course_msg->drive.speed;
  cur_course_ = course_msg->drive.steering_angle;
  if(0 <= cur_course_ && cur_course_ < 270) cur_course_ = 90 - cur_course_;
        else cur_course_ = 450 - cur_course_;
  is_course_ = true;
  //ROS_INFO("COURSE CALLBACK");
}


float calcSteeringAngle() {
  for(int i=0;i<waypoints_size_;i++) 
  {	
    double dist = calcPlaneDist(cur_pose_, waypoints_[i].pose);
    if(dist>lookahead_dist_)
    {
      target_index_ = i;
      waypoint_target_index_ = waypoints_[i].waypoint_index;
      ROS_INFO("target_index: %d ld: %f",target_index_,lookahead_dist_);
      break;
    }
  }

  double wheel_base = 1.04;
  double steering_angle;
  double target_x = waypoints_[target_index_].pose.pose.position.x;
  double target_y = waypoints_[target_index_].pose.pose.position.y;
  //ROS_INFO("TARGET X=%f, TARGET Y=%f", target_x, target_y);

  double dx = target_x - cur_pose_.pose.position.x;
  double dy = target_y - cur_pose_.pose.position.y;
  double temp_theta = atan2(dy,dx) *180.0/M_PI;
  if (temp_theta <= 90.0) temp_theta = 90.0 - temp_theta;
  else temp_theta = 450.0 - temp_theta;
  double deg_alpha = (temp_theta - cur_course_);
  if (deg_alpha <= -180.0) deg_alpha += 360.0;
  else if(deg_alpha > 180.0) deg_alpha -= 360.0;
  cout<<"deg_alpha = "<<deg_alpha<<endl;
  double sin_alpha = sin(deg_alpha * M_PI/180.0);
  double cur_steer = atan2(2.0 * wheel_base * sin_alpha / dist, 1.0);
  cur_steer = cur_steer * 180.0 / M_PI;
  return cur_steer;
}
