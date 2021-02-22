  // if course (vehicle's heading degree) is degree (North :0, East :90, South: 180, West :270) : change for atan2() system

void CourseCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr &course_msg) {
  cur_course_ = course_msg->drive.steering_angle;
  if(0<=cur_course_ && 180.0 > cur_course_){        //set cur_course_ system to same as atan2 system (+x:North, +y:West)
    cur_course_ = -1.0 * cur_course_;
  }
  else if(180.0 <= cur_course_ && cur_course_ < 360.0){
    cur_course_ = 360.0 - cur_course_;
  }
}

  // atan2() system for pure pursuit algorithm, and position of target, current pose is in utm-system now.

float calcSteeringAngle() {
  for(int i=0;i<waypoints_size_;i++) {
    double dist = calcPlaneDist(cur_pose_, waypoints_[i].pose);
    if(dist>lookahead_dist_){
      target_index_=i;
      waypoint_target_index_ = waypoints_[i].waypoint_index;
      break;
    }
  float steering_angle;
  float target_x = waypoints_[target_index_].pose.pose.position.x;
  float target_y = waypoints_[target_index_].pose.pose.position.y;
  float cur_x = cur_pose_.pose.position.x;
  float cur_y = cur_pose_.pose.position.y;
  //ROS_INFO("TARGET X=%f, TARGET Y=%f", target_x, target_y);
  float dx = target_x - cur_x;
  float dy = target_y - cur_y;

  float heading = atan2(dy,dx);
  float angle = heading * 180.0/M_PI;
  float alpha = (angle - cur_course_)*M_PI/180.0;
  float cur_steer;
  float dist_to_target = sqrtf(powf(dx,2)+powf(dy,2));
  float wheel_base = 1.04;
  cur_steer = atan2(2.0*wheel_base*sin(alpha)/dist_to_target, 1.0);
  return cur_steer;
}
