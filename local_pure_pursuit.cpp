	double calcSteeringAngle() {
		double  wheel_base = 1.04;
		double steering_angle;
		double target_x = waypoints_[target_index_].pose.pose.position.x;
		double target_y = waypoints_[target_index_].pose.pose.position.y;
		// cur_steer = atan(2*wheel_base*sin(alpha)/target_ld_value);
		double cur_steer;
		double local_target_x, local_target_y;
		double local_theta_rad;
		double local_distance;
		local_target_x, local_target_y, local_theta_rad, local_distance = transform2local(target_x,target_y,cur_pose_,cur_course_);
		cur_steer = atan(2.0*wheel_base*sin(local_theta_rad)/local_distance);	
		if(isfirst_steer_) 
		{
	   		prev_steer_ = cur_steer;
			isfirst_steer_ = false;
        		}
		return cur_steer;
	}
	
	double transform2local(double t_x, double t_y, double cur_position, double cur_course_)  // transform to local system for Pure-Pursuit algorithm
	{
		double local_x = 0.0;
		double local_y = 0.0;
		double point_x = 0.0;
		double point_y = 0.0;
		//double local_r = 0.0;
		double local_theta = 0.0;
		double temp_posepoint_x = cur_position.pose.position.x;
		double temp_posepoint_y = cur_position.pose.position.y;
		//ROS_INFO("way,pose =  %f,%f",t_x,t_y);
		point_x = t_x - temp_posepoint_x;
		point_y = t_y - temp_posepoint_y;
		//ROS_INFO("point_offset = %f,%f",point_x,point_y);
		float local_r= sqrtf(powf(point_x, 2) + powf(point_y, 2));

		if ((point_x >= 0) && (point_y > 0)){
			local_theta = 90.0-atan(point_y/point_x)*180/M_PI - cur_course_;
			if (local_theta < -270) local_theta = local_theta + 360;
		}
		else if ((point_x >= 0) && (point_y < 0)){
			local_theta = 90.0 - atan(point_y/point_x)*180/M_PI - cur_course_;
		}
		else if ((point_x < 0) && (point_y < 0)){
			local_theta = 270.0 - atan(point_x/point_y)*180/M_PI - cur_course_;
		}
		else if ((point_x < 0) && (point_y > 0)){
			local_theta = 270.0 - atan(point_y/point_x)*180/M_PI - cur_course_;
			if (local_theta > 270) local_theta = local_theta - 360;	
		}
		
		if(local_theta<-180) local_theta += 360;
		else if(local_theta>180) local_theta -= 360;

		float local_theta_rad = local_theta*M_PI/180;
		local_x = local_r * cos(local_theta_rad);
		local_y = -(local_r * sin(local_theta_rad));
		return local_x, local_y, local_theta_rad, local_r; 
	}
