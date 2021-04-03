#include <ros/ros.h>
#include <math.h>
#include <vector>
#include <cmath>
#include <iostream>

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <obstacle_detector/Obstacles.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/Marker.h>

using namespace std;

class NarrowPath {

private:

	// Value
	double steer_angle_;
	double start_distance_;
	double end_distance_;
	int start_signal_;

	// Flag
	bool approach_flag_;
	bool start_flag_;
	bool finish_flag_;

	// Node
	ros::NodeHandle nh_;
	ros::Publisher ackermann_pub_, pub1_, pub2_, pub3_;
	ros::Subscriber sub_, sub1_, sub2_, sub3_;

	// Message
	geometry_msgs::Point wayPoint_;
	ackermann_msgs::AckermannDriveStamped ackerData_;
	
	vector<pair<double,double>> obstacle_right_vector_;  
	vector<pair<double,double>> obstacle_left_vector_;  
	vector<geometry_msgs::PoseStamped> center_vec_;
	ros::Publisher marker_pub;

public:
	NarrowPath() {
		initSetup();
		ROS_INFO("Narrow Path INITIALIZED.");
	}
	
	~NarrowPath(){
		obstacle_right_vector_.clear();
		obstacle_left_vector_.clear();
		ROS_INFO("Obstacle Loader Terminated.");
	}
	

	void initSetup() {
		approach_flag_ = false;
		start_flag_ = true;
		finish_flag_ = false;
		steer_angle_ = 0.0;
		
		setPoint(1, 0, 0);

		start_signal_ = 0;

		start_distance_ = 0.1;
		end_distance_ = 0.1;
	
		ackermann_msgs::AckermannDriveStamped ackerData_;
		sub1_ = nh_.subscribe("narrow_path_raw_obstacles", 1, &NarrowPath::obstacleCallback,this);
		sub2_ = nh_.subscribe("narrow_path_approach_raw_obstacles", 1, &NarrowPath::approachCallback,this);
		sub3_ = nh_.subscribe("narrow_path_escape_raw_obstacles", 1, &NarrowPath::escapeCallback,this);
		ackermann_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("ctrl_cmd", 10);
		marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_mark",1);
	}

	void setPoint(float x, float y, float z) {
		wayPoint_.x = x;
		wayPoint_.y = y;
		wayPoint_.z = z;	
	}

	void approachCallback(const obstacle_detector::Obstacles data) {

		if (start_flag_){
			ROS_INFO("approache_cb");
			narrowPathingApproach(data);
			double nearest_x_ = 1.0;

			for(auto segment_data : data.segments) {
				if (nearest_x_ > segment_data.first_point.x) {
					nearest_x_ = segment_data.first_point.x;
				}

				if (nearest_x_ > segment_data.last_point.x) {
					nearest_x_ = segment_data.last_point.x;
				}
			}

			if (nearest_x_ < start_distance_) {
				start_flag_ = false;
				approach_flag_ = true;
			}
		}
	}

	void obstacleCallback(const obstacle_detector::Obstacles data) {
		if (approach_flag_) {
			obstacle_right_vector_.clear();
			obstacle_left_vector_.clear();
			center_vec_.clear();
			ROS_INFO("obstacleCallback called");

			get_obstacle(data);
			get_center(obstacle_right_vector_, obstacle_left_vector_);
			
			double farthest_x = 0.05;
			
			for (auto segment_data : data.segments) {
				if (farthest_x < segment_data.first_point.x) {
					farthest_x = segment_data.first_point.x;
				}

				if (farthest_x <  segment_data.last_point.x) {

					farthest_x = segment_data.last_point.x;
				}
			}

			if (farthest_x < end_distance_) {
				finish_flag_ = true;
			}
			
		}
	}


	void escapeCallback(const obstacle_detector::Obstacles obs) {
		geometry_msgs::Point finish_point = farthestPoint(obs);
	

		if (finish_flag_) {
			if (fabs(calcDistance(wayPoint_, finish_point)) < 0.7) {
				ackerData_.drive.steering_angle = -10;
				ackerData_.drive.speed = 3;
				approach_flag_ = false;
				ackermann_pub_.publish(ackerData_);
				ROS_INFO("##############################");
				ros::shutdown();
			}
			
		}
	}

	void get_obstacle(const obstacle_detector::Obstacles data){
		ackerData_.drive.speed = 3; // throttle
		
		//##################### check if segments is empty or not############
		int size_segments = sizeof(data)/sizeof(data.segments[0]);
		geometry_msgs::PoseStamped obstacle_pose; 
		if (size_segments >= 2){
			double x_center = 0;
			double y_center = 0; 
			double temp_x = 0;
			double temp_y = 0;
			int count = 0;
			for (auto segment_data : data.segments){
				temp_x = (segment_data.first_point.x + segment_data.last_point.x)/2;
				temp_y = (segment_data.first_point.y + segment_data.last_point.y)/2;
				obstacle_pose.pose.position.x = temp_x;
				obstacle_pose.pose.position.y = temp_y;
				if (obstacle_pose.pose.position.y <= 0){
					obstacle_right_vector_.push_back(pair<double,double>(obstacle_pose.pose.position.x,obstacle_pose.pose.position.y));
				}
				if (obstacle_pose.pose.position.y > 0){
					obstacle_left_vector_.push_back(pair<double,double>(obstacle_pose.pose.position.x,obstacle_pose.pose.position.y));
				}				
				x_center = x_center + segment_data.first_point.x;
				//cout << "first x point is : " << segment_data.first_point.x << endl;
				x_center = x_center + segment_data.last_point.x;
				y_center = y_center + segment_data.first_point.y;
				//cout << "first y point is : " << segment_data.first_point.y << endl;
				y_center = y_center + segment_data.last_point.y;
				count++;
			}
			cout << "count is " << count << endl;
			x_center = x_center / size_segments;
			y_center = y_center / size_segments;
			/*if (!obstacle_vector_.empty()){
				for (int i=0; i<obstacle_vector_.size(); i++){
					cout << "obstacle_vector point x is  ::  " << obstacle_vector_[i].pose.position.x << endl;
					cout << "obstacle_vector point y is  ::  " << obstacle_vector_[i].pose.position.y << endl;
				}
				int size_ = (obstacle_left_vector_.size() > obstacle_right_vector_.size()) ? obstacle_right_vector_.size() : obstacle_left_vector_.size();
				for (int i = 0; i < size_; i++){
					cout << "obstacle right point of x is " << obstacle_right_vector_[i].pose.position.x << endl;
					cout << "obstacle right point of y is " << obstacle_right_vector_[i].pose.position.y << endl;
					cout << "obstacle left point of x is " << obstacle_left_vector_[i].pose.position.x << endl;
					cout << "obstacle left point of y is " << obstacle_left_vector_[i].pose.position.y << endl;
				}
			}*/
			setPoint(x_center, y_center, 0);
			start_signal_ = 1;
			steer_angle_ = -atan((wayPoint_.y / wayPoint_.x));
			ackerData_.drive.steering_angle = double(steer_angle_ * 180.0 / M_PI);
		} 
		else {
			ackerData_.drive.steering_angle = double(steer_angle_ * 180.0 / M_PI);
		}

		if (ackerData_.drive.steering_angle > 26) {
			ackerData_.drive.steering_angle = 26;
		}
		else if (ackerData_.drive.steering_angle < -26) {
			ackerData_.drive.steering_angle = -26;
		}
		//cout << "steering angle is " << steer_angle_ << endl;
		if (!finish_flag_) {
			ackermann_pub_.publish(ackerData_);
		}
	}
	
	void get_center(vector<pair<double,double>> obstacle_right_vector_, vector<pair<double,double>> obstacle_left_vector_){
	
		uint32_t viz = visualization_msgs::Marker::CUBE;
		visualization_msgs::Marker marker;
		marker.header.frame_id = "/center_point";
		marker.header.stamp = ros::Time::now();
		marker.ns = "basic_shapes";
		marker.id = 0;
		marker.type = viz;
		marker.pose.position.x = 0.0;
		marker.pose.position.y = 0.0;
		marker.pose.position.z = 0.0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 0.1;
		marker.scale.y = 0.1;
		marker.scale.z = 0.0;
		marker.color.r = 0.0f;
		marker.color.g = 1.0f;
		marker.color.b = 0.0f;
		marker.color.a = 1.0;
		marker.lifetime = ros::Duration();
	
		sort(obstacle_right_vector_.begin(),obstacle_right_vector_.end());
		sort(obstacle_left_vector_.begin(),obstacle_left_vector_.end());
		int size_ = (obstacle_left_vector_.size() > obstacle_right_vector_.size()) ? obstacle_right_vector_.size() : obstacle_left_vector_.size();
		double temp_x = 0;
		double temp_y = 0;
		geometry_msgs::PoseStamped temp_pose;
		for (int i=0; i<size_-1; i++){
			temp_x = (obstacle_right_vector_[i].first + obstacle_left_vector_[i].first)/2;
			temp_y = (obstacle_right_vector_[i].second + obstacle_left_vector_[i].second)/2;
			temp_pose.pose.position.x = temp_x;
			temp_pose.pose.position.y = temp_y;
			center_vec_.push_back(temp_pose);
			temp_x = (obstacle_right_vector_[i].first + obstacle_left_vector_[i].first)/2;
			temp_y = (obstacle_right_vector_[i+1].second + obstacle_left_vector_[i+1].second)/2;
			temp_pose.pose.position.x = temp_x;
			temp_pose.pose.position.y = temp_y;
			center_vec_.push_back(temp_pose);
		}
		temp_x = (obstacle_right_vector_[size_-1].first + obstacle_left_vector_[size_-1].first)/2;  // 마지막에는 끝 좌표의 중점만을 잡기위해 따로 계산한다.
		temp_y = (obstacle_right_vector_[size_-1].second + obstacle_left_vector_[size_-1].second)/2;
		temp_pose.pose.position.x = temp_x;
		temp_pose.pose.position.y = temp_y;
		center_vec_.push_back(temp_pose);
		for (int i=0; i < center_vec_.size(); i++){
			cout << "center x point is " << center_vec_[i].pose.position.x << "  and y point is  " << center_vec_[i].pose.position.y << endl;
			marker.pose.position.x = center_vec_[i].pose.position.x;
			marker.pose.position.y = center_vec_[i].pose.position.y;
			marker_pub.publish(marker);
		}
		//중점 잡는거 for
	}
	
	void narrowPathingApproach(const obstacle_detector::Obstacles data){
				
		double x_center = 0.0;	
		double y_center = 0.0;
		double nearest_x = 1.0;
		double nearest_y_1 = -1.0;
		double nearest_y_2 = 1.0;

		for(auto segment_data : data.segments){

			if (nearest_x > segment_data.first_point.x) {
				nearest_x = segment_data.first_point.x;
			}
		
			if (nearest_x > segment_data.last_point.x) {
				nearest_x = segment_data.last_point.x;
			}

			if (segment_data.first_point.y < 0) {
				if (nearest_y_1 < segment_data.first_point.y) {
					nearest_y_1 = segment_data.first_point.y;
				}
				if (nearest_y_1 < segment_data.last_point.y) {
					nearest_y_1 = segment_data.last_point.y;
				}
			}

			if (segment_data.first_point.y >= 0) {
				if (nearest_y_2 > segment_data.first_point.y) {
					nearest_y_2 = segment_data.first_point.y;
				}
				if (nearest_y_2 > segment_data.last_point.y) {
					nearest_y_2 = segment_data.last_point.y;
				}
			}
		}
	
		x_center = nearest_x;
		y_center = nearest_y_1 + nearest_y_2;

		x_center = x_center;
		y_center = y_center/2;
		setPoint(x_center, y_center, 0);
		
		start_signal_ = 1;
		ackerData_.drive.speed = 3;	
		steer_angle_ = atan(wayPoint_.y/wayPoint_.x);

		ackerData_.drive.steering_angle = int((-steer_angle_/M_PI) * 1.04);

		if (ackerData_.drive.steering_angle > 26) {
			ackerData_.drive.steering_angle = 26;
		}
		else if (ackerData_.drive.steering_angle < -26)	{
			ackerData_.drive.steering_angle = -26;
		}
		cout << "steering angle is " << steer_angle_ << endl;
		if (!finish_flag_) {
			ackermann_pub_.publish(ackerData_);
		}

				
	}

	geometry_msgs::Point farthestPoint(const obstacle_detector::Obstacles data) {
		double x_center = 0;
		double y_center = 0;
		double far_x = 0;
		double far_y_1 = 0;
		double far_y_2 = 0;

		for (auto segment_data : data.segments) {
			if (far_x < segment_data.first_point.x){
				far_x = segment_data.first_point.x;
		}
			if (far_x < segment_data.last_point.x) {
				far_x = segment_data.last_point.x;
		}

			if (segment_data.first_point.y < 0) {
				if (far_y_1 > segment_data.first_point.y) {
					far_y_1 = segment_data.first_point.y;
				}
				if (far_y_1 > segment_data.last_point.y) {
					far_y_1 = segment_data.last_point.y;
				}	
		}

			if (segment_data.first_point.y >= 0) {
				if (far_y_2 < segment_data.first_point.y) {
					far_y_2 = segment_data.first_point.y;
				}
				if (far_y_2 < segment_data.last_point.y) {
					far_y_2 = segment_data.last_point.y;
				}
			}
		}

		x_center = far_x;
		y_center = far_y_1 + far_y_2;
		
		x_center = x_center;
		y_center = y_center/2;
		setPoint(x_center, y_center ,0);

		return wayPoint_;
		
	}

 	double calcDistance(geometry_msgs::Point point1, geometry_msgs::Point point2) {
		return pow((point1.x- point2.x), 2) + pow((point1.y - point2.y), 2);
	}
	
/*
	inline void  delaunay_validate(const vector<double>& poses, const double e){
		delaunator::Delaunator d(poses);
		for (int i=0; i < d.halfedges.size(); i++) {
			const auto i
		}
	}
*/
};
int main(int argc, char **argv) {
	ros::init(argc, argv, "narrow_path_node");
	NarrowPath narrowPath;
	ros::spin();
	return 0;
}
