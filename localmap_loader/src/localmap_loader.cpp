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
#include <localmap_loader/PtsVector.h>

using namespace std;

class Localmap_Loader {

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
	ros::Publisher centers_pub_, leftobs_pub_, rightobs_pub_;
	ros::Subscriber sub_, sub1_, sub2_, sub3_;

	// Message
	geometry_msgs::Point wayPoint_;
	ackermann_msgs::AckermannDriveStamped ackerData_;
	localmap_loader::PtsVector left_msg_;
	localmap_loader::PtsVector right_msg_;
	localmap_loader::PtsVector center_msg_;
	
	// Vector
	//vector for calculate the center points (center_vec_)
	vector<pair<double,double>> obstacle_right_vector_;   
	vector<pair<double,double>> obstacle_left_vector_; 
	//vector for publish message
	vector<geometry_msgs::PoseStamped> leftobs_; 
	vector<geometry_msgs::PoseStamped> rightobs_; 
	vector<geometry_msgs::PoseStamped> center_vec_;

public:
	Localmap_Loader() {
		initSetup();
		ROS_INFO("Localmap_Loader INITIALIZED.");
	}
	
	~Localmap_Loader(){
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
		sub1_ = nh_.subscribe("localmap_loader_raw_obstacles", 1, &Localmap_Loader::obstacleCallback,this);
		sub2_ = nh_.subscribe("localmap_loader_approach_raw_obstacles", 1, &Localmap_Loader::approachCallback,this);
		sub3_ = nh_.subscribe("localmap_loader_escape_raw_obstacles", 1, &Localmap_Loader::escapeCallback,this);
		ackermann_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("ctrl_cmd", 10);
		centers_pub_ = nh_.advertise<localmap_loader::PtsVector>("center_pts", 10);
		leftobs_pub_ = nh_.advertise<localmap_loader::PtsVector>("left_obstacle_pts", 10);
		rightobs_pub_ = nh_.advertise<localmap_loader::PtsVector>("right_obstacle_pts", 10);
	}

	void setPoint(float x, float y, float z) {
		wayPoint_.x = x;
		wayPoint_.y = y;
		wayPoint_.z = z;	
	}

	void approachCallback(const obstacle_detector::Obstacles data) {

		if (start_flag_){
			ROS_INFO("approache_cb");
			center_vec_.clear();
			ObstacleApproach(data);
			//Todo :: find the start center point to set the position of platform for starting mission
			//centers_pub_.publish(center_vec_);
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
			center_vec_.clear(); // posestamped vector for msg
			leftobs_.clear();
			rightobs_.clear();
			ROS_INFO("obstacleCallback called");

			get_obstacle(data);
			
			leftobs_pub_.publish(left_msg_);
			rightobs_pub_.publish(right_msg_);
			
			get_center(obstacle_right_vector_, obstacle_left_vector_);
			
			centers_pub_.publish(center_msg_);
			
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
					rightobs_.push_back(obstacle_pose);
				}
				if (obstacle_pose.pose.position.y > 0){
					obstacle_left_vector_.push_back(pair<double,double>(obstacle_pose.pose.position.x,obstacle_pose.pose.position.y));
					leftobs_.push_back(obstacle_pose);
				}				
				x_center = x_center + segment_data.first_point.x;
				//cout << "first x point is : " << segment_data.first_point.x << endl;
				x_center = x_center + segment_data.last_point.x;
				y_center = y_center + segment_data.first_point.y;
				//cout << "first y point is : " << segment_data.first_point.y << endl;
				y_center = y_center + segment_data.last_point.y;
				count++;
			}
			
			left_msg_.ptsvec = leftobs_;
			right_msg_.ptsvec = rightobs_;
				
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
		center_msg_.ptsvec = center_vec_;
		for (int i=0; i < center_vec_.size(); i++){
			cout << "center x point is " << center_vec_[i].pose.position.x << "  and y point is  " << center_vec_[i].pose.position.y << endl;
		}
		//중점 잡는거 for
	}
	
	void ObstacleApproach(const obstacle_detector::Obstacles data){
				
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
	ros::init(argc, argv, "localmap_loader_node");
	Localmap_Loader LL;
	ros::spin();
	return 0;
}
