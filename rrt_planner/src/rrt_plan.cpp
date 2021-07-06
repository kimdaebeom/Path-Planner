#include <ros/ros.h>
#include <rrt_planner/spline.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <obstacle_detector/Obstacles.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
​
​
#include <math.h>
#include <time.h>
#include <cmath>
#include <random>
#include <vector>
#include <iostream>
​
using naemspace std;
​
class RRT{
	
private:
	ros::NodeHandle nh_;
​
	int start_signal_
​
	double dist_;
	double start_distance_;
	double end_distance_;
	double look_ahead_distance_;
	double steer_angle_;
​
	//check route
	bool is_roote =false;
	bool is_best_roote=false;
	
	//flag
	bool approach_flag_;
	bool start_flag_;
	bool finish_flag_;
​
​	geometry_msgs::PoseStamped cur_pose_;
	cur_pose_.pose.x=0;
	cur_pose_.pose.y=0;

​	vector<geometry_msgs::PoseStamped> leftobs_; 
	vector<geometry_msgs::PoseStamped> rightobs_; 
	vector<geometry_msgs::PoseStamped> center_vec_;
	vector<double> parent_;
​
	ros::Publisher path_pub_;
	ros::Publisher ackermann_pub_;
​
	ros::Subscriber center_sub_;
	ros::Subscriber leftobs_sub_;
	ros::Subscriber rightobs_pub_
public:
	RRT(){
		initSetup();
		ROS_INFO("RRT PATH PLANNING INITIALIZED.");
	}
	~RRT(){
		ROS_INFO("PATH ALREADY GIVEN.")
	}
​
​
	void initSetup(){
		approach_flag_ = false;
		start_flag_ = true;
		finish_flag_ = false;
		steer_angle_ = 0.0;
​
		start_signal_=0;
​
		start_distance_ = 0.1;
		end_distanch_ = 0.1;
​
​
		center_sub_=nh_.subscribe("center_pts",10,centercallback);
		leftobs_sub_=nh_.subscribe("left_obstacle_pts",10,leftobscallback);
		rightobs_sub_=nh_.subscribe("right_obstacle_pts",10,rightobscallback);
​
	}
	
	void centercallback(const localmap_loader::PtsVector &center_vec){
		//center vector 중점 값을 받아오고 ->sub
		//그 값을 waypoint로 지정
		//way값 publish-> visualize에 쓴다.
​		center_vec_.clear();
		for (int i=0; i<center_vec.size(); i++){
			center_vec_.pushback(center_vec[i]);
			i++;
		}
	}
​
	void leftobscallback(const localmap_loader::PtsVector &left_vec){
		//left vector 값을 계속해서 받아오고 ->sub
		//좌측 범위 (random함수에서 범위) 
		leftobs_.clear();
		for (int i=0; i<left_vec.size(); i++){
			leftobs_.pushback(left_vec[i]);
			i++;
		}
	}
​
	void rightobscallback(const localmap_loader::PtsVector &right_vec){
		//right vector 값을 계속해서 받아오고 ->sub
		//우측 범위 (random함수에서 범위) 
​		rightobs_.clear();
		for (int i=0; i<right_vec.size(); i++){
			rightobs_.pushback(right_vec[i]);
			i++;
		}
	}
​
	void get_random_node(){
		//난수 함수 사용. https://blog.naver.com/anciid/221373642150 left_vect< <right_vect
		//dist 함수를통해 y좌표는 1m 범위 이내의 점들을 0.1m단위로 randomnode로 설정
		//->x는 vector범위 이내 random y는 cal_distance를 통해 1m내에서 0.1m단위로 
		//random node는 배열으로 설정.
		//각 배열은 array1 1m기준 array2 2m기준....	
		//random node -> publish	
	}
​
​
	void check_collision(){
		//left_vect와 right_vect를 불러와서 각 점으로 부터 random node까지 거리를 계산
		//거리를 cost2로 지정
		//if cost2< 차폭 ->cost2=0 or -index
		//if cost2>차폭  ->cost2= 거리의 역수.
	}
​
	void get_nearest_node_index(){
		//random node의 배열에서 각 cost를 비교
		//각 배열별 대표 point가 나오면 best_way로 저장
		//path라는 array에 best_way를 하나씩 넣어줌.
		//path return
	}
	
​
	double calc_distance(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2){
		//그냥 거리 구하는 식
​		return sqrt(pow((pose1.x-pose2.x),2) + pow((pose1.y-pose2.y),2));
	}
​
​
	void planning(){
		//return 받은 path를 이용,,??? 필요한 함순가... 그냥 종합해서 main에 쓸 함수임.
	
	}
	 
	double calc_steering_angle(){
		//angle을 구하는데 차량의 위치는 계속 0,0으로 갱신되기에
		//best_way+1 index와 arctan
		//steering_angle publish
	}
​
​
	//to think
	//차량의 위치가 0,0으로 갱신되기 때문에 각 way의 xy는 이전의 way의 xy만큼 감소시켜줘야함
	//
​
​
​
​
​
​
​
​
​
	
​
​
}
​
​
​
​
​
​
​
​
​
​
​
​
​
​
​
​
​
​
​
​
​
​
​
​
​
​
​
		
	}
