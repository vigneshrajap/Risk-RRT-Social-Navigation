#include <ros/ros.h>
#include <riskrrt/PoseTwistStamped.h>
#include <riskrrt/Trajectory.h>//ros only have path msg, no trajectory
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <vector>
#include <icars_2d_map_manager/Status.h>  // Car pose
#include <icarsCarControlWebsocket/SimRefs.h>  // for getting the control vector
#include <icars_2d_map_manager/Path.h>  // Trajectory pose
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <riskrrt/OccupancyGridArray.h>
#include <tf/transform_broadcaster.h>
// #include <tf/Vector3.h>

#include "riskrrt/social_navigation.hpp"

#include <nav_msgs/OccupancyGrid.h> // OG map from "submap" topic
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <angles/angles.h>

//Global Varaibles Definitions
geometry_msgs::Pose local_goal[100],global_goal,final_goal,p_space,o_space;
icars_2d_map_manager::Status msg_pf_stat;
bool goal_received;
icarsCarControlWebsocket::SimRefs msg_ctr, msg_ctr_finale;
double robotLength = 2.588;
double robotWidth = 1.5110;
double timeStep = 0.5;
int nbMap= 5;
geometry_msgs::Pose error_traj;
custom_pose robot_pose;
nav_msgs::OccupancyGrid grid;
riskrrt::OccupancyGridArray og_array; // array of maps, time of map = timeStep * position in array. TODO: multiple gridArray
bool obstacle_detected = false;
bool goal_reached = false;
double obs_yaw;
double margin = 1.50 * robotLength;
double margin_1 = 0.4 * margin;
double margin_2 = 0.5 * margin;
bool check_for_obstacle = false;
double p_marker_size_A,p_marker_size_B,o_marker_size_A,o_marker_size_B;
double linear_velocity_max = 5.0;
double New_A_lc = 3.0; 
double New_B_lc = 2.0;
double A_lc, B_lc;


int gridIFromPose(geometry_msgs::Pose pose){
 return (int)round((pose.position.x - og_array.array[0].info.origin.position.x) / og_array.array[0].info.resolution);
}

int gridJFromPose(geometry_msgs::Pose pose){
 return (int)round((pose.position.y - og_array.array[0].info.origin.position.y) / og_array.array[0].info.resolution);
}

int gridIndexFromCoord(int i, int j){
  return i + og_array.array[0].info.width * j;
}

// for getting odometry data 
void odometryCallback(const nav_msgs::Odometry& msg) 
  {
    geometry_msgs::Twist currentVelocity;
    currentVelocity = msg.twist.twist;
  }

//car current pose
void poseCallback(const icars_2d_map_manager::Status msg){
  robot_pose.x = msg.xLocal;
  robot_pose.y = msg.yLocal;
  robot_pose.theta = msg.yaw+1.57;

  msg_pf_stat.header=msg.header;  // for obtaining the exact time stamp of car's current position
}

// Goal Information
void goalCallback(const icars_2d_map_manager::Path msg){

for(int i=0;i<msg.points.size();i++)
{
  local_goal[i].position.x  = msg.points[i].x;
  local_goal[i].position.y = msg.points[i].y;
  local_goal[i].orientation = tf::createQuaternionMsgFromYaw(msg.points[i].yaw);
}
  global_goal.position.x  = msg.xGlobalOrig;
  global_goal.position.y = msg.yGlobalOrig;
  global_goal.orientation = tf::createQuaternionMsgFromYaw(msg.origHeading);

  goal_received = true;

}

// braking
geometry_msgs::Twist brake(){
  
  geometry_msgs::Twist corrected_control_pp;
  
  corrected_control_pp.linear.x = 0.0;
  corrected_control_pp.angular.z = 0.0;

    msg_ctr.steerAng=corrected_control_pp.angular.z;
    msg_ctr.brakeStrength= 100.0;
    msg_ctr.gasForce=corrected_control_pp.linear.x;
    
	return corrected_control_pp;
};

// obtaining the 2D occupancy grid information
void ogArrayCallback(const riskrrt::OccupancyGridArray msg){
  og_array = msg;
}

// obtaining the p-space obstacle information
void humanmarkerpCallback(const visualization_msgs::MarkerArray msg){

if(msg.markers.size()>0){
  p_space.position.x = msg.markers[0].pose.position.x;
  p_space.position.y = msg.markers[0].pose.position.y;
  p_space.position.z = msg.markers[0].pose.position.z;
  p_space.orientation = msg.markers[0].pose.orientation;
  obs_yaw= tf::getYaw(p_space.orientation);
  p_marker_size_A = msg.markers[0].scale.x;
  p_marker_size_B = msg.markers[0].scale.y;
 }
}

// obtaining the o-space obstacle information
void humanmarkeroCallback(const visualization_msgs::MarkerArray msg){
if(msg.markers.size()>0){
  o_space.position.x = msg.markers[0].pose.position.x;
  o_space.position.y = msg.markers[0].pose.position.y;
  o_space.position.z = msg.markers[0].pose.position.z;
  o_space.orientation = msg.markers[0].pose.orientation;
  o_marker_size_A = msg.markers[0].scale.x;
  o_marker_size_B = msg.markers[0].scale.y;
 }
}

//get the pose of the new node for a differential robot
custom_pose robotKinematic(custom_pose pose, icarsCarControlWebsocket::SimRefs control){
  double rotation_radius;
  double delta_theta, delta_x, delta_y;
  double delta_xc, delta_yc;

//paramteres definition for odometry calculation of CLMR
  double delta_r, delta_ori, Nonholonomicconstraint_1,Nonholonomicconstraint_2,rotation_radius_curvature, curvature;
  custom_pose new_pose_CLMR;
  
  if(control.gasForce == 0.0){
    delta_r = control.gasForce;
    rotation_radius_curvature =robotLength/(tan(control.steerAng));
    delta_ori = (delta_r*timeStep)/rotation_radius_curvature;
	}

   else if(control.steerAng == 0.0){
    delta_r = control.gasForce;
    rotation_radius_curvature = 0.0;
    delta_ori = 0.0;
  }

  else{
   //curvature calculation for CLMR
    rotation_radius_curvature=robotLength/(tan(control.steerAng));
    delta_r = (control.gasForce );
    delta_ori = (delta_r*timeStep)/rotation_radius_curvature;
  }

   delta_x = ((delta_r* timeStep) * cos(pose.theta));
   delta_y = ((delta_r* timeStep) * sin(pose.theta));

    //new pose prediction for CLMR
     new_pose_CLMR.x = pose.x + delta_x;
     new_pose_CLMR.y= pose.y + delta_y;
     new_pose_CLMR.theta = pose.theta + delta_ori*timeStep;

  return new_pose_CLMR;
}

//returns true if robot close enough to the goal (threshold set by user)
double isGoalReached(){
  //this is a distance criteria, orientation is not taken into account
  return (sqrt(pow(error_traj.position.x, 2) + pow(error_traj.position.y, 2)));
}

//returns true if robot close enough to the obstacle (threshold set by user)
double nearestobstacle(geometry_msgs::Pose dist_pose){
  //this is a distance criteria, orientation is not taken into account
  return (sqrt(pow(robot_pose.x-dist_pose.position.x, 2) + pow(robot_pose.y-dist_pose.position.y, 2)));
}

// Tranformationmatrix
void TransformationMatrix(geometry_msgs::Pose obs_pose, custom_pose robot_pose,Eigen::Matrix3d rot,double theta_diff_obs_goal){

     rot(0,0)=double(cos(theta_diff_obs_goal));
     rot(0,1)=double(-sin(theta_diff_obs_goal));
     rot(0,2)=double(0);
     rot(1,0)=double(sin(theta_diff_obs_goal));
     rot(1,1)=double(cos(theta_diff_obs_goal));
     rot(1,2)=double(0);
     rot(2,0)=double(0);
     rot(2,1)=double(0);
     rot(2,2)=double(1);
}

void setTransform(Eigen::Matrix3d &rot, Eigen::Vector3d &t, Eigen::Matrix4d &Tf)
{

Tf.block(0,0,3,3) << rot;
Tf.block(0,3,3,1) << t;
Tf.block(3,0,1,4) << 0.0,0.0,0.0,1.0;

}

//reaching the target controller - FIRST CONTROLLER
icarsCarControlWebsocket::SimRefs control_commands( geometry_msgs::Twist corrected_control)
{
 geometry_msgs::Pose theoretical_pose, temp_pose_1;

  int i, j, k, ped_grid_i, ped_grid_j;
  int arg_vel_obs=0;
  int max_i, min_i;
  int max_j, min_j;

double error = isGoalReached(); // ERROR W.r.t TO FINAL GOAL

if (corrected_control.angular.z!=0 && corrected_control.linear.x!=0){             

for(k=0;k<nbMap;k++){

   temp_pose_1.position.x = robot_pose.x + k*timeStep*arg_vel_obs*cos(robot_pose.theta);
   temp_pose_1.position.y = robot_pose.y + k*timeStep*arg_vel_obs*cos(robot_pose.theta);

   ped_grid_i = gridIFromPose(temp_pose_1);
   ped_grid_j = gridJFromPose(temp_pose_1);

    min_i = ped_grid_i - 2;
    max_i = ped_grid_i + 2;
    min_j = ped_grid_j - 2;
    max_j = ped_grid_j + 2;

     for(i=min_i ; i<=max_i ; i++){
      for(j=min_j ; j<=max_j ; j++){

// PART 1: BRAKE IF OBSTACLE DETECTED 
        if(og_array.array[k].data[gridIndexFromCoord(i,j)] != 0)
        {
        corrected_control = brake();
        ROS_INFO("OBSTACLE DETECTED AHEAD");
        obstacle_detected = true;
        }

      }
        }
}

if(obstacle_detected == false)
 {
// PART 2: BRAKE IF GOAL REACHED
if (error < 1.5 && (msg_ctr.brakeStrength!=100.0))
{
 corrected_control = brake();
 ROS_INFO("GOAL REACHED");
 goal_reached = true;
 } 

// PART 3: SENDING REQUIRED VELOCITY COMMANDS TO REACH THE GOAL
if(goal_reached == false){
    msg_ctr.steerAng=corrected_control.angular.z;
    msg_ctr.brakeStrength=0.0;
    msg_ctr.gasForce=corrected_control.linear.x;
     }
 }

} 
    
else{
        corrected_control = brake();
        ROS_INFO("WAITING FOR GRID MAP INFO");
    }

return msg_ctr;

}

int checkforobstacle(custom_pose robot_pose,geometry_msgs::Pose final_goal){
// creating points
int count =0;
int count_1 =0;
int numOfPoints = 50;
Point* points;
points = new Point[numOfPoints];

geometry_msgs::Pose error_obs;

// error with respect to x and y
error_obs.position.x= robot_pose.x-p_space.position.x;
error_obs.position.y= robot_pose.y-p_space.position.y;

double d_obs = sqrt((error_obs.position.x * error_obs.position.x) + (error_obs.position.y * error_obs.position.y));

    double m = (final_goal.position.y - robot_pose.y)/(final_goal.position.x-robot_pose.x);
    double c = robot_pose.y - m*robot_pose.x;

    double increment = ((final_goal.position.x-robot_pose.x)/numOfPoints);

        points[0].x = robot_pose.x+increment;
        points[0].y = (m*points[0].x) + c;

   for(int i= 1; i<numOfPoints ; i++) {
        points[i].x = increment+ points[i-1].x;
        points[i].y = (m*points[i].x) + c;
    }

for(int i=0; i<numOfPoints; i++){
if (((points[i].x ) < (p_marker_size_A + (p_space.position.x) + 2.0)) && (points[i].x) > ( p_space.position.x - p_marker_size_A - 2.0)){
 if (((points[i].y ) < (p_marker_size_B + (p_space.position.y)+ 5.0)) && (points[i].y) > ( p_space.position.y - p_marker_size_B - 5.0)){
count =1;
// std::cout << "i:" << i << std::endl;

if (count == 1 && d_obs <= 25){
count_1 = 1;
}

   } 
  }
 } 
 
/*
if (count == 0) {
for(int i=0; i<numOfPoints; i++){
if (points[i].x - (o_marker_size_A+o_space.position.x) < (o_marker_size_A+1.5) && points[i].x - (o_marker_size_A+o_space.position.x) > -(o_marker_size_A+1.5)){
 if (points[i].y - (o_marker_size_B+o_space.position.y) < (o_marker_size_B+0.5) && points[i].y -(o_marker_size_B+o_space.position.y) > -(o_marker_size_B+0.5)){
 check_for_obstacle = true;
count =1;
   } 
  }
 } 
} */


return count_1;
}


//reaching the target controller - FIRST CONTROLLER
geometry_msgs::Twist attractiontothecontroller(custom_pose car_pose,geometry_msgs::Pose final_goal)
{
geometry_msgs::Twist control;
double l_1 = 0.5;
double k_gain = 0.6; // gain for the proposed contorl law

// error with respect to x and y
error_traj.position.x= car_pose.x-final_goal.position.x;
error_traj.position.y= car_pose.y-final_goal.position.y;

double d = sqrt((error_traj.position.x*error_traj.position.x) + (error_traj.position.y*error_traj.position.y));
double theta_tilde = ((atan(error_traj.position.y/error_traj.position.x)) - car_pose.theta);


// proposed control law
double v =  (-k_gain) * ((cos(car_pose.theta)* error_traj.position.x) + (sin(car_pose.theta) * error_traj.position.y));
double omega = (-k_gain) * ((-sin(car_pose.theta)* error_traj.position.x/l_1) + (cos(car_pose.theta) * error_traj.position.y)/l_1);

    control.angular.z= omega;
    control.linear.x= v;

return control;

}

// reaching the obstacle avoidance controller - SECOND CONTROLLER
void obstacle_avoidance_controller(custom_pose robot_pose,icarsCarControlWebsocket::SimRefs &msg_ctr, double &New_A_lc,double &New_B_lc,double &A_lc,double &B_lc)
{

//initializations
geometry_msgs::Pose closest_obstacle, diff_obs_goal,new_robot_pose, changednew_robot_pose;
custom_pose robot_trajectory,robot_trajectory_d;
Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
Eigen::Matrix4d updated_rot, TFLocal=Eigen::Matrix4d::Identity();
Eigen::Vector3d tr;
double k_p = 0.6;
double mou = 0.6;
// double C = 30.0;

 double distance_1 = nearestobstacle(p_space);
 double distance_2 = nearestobstacle(o_space);
 if (distance_1<distance_2){
 closest_obstacle=p_space;

// actual A_lc and B_lc values
A_lc = p_marker_size_A + margin + 2.588;
B_lc = p_marker_size_B + margin + 1.5110;

 }
 else{
closest_obstacle=o_space;

// actual A_lc and B_lc values
A_lc = o_marker_size_A + margin + 2.588;
B_lc = o_marker_size_B + margin + 1.5110;
}


// difference between obstacle and goal
diff_obs_goal.position.x = closest_obstacle.position.x - final_goal.position.x;
diff_obs_goal.position.y = closest_obstacle.position.y - final_goal.position.y;

double theta_diff_obs_goal = ((atan(diff_obs_goal.position.y/diff_obs_goal.position.x)));

// creating a specfic reference frame with obstacle as a origin facing towards the target
tf::TransformBroadcaster br;
tf::Transform transform;
transform.setOrigin(tf::Vector3(closest_obstacle.position.x,closest_obstacle.position.y,0.0));
tf::Quaternion q;
q.setRPY(0,0,theta_diff_obs_goal);
transform.setRotation(q);
br.sendTransform(tf::StampedTransform(transform,msg_pf_stat.header.stamp,"/FLUENCE/submap", "/obstacle_frame"));

// changing the transformations of car w.r.t to specific reference frame
 TransformationMatrix(closest_obstacle,robot_pose,rot,theta_diff_obs_goal);
 tr << closest_obstacle.position.x, closest_obstacle.position.y, 0;
 setTransform(rot,tr,TFLocal);
 updated_rot = TFLocal.inverse();

// new car position w.r.t new reference frame
new_robot_pose.position.x = (robot_pose.x * updated_rot(0,0)) + (robot_pose.y * updated_rot(0,1)) + (0 * updated_rot(0,2)) + (1 * updated_rot(0,3));
new_robot_pose.position.y = (robot_pose.x * updated_rot(1,0)) + (robot_pose.y * updated_rot(1,1)) + (0 * updated_rot(1,2)) + (1 * updated_rot(1,3));

changednew_robot_pose.position.x = new_robot_pose.position.y;
changednew_robot_pose.position.y = new_robot_pose.position.x;

// obtaining the new A_lc' and B_lc' values of the limit_cycle
if (changednew_robot_pose.position.x <= 0){
New_A_lc = A_lc - margin_1;
New_B_lc = B_lc - margin_1; 

// ROS_INFO("Attraction");
}
else{
New_A_lc = New_A_lc + margin_2;
New_B_lc = New_B_lc + margin_2; 

// ROS_INFO("Replusion");
}


// deriving the cartesian equation of ellipse (A,B,C,D,E,f)
double ellipse_angle = -45 * 3.14/180;
double A = (pow((New_A_lc),2) * pow(sin(ellipse_angle),2)) + (pow((New_B_lc),2) * pow(cos(ellipse_angle),2));
double B = 2 * (pow((New_B_lc),2) - pow((New_A_lc),2)) * (sin(ellipse_angle) * cos(ellipse_angle));
double C = (pow((New_A_lc),2) * pow(cos(ellipse_angle),2)) + (pow((New_B_lc),2) * pow(sin(ellipse_angle),2));
double D = (-2*(A * closest_obstacle.position.x))-(closest_obstacle.position.y * (B));
double E = (-(B * closest_obstacle.position.x))-(2 * closest_obstacle.position.y * (C));
double F = pow((A * closest_obstacle.position.x),2) + (closest_obstacle.position.x * closest_obstacle.position.y * (B)) + pow((C * closest_obstacle.position.y),2) - (pow((New_A_lc),2)*pow((New_B_lc),2));

double Ae = A/abs(F);
double Be = 2*B/abs(F);
double Ce = C/abs(F);

// calulcation of set points
if (changednew_robot_pose.position.y >= 0){

// clockwise trajectory motion


robot_trajectory.x = changednew_robot_pose.position.y + (mou * changednew_robot_pose.position.x) * (1-((pow(changednew_robot_pose.position.x,2)/pow(New_A_lc,2)))-((pow(changednew_robot_pose.position.y,2)/pow(New_B_lc,2)))-(C* changednew_robot_pose.position.x*changednew_robot_pose.position.y));

robot_trajectory.y = -changednew_robot_pose.position.x + (mou * changednew_robot_pose.position.y) * (1-((pow(changednew_robot_pose.position.x,2)/pow(New_A_lc,2)))-((pow(changednew_robot_pose.position.y,2)/pow(New_B_lc,2)))-(C* changednew_robot_pose.position.x*changednew_robot_pose.position.y));


/*
robot_trajectory.x = 1 * (changednew_robot_pose.position.y * Ce + 0.5 * Be * changednew_robot_pose.position.x) + mou * changednew_robot_pose.position.x * (1-(Ae * pow(changednew_robot_pose.position.x,2)))-(Be * changednew_robot_pose.position.x * changednew_robot_pose.position.y)-(Ce * pow(changednew_robot_pose.position.y,2));

robot_trajectory.y = -1 * (changednew_robot_pose.position.x * Ae + 0.5 * Be * changednew_robot_pose.position.y) + mou * changednew_robot_pose.position.y * (1-(Ae * pow(changednew_robot_pose.position.x,2)))-(Be * changednew_robot_pose.position.x * changednew_robot_pose.position.y)-(Ce * pow(changednew_robot_pose.position.y,2));
*/

//derivative of the previous trajectory
robot_trajectory_d.x = 1 * (robot_trajectory.y * Ce + 0.5 * Be * robot_trajectory.x) + mou * robot_trajectory.x * (1-(Ae * pow(changednew_robot_pose.position.x,2)))-(Be * changednew_robot_pose.position.x * changednew_robot_pose.position.y)-(Ce * pow(changednew_robot_pose.position.y,2)) + mou * changednew_robot_pose.position.x * (1-( 2* Ae * changednew_robot_pose.position.x * robot_trajectory.x))-(Be * changednew_robot_pose.position.x * robot_trajectory.y)- (Be * changednew_robot_pose.position.y * robot_trajectory.x) -(2 * Ce * changednew_robot_pose.position.y * robot_trajectory.y);

robot_trajectory_d.y = -1 * (robot_trajectory.x * Ae + 0.5 * Be * robot_trajectory.y) + mou * robot_trajectory.y * (1-(Ae * pow(changednew_robot_pose.position.x,2)))-(Be * changednew_robot_pose.position.x * changednew_robot_pose.position.y)-(Ce * pow(changednew_robot_pose.position.y,2)) + mou * changednew_robot_pose.position.x * (1-(2 * Ae * changednew_robot_pose.position.x * robot_trajectory.x))-(Be * changednew_robot_pose.position.x * robot_trajectory.y)- (Be * changednew_robot_pose.position.y * robot_trajectory.x) -(2 * Ce * changednew_robot_pose.position.y * robot_trajectory.y);

// ROS_INFO("clockwise");
}
else{

// counter clockwise trajectory motion

robot_trajectory.x = -changednew_robot_pose.position.y + (mou * changednew_robot_pose.position.x) * (1-((pow(changednew_robot_pose.position.x,2)/pow(New_A_lc,2)))-((pow(changednew_robot_pose.position.y,2)/pow(New_B_lc,2)))-(C* changednew_robot_pose.position.x*changednew_robot_pose.position.y));

robot_trajectory.y = changednew_robot_pose.position.x + (mou * changednew_robot_pose.position.y) * (1-((pow(changednew_robot_pose.position.x,2)/pow(New_A_lc,2)))-((pow(changednew_robot_pose.position.y,2)/pow(New_B_lc,2)))-(C* changednew_robot_pose.position.x*changednew_robot_pose.position.y));

/*
robot_trajectory.x = -1 * (changednew_robot_pose.position.y * Ce + 0.5 * Be * changednew_robot_pose.position.x) + mou * changednew_robot_pose.position.x * (1-(Ae * pow(changednew_robot_pose.position.x,2)))-(Be * changednew_robot_pose.position.x * changednew_robot_pose.position.y)-(Ce * pow(changednew_robot_pose.position.y,2));

robot_trajectory.y = 1 * (changednew_robot_pose.position.x * Ae + 0.5 * Be * changednew_robot_pose.position.y) + mou * changednew_robot_pose.position.y * (1-(Ae * pow(changednew_robot_pose.position.x,2)))-(Be * changednew_robot_pose.position.x * changednew_robot_pose.position.y)-(Ce * pow(changednew_robot_pose.position.y,2));
*/
//derivative of the previous trajectory
robot_trajectory_d.x = -1 * (robot_trajectory.y * Ce + 0.5 * Be * robot_trajectory.x) + mou * robot_trajectory.x * (1-(Ae * pow(changednew_robot_pose.position.x,2)))-(Be * changednew_robot_pose.position.x * changednew_robot_pose.position.y)-(Ce * pow(changednew_robot_pose.position.y,2)) + mou * changednew_robot_pose.position.x * (1-( 2* Ae * changednew_robot_pose.position.x * robot_trajectory.x))-(Be * changednew_robot_pose.position.x * robot_trajectory.y)- (Be * changednew_robot_pose.position.y * robot_trajectory.x) -(2 * Ce * changednew_robot_pose.position.y * robot_trajectory.y);

robot_trajectory_d.y = 1 * (robot_trajectory.x * Ae + 0.5 * Be * robot_trajectory.y) + mou * robot_trajectory.y * (1-(Ae * pow(changednew_robot_pose.position.x,2)))-(Be * changednew_robot_pose.position.x * changednew_robot_pose.position.y)-(Ce * pow(changednew_robot_pose.position.y,2)) + mou * changednew_robot_pose.position.x * (1-(2 * Ae * changednew_robot_pose.position.x * robot_trajectory.x))-(Be * changednew_robot_pose.position.x * robot_trajectory.y)- (Be * changednew_robot_pose.position.y * robot_trajectory.x) -(2 * Ce * changednew_robot_pose.position.y * robot_trajectory.y);

// ROS_INFO("counter clockwise");
}

// new car position w.r.t new reference frame
 // robot_trajectory.x = (robot_trajectory.x * TFLocal(0,0)) + (robot_trajectory.y * TFLocal(0,1)) + (0 * TFLocal(0,2)) + (1 * TFLocal(0,3));
 // robot_trajectory.y = (robot_trajectory.x * TFLocal(1,0)) + (robot_trajectory.y * TFLocal(1,1)) + (0 * TFLocal(1,2)) + (1 * TFLocal(1,3));

robot_trajectory.theta = atan(robot_trajectory.y/robot_trajectory.x);
robot_trajectory_d.theta = atan(robot_trajectory_d.y/robot_trajectory_d.x);

double theta_error = robot_trajectory.theta  - robot_pose.theta;
double linear_velocity = 0.000000000000000000000000000001;
// double d = sqrt(pow((robot_pose.x-final_goal.position.x),2) + pow((robot_pose.y-final_goal.position.y),2));

// double linear_velocity = linear_velocity_max * exp(-1/d) * cos(theta_error);
// double angular_velocity = (linear_velocity * sin(theta_error) / (d)) + k_p * theta_error;

// double angular_velocity = (((robot_trajectory.theta + timeStep)- (robot_trajectory.theta - timeStep))/2*timeStep) + k_p * theta_error;

// double dy = (((robot_trajectory.y + timeStep)- (robot_trajectory.y - timeStep))/2*timeStep);
// double dx = (((robot_trajectory.x + timeStep)- (robot_trajectory.x - timeStep))/2*timeStep);

double angular_velocity =  (robot_trajectory_d.theta)+ k_p * theta_error;
double steering_angle=angles::to_degrees(angular_velocity);

  // std::cout << "angular_velocity : " << angular_velocity << std::endl;
    msg_ctr.steerAng= angular_velocity;
    msg_ctr.brakeStrength=0.0;
    msg_ctr.gasForce=linear_velocity;

// return msg_ctr;
}

// MAIN FUNCTION BEGINS
int main(int argc, char** argv){
  
  ros::init(argc, argv, "social_navigation_control");  
  ros::NodeHandle n;

  int index;
  double duration;
  double timeStep;
  geometry_msgs::Twist corrected_control;
  custom_pose next_pose;
  double orientation_d;
  geometry_msgs::Pose error;
  std_msgs::Int8 switchcontroller;
  geometry_msgs::Pose reactive_values;

 
//Publishers
  ros::Publisher controlPublisher_car = n.advertise<icarsCarControlWebsocket::SimRefs>("/FLUENCE/sim/refs", 100);
  ros::Publisher goal_marker_pub = n.advertise<visualization_msgs::Marker>( "goal_marker_2", 0 );
  visualization_msgs::Marker gmarker;


  ros::Publisher error_pose = n.advertise<geometry_msgs::Pose>("error", 1);  
  ros::Publisher switchcontrollerPublisher = n.advertise<std_msgs::Int8>("switch_controller",1);
  ros::Publisher reactive = n.advertise<geometry_msgs::Pose>("/reactive", 1);  

//Subscribers
  ros::Subscriber odometrySubscriber = n.subscribe("/FLUENCE/odometry/attitude", 1,&odometryCallback);
  ros::Subscriber poseSubscriber = n.subscribe("/FLUENCE/map_manager/Status", 1, poseCallback);
  ros::Subscriber goalSubscriber = n.subscribe<icars_2d_map_manager::Path>("/FLUENCE/map_manager/Path", 1, &goalCallback);
  ros::Subscriber ogArraySubscriber = n.subscribe<riskrrt::OccupancyGridArray>("/ogarray", 1, &ogArrayCallback);

  ros::Subscriber vis_pub_p = n.subscribe<visualization_msgs::MarkerArray>("/human_marker_p", 1, &humanmarkerpCallback);
  ros::Subscriber vis_pub_o = n.subscribe<visualization_msgs::MarkerArray>("/human_marker_o", 1, &humanmarkeroCallback);

while(ros::ok()){

if(og_array.array.size() > 0)
{
  if(goal_received){

  final_goal = local_goal[50];
// Marker for goal
        gmarker.header.frame_id = "/FLUENCE/submap";
        gmarker.header.stamp = msg_pf_stat.header.stamp;
        gmarker.ns = "goal";
        gmarker.id = 1;
        gmarker.type = visualization_msgs::Marker::SPHERE;
        gmarker.action = visualization_msgs::Marker::ADD;
        gmarker.pose.position.x = final_goal.position.x;
        gmarker.pose.position.y = final_goal.position.y;
        gmarker.pose.position.z = 0.15;
        gmarker.scale.x = 1;
        gmarker.scale.y = 1;
        gmarker.scale.z = 1;
        gmarker.color.a = 1.0; // Don't forget to set the alpha!
        gmarker.color.r = 0.0;
        gmarker.color.g = 0.0;
        gmarker.color.b = 1.0;
        goal_marker_pub.publish(gmarker);

error.position.x = robot_pose.x - final_goal.position.x;
error.position.y = robot_pose.y - final_goal.position.y;
   
////////////////////////////////

// SWITCHING BETWEEN CONTROLLERS
int count = checkforobstacle(robot_pose,final_goal);
 // std::cout << "count:" << count << std::endl;

// std::cout << "robot_pose.x:" << robot_pose.x << "robot_pose.y:" << robot_pose.y << std::endl;

/*if (robot_pose.x <= (p_space.position.x) && robot_pose.y <= (p_space.position.y)){
count_1 =1;
 }*/

 switchcontroller.data = count;

if (count == 0){

// Attraction to the target controller
corrected_control = attractiontothecontroller(robot_pose,final_goal);

// 3 CASES FOR SENDING THE CONTROL COMMANDS TO THE CAR
 msg_ctr = control_commands(corrected_control);

// msg_ctr.brakeStrength = 50.0;
// ROS_INFO("sfqqqqqqqqqqv");
}
////////////////////////////////////
else{

/*
int count_main = count/2;
if (points[count_main].x - (robot_pose.x) < (2.5) && points[count_main].x - (robot_pose.x) > -(2.5)){
 if (points[count_main].y - (robot_pose.y) < (2.5) && points[count_main].y -(robot_pose.y) > -(2.5)){
    msg_ctr.steerAng=0.0;
    msg_ctr.brakeStrength=50.0;
    msg_ctr.gasForce=0.0;
ROS_INFO("Brakeeeeeeeee");
   }   
 } 
*/

// if (p_space.position.x - (robot_pose.x) < (25.0) && p_space.position.x - (robot_pose.x) > -(25.0)){
 //if (p_space.position.y - (robot_pose.y) < (25.0) && p_space.position.y - (robot_pose.y) > -(25.0)){
  
// Obstacle avoidance algorithm
 obstacle_avoidance_controller(robot_pose,msg_ctr,New_A_lc, New_B_lc, A_lc, B_lc);

/*
// introducing time instant for finding set-points
  Node* node;
  node = new Node[10];
  node[0].time = msg_pf_stat.header.stamp;
  node[0].pose = robot_pose;
  node[0].vel = msg_ctr;

for(int k=1;k<10;k++){

  next_pose = robotKinematic(next_pose, msg_ctr);
  msg_ctr = obstacle_avoidance_controller(next_pose,msg_ctr);

  node[k].time = node[k-1].time + ros::Duration(timeStep);
  node[k].pose = next_pose;
  node[k].vel = msg_ctr;
  
   } // end of for loop 

  } // end of else loop
 }*/


reactive_values.position.x = New_A_lc;
reactive_values.position.y = New_B_lc;

}// end of outer else loop
///////////////////////////////////////////////

/*
double k_p = 0.30;

// error with respect to x and y
error_traj.position.x= robot_pose.x - final_goal.position.x;
error_traj.position.y= robot_pose.y - final_goal.position.y;

double d = sqrt((error_traj.position.x*error_traj.position.x) + (error_traj.position.y*error_traj.position.y));
double theta_error =  orientation_d - robot_pose.theta;
double linear_velocity = linear_velocity_max * exp(-0.30/d) * cos(theta_error);
double angular_velocity = orientation_d + k_p * theta_error;

    msg_ctr_finale.steerAng=angular_velocity;
    msg_ctr_finale.brakeStrength=0.0;
    msg_ctr_finale.gasForce=1.0;
*/
    controlPublisher_car.publish(msg_ctr);
    error_pose.publish(error);
    reactive.publish(reactive_values);
    switchcontrollerPublisher.publish(switchcontroller);


    ros::spinOnce();
    } //check for goal_received
   } // for the og_array check
    ros::spinOnce();
  }// for the while loop
  return 0;
}// for the main
