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
#include <angles/angles.h>
#include <vector>

#include <icars_2d_map_manager/Status.h>  // Car pose
#include <icarsCarControlWebsocket/SimRefs.h>  // for getting the control
#include <icars_2d_map_manager/Path.h> // for obtaining trajectory as points
#include <riskrrt/OccupancyGridArray.h> //getting the occupancy grid data of the submap
#include <std_msgs/Int8.h>


using namespace std;


// Global Parameters
geometry_msgs::Pose robot_pose;
riskrrt::Trajectory trajectory;
icars_2d_map_manager::Path trajectoryPath;
icarsCarControlWebsocket::SimRefs msg_ctr;
int _nextWayPoint;
double _epsilon = 1e-6;
icars_2d_map_manager::Status msg_pf_stat;
double robotLength = 2.588;
double maxSteeringAngle = 30.0;
riskrrt::OccupancyGridArray og_array;//array of maps, time of map = timeStep * position in array.



    geometry_msgs::Pose getCurrentPose();
    /// Returns the lookahead distance for the given pose
    double getLookAheadDistance(const geometry_msgs::Pose& pose);
    /// Returns the lookahead angle for the given pose in [rad]
    double getLookAheadAngle(const geometry_msgs::Pose& pose);
    /// Returns the current lookahead distance threshold
    double getLookAheadThreshold();
    /// Returns the lookahead distance for the given pose
    double getArcDistance(const geometry_msgs::Pose& pose);
    /// Returns the next way point by linear search from the current waypoint
    int getNextWayPoint(int _nextWayPoint);    
    /// Returns the current closest waypoint
    int getClosestWayPoint();    


double getLookAheadThreshold()  {

geometry_msgs::Twist _currentVelocity;
double _lookAheadRatio = 3.0; // w.r.t. velocity
// std::cout << _currentVelocity.linear.x << std::endl;
// return _lookAheadRatio *_currentVelocity.linear.y;
return _lookAheadRatio;

  }

 geometry_msgs::Pose getCurrentPose() {
geometry_msgs::Pose pose;
    // pose.header = trajectoryPath.header;
    pose.position.x = robot_pose.position.x;
    pose.position.y = robot_pose.position.y;
    pose.position.z = 1e-8;
    pose.orientation = robot_pose.orientation;

    return pose;
  }

int getNextWayPoint(int _nextWayPoint,riskrrt::Trajectory trajectory,geometry_msgs::Pose estimated_pose) {   

 // std::cout << "_nextWayPoint : " << WayPoint << std::endl;

    if (!trajectory.poses.empty()) {

      if (_nextWayPoint >= 0) {

        geometry_msgs::Pose origin = getCurrentPose();

        tf::Vector3 v_1(origin.position.x,
                        origin.position.y,
                        origin.position.z);

      //  std::cout << "origin.pose.position.x :" << origin.pose.position.y << std::endl;

        double lookAheadThreshold = getLookAheadThreshold();

        for (int i = _nextWayPoint; i < trajectory.poses.size();++i) {                
          tf::Vector3 v_2(trajectory.poses[i].pose.position.x,
                          trajectory.poses[i].pose.position.y,
                          1e-8);
          
//  std::cout << "trajectoryPath.points y:" << trajectory.poses[i].pose.position.y << std::endl;

          if (tf::tfDistance(v_1, v_2) > lookAheadThreshold)            
            return i;
        }
        return _nextWayPoint;
      }
      else
        return 0;
    }

    return -1;
  }

  double getArcDistance(const geometry_msgs::Pose& pose) {
    double lookAheadDistance = getLookAheadDistance(pose);
    double lookAheadAngle = getLookAheadAngle(pose);

    if (std::abs(std::sin(lookAheadAngle)) >= _epsilon)
      return lookAheadDistance/sin(lookAheadAngle)*lookAheadAngle;
    else
      return lookAheadDistance;
  }

  int getClosestWayPoint(){
    if (!trajectory.poses.empty()) {
      int closestWaypoint = -1;
      double minDistance = -1.0;
      geometry_msgs::Pose conv_trajectory[trajectory.poses.size()];
      
      for (int i = 0; i < trajectory.poses.size(); ++i) {

      conv_trajectory[i]=trajectory.poses[i].pose;
     // conv_trajectory[i].header=trajectory.header;

        double distance = getArcDistance(conv_trajectory[i]);
        
        if ((minDistance < 0.0) || (distance < minDistance)) {
          closestWaypoint = i;
          minDistance = distance;
        }
      }
      
      return closestWaypoint;
    }

    return -1;
  }

//Pure Pursuit controller
//returns a corrected control from the robot theoretical, estimated pose and current speed
void Purepursuit(riskrrt::Trajectory trajectory,geometry_msgs::Pose theoretical_pose, geometry_msgs::Pose estimated_pose, geometry_msgs::Twist control,int *_nextWayPoint,  geometry_msgs::Twist *corrected_control_pp){

  double steering_angle;

  corrected_control_pp->linear.x = 1.0;

if (!trajectory.poses.empty()) {

int ClosestWayPoint = getClosestWayPoint();

// std::cout << "ClosestWayPoint:" << ClosestWayPoint << std::endl;

*_nextWayPoint= getNextWayPoint(ClosestWayPoint,trajectory,estimated_pose);

//  std::cout << "_nextWayPoint:" << *_nextWayPoint << std::endl;

  double lookAheadDistance = getLookAheadDistance(trajectory.poses[*_nextWayPoint].pose);
  double lookAheadAngle = getLookAheadAngle(trajectory.poses[*_nextWayPoint].pose);

  //  std::cout << "lookAheadDistance:" << lookAheadDistance << std::endl;
  // std::cout << "lookAheadAngle:" << lookAheadAngle << std::endl;

steering_angle= atan((2*robotLength*sin(lookAheadAngle))/lookAheadDistance);

steering_angle=angles::to_degrees(steering_angle);

 // std::cout << "steering_angle:" << steering_angle << std::endl;

if (steering_angle < maxSteeringAngle)
{

if(steering_angle > (-maxSteeringAngle))
{

 corrected_control_pp->angular.z = steering_angle;
 // ROS_INFO("Perfect!");
}

}

else
{
  corrected_control_pp->linear.x = 0.0;
  corrected_control_pp->angular.z = 0.0;

 ROS_INFO("Steering limit exceeded!");

    msg_ctr.steerAng=corrected_control_pp->angular.z;
    msg_ctr.brakeStrength= 50.0;
    msg_ctr.gasForce=corrected_control_pp->linear.x;
    
}


//return corrected_control_pp;
}

}

double getLookAheadDistance(const
      geometry_msgs::Pose& pose) {
    geometry_msgs::Pose origin = getCurrentPose();

    tf::Vector3 v1(origin.position.x,
                   origin.position.y,
                   origin.position.z);
    tf::Vector3 v2(pose.position.x,
                   pose.position.y,
                   pose.position.z);
    
    return tf::tfDistance(v1, v2);
  }
  
  double getLookAheadAngle(const
    geometry_msgs::Pose& pose) {
    geometry_msgs::Pose origin = getCurrentPose();

double alpha = (tf::getYaw(pose.orientation)-(tf::getYaw(robot_pose.orientation)+1.57));

  // std::cout << "alpha:" << alpha << std::endl;
    
  // std::cout << "tf::getYaw(pose.orientation):" << tf::getYaw(pose.orientation) << "tf::getYaw(robot_pose.orientation):" << tf::getYaw(robot_pose.orientation) << std::endl;

    return alpha;
  }


geometry_msgs::Twist brake(){
  
  geometry_msgs::Twist corrected_control_pp;
  
  corrected_control_pp.linear.x = 0.0;
  corrected_control_pp.angular.z = 0.0;

    msg_ctr.steerAng=corrected_control_pp.angular.z;
    msg_ctr.brakeStrength= 50.0;
    msg_ctr.gasForce=corrected_control_pp.linear.x;
    
	return corrected_control_pp;
};


//get the pose of the new node for a differential robot
geometry_msgs::Pose robotKinematic(geometry_msgs::Pose pose, geometry_msgs::Twist control, double duration){
  double rotation_radius;
  double delta_theta, delta_x, delta_y;
  double delta_xc, delta_yc;

//paramteres definition for odometry calculation of CLMR
  double delta_r, delta_ori,    Nonholonomicconstraint_1,Nonholonomicconstraint_2,rotation_radius_curvature, curvature;
  geometry_msgs::Pose new_pose_CLMR;
  
  if(control.linear.x == 0.0){
   // delta_theta = control.twist.angular.z * params.timeStep;
   // delta_x = 0.0;
   // delta_y = 0.0;
   // delta_ori = control.twist.angular.z * params.timeStep;

    delta_r = control.linear.x;
    rotation_radius_curvature =robotLength/(tan(control.angular.z* duration));
    delta_ori = (delta_r)/rotation_radius_curvature;
	}

   else if(control.angular.z == 0.0){
   // delta_theta = 0.0;
   // delta_x = control.twist.linear.x * params.timeStep;
   // delta_y = 0.0;

    delta_r = control.linear.x * duration;
    rotation_radius_curvature = 0.0;
    delta_ori = 0.0;

 // ROS_INFO("i am here in 2nd");
  }

  else{
   // rotation_radius = control.twist.linear.x / control.twist.angular.z;
   // delta_theta = control.twist.angular.z * params.timeStep;
   // delta_x = rotation_radius * sin(delta_theta);
   // delta_y = rotation_radius * (1.0 - cos(delta_theta));

   //odometry calculation for CLMR

    rotation_radius_curvature=robotLength/(tan(control.angular.z* duration));
    delta_r = (control.linear.x * (duration));
    delta_ori = (delta_r)/rotation_radius_curvature;
   
// curvature=(tan(control.twist.angular.z* params.timeStep))/params.robotLength;

// std::cout << "rotation_radius: " << rotation_radius_curvature << std::endl;
// std::cout << "curvature: " << curvature << std::endl;
 // ROS_INFO("i am here in 3rd");

  }

 // new_pose.x = pose.x + (delta_x * cos(pose.theta) - delta_y * sin(pose.theta));
 // new_pose.y = pose.y + (delta_x * sin(pose.theta) + delta_y * cos(pose.theta));
 // new_pose.theta = atan2(sin(pose.theta + delta_theta), cos(pose.theta + delta_theta));

   delta_x = (delta_r * cos(tf::getYaw(pose.orientation) + (delta_ori* duration)));
   delta_y = (delta_r * sin(tf::getYaw(pose.orientation) + (delta_ori* duration)));

   //  delta_xc = pose.x - rotation_radius_curvature*sin(pose.theta);
   //  delta_yc = pose.y + rotation_radius_curvature*cos(pose.theta);

    //new pose prediction for CLMR
     new_pose_CLMR.position.x = pose.position.x + delta_x;
     new_pose_CLMR.position.y= pose.position.y + delta_y;
     new_pose_CLMR.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(pose.orientation) + delta_ori* duration);
/*
if(delta_ori != 0.0)
{
     new_pose_CLMR.x = delta_xc + rotation_radius_curvature*sin(pose.theta+delta_ori);
     new_pose_CLMR.y = delta_xc - rotation_radius_curvature*cos(pose.theta+delta_ori);
     new_pose_CLMR.theta = pose.theta + delta_ori;
}

else
{
     new_pose_CLMR.x = delta_xc + delta_r*cos(pose.theta);
     new_pose_CLMR.y = delta_xc + delta_r*sin(pose.theta);
     new_pose_CLMR.theta = pose.theta + delta_ori;
}
 // new_pose_CLMR.x = pose.x -rotation_radius_curvature + rotation_radius_curvature*cos(pose.theta + (control.twist.angular.z* params.timeStep));
 // new_pose_CLMR.y = pose.y + rotation_radius_curvature*sin(pose.theta + (control.twist.angular.z* params.timeStep));

*/
    //  new_pose_CLMR.theta = pose.theta + delta_ori;

// Nonholonomicconstraint_1 = ((-delta_x*sin(new_pose_CLMR.theta))+(delta_y*cos(new_pose_CLMR.theta)));
  
/*Nonholonomicconstraint_2 = ((delta_x*delta_x) + (delta_y*delta_y));

 // std::cout << "Non-holonomic constraint 2 : " << Nonholonomicconstraint_2 << std::endl;
 // std::cout << "right side : " << ((rotation_radius_curvature*rotation_radius_curvature)* (delta_ori*delta_ori)) << std::endl;

if(Nonholonomicconstraint_1 < 0.1)
{
 if((Nonholonomicconstraint_2 >= ((rotation_radius_curvature*rotation_radius_curvature)* (delta_ori*delta_ori))))
{
  return new_pose_CLMR;
} */


  return new_pose_CLMR;
}


//get the pose of the new node for a differential robot
geometry_msgs::Pose robotKinematic1(geometry_msgs::Pose pose, geometry_msgs::Twist control){
  double rotation_radius;
  double delta_theta, delta_x, delta_y;
  double delta_xc, delta_yc;

//paramteres definition for odometry calculation of CLMR
  double delta_r, delta_ori,   Nonholonomicconstraint_1,Nonholonomicconstraint_2,rotation_radius_curvature, curvature;
  geometry_msgs::Pose new_pose_CLMR;
  
  if(control.linear.x == 0.0){
    delta_r = control.linear.x;
    rotation_radius_curvature =robotLength/(tan(control.angular.z));
    delta_ori = (delta_r)/rotation_radius_curvature;
	}

   else if(control.angular.z == 0.0){
    delta_r = control.linear.x;
    rotation_radius_curvature = 0.0;
    delta_ori = 0.0;

 // ROS_INFO("i am here in 2nd");
  }

  else{
   //odometry calculation for CLMR

    rotation_radius_curvature=robotLength/(tan(control.angular.z));
    delta_r = (control.linear.x);
    delta_ori = (delta_r)/rotation_radius_curvature;
   
// curvature=(tan(control.twist.angular.z* params.timeStep))/params.robotLength;

 // std::cout << " delta_ori: " <<  delta_ori << std::endl;
// std::cout << "curvature: " << curvature << std::endl;
 // ROS_INFO("i am here in 3rd");

  }

 // new_pose.x = pose.x + (delta_x * cos(pose.theta) - delta_y * sin(pose.theta));
 // new_pose.y = pose.y + (delta_x * sin(pose.theta) + delta_y * cos(pose.theta));
 // new_pose.theta = atan2(sin(pose.theta + delta_theta), cos(pose.theta + delta_theta));

   delta_x = (delta_r * cos(tf::getYaw(pose.orientation) + (delta_ori)));
   delta_y = (delta_r * sin(tf::getYaw(pose.orientation) + (delta_ori)));

   //  delta_xc = pose.x - rotation_radius_curvature*sin(pose.theta);
   //  delta_yc = pose.y + rotation_radius_curvature*cos(pose.theta);

  // std::cout << " delta_x: " <<  delta_x << " delta_y: " <<  delta_y << std::endl;

    //new pose prediction for CLMR
     new_pose_CLMR.position.x = pose.position.x + delta_x;
     new_pose_CLMR.position.y= pose.position.y + delta_y;
     new_pose_CLMR.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(pose.orientation) + delta_ori);

  return new_pose_CLMR;
}


//returns the index of the last visited node in the trajectory
int interpolation(riskrrt::Trajectory trajectory, double timeStep){
  
  int index;
  ros::Time present;
  
  present = msg_pf_stat.header.stamp;
  
  //there is only root in the trajectory, wait for an actual trajectory
	if(trajectory.poses.size() <= 1){
    index = -1;
	}
  //trajectory hasn't started yet (root time is in the future)
	else if(trajectory.poses.front().time > present){
    index = -1;
	}
  //trajectory has already ended (last node of trajectory is in the past)
	else if(trajectory.poses.back().time < present){
		index = -1;
	}
  else{
    index = floor((present - trajectory.poses.front().time).toSec() / timeStep);
  }
    
	return index;
}

void trajectoryCallback(const riskrrt::Trajectory::ConstPtr& msg){
  trajectory = *msg;
}

// for getting odometry data 
void odometryCallback(const nav_msgs::Odometry& msg) 
  {
    geometry_msgs::Twist _currentVelocity;
    _currentVelocity = msg.twist.twist;
  }

void poseCallback(const icars_2d_map_manager::Status msg){
  robot_pose.position.x = msg.xLocal;
  robot_pose.position.y = msg.yLocal;
  robot_pose.orientation = tf::createQuaternionMsgFromYaw(msg.yaw);

// std::cout << "w:"<<  robot_pose.orientation.w << "x:"<< robot_pose.orientation.x << "y:"<< robot_pose.orientation.y << "z:"<< robot_pose.orientation.z << std::endl;

  msg_pf_stat.header=msg.header;
}


void trajectoryPathCallback(const icars_2d_map_manager::Path msg){
  trajectoryPath = msg;
}


  void ogArrayCallback(const riskrrt::OccupancyGridArray msg){
  og_array = msg;
}

int main(int argc, char** argv){
  
  ros::init(argc, argv, "riskrrt_controller_pp");
  
  ros::NodeHandle n;
  
  geometry_msgs::Pose theoretical_pose,new_pose;
  geometry_msgs::Twist raw_control;
  int index;
  double duration,duration_1;
  double timeStep;
  double  robotLength;
  double maxSteeringAngle;
  geometry_msgs::Twist corrected_control;
  geometry_msgs::Twist corrected_control_pp;
  std_msgs::Bool controller_feedback;
  std_msgs::Int8 lookaheadpoint;
  int waypoint=-1;
  
  n.param("timeStep", timeStep, 0.5);
  n.param("robotLength", robotLength, 2.588);


//Publishers
  ros::Publisher controlPublisher_car = n.advertise<icarsCarControlWebsocket::SimRefs>("/FLUENCE/sim/refs", 100);
 // ros::Publisher controlPublisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
  //publish a msg to tell the rrt if the robot is too far from the trajectory
  ros::Publisher controllerFeedbackPublisher = n.advertise<std_msgs::Bool>("controller_feedback",1);

//send the lookaheadpoint    
ros::Publisher lookaheadpointPublisher = n.advertise<std_msgs::Int8>("lookaheadpoint_feedback",1);
  
// Subscribers
  ros::Subscriber trajectorySubscriber = n.subscribe("/traj", 1, trajectoryCallback);
  ros::Subscriber poseSubscriber = n.subscribe("/FLUENCE/map_manager/Status", 1, poseCallback);
  ros::Subscriber trajectoryPathSubscriber = n.subscribe<icars_2d_map_manager::Path>("/trajectory/Path", 1, trajectoryPathCallback);
  ros::Subscriber _odometrySubscriber = n.subscribe("/FLUENCE/odometry/attitude", 1,&odometryCallback);
  ros::Subscriber ogArraySubscriber = n.subscribe<riskrrt::OccupancyGridArray>("/ogarray", 1,ogArrayCallback);

  ros::Rate loop_rate(20.0);
  trajectory.exists.data = false;
  
  while(ros::ok()){
    
    ros::spinOnce();
    
    if(trajectory.exists.data){
      
      //what is the last node of the trajectory the robot visited
      index = interpolation(trajectory, timeStep);
      
      if(index != -1){
        
        //time difference between present time and timestamp of the last visited node (s)
        duration = (msg_pf_stat.header.stamp - trajectory.poses[index].time).toSec();
        //where should the robot be on the trajectory at present time
        theoretical_pose = robotKinematic(trajectory.poses[index].pose, trajectory.poses[index+1].twist, duration);
        //compute corrected control
        // corrected_control = kanayama(theoretical_pose, robot_pose, trajectory.poses[index+1].twist);
        
Purepursuit(trajectory,theoretical_pose, robot_pose, trajectory.poses[index+1].twist, &waypoint,&corrected_control_pp);
        
// std::cout<<"_nextWayPoint:"<<waypoint<<std::endl;

//check the difference between the estimated robot pose and the theoretical pose
    double pose_error = sqrt(pow(trajectory.poses[waypoint].pose.position.x - robot_pose.position.x, 2) + pow(trajectory.poses[waypoint].pose.position.y - robot_pose.position.y, 2));

 new_pose = robotKinematic1(robot_pose, corrected_control_pp);

 // std::cout<<"new_pose.x:"<<new_pose.position.x <<"new_pose.y:"<<new_pose.position.y<<std::endl;

double new_pose_error = sqrt(pow(trajectory.poses[waypoint].pose.position.x - new_pose.position.x, 2) + pow(trajectory.poses[waypoint].pose.position.y - new_pose.position.y, 2));

double lookAheadThreshold_1 = getLookAheadThreshold();

 controller_feedback.data = (pose_error < (lookAheadThreshold_1+0.5));
 lookaheadpoint.data = waypoint;


if(pose_error > (lookAheadThreshold_1+1.0))
{
        corrected_control_pp = brake();
}

else{
    msg_ctr.steerAng=corrected_control_pp.angular.z;
    msg_ctr.brakeStrength=0.0;
    msg_ctr.gasForce=corrected_control_pp.linear.x;


}

        
      }
      else{
       // corrected_control = brake();
        corrected_control_pp = brake();
        controller_feedback.data = true;
      }
      
    }
    else{
     // corrected_control = brake();
        corrected_control_pp = brake();
      controller_feedback.data = true;
    }
    
    //publish the corrected control
  //  controlPublisher.publish(corrected_control);
    //publish a message to tell the rrt if the robot position is ok
    controllerFeedbackPublisher.publish(controller_feedback);
    lookaheadpointPublisher.publish(lookaheadpoint);

    controlPublisher_car.publish(msg_ctr);
    
    loop_rate.sleep();
  }

  return 0;
}
