#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <riskrrt/Trajectory.h>
#include "riskrrt/riskrrt.hpp"
#include <Eigen/Dense>
#include <nav_msgs/Path.h>

using namespace std;

 void fromEulerToRotationMatrixYXZ(double y,double p,double r, Eigen::Matrix3d &rot)
{
     tf::Matrix3x3 m;
     m.setRPY(p,r,y);

     rot(0,0)=double(m[0][0]);
     rot(0,1)=double(m[0][1]);
     rot(0,2)=double(m[0][2]);
     rot(0,2)=double(m[0][2]);
     rot(1,0)=double(m[1][0]);
     rot(1,1)=double(m[1][1]);
     rot(1,2)=double(m[1][2]);
     rot(2,0)=double(m[2][0]);
     rot(2,1)=double(m[2][1]);
     rot(2,2)=double(m[2][2]);

} 

void setTransform(Eigen::Matrix3d &rot, Eigen::Vector3d &t, Eigen::Matrix4d &Tf)
{

Tf.block(0,0,3,3) << rot;
Tf.block(0,3,3,1) << t;
Tf.block(3,0,1,4) << 0.0,0.0,0.0,1.0;

}

//gets the corresponding TF to a global position given by x,y and heading
void getTFFFromXYHd(double x,double y,double hd, Eigen::Matrix4d &TFPos)
{
Eigen::Matrix3d rot;
Eigen::Vector3d tr;

double pt,yw,rl;

tr(0)=x;
tr(1)=y;
tr(2)=0;

pt=0.0;
rl=0.0;
yw=(hd-M_PI/2.0);

fromEulerToRotationMatrixYXZ(yw,pt,rl,rot);
setTransform(rot,tr,TFPos);
}


int main(int argc, char **argv){
 
  //Initializing ROS node
  ros::init(argc, argv, "rrt");
  ros::NodeHandle n;
  Eigen::Matrix4d TFMapOrig, TFGlobal,TFLocal=Eigen::Matrix4d::Identity();
  Eigen::Matrix3d rot;
  Eigen::Vector3d tr;
  geometry_msgs::Pose error;


  //Reading parameters from yaml file (description in yaml file)
  Params params;
  n.param("timeStep", params.timeStep,0.5);
  n.param("maxDepth", params.maxDepth, 100);
  n.param("nv", params.nv,3);
  n.param("nphi", params.nphi,10);
  n.param("threshold", params.threshold,0.9);
  n.param("socialWeight", params.socialWeight, 10.0);
  n.param("rotationWeight", params.rotationWeight, 5.0);
  n.param("growTime", params.growTime, 0.5);
  n.param("bias", params.bias, 0.005);
  n.param("goalTh", params.goalTh,0.5);
  n.param("windowSize", params.windowSize, 15.0);
  n.param("robotLength", params.robotLength, 2.588);
  n.param("robotWidth", params.robotWidth, 1.5110);
  n.param("vMin", params.vMin, 0.0);
  n.param("vMax", params.vMax, 1.0);
  n.param("accMax", params.accMax, 1.0);
  n.param("omegaMax", params.omegaMax,5.0);
  n.param("accOmegaMax", params.accOmegaMax,20.0);
  n.param("maxSteeringAngle", params.maxSteeringAngle,30.0);

 //std::cout << params.socialWeight  << std::endl;
  
  //maxdepth is the maximum depth a node can have to grow, thus this node can have a son for which we will compute the risk and try to access an inexistent grid layer
  params.maxDepth = params.maxDepth - 1;
  
  //creating rrt 
  RRT* rrt;
  rrt = new RRT(params);
  
  //publishers
  ros::Publisher traj_pub = n.advertise<riskrrt::Trajectory>("traj", 10);
  ros::Publisher node_markers_pub = n.advertise<visualization_msgs::MarkerArray>("node_markers", 10);
  ros::Publisher path_markers_pub = n.advertise<visualization_msgs::MarkerArray>("path_markers", 10);
  ros::Publisher line_strip_pub = n.advertise<visualization_msgs::Marker>("line_strip", 10);
  
  ros::Publisher robot_marker_pub = n.advertise<visualization_msgs::Marker>( "robot_marker", 0 );
  visualization_msgs::Marker marker;
  ros::Publisher goal_marker_pub = n.advertise<visualization_msgs::Marker>( "goal_marker", 0 );
  visualization_msgs::Marker gmarker;

  ros::Publisher pub_pathMM = n.advertise<icars_2d_map_manager::Path>("trajectory/Path", 1);
  ros::Publisher error_pose = n.advertise<geometry_msgs::Pose>("error", 1);  


  //subscribers
  rrt->initOgArraySub();
  rrt->initOdomSub();
  rrt->initPoseSub();
  rrt->initGoalSub();
  rrt->initcontrollerFeedbackSub();

  rrt->initlookaheadSub();
  
  //timers
  ros::WallTime growing_start_time;
  ros::WallTime present;
  ros::WallDuration time_spent_growing;
  ros::WallDuration total_time_to_grow(params.growTime);
  
  //constants & variables
  double distance_to_goal_th = params.goalTh;
  int i;
  double xGlobal,yGlobal;
  
  //goal flag to start rrt
  rrt->goal_received = false;
  int j=50;


  while (ros::ok())
  {
    if(rrt->og_array.array.size() > 0)
   {
    if(rrt->goal_received)//TODO: always on rrt
    {

        // Sub_Goal point transformations
        fromEulerToRotationMatrixYXZ(rrt->local_goal[j].theta,0,0,rot);
        tr << rrt->local_goal[j].x, rrt->local_goal[j].y, 0;
        setTransform(rot,tr,TFLocal);
        getTFFFromXYHd(rrt->global_goal.x,rrt->global_goal.y,rrt->global_goal.theta,TFMapOrig);

        TFGlobal=TFMapOrig*TFLocal; //current global position using local position and the position of submap's origin

         rrt->final_goal.x=TFLocal(0,3);
         rrt->final_goal.y=TFLocal(1,3);
        // transformations ends here


        marker.header.frame_id = "/FLUENCE/submap";
        marker.header.stamp = rrt->msg_pf_stat.header.stamp;
        marker.ns = "robot";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = rrt->robot_pose.x;
        marker.pose.position.y = rrt->robot_pose.y;
        marker.pose.position.z = 0;
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(rrt->robot_pose.theta);
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 5.0;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        //marker.color.g = 1.0;
        //marker.color.b = 1.0;
        marker.mesh_use_embedded_materials = true;
        marker.mesh_resource = "package://riskrrt/meshes/fluence.dae";
        robot_marker_pub.publish(marker);

      //initializing rrt
      rrt->init();

      ROS_INFO("GOAL RECEIVED");
      
      while(!(rrt->isGoalReached()) && ros::ok())//while goal not reached
      {
      
        gmarker.header.frame_id = "/FLUENCE/submap";
        gmarker.header.stamp = rrt->msg_pf_stat.header.stamp;
        gmarker.ns = "goal";
        gmarker.id = 1;
        gmarker.type = visualization_msgs::Marker::SPHERE;
        gmarker.action = visualization_msgs::Marker::ADD;
        gmarker.pose.position.x = rrt->final_goal.x;
        gmarker.pose.position.y = rrt->final_goal.y;
        gmarker.pose.position.z = 0.15;
        gmarker.scale.x = 1;
        gmarker.scale.y = 1;
        gmarker.scale.z = 1;
        gmarker.color.a = 1.0; // Don't forget to set the alpha!
        gmarker.color.r = 0.0;
        gmarker.color.g = 0.0;
        gmarker.color.b = 1.0;
        goal_marker_pub.publish( gmarker );
        

error.position.x = rrt->root->pose.x - rrt->final_goal.x;
error.position.y = rrt->root->pose.y - rrt->final_goal.y;
// std::cout << "error x:" << error.x << std::endl;

// std::cout<<"root->pose.x:"<<rrt->root->pose.x<<"rrt->root->pose.y:"<<rrt->root->pose.y<<std::endl;
  /*      
      
        marker.header.frame_id = "/map";
        marker.header.stamp = rrt->msg_pf_stat.header.stamp;
        marker.ns = "robot";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = rrt->robot_pose.x;
        marker.pose.position.y = rrt->robot_pose.y;
        marker.pose.position.z = 0;
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(rrt->robot_pose.theta);
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 2.0;
        //marker.color.a = 1.0; // Don't forget to set the alpha!
        //marker.color.r = 1.0;
        //marker.color.g = 1.0;
        //marker.color.b = 1.0;
        marker.mesh_use_embedded_materials = true;
        marker.mesh_resource = "package://icars/fluence/car_description/meshes/velodyne.dae";
        robot_marker_pub.publish(marker); 
 */   
        
        
        growing_start_time = ros::WallTime::now();
        time_spent_growing = ros::WallTime::now() - growing_start_time;
        i=0;

         // ROS_INFO("hEY");
        while(time_spent_growing < total_time_to_grow && ros::ok())//while there is still some time left
        {
          //make the tree grow
          rrt->grow();
          i++;
          //update the timer to know how much time we have left to grow
          time_spent_growing = ros::WallTime::now() - growing_start_time;
          
        }//time is up, end of growing part
        
        ////find the best partial path toward the final goal
        //rrt->findPath();

        //robot is at the right place at the right time
        if(rrt->robot_on_traj){
          //update the tree with the latest map
          rrt->update();
          //ROS_INFO("robot_on_traj");
        }
        else{
          //robot failed to follow the trajectory, start over from current location
         rrt->init();
          ROS_INFO("REINIT");
        }
        
        //find the best partial path toward the final goal
        rrt->findPath();
          // rrt->root->pose=rrt->robot_pose;
////////////////////////////////////////////////////////
  icars_2d_map_manager::Path traj_pose1;

if(rrt->traj_msg.exists.data)
{
  traj_pose1.header = rrt->traj_msg.header;
  traj_pose1.header.stamp = rrt->msg_pf_stat.header.stamp;

  for (int i=0;i<int(rrt->traj_msg.poses.size());i++)
  {
      icars_2d_map_manager::PathPoint pt;
     geometry_msgs::PoseStamped path_pose; 

      pt.idx=i;
      pt.x=rrt->traj_msg.poses[i].pose.position.x;
      pt.y=rrt->traj_msg.poses[i].pose.position.y;
      pt.yaw=tf::getYaw(rrt->traj_msg.poses[i].pose.orientation);
     // pt.speed=pathPointList[i].speed;
     // pt.type=pathPointList[i].type;
      traj_pose1.points.push_back(pt);

  }
}
pub_pathMM.publish(traj_pose1);
error_pose.publish(error);

//////////////////////////////////////////////////////////////

        
        //publish the trajectory and markers
        traj_pub.publish(rrt->traj_msg);
        node_markers_pub.publish(rrt->node_markers);
        path_markers_pub.publish(rrt->path_markers);
        line_strip_pub.publish(rrt->line_strip);
        rrt->node_markers.markers.clear();
        rrt->path_markers.markers.clear();
        
        ros::spinOnce();
        
      }//goal reached
      
      //set the trajectory flag to false and publish it to stop controller
      rrt->traj_msg.exists.data = false;
      traj_pub.publish(rrt->traj_msg);
      ROS_INFO("GOAL REACHED");
      rrt->goal_received = false;

/* if(rrt->isGoalReached()){

rrt->final_goal.x = rrt->local_goal[40].x;
rrt->final_goal.y = rrt->local_goal[40].y;
 ROS_INFO("NEW_GOAL INITIATED");
  }
*/

    }
    ros::spinOnce();



   }
    ros::spinOnce();
    
  }//end ros::ok()

  return 0;
}
