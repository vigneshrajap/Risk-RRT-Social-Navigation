#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h> // OG map from "submap" topic
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <riskrrt/OccupancyGridArray.h>
#include <riskrrt/PoseTwistStamped.h>
#include <riskrrt/Trajectory.h>
#include <vector>
#include <iostream>
#include <tf/tf.h>
#include <algorithm>
#include <visualization_msgs/MarkerArray.h>
#include <icars_object_ekf/Object.h>  // for pedestrain pose
#include <Eigen/Dense>  // for eigen transformations
//#include "riskrrt/riskrrt.hpp"  // for getting marker stamp alone


using namespace std;

struct custom_pose{//ros pose msgs use quaternion and the function provided by the tf library to get yaw from quaternion is terrible, hence this.
  double x;//in meters
  double y;//in meters
  double theta;//in radians (-PI, PI)
};


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



nav_msgs::OccupancyGrid grid;
//nav_msgs::OccupancyGrid modified_grid;
riskrrt::OccupancyGridArray og_array;
int nbMap;
double timeStep;
double arg_vel;
double arg_vel_p;
double arg_vel_o;
int ped_num;
custom_pose temp_pose, ped_pose, obstacle, temp_pose_1,ped_pose_1,ped_pose_2,ped_pose_3;

visualization_msgs::MarkerArray marker_array;
visualization_msgs::Marker marker;

visualization_msgs::MarkerArray marker_array1;
visualization_msgs::Marker marker1;

visualization_msgs::MarkerArray marker_array2;
visualization_msgs::Marker marker2;

visualization_msgs::MarkerArray p_marker_array;
visualization_msgs::Marker p_marker;

visualization_msgs::MarkerArray o_marker_array;
visualization_msgs::Marker o_marker;

visualization_msgs::MarkerArray ellipse_o_marker_array; // ellipse shape around o_space_obstacle
visualization_msgs::Marker ellipse_o_marker;

visualization_msgs::MarkerArray ellipse_p_marker_array; // ellipse shape around p_space_obstacle
visualization_msgs::Marker ellipse_p_marker;

int gridIFromPose(custom_pose pose){
  return (int)round((pose.x - grid.info.origin.position.x) / grid.info.resolution);
 // std::cout << grid.info.resolution << std::endl;
}

int gridJFromPose(custom_pose pose){
  return (int)round((pose.y - grid.info.origin.position.y) / grid.info.resolution);
}

int gridIndexFromCoord(int i, int j){
  return i + grid.info.width * j;
}

int gridIFromIndex(int index){
  return  index % grid.info.width;
}

int gridJFromIndex(int index){
  return floor(index / grid.info.width);
}

custom_pose poseFromGridCoord(int i, int j){
  custom_pose pose;
  pose.x = grid.info.resolution * i + grid.info.origin.position.x;
	pose.y = grid.info.resolution * j + grid.info.origin.position.y;
  return pose;
}

int gridIndexFromPose(custom_pose pose){
  int index, i, j;
  i = gridIFromPose(pose);
  j = gridJFromPose(pose);
  index = gridIndexFromCoord(i, j);
  return index;
}

custom_pose poseFromGridIndex(int index){//not really a pose but rather a point
  int i, j;
  custom_pose pose;
  
  i = gridIFromIndex(index);
  j = gridJFromIndex(index);
  pose = poseFromGridCoord(i, j);
  
  return pose;
}

void OGCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) // Building the grid from occupancy map
{


  grid = *msg;
// std::cout << grid.info.resolution << std::endl;
  int i;
  og_array.array.clear();
  for(i=0;i<nbMap;i++){
    og_array.array.push_back(grid);
  }


}


void pedposeCallback(const icars_object_ekf::Object msg)  // For updating the pedestrain pose in the OG map
{

  int i, j, k;
  int ped_grid_i;
  int ped_grid_j;

  int max_i, min_i;
  int max_j, min_j;

  int ped_grid_i_1;
  int ped_grid_j_1;

  int max_i_1, min_i_1;
  int max_j_1, min_j_1;

  Eigen::Matrix4d TFMapOrig, TFGlobal,TFLocal=Eigen::Matrix4d::Identity();
  Eigen::Matrix3d rot;
  Eigen::Vector3d tr;

// if(msg.size()>0)
// { 

   ped_pose.x = msg.x;
   ped_pose.x = msg.y;
   ped_pose.theta = msg.ori;
   arg_vel=msg.speed;

   ped_pose_1.x = - 0.1;
   ped_pose_1.y = 32;
   ped_pose_1.theta = 1.57;

   ped_pose_2.x = -1.055;
   ped_pose_2.y = 48;
   ped_pose_2.theta = 150.57;

   ped_pose_3.x = -1.050;
   ped_pose_3.y = 46;
   ped_pose_3.theta = 40.57;

   arg_vel_p=0;
   arg_vel_o=0;

        // Sub_Goal point transformations
        fromEulerToRotationMatrixYXZ(msg.ori,0,0,rot);
        tr << msg.x, msg.y, 0;
        setTransform(rot,tr,TFGlobal);
        getTFFFromXYHd(obstacle.x,obstacle.y,obstacle.theta,TFMapOrig);

        TFLocal=TFGlobal*(TFMapOrig.inverse()); //current global position using local position and the position of submap's origin

      // ped_pose.x=TFLocal(0,3);
      // ped_pose.y=TFLocal(1,3);
      // ped_pose.theta = TFLocal(1,2);
      // ped_pose.x=0.525;
      // ped_pose.y=57;
      // ped_pose.theta = TFLocal(1,2);
        // transformations ends here

 // std::cout<<"actual"<<msg.x<<"global"<<TFGlobal(0,3)<<"local"<<TFLocal(0,3)<<std::endl;


  marker.header.frame_id = "/FLUENCE/submap";
  marker.header.stamp = ros::Time();
  marker.ns = "humans";
  marker.id = rand();
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = ped_pose_1.x;
  marker.pose.position.y = ped_pose_1.y;
  marker.pose.position.z = 0;
  //marker.pose.orientation = msg->pose.pose.orientation;
  marker.pose.orientation = tf::createQuaternionMsgFromYaw(ped_pose_1.theta);
  marker.scale.x = 0.025;
  marker.scale.y = 0.025;
  marker.scale.z = 0.025;
  //marker.color.a = 1.0; // Don't forget to set the alpha!
  //marker.color.r = 1.0;
  //marker.color.g = 1.0;
  //marker.color.b = 1.0;
  marker.lifetime = ros::Duration(0.125);
  marker.mesh_use_embedded_materials = true;
  marker.mesh_resource = "package://riskrrt/meshes/female2/models/female2.dae";
  marker_array.markers.push_back(marker);

  marker1.header.frame_id = "/FLUENCE/submap";
  marker1.header.stamp = ros::Time();
  marker1.ns = "humans1";
  marker1.id = rand();
  marker1.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker1.action = visualization_msgs::Marker::ADD;
  marker1.pose.position.x = ped_pose_2.x;
  marker1.pose.position.y = ped_pose_2.y;
  marker1.pose.position.z = 0;
  //marker.pose.orientation = msg->pose.pose.orientation;
  marker1.pose.orientation = tf::createQuaternionMsgFromYaw(ped_pose_2.theta);
  marker1.scale.x = 0.025;
  marker1.scale.y = 0.025;
  marker1.scale.z = 0.025;
  //marker.color.a = 1.0; // Don't forget to set the alpha!
  //marker.color.r = 1.0;
  //marker.color.g = 1.0;
  //marker.color.b = 1.0;
  marker1.lifetime = ros::Duration(0.125);
  marker1.mesh_use_embedded_materials = true;
  marker1.mesh_resource = "package://riskrrt/meshes/female2/models/female2.dae";
  marker_array1.markers.push_back(marker1);

  marker2.header.frame_id = "/FLUENCE/submap";
  marker2.header.stamp = ros::Time();
  marker2.ns = "humans2";
  marker2.id = rand();
  marker2.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker2.action = visualization_msgs::Marker::ADD;
  marker2.pose.position.x = ped_pose_3.x;
  marker2.pose.position.y = ped_pose_3.y;
  marker2.pose.position.z = 0;
  //marker.pose.orientation = msg->pose.pose.orientation;
  marker2.pose.orientation = tf::createQuaternionMsgFromYaw(ped_pose_3.theta);
  marker2.scale.x = 0.025;
  marker2.scale.y = 0.025;
  marker2.scale.z = 0.025;
  //marker.color.a = 1.0; // Don't forget to set the alpha!
  //marker.color.r = 1.0;
  //marker.color.g = 1.0;
  //marker.color.b = 1.0;
  marker2.lifetime = ros::Duration(0.125);
  marker2.mesh_use_embedded_materials = true;
  marker2.mesh_resource = "package://riskrrt/meshes/female2/models/female2.dae";
  marker_array2.markers.push_back(marker2);

  p_marker.header.frame_id = "/FLUENCE/submap";
  p_marker.header.stamp = ros::Time();
  p_marker.ns = "humans_p";
  p_marker.id = rand();
  p_marker.type = visualization_msgs::Marker::CYLINDER;
  p_marker.action = visualization_msgs::Marker::ADD;
  p_marker.pose.position.x = ped_pose_1.x;
  p_marker.pose.position.y = ped_pose_1.y;
  p_marker.pose.position.z = 0;
  //marker.pose.orientation = msg->pose.pose.orientation;
  p_marker.pose.orientation = tf::createQuaternionMsgFromYaw(ped_pose_1.theta);
  p_marker.scale.x = 2.5;
  p_marker.scale.y = 2;
  p_marker.scale.z = 0.2;
  p_marker.color.a = 1.0; // Don't forget to set the alpha!
  //marker.color.r = 1.0;
  //marker.color.g = 1.0;
  p_marker.color.b = 0.2;
  p_marker.lifetime = ros::Duration(0.125);
  //p_marker.mesh_use_embedded_materials = true;
  //p_marker.mesh_resource = "package://riskrrt/meshes/female2/models/female2.dae";
  p_marker_array.markers.push_back(p_marker);

  o_marker.header.frame_id = "/FLUENCE/submap";
  o_marker.header.stamp = ros::Time();
  o_marker.ns = "humans_p";
  o_marker.id = rand();
  o_marker.type = visualization_msgs::Marker::CYLINDER;
  o_marker.action = visualization_msgs::Marker::ADD;
  o_marker.pose.position.x = (ped_pose_2.x+ped_pose_3.x)/2;
  o_marker.pose.position.y = (ped_pose_2.y+ped_pose_3.y)/2;
  o_marker.pose.position.z = 0;
  //marker.pose.orientation = msg->pose.pose.orientation;
  o_marker.pose.orientation = tf::createQuaternionMsgFromYaw((ped_pose_2.theta+ped_pose_3.theta)/2);
  o_marker.scale.x = 4;
  o_marker.scale.y = 2.5;
  o_marker.scale.z = 0.2;
  o_marker.color.a = 1.0; // Don't forget to set the alpha!
  //marker.color.r = 1.0;
  o_marker.color.g = 1.0;
  o_marker.color.b = 0.2;
  o_marker.lifetime = ros::Duration(0.125);
  //p_marker.mesh_use_embedded_materials = true;
  //p_marker.mesh_resource = "package://riskrrt/meshes/female2/models/female2.dae";
  o_marker_array.markers.push_back(o_marker);


  ellipse_p_marker.header.frame_id = "/FLUENCE/submap";
  ellipse_p_marker.header.stamp = ros::Time();
  ellipse_p_marker.ns = "humans_p";
  ellipse_p_marker.id = rand();
  ellipse_p_marker.type = visualization_msgs::Marker::CYLINDER;
  ellipse_p_marker.action = visualization_msgs::Marker::ADD;
  ellipse_p_marker.pose.position.x = ped_pose_1.x;
  ellipse_p_marker.pose.position.y = ped_pose_1.y;
  ellipse_p_marker.pose.position.z = 0;
  //marker.pose.orientation = msg->pose.pose.orientation;
  ellipse_p_marker.pose.orientation = tf::createQuaternionMsgFromYaw(ped_pose_1.theta);
  ellipse_p_marker.scale.x = 6.0;
  ellipse_p_marker.scale.y = 5.0;
  ellipse_p_marker.scale.z = 0.2;
  ellipse_p_marker.color.a = 0.3; // Don't forget to set the alpha!
  //marker.color.r = 1.0;
  ellipse_p_marker.color.g = 0.2;
  ellipse_p_marker.color.b = 0.2;
  ellipse_p_marker.lifetime = ros::Duration(0.125);
  //p_marker.mesh_use_embedded_materials = true;
  //p_marker.mesh_resource = "package://riskrrt/meshes/female2/models/female2.dae";
  ellipse_p_marker_array.markers.push_back(ellipse_p_marker);


  ellipse_o_marker.header.frame_id = "/FLUENCE/submap";
  ellipse_o_marker.header.stamp = ros::Time();
  ellipse_o_marker.ns = "humans_p";
  ellipse_o_marker.id = rand();
  ellipse_o_marker.type = visualization_msgs::Marker::CYLINDER;
  ellipse_o_marker.action = visualization_msgs::Marker::ADD;
  ellipse_o_marker.pose.position.x = (ped_pose_2.x+ped_pose_3.x)/2;
  ellipse_o_marker.pose.position.y = (ped_pose_2.y+ped_pose_3.y)/2;
  ellipse_o_marker.pose.position.z = 0;
  //marker.pose.orientation = msg->pose.pose.orientation;
  ellipse_o_marker.pose.orientation = tf::createQuaternionMsgFromYaw((ped_pose_2.theta+ped_pose_3.theta)/2);
  ellipse_o_marker.scale.x = 7;
  ellipse_o_marker.scale.y = 5.5;
  ellipse_o_marker.scale.z = 0.2;
  ellipse_o_marker.color.a = 0.3; // Don't forget to set the alpha!
  //marker.color.r = 1.0;
  ellipse_o_marker.color.g = 0.2;
  ellipse_o_marker.color.b = 0.2;
  ellipse_o_marker.lifetime = ros::Duration(0.125);
  //p_marker.mesh_use_embedded_materials = true;
  //p_marker.mesh_resource = "package://riskrrt/meshes/female2/models/female2.dae";
  ellipse_o_marker_array.markers.push_back(ellipse_o_marker);

for(k=0;k<nbMap;k++){
    
    temp_pose.x = ped_pose.x + k*timeStep*arg_vel*cos(ped_pose.theta);
    temp_pose.y = ped_pose.y + k*timeStep*arg_vel*sin(ped_pose.theta);
    
   // temp_pose.x =  0.01;
   // temp_pose.y =   65;

    ped_grid_i = gridIFromPose(temp_pose);
    ped_grid_j = gridJFromPose(temp_pose);
    
// std::cout << "temp pose x:" << temp_pose.x << " y:" << temp_pose.y << std::endl;
 // std::cout << " i:" << ped_grid_i << " j:" << ped_grid_j << std::endl;
    min_i = max(ped_grid_i - 5, 0);
    max_i = min(ped_grid_i + 5, (int)grid.info.width);
    min_j = max(ped_grid_j - 5, 0);
    max_j = min(ped_grid_j + 5, (int)grid.info.height);
    // std::cout << "min_i:"<< min_i << " max_i:"<<max_i << " min_j:" << min_j << " min_j:"<< max_j << std::endl;
   //  std::cout << gridIndexFromCoord(min_i,min_j) << std::endl;
    for(i=min_i ; i<=max_i ; i++){
      for(j=min_j ; j<=max_j ; j++){
        og_array.array[k].data[gridIndexFromCoord(i,j)] = 100;
      }
    }
  }


if(p_marker.pose.position.x > 0)
{
for(k=0;k<nbMap;k++){
   temp_pose_1.x = p_marker.pose.position.x + k*timeStep*arg_vel_p*cos(1.57);
   temp_pose_1.y = p_marker.pose.position.y + k*timeStep*arg_vel_p*cos(1.57);


    ped_grid_i_1 = gridIFromPose(temp_pose_1);
    ped_grid_j_1 = gridJFromPose(temp_pose_1);
    
 // std::cout << "temp pose x:" << temp_pose.x << " y:" << temp_pose.y << std::endl;
 // std::cout << " i:" << ped_grid_i << " j:" << ped_grid_j << std::endl;
    min_i_1 = max(ped_grid_i_1 - 3, 0);
    max_i_1 = min(ped_grid_i_1 + 3, (int)grid.info.width);
    min_j_1 = max(ped_grid_j_1 - 3, 0);
    max_j_1 = min(ped_grid_j_1 + 3, (int)grid.info.height);
   // std::cout << "min_i:"<< min_i << " max_i:"<<max_i << " min_j:" << min_j << " min_j:"<< max_j << std::endl;
   //  std::cout << gridIndexFromCoord(min_i,min_j) << std::endl;
    for(i=min_i_1 ; i<=max_i_1 ; i++){
      for(j=min_j_1 ; j<=max_j_1 ; j++){
        og_array.array[k].data[gridIndexFromCoord(i,j)] = 100;
      }
    }

  }
}


if(o_marker.pose.position.x > 0)
{
for(k=0;k<nbMap;k++){
   temp_pose_1.x = o_marker.pose.position.x+ k*timeStep*arg_vel_o*cos(1.57);
   temp_pose_1.y = o_marker.pose.position.y+ k*timeStep*arg_vel_o*cos(1.57);

    ped_grid_i_1 = gridIFromPose(temp_pose_1);
    ped_grid_j_1 = gridJFromPose(temp_pose_1);
    
// std::cout << "temp pose x:" << temp_pose.x << " y:" << temp_pose.y << std::endl;
 // std::cout << " i:" << ped_grid_i << " j:" << ped_grid_j << std::endl;
    min_i_1 = max(ped_grid_i_1 - 6, 0);
    max_i_1 = min(ped_grid_i_1 + 6, (int)grid.info.width);
    min_j_1 = max(ped_grid_j_1 - 6, 0);
    max_j_1 = min(ped_grid_j_1 + 6, (int)grid.info.height);
    // std::cout << "min_i:"<< min_i << " max_i:"<<max_i << " min_j:" << min_j << " min_j:"<< max_j << std::endl;
   //  std::cout << gridIndexFromCoord(min_i,min_j) << std::endl;
    for(i=min_i_1 ; i<=max_i_1 ; i++){
      for(j=min_j_1 ; j<=max_j_1 ; j++){
        og_array.array[k].data[gridIndexFromCoord(i,j)] = 100;
      }
    }

  }
}

// }



}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "og_map");

  ros::NodeHandle n;
  
 //Reading parameters from yaml file (description in yaml file)
 // Params params;
 n.param("timeStep", timeStep,0.5);
 n.param("maxDepth", nbMap, 10);


// std::cout << nbMap << timeStep << std::endl;
// int nbMap =10;
// double timeStep = 0.5;

  int i;


  //Subscribers
  ros::Subscriber og_sub = n.subscribe("/FLUENCE/submap", 1, OGCallback);
  ros::Subscriber ped_sub = n.subscribe<icars_object_ekf::Object>("/FLUENCE/obstacle/filtered", 1, pedposeCallback);

  //Publishers
  ros::Publisher og_pub = n.advertise<riskrrt::OccupancyGridArray>("/ogarray", 1);

  ros::Rate loop_rate(10);
  ros::Publisher vis_pub = n.advertise<visualization_msgs::MarkerArray>( "human_marker", 0 );
  ros::Publisher vis_pub1 = n.advertise<visualization_msgs::MarkerArray>( "human_marker1", 0 );
  ros::Publisher vis_pub2 = n.advertise<visualization_msgs::MarkerArray>( "human_marker2", 0 );
  ros::Publisher vis_pub_p = n.advertise<visualization_msgs::MarkerArray>( "human_marker_p", 0 );
  ros::Publisher vis_pub_o = n.advertise<visualization_msgs::MarkerArray>( "human_marker_o", 0 );
  ros::Publisher ellipse_vis_pub_p = n.advertise<visualization_msgs::MarkerArray>( "ellipse_human_marker_p", 0 );
  ros::Publisher ellipse_vis_pub_o = n.advertise<visualization_msgs::MarkerArray>( "ellipse_human_marker_o", 0 );

  while (ros::ok())
  {

    marker_array.markers.clear();
    marker_array1.markers.clear();
    marker_array2.markers.clear();
    p_marker_array.markers.clear();
    o_marker_array.markers.clear();
    ellipse_p_marker_array.markers.clear();
    ellipse_o_marker_array.markers.clear();

    ros::spinOnce();

    og_pub.publish(og_array);
    og_array.array.clear();
  for(i=0;i<nbMap;i++){
    og_array.array.push_back(grid);
  }

    // std::cout << og_array.array[0].info.origin.position.x << std::endl;

    vis_pub.publish(marker_array);
    marker_array.markers.clear();

    vis_pub1.publish(marker_array1);
    marker_array1.markers.clear();

    vis_pub2.publish(marker_array2);
    marker_array2.markers.clear();

    vis_pub_p.publish(p_marker_array);
    p_marker_array.markers.clear();

    vis_pub_o.publish(o_marker_array);
    o_marker_array.markers.clear();

    ellipse_vis_pub_p.publish(ellipse_p_marker_array);
    ellipse_p_marker_array.markers.clear();

    ellipse_vis_pub_o.publish(ellipse_o_marker_array);
    ellipse_o_marker_array.markers.clear();

    loop_rate.sleep();
  }
  return 0;
}
