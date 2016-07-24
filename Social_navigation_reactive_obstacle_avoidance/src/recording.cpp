#include <iostream>
#include <fstream>

//ROS
#include "ros/ros.h"

#include <std_msgs/String.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include "tf/tf.h"

#include <icars_2d_map_manager/Status.h>  // Car pose
#include <icars_2d_map_manager/Path.h>  // trajectory
#include <icarsCarControlWebsocket/SimRefs.h>  // for getting the control vector

#include "riskrrt/riskrrt.hpp"

//Namespaces
using namespace std;
//using namespace Eigen;
//using Eigen::MatrixXd;

bool registerTimeStamp, registerDistTravelled, registerPosition, registerLatErr, registerOriErr, registerCurvature, registerSpeed,registerNextRoadItem, registerDistToNextRoadItem;
bool registerPotGas;
int id_prev;
int count_id;


geometry_msgs::Pose error;
geometry_msgs::Pose error1_t;
geometry_msgs::PoseStamped dat;
icarsCarControlWebsocket::SimRefs control1;
geometry_msgs::Pose traj1;
geometry_msgs::Pose car_pose1;
int switch_controller1;
geometry_msgs::Pose reactive1;

geometry_msgs::PoseStamped datWithCov;
std::vector<geometry_msgs::PoseStamped> poses;
std::vector<geometry_msgs::PoseStamped> posesCam;

std::vector<geometry_msgs::PoseStamped> posesEKF;

void StatusInputCallback(geometry_msgs::Pose msg);
void switchcontrollerCallback(std_msgs::Int8 msg1);
void controlCallback(icarsCarControlWebsocket::SimRefs msg2);
void reactiveCallback(geometry_msgs::Pose msg3);
/*void trajCallback(icars_2d_map_manager::Path msg3);
void poseCallback(icars_2d_map_manager::Status msg4);*/

void registerDataOnFile1(std_msgs::Int8 switch_controller);
void registerDataOnFile(geometry_msgs::Pose error_pose);
void registerDataOnFile2(icarsCarControlWebsocket::SimRefs control);
void registerDataOnFile3(geometry_msgs::Pose reactive);
/*void registerDataOnFile3(icars_2d_map_manager::Path traj);
void registerDataOnFile4(icars_2d_map_manager::Status car_pose);*/

//void registerDataOnFile(icars_2d_map_manager::Status Dat);


void registerDataOnFile1(std_msgs::Int8 switch_controller)
{
       switch_controller1 = switch_controller.data;
 
std::ofstream resultOutput1;

    resultOutput1.open("/home/user/catkin_ws/src/vignesh/riskrrt/switch_controller.txt", std::ofstream::out | std::ofstream::app);
	    if (resultOutput1.fail())
	    	{
      		std::cout<<"ERROR!! Unable to open fileNameData output file 1!"<<std::endl;
    	    	}
           else
    		{
      		resultOutput1 << std::setprecision(10);
				
			  resultOutput1 << switch_controller1 << ";";
		    }
  	resultOutput1 << endl;
	resultOutput1.close();

}

void registerDataOnFile(geometry_msgs::Pose error_pose)
{

         error.position.x = error_pose.position.x;
         error.position.y = error_pose.position.y;

        std::ofstream resultOutput;

    //cout << " Registering:";.c_str()
    resultOutput.open("/home/user/catkin_ws/src/vignesh/riskrrt/error.txt", std::ofstream::out | std::ofstream::app);
	    if (resultOutput.fail())
	    	{
      		std::cout<<"ERROR!! Unable to open fileNameData output file!"<<std::endl;
    	    	}
           else
    		{
      		resultOutput << std::setprecision(10);
				
		
			  resultOutput << error.position.x << ";";
			  resultOutput << error.position.y << ";";
			  // resultOutput << control1.gasForce << ";";
		    }
  	resultOutput << endl;
	resultOutput.close();

}



void registerDataOnFile2(icarsCarControlWebsocket::SimRefs control)
{

//if (!control.empty()) {

	control1.steerAng=control.steerAng;
	control1.brakeStrength=control.brakeStrength;
	control1.gasForce=control.gasForce;
	// dat.pose.position.y=Dat.points[i].y;

//for(int i=0;i<Dat.points.size();i++)
//{
	// dat.pose.position.x=Dat.points[i].x;
	// dat.pose.position.y=Dat.points[i].y;
	// dat.pose.orientation=tf::createQuaternionMsgFromYaw(Dat.points[i].yaw);;
	// dat.pose.position.z=-Dat.pose.position.y; //08/04: Changed the sign as its inversed .////try - symbol to see if correspods Z axis motion
  // dat.pose.orientation.x=dat.pose.orientation.x;
  // dat.pose.orientation.y=dat.pose.orientation.y;
  // dat.pose.orientation.z=dat.pose.orientation.z;
  // dat.pose.orientation.w=dat.pose.orientation.w;
    // dat.header=Dat.header;
	// dat.header= Dat.header;

	
        std::ofstream resultOutput2;

    //cout << " Registering:";.c_str()
    resultOutput2.open("/home/user/catkin_ws/src/vignesh/riskrrt/control.txt", std::ofstream::out | std::ofstream::app);
	    if (resultOutput2.fail())
	    	{
      		std::cout<<"ERROR!! Unable to open fileNameData output file 2!"<<std::endl;
    	    	}
           else
    		{
      		resultOutput2 << std::setprecision(10);
				
				/*if( Dat.isKeyframe) 
				if(Dat.id!=id_prev)
				{
				{
					for (int i=0; i<6; i++)
					{	
						cout << " Pose_"<< count_id <<"  "<< Dat.camToWorld[i]<< ";";
						cout << endl;	
						resultOutput << Dat.camToWorld[i] << ";";
					}
					resultOutput << endl;
					resultOutput.close();
					id_prev=Dat.id;
					count_id++;
				}
    		}*/	
			//resultOutput << dat.pose.position.x << ";";
			//resultOutput << dat.pose.position.y << ";";
			//resultOutput << dat.pose.position.z<< ";";
			//resultOutput << dat.pose.orientation.x<< ";";
			//resultOutput << dat.pose.orientation.y<< ";";
			//resultOutput << dat.pose.orientation.z<< ";";
			//resultOutput << dat.pose.orientation.w<< ";";
			  resultOutput2 << control1.steerAng << ";";
			  resultOutput2 << control1.brakeStrength << ";";
			  resultOutput2 << control1.gasForce << ";";
		    }
  	resultOutput2 << endl;
	resultOutput2.close();

}



void registerDataOnFile3(geometry_msgs::Pose reactive)
{
	 reactive1.position.x=reactive.position.x;
	 reactive1.position.y=reactive.position.y;

	//traj1.orientation=tf::createQuaternionMsgFromYaw(traj.points[i].yaw);;

	// traj1.header= traj.header;
	
        std::ofstream resultOutput3;

    //cout << " Registering:";.c_str()
    resultOutput3.open("/home/user/catkin_ws/src/vignesh/riskrrt/reactive.txt", std::ofstream::out | std::ofstream::app);
	    if (resultOutput3.fail())
	    	{
      		std::cout<<"ERROR!! Unable to open fileNameData output file 3!"<<std::endl;
    	    	}
           else
    		{
      		resultOutput3 << std::setprecision(10);
				
			resultOutput3 << reactive1.position.x << ";";
			resultOutput3 << reactive1.position.y << ";";
			//resultOutput << dat.pose.position.z<< ";";
			//resultOutput3 << traj1.orientation.x<< ";";
			//resultOutput3 << traj1.orientation.y<< ";";
			//resultOutput3 << traj1.orientation.z<< ";";
			//resultOutput3 << traj1.orientation.w<< ";";
		
		    }
  	resultOutput3 << endl;
	resultOutput3.close();

}

/*
void registerDataOnFile4(icars_2d_map_manager::Status car_pose)
{
// if (!traj.points.empty()) {

//for(int i=0;i<traj.points.size();i++)
//{
	 car_pose1.position.x=car_pose.xLocal;
	 car_pose1.position.y=car_pose.yLocal;

	 car_pose1.orientation=tf::createQuaternionMsgFromYaw(car_pose.yaw);;

	// car_pose1.header= traj.header;
	
        std::ofstream resultOutput4;

    //cout << " Registering:";.c_str()
    resultOutput4.open("/home/user/catkin_ws/src/vignesh/riskrrt/car_pose.txt", std::ofstream::out | std::ofstream::app);
	    if (resultOutput4.fail())
	    	{
      		std::cout<<"ERROR!! Unable to open fileNameData output file 4!"<<std::endl;
    	    	}
           else
    		{
      		resultOutput4 << std::setprecision(10);
				
			resultOutput4 << car_pose1.position.x << ";";
			resultOutput4 << car_pose1.position.y << ";";
			//resultOutput << dat.pose.position.z<< ";";
			resultOutput4 << car_pose1.orientation.x<< ";";
			resultOutput4 << car_pose1.orientation.y<< ";";
			resultOutput4 << car_pose1.orientation.z<< ";";
			resultOutput4 << car_pose1.orientation.w<< ";";
		
		    }
  	resultOutput4 << endl;
	resultOutput4.close();
}


*/

//Callback to receive the error status

void StatusInputCallback(geometry_msgs::Pose msg)
{
      registerDataOnFile(msg);
      
}


void switchcontrollerCallback(std_msgs::Int8 msg1)
{
      registerDataOnFile1(msg1);
    
} 

void controlCallback(icarsCarControlWebsocket::SimRefs msg2)
{
      registerDataOnFile2(msg2);
      
}


void reactiveCallback(geometry_msgs::Pose msg3)
{
      registerDataOnFile3(msg3);
      
}
/*
void poseCallback(icars_2d_map_manager::Status msg4)
{
      registerDataOnFile4(msg4);
      
}

*/

int main (int argc, char** argv)
{
  std::ofstream resultOutput;
  std::ofstream resultOutput1;
  std::ofstream resultOutput2;
  std::ofstream resultOutput3;
  std::ofstream resultOutput4;


  //Connect to ROS
  ros::init(argc, argv, "Data_registerer_node1");
  ROS_INFO("Node Data_registerer_node Connected to roscore");

  ros::NodeHandle local_nh("~");//ROS local Handler

  std::string recording;

 resultOutput.open("/home/user/catkin_ws/src/vignesh/riskrrt/error.txt", std::ofstream::out | std::ofstream::trunc);

  cout << "Registering:";
  cout << endl ;	
      	
      resultOutput << " error x;";
      resultOutput << " error y;";
   //   resultOutput << " gas Force;";

   //   resultOutput << " X;";
    //  resultOutput << " Y;";
    //  resultOutput << " Z;";
    // resultOutput << " qat1;";
    // resultOutput << " qat2;";
    // resultOutput << " qat3;";
    // resultOutput << " qat4;";      
      resultOutput << endl;

 resultOutput1.open("/home/user/catkin_ws/src/vignesh/riskrrt/switch_controller.txt", std::ofstream::out | std::ofstream::trunc);

      resultOutput1 << " Switch_controller;";
      resultOutput1 << endl;


 resultOutput2.open("/home/user/catkin_ws/src/vignesh/riskrrt/control.txt", std::ofstream::out | std::ofstream::trunc);

      resultOutput2 << " steerAng;";
      resultOutput2 << " brakeStrength;";
      resultOutput2 << " gas Force;";
      resultOutput2 << endl;

 resultOutput3.open("/home/user/catkin_ws/src/vignesh/riskrrt/reactive.txt", std::ofstream::out | std::ofstream::trunc);

       resultOutput3 << " X;";
       resultOutput3 << " Y;";
      // resultOutput << " Z;";
      // resultOutput3 << " qat1;";
    // resultOutput3 << " qat2;";
    // resultOutput3 << " qat3;";
    //  resultOutput3 << " qat4;"; 
      resultOutput3 << endl;
/*
 resultOutput4.open("/home/user/catkin_ws/src/vignesh/riskrrt/car_pose.txt", std::ofstream::out | std::ofstream::trunc);

       resultOutput4 << " X;";
       resultOutput4 << " Y;";
      // resultOutput << " Z;";
     resultOutput4 << " qat1;";
     resultOutput4 << " qat2;";
     resultOutput4 << " qat3;";
     resultOutput4 << " qat4;"; 
      resultOutput4 << endl;

*/
	ros::Rate rate(10.0);


  //Subscribing
	ROS_INFO("Subscribing to topics\n");
/*	ros::Subscriber status_sub=local_nh.subscribe<geometry_msgs::Pose> ("/error", 20, StatusInputCallback);
	ros::Subscriber errortraj_sub=local_nh.subscribe<geometry_msgs::Pose> ("/error_traj", 20, errortrajCallback);
	ros::Subscriber control_sub=local_nh.subscribe<icarsCarControlWebsocket::SimRefs> ("/FLUENCE/sim/refs", 20, controlCallback);
	ros::Subscriber traj_sub=local_nh.subscribe<icars_2d_map_manager::Path> ("/trajectory/Path", 20, trajCallback);
	ros::Subscriber pose_sub=local_nh.subscribe<icars_2d_map_manager::Status> ("/FLUENCE/map_manager/Status", 20, poseCallback);
	// ros::Subscriber statuswCov_sub=local_nh.subscribe< geometry_msgs::PoseWithCovarianceStamped> ("/robot_pose_ekf/odom_combined", 20, StatusWithCovCallback); */

	ros::Subscriber control_sub=local_nh.subscribe<icarsCarControlWebsocket::SimRefs> ("/FLUENCE/sim/refs", 20, controlCallback);
        ros::Subscriber status_sub=local_nh.subscribe<geometry_msgs::Pose> ("/error", 20, StatusInputCallback);
        ros::Subscriber switchcontroller_sub =local_nh.subscribe<std_msgs::Int8>("/switch_controller",20, switchcontrollerCallback);
	ros::Subscriber reactive_sub=local_nh.subscribe<geometry_msgs::Pose> ("/reactive", 20, reactiveCallback);

  //Main loop
  ros::spin();
}
  
