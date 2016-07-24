#include "riskrrt/riskrrt.hpp"
#include <cmath>
#include <algorithm>
#include <icars_2d_map_manager/Path.h>
#include <geometry_msgs/Point.h>

using namespace std;

RRT::RRT(Params params){
  this->params = params;
  
}

RRT::~RRT(){
  
}

//returns the node score given the goal to reach (final or random)
//the bigger the weight, the better the node
double RRT::computeNodeWeight(Node* node, custom_pose goal){

  double weight;
  
  weight = 1.0 / (params.socialWeight * node->risk + trajLength(node->pose, goal)) * 1e-9;
  
  return weight;
}

//return a distance from pose to goal based on both euclidian distance and orientation
double RRT::trajLength(custom_pose pose, custom_pose goal){
  
  double pose_euclide_distance;
  double root_euclide_distance;
  double position_improvement;
  double rotation_diff;
  double distance_from_goal;
  
  //distance from pose to goal
  pose_euclide_distance = sqrt(pow(pose.x - goal.x, 2) + pow(pose.y - goal.y, 2));
  //distance from root to goal
  root_euclide_distance = sqrt(pow(root->pose.x - goal.x, 2) + pow(root->pose.y - goal.y, 2));
  //position improvement with respect to the goal
  //ratio < 1 means we are getting closer to the goal
  //ratio > 1 means we are getting further away from the goal

// std::cout<<"root->pose x:"<< root->pose.x<<"root->pose y:"<< root->pose.y<<std::endl;

  position_improvement = pose_euclide_distance / root_euclide_distance;
  //angle difference between pose orientation and the pose-goal vector
  rotation_diff = atan2(goal.y - pose.y, goal.x - pose.x) - pose.theta;
  //angle must be in [-PI;PI]
  if(rotation_diff > M_PI){
    rotation_diff -= 2 * M_PI;
  }
  if(rotation_diff < -M_PI){
    rotation_diff += 2 * M_PI;
  }
  distance_from_goal = position_improvement + params.rotationWeight * fabs(rotation_diff);
  
  return distance_from_goal;
}

//extends the tree from by creating a new node given an already existing node and a goal
void RRT::extend(Node* node, custom_pose random_goal){
  Node* new_node;
  new_node = new Node;
  custom_pose expected_pose;
  int best_control_index;
  double control_score, best_control_score;
  int i, j;
  bool node_still_open;
  
  best_control_score = 0.0;
  //choose the best control among the (still open) possible controls
  for(i=0; i<node->possible_controls.size(); i++){
    if(node->possible_controls[i].open){
      //compute what pose would be obtained with that control
      expected_pose = robotKinematic(node->pose, node->possible_controls[i]);
      //compute the score for that pose
      control_score = computeControlScore(expected_pose, random_goal);
      if(control_score >= best_control_score){
        best_control_score = control_score;
        best_control_index = i;
      }
    }
  }
  
  //create the new node from the best control
  new_node->time = node->time + ros::Duration(params.timeStep);
  new_node->pose = robotKinematic(node->pose, node->possible_controls[best_control_index]);
  new_node->vel = node->possible_controls[best_control_index].twist;
  new_node->parent = node;
  new_node->sons.clear();
  new_node->possible_controls = discretizeVelocities(new_node);
  new_node->depth = node->depth + 1;
  new_node->risk = computeNodeRisk(new_node);
  new_node->isFree = (new_node->risk <= params.threshold);
  new_node->isOpen = true;
  //add new node to the tree (even if it is in collision, the candidate nodes vector is here to sort the nodes)
  node->sons.push_back(new_node);
  //close the control used to create the new node so it is impossible to create a duplicate node later
  node->possible_controls[best_control_index].open = false;
  
  //add the newly created node to the candidates vector (if it is free) to be taken into account for future expansion during the same growing phase
  if(new_node->isFree && new_node->depth < params.maxDepth){
    candidate_nodes.push_back(new_node);
  }
  
  //check if the last opened control was used and, if so, close the node so that it won't be selected as best node again
  node_still_open = false;
  for(j=0; j<node->possible_controls.size(); j++){
    node_still_open = node_still_open || node->possible_controls[j].open;
  }
  node->isOpen = node_still_open;
}

//read the risks from the grid and compute a global risk
double RRT::computeNodeRisk(Node* node){
  
  //robot's footprint is assumed to be a rectangle
	custom_pose front_left;
  custom_pose front_right;
  custom_pose rear_left;
  custom_pose rear_right;
	int grid_front_left_x, grid_front_left_y;
	int grid_front_right_x, grid_front_right_y;
	int grid_rear_left_x, grid_rear_left_y;
	int grid_rear_right_x, grid_rear_right_y;
	double l, w;
  double node_theta;
	vector<int> grid_cells;
  int i, j;
  double risk, max_risk;
  int grid_max_x, grid_max_y, grid_min_x, grid_min_y;
  
  //the wheel axis is assumed to be in the middle
  l = params.robotLength/2.0;
  w = params.robotWidth/2.0;
	
	//computing the poses of each corner of the robot's footprint
	front_left.x = node->pose.x + (l * cos(node->pose.theta) + w * cos(node->pose.theta + M_PI/2.0));
	front_left.y = node->pose.y + (l * sin(node->pose.theta) + w * sin(node->pose.theta + M_PI/2.0));
	front_right.x = node->pose.x + (l * cos(node->pose.theta) + w * cos(node->pose.theta - M_PI/2.0));
	front_right.y = node->pose.y + (l * sin(node->pose.theta) + w * sin(node->pose.theta - M_PI/2.0));
	rear_left.x = node->pose.x + (l * cos(node->pose.theta + M_PI) + w * cos(node->pose.theta  + M_PI - M_PI/2.0));
	rear_left.y = node->pose.y + (l * sin(node->pose.theta + M_PI) + w * sin(node->pose.theta  + M_PI - M_PI/2.0));
	rear_right.x = node->pose.x + (l * cos(node->pose.theta + M_PI) + w * cos(node->pose.theta + M_PI + M_PI/2.0));
	rear_right.y = node->pose.y + (l * sin(node->pose.theta + M_PI) + w * sin(node->pose.theta + M_PI + M_PI/2.0));
	
	//the corners poses in grid coordinates
  grid_front_left_x = gridIFromPose(front_left);
  grid_front_left_y = gridJFromPose(front_left);
  grid_front_right_x = gridIFromPose(front_right);
  grid_front_right_y = gridJFromPose(front_right);
  grid_rear_left_x = gridIFromPose(rear_left);
  grid_rear_left_y = gridJFromPose(rear_left);
  grid_rear_right_x = gridIFromPose(rear_right);
  grid_rear_right_y = gridJFromPose(rear_right);
	
	//testing: simple bounding box TODO: exact footprint
	grid_max_x = max(grid_front_left_x, max(grid_front_right_x, max(grid_rear_left_x, grid_rear_right_x)));
	grid_max_y = max(grid_front_left_y, max(grid_front_right_y, max(grid_rear_left_y, grid_rear_right_y)));
	grid_min_x = min(grid_front_left_x, min(grid_front_right_x, min(grid_rear_left_x, grid_rear_right_x)));
	grid_min_y = min(grid_front_left_y, min(grid_front_right_y, min(grid_rear_left_y, grid_rear_right_y)));
	
  //creating a list of all the grid cells within the robot's footprint
	for(i=grid_min_x ; i<=grid_max_x ; i++){
		for(j=grid_min_y ; j<=grid_max_y ; j++){
			if (i >=0 && i < (int)og_array.array[0].info.width && j >=0 && j< (int)og_array.array[0].info.height){
				grid_cells.push_back(gridIndexFromCoord(i,j));
			}
		}
	}
  
  //going through all the cells in robot footprint and getting the maximum risk
  max_risk = 0.0;
  for(i=0; i<grid_cells.size(); i++){	  
    risk = og_array.array[node->depth].data[grid_cells[i]];
    max_risk = max(risk, max_risk);
  }
  
  //risk propagation from a node to his sons if their risk is lower
  if(node->parent != NULL){
    max_risk = max(max_risk, node->parent->risk);
  }
  

	return max_risk;
}

//returns control score given the pose that would be obtained by applying the control during timestep and a goal
//the bigger the score, the better the control
double RRT::computeControlScore(custom_pose expected_pose,  custom_pose random_goal){
  double distance_from_random_goal;
  double score;
  
  distance_from_random_goal = trajLength(expected_pose, random_goal);
  score = 1.0 / distance_from_random_goal + 1e-9;
  
  return score;
}

//initialisation, creates a root at robot location
void RRT::init(){
  //creating a new node at the robot current location, with its current speed
  Node* node;
  node = new Node;
  node->time = msg_pf_stat.header.stamp;
  node->pose = robot_pose;
  node->vel = robot_vel;
  node->parent = NULL;
  node->sons.clear();
  node->possible_controls = discretizeVelocities(node);
  node->isOpen = true;
  node->depth = 0;
  node->risk = computeNodeRisk(node);
  node->isFree = (node->risk <= params.threshold);
  //set this new node as the tree root
  root = node;
  //set root as the best node to reach the final goal (this is needed to set the window in order to choose the next random goal)
  best_node = node;
  //the tree root is the first candidate to grow
  candidate_nodes.clear();
  candidate_nodes.push_back(root);

}

//adds 1 new node to the tree
void RRT::grow(){
  Node* best_node_to_grow;
  custom_pose random_goal;

  //choosing a random point in window
 random_goal = chooseRandomGoal();
// random_goal.x = 0.5;
// random_goal.y = 28;
// std::cout<< "random_goal x:" << random_goal.x << "random_goal y:" << random_goal.y << std::endl;

  //choosing the best node within the tree to grow toward the random point
  best_node_to_grow = chooseBestNode(random_goal);

  // std::cout<< "best_node_to_grow->pose x:" << best_node_to_grow->pose.x << "best_node_to_grow->pose y:" << best_node_to_grow->pose.y << std::endl;

  //checking if there is an actual best node (in some situations, it is possible to have none. for instance, if root is in collision)
  if(best_node_to_grow != NULL){
    //extend the tree from the best node to the random point by choosing the best control
    extend(best_node_to_grow, random_goal);
  }
  else ROS_INFO("ROBOT STUCK");
}

void RRT::grow_to_goal(Node* best_node, custom_pose goal){
  //TODO
}

//returns a random pose inside a user defined window
custom_pose RRT::chooseRandomGoal(){
  double window_size;
  double random_score;
  int window_size_grid;
  int x_min_limit, x_max_limit, y_min_limit, y_max_limit;
  int random_x, random_y;
  custom_pose random_goal;
  
  //getting a random number between 0 and 99 for bias
  random_score = (rand() % 100)/100.0;
  //computing the window size in grid cells
  window_size_grid = (int)floor(params.windowSize / og_array.array[0].info.resolution);
 
  // std::cout<< "window_size_grid:" << window_size_grid << std::endl;

  //reducing the window size if it exceeds the map dimensions
  x_min_limit = max(gridIFromPose(best_node->pose) - (window_size_grid), 0);
  x_max_limit = min(gridIFromPose(best_node->pose) + (window_size_grid), (int)og_array.array[0].info.width);
  y_min_limit = max(gridJFromPose(best_node->pose) - (window_size_grid), 0);
  y_max_limit = min(gridJFromPose(best_node->pose) + (window_size_grid), (int)og_array.array[0].info.height);

  // std::cout<<"best_node->pose x:"<< best_node->pose.x<<"best_node->pose y:"<< best_node->pose.y << std::endl;

 // std::cout<< "x_min_limit:" << x_min_limit << "x_max_limit" << x_max_limit << std::endl;

  if(random_score > params.bias){
    //choosing random position within limits (window or map edges)
    random_x = x_min_limit + rand() % (x_max_limit - x_min_limit);
    random_y = y_min_limit + rand() % (y_max_limit - y_min_limit);
    //same position but in map coordinates
    random_goal = poseFromGridCoord(random_x, random_y);

  }
  else{
    //choosing the final goal as random goal with a set probability (bias)
    random_goal = final_goal;


  } 
   // random_goal.x = -10;
   // random_goal.y = 16;
    // std::cout<<"random_goal x:"<< random_goal.x<<"random_goal y:"<< random_goal.y<<std::endl;
  return random_goal;
}


int getNextWayPoint(int _nextWayPoint,riskrrt::Trajectory traj_msg) {   

 // std::cout << "_nextWayPoint : " << WayPoint << std::endl;

  custom_pose robot_pose;//pose of the robot
  vector<Node*> traj;//trajectory with pointer to the nodes, used to update tree

    if (!traj_msg.poses.empty()) {

      if (_nextWayPoint >= 0) {

        tf::Vector3 v_1(robot_pose.x,
                        robot_pose.y,
                        0.015);

        double lookAheadThreshold = 3.0;

        for (int i = _nextWayPoint; i < traj_msg.poses.size();++i) {                
          tf::Vector3 v_2(traj_msg.poses[i].pose.position.x,
                          traj_msg.poses[i].pose.position.y,
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

//updates the tree, deletes unreachable nodes and updates remaining nodes
void RRT::update(){
  
  int index;
  ros::Time present;

  
  present = msg_pf_stat.header.stamp;
  
	if(traj.size() <= 1){
    index = -1;
	}
	else if(traj.front()->time > present){
    index = -1;

	}
	else if(traj.back()->time < present){
		index = -1;
	}
  else{
    index = floor((present - traj.front()->time).toSec() / params.timeStep);
  }
  
  //to delete the unreachable nodes, there has to be an actual trajectory with at least 2 nodes (root and one other node)
  //if(traj.size() > 1){
    //if(ros::Time::now() >= traj[1]->time){

// int nextWayPoint= getNextWayPoint(nextWayPoint,traj_msg);

// std::cout<< "nextWayPoint:" << nextWayPoint << std::endl;

if (nextWayPoint==0) {
nextWayPoint=nextWayPoint+1;
}


if(index != -1 && index != 0){



 deleteUnreachableNodes(traj[nextWayPoint]);
      //making the first node in trajectory the new root

root =traj[nextWayPoint];
root->depth=0;
root->parent = NULL; 
    }
    //}
  //}
  //update remaining tree
  updateNodes();
}

//depth first update of all nodes in tree, also creates node markers
void RRT::updateNodes(){
  Node* current_node;
  vector<Node*> node_stack;
  int i;
  int nb_nodes;
  visualization_msgs::Marker node_marker;
  
  nb_nodes = 0;
  candidate_nodes.clear();
  
  //robot is always somewhere between root and the first node of the trajectory
  //thus, when the robot is moving, root and its descendants (except the subtree defined by the first node of the trajectory) are unreachable
  //there is no need in updating them or adding them in the candidate nodes vector
  //if(traj.size() > 1){
    //node_stack.push_back(traj[1]);
  //}
  //else{
    node_stack.push_back(root);
  //}
  while(!node_stack.empty()){
    current_node = node_stack.back();
    node_stack.pop_back();
    if(!current_node->sons.empty()){
      for(i=0; i<current_node->sons.size(); i++){
        node_stack.push_back(current_node->sons[i]);
      }
    }
    //update the depth
    if(current_node->parent != NULL){
      current_node->depth = current_node->parent->depth + 1;
    }
    current_node->risk = computeNodeRisk(current_node);
    current_node->isFree = (current_node->risk <= params.threshold);
    //add the "good" nodes to the candidate nodes vector
    if(current_node->isFree && current_node->depth < params.maxDepth && current_node->isOpen){
      candidate_nodes.push_back(current_node);
    }

// std::cout<<"current_node->risk:"<<current_node->risk<<std::endl;

    nb_nodes++;
    //rviz marker for nodes
    node_marker.header.frame_id = "/FLUENCE/submap";
    node_marker.header.stamp = msg_pf_stat.header.stamp;
    node_marker.id = nb_nodes;
    node_marker.ns = "tree";
    node_marker.type = visualization_msgs::Marker::SPHERE;
    node_marker.action = visualization_msgs::Marker::ADD;
    node_marker.pose.position.x = current_node->pose.x;
    node_marker.pose.position.y = current_node->pose.y;
    node_marker.pose.position.z = 0.05;
    node_marker.scale.x = 0.1;
    node_marker.scale.y = 0.1;
    node_marker.scale.z = 0.1;
    node_marker.color.r = 0.0;
    node_marker.color.g = 1.0;
    node_marker.color.b = 0.0;
    node_marker.color.a = 1.0;
    node_marker.lifetime = ros::Duration(1.0);
    node_markers.markers.push_back(node_marker);
    
  }
}

//deletes the tree minus the subtree defined by new_root
void RRT::deleteUnreachableNodes(Node* new_root){
  Node* current_node;
  vector<Node*> node_stack;
  int i;
  
  node_stack.push_back(root);
  while(!node_stack.empty()){
    current_node = node_stack.back();
    node_stack.pop_back();
    if(!current_node->sons.empty()){
      for(i=0; i<current_node->sons.size(); i++){
        if(current_node->sons[i] != new_root){
          node_stack.push_back(current_node->sons[i]);
        }
      }
    }
    current_node->sons.clear();
    current_node->parent = NULL;
    current_node->possible_controls.clear();
    delete current_node;
  }
}

//returns the best node among candidates nodes with respect to goal
Node* RRT::chooseBestNode(custom_pose goal){
  
  Node* best_rated_node;
  Node* current_node;
  double best_weight;
  double current_weight;
  int i;
  
  best_rated_node = NULL;
  best_weight = 0.0;
  for(i=0; i<candidate_nodes.size(); i++){
    current_node = candidate_nodes[i];
    current_weight = computeNodeWeight(current_node, goal);

// std::cout<< current_weight << std::endl;
    // current_node->isOpen=true;
    //node weight is better, node is free and open, node depth is less than maximum depth

     if(best_weight <= current_weight && current_node->isOpen){
  // if(current_node->isOpen){
      best_rated_node = current_node;
      best_weight = current_weight;
    }
  }
  
  return best_rated_node;
}

//select the best node with respect to the final goal and creates the path from root to that node
void RRT::findPath(){
  riskrrt::PoseTwistStamped pose_twist;
  Node* current_node;
  visualization_msgs::Marker path_marker; 
  visualization_msgs::Marker line_strip; 
  geometry_msgs::Point p;
  
  traj_msg.poses.clear();
  traj.clear();
  //find the best node to go toward the final goal and add it to the trajectory
  best_node = chooseBestNode(final_goal);

// std::cout<<"best_node->risk:"<<best_node->risk<<std::endl;
  //if there is no best node, pick root
  if(best_node == NULL){
    best_node = root;
  }

// std::cout<<"root->pose.x in:"<<root->pose.x<<"rrt->root->pose.y in:"<<root->pose.y<<std::endl;

  //fill the twist message with node attributes
  pose_twist.pose.position.x = best_node->pose.x;
  pose_twist.pose.position.y = best_node->pose.y;
  pose_twist.pose.orientation = tf::createQuaternionMsgFromYaw(best_node->pose.theta);
  pose_twist.twist = best_node->vel;
  pose_twist.time = best_node->time;
  current_node = best_node;
  traj_msg.poses.push_back(pose_twist);
  traj.push_back(current_node);
  //add all bestnode's ancestor to the trajectory until root is reached
  while(current_node->parent != NULL){
    current_node = current_node->parent;
    pose_twist.pose.position.x = current_node->pose.x;
    pose_twist.pose.position.y = current_node->pose.y;
    pose_twist.pose.orientation = tf::createQuaternionMsgFromYaw(current_node->pose.theta);
    pose_twist.twist = current_node->vel;
    pose_twist.time = current_node->time;
    traj_msg.poses.insert(traj_msg.poses.begin(), pose_twist);
    traj_msg.header = msg_pf_stat.header;
    traj.insert(traj.begin(), current_node);
    
    
    //create a marker for the nodes in the trajectory
    path_marker.header.frame_id = "/FLUENCE/submap";
    path_marker.header.stamp = msg_pf_stat.header.stamp;
    path_marker.id = rand();
    path_marker.ns = "tree";
    path_marker.type = visualization_msgs::Marker::SPHERE;
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.pose.position.x = current_node->pose.x;
    path_marker.pose.position.y = current_node->pose.y;
    path_marker.pose.position.z = 0.05;
    path_marker.scale.x = 0.3;
    path_marker.scale.y = 0.3;
    path_marker.scale.z = 0.3;
    path_marker.color.r = 1.0;
    path_marker.color.g = 0.0;
    path_marker.color.b = 0.0;
    path_marker.color.a = 1.0;
    path_marker.lifetime = ros::Duration(1.0);
  //  path_markers.markers.push_back(path_marker);

    //create a marker for the nodes in the trajectory
    line_strip.header.frame_id = "/FLUENCE/submap";
    line_strip.header.stamp = msg_pf_stat.header.stamp;
    line_strip.id = 1;
    line_strip.ns = "tree";
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    p.x = current_node->pose.x;
    p.y = current_node->pose.y;
    p.z = 0.05;
    line_strip.scale.x = 0.1;
    // path_marker.scale.y = 0.3;
    // path_marker.scale.z = 0.3;
    line_strip.color.r = 1.0;
    line_strip.color.g = 0.0;
    line_strip.color.b = 0.0;
    line_strip.color.a = 1.0;
    line_strip.lifetime = ros::Duration(1.0);
    line_strip.points.push_back(p);

    path_markers.markers.push_back(line_strip);
    
  }
  //set the trajectory flag to stop robot from executing deprecated trajectories if this trajectory is empty
  traj_msg.exists.data = (traj_msg.poses.size() > 1);  
}

//returns true if robot close enough to the goal (threshold set by user)
bool RRT::isGoalReached(){
  //this is a distance criteria, orientation is not taken into account
  return (sqrt(pow(root->pose.x - final_goal.x, 2) + pow(root->pose.y - final_goal.y, 2)) < params.goalTh);
}

//returns a list of all possible controls for a node
vector<Control> RRT::discretizeVelocities(Node* node){
  
  vector<Control> controls;
  Control control;
  double min_linear_vel;
  double max_linear_vel;
  double min_angular_vel;
  double max_angular_vel;
  int i, j;
  double delta_linear, delta_angular;
  
  //compute maximum and minimum velocities that can be reached from that node given the speed and acceleration limits of the robot
  min_linear_vel = node->vel.linear.x - params.accMax * params.timeStep;
  max_linear_vel = node->vel.linear.x + params.accMax * params.timeStep;
  min_angular_vel = node->vel.angular.z - params.maxSteeringAngle * params.timeStep;//right
  max_angular_vel = node->vel.angular.z + params.maxSteeringAngle * params.timeStep;//left
  
  //make sure that speed limits are not exceeded
  min_linear_vel = max(params.vMin, min_linear_vel);
  max_linear_vel = min(params.vMax, max_linear_vel);
  min_angular_vel = max(-params.maxSteeringAngle, min_angular_vel);
  max_angular_vel = min(params.maxSteeringAngle, max_angular_vel);
  //min_angular_vel = -params.maxSteeringAngle;
  //max_angular_vel = params.maxSteeringAngle;


  //create the set of controls
  delta_linear = (max_linear_vel - min_linear_vel) / double(params.nv);
  delta_angular = (max_angular_vel - min_angular_vel) / double(params.nphi);
  for(i=0; i<params.nv; i++){
    control.twist.linear.x = min_linear_vel + i * delta_linear;
    for(j=0; j<(params.nphi); j++){
      control.twist.angular.z = min_angular_vel + j * delta_angular;
      control.open = true;
      controls.push_back(control);
    }
  }
  
  return controls;
}

//get the pose of the new node for a differential robot
custom_pose RRT::robotKinematic(custom_pose pose, Control control){
  custom_pose new_pose;
  double rotation_radius;
  double delta_theta, delta_x, delta_y;
  double delta_xc, delta_yc;

//paramteres definition for odometry calculation of CLMR
  double delta_r, delta_ori, Nonholonomicconstraint_1,Nonholonomicconstraint_2,rotation_radius_curvature, curvature;
  custom_pose new_pose_CLMR;
  
  if(control.twist.linear.x == 0.0){
  /* 
    delta_theta = control.twist.angular.z * params.timeStep;
    delta_x = 0.0;
    delta_y = 0.0;
    delta_ori = control.twist.angular.z * params.timeStep;  */

    delta_r = control.twist.linear.x;
    rotation_radius_curvature =params.robotLength/(tan(control.twist.angular.z));
    delta_ori = (delta_r*params.timeStep)/rotation_radius_curvature;

// ROS_INFO("i am here in 1st");

	}

   else if(control.twist.angular.z == 0.0){
/*  delta_theta = 0.0;
    delta_x = control.twist.linear.x * params.timeStep;
    delta_y = 0.0;  */

    delta_r = control.twist.linear.x;
    rotation_radius_curvature = 0.0;
    delta_ori = 0.0; 

// ROS_INFO("i am here in 2nd");
  }

  else{
 /*   rotation_radius = control.twist.linear.x / control.twist.angular.z;
    delta_theta = control.twist.angular.z * params.timeStep;
    delta_x = rotation_radius * sin(delta_theta);
    delta_y = rotation_radius * (1.0 - cos(delta_theta)); */

   //odometry calculation for CLMR
    
    rotation_radius_curvature=params.robotLength/(tan(control.twist.angular.z));
    delta_r = (control.twist.linear.x);
    delta_ori = (delta_r* params.timeStep)/rotation_radius_curvature;
   
// curvature=(tan(control.twist.angular.z* params.timeStep))/params.robotLength;

// std::cout << "rotation_radius: " << rotation_radius_curvature << std::endl;
 // std::cout << "delta_ori: " << delta_ori << std::endl;

  }
/* 
  new_pose.x = pose.x + (delta_x * cos(pose.theta) - delta_y * sin(pose.theta));
  new_pose.y = pose.y + (delta_x * sin(pose.theta) + delta_y * cos(pose.theta));
  new_pose.theta = atan2(sin(pose.theta + delta_theta), cos(pose.theta + delta_theta)); */

   delta_x = ((delta_r* params.timeStep) * cos(pose.theta));
   delta_y = ((delta_r* params.timeStep) * sin(pose.theta));

   //  delta_xc = pose.x - rotation_radius_curvature*sin(pose.theta);
   //  delta_yc = pose.y + rotation_radius_curvature*cos(pose.theta);

    //new pose prediction for CLMR
     new_pose_CLMR.x = pose.x + delta_x ;
     new_pose_CLMR.y = pose.y + delta_y ;
     new_pose_CLMR.theta = pose.theta + delta_ori* params.timeStep;

 // std::cout << "delta_ori: " << delta_ori<< "new_pose_CLMR.theta: " << new_pose_CLMR.theta << std::endl;

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

/*else
{
ROS_INFO("Non-holonomic constraints are not satisfied");
}*/


//bunch of functions to help switching between coordinates types
int RRT::gridIndexFromPose(custom_pose pose){
  int index, i, j;
  i = gridIFromPose(pose);
  j = gridJFromPose(pose);
  index = gridIndexFromCoord(i, j);
  return index;
}

int RRT::gridIFromPose(custom_pose pose){
  return (int)round((pose.x - og_array.array[0].info.origin.position.x) / og_array.array[0].info.resolution);
}

int RRT::gridJFromPose(custom_pose pose){
  return (int)round((pose.y - og_array.array[0].info.origin.position.y) / og_array.array[0].info.resolution);
}

int RRT::gridIndexFromCoord(int i, int j){
  return i + og_array.array[0].info.width * j;
}

int RRT::gridIFromIndex(int index){
  return  index % og_array.array[0].info.width;
}

int RRT::gridJFromIndex(int index){
  return floor(index / og_array.array[0].info.width);
}

custom_pose RRT::poseFromGridIndex(int index){
  int i, j;
  custom_pose pose;
  
  i = gridIFromIndex(index);
  j = gridJFromIndex(index);
  pose = poseFromGridCoord(i, j);
  
  return pose;
}

custom_pose RRT::poseFromGridCoord(int i, int j){
  custom_pose pose;
  pose.x = og_array.array[0].info.resolution * i + og_array.array[0].info.origin.position.x;
  pose.y = og_array.array[0].info.resolution * j + og_array.array[0].info.origin.position.y;
  return pose;
}
//end of functions to help switching between coordinates types


//subscribers initializations and callbacks
void RRT::controllerFeedbackCallback(const std_msgs::Bool::ConstPtr& msg){
  robot_on_traj = msg->data;
}

void RRT::initcontrollerFeedbackSub(){
  controllerFeedbackSubscriber = nodeHandle.subscribe("/controller_feedback", 1, &RRT::controllerFeedbackCallback, this);
}

void RRT::ogArrayCallback(const riskrrt::OccupancyGridArray::ConstPtr& msg){
  og_array = *msg;
}

void RRT::initOgArraySub(){
  ogArraySubscriber = nodeHandle.subscribe<riskrrt::OccupancyGridArray>("/ogarray", 1, &RRT::ogArrayCallback, this);
}

void RRT::odomCallback(const nav_msgs::Odometry msg){
  robot_vel = msg.twist.twist;
}

void RRT::initOdomSub(){
  odomSubscriber = nodeHandle.subscribe("/FLUENCE/odometry/attitude", 1, &RRT::odomCallback, this);
}

void RRT::poseCallback(const icars_2d_map_manager::Status msg){
  
/*
  robot_pose_test.x = msg.xGlobal;
  robot_pose_test.y = msg.yGlobal;
  robot_pose_test.theta = msg.heading; 
*/

  robot_pose.x = msg.xLocal;
  robot_pose.y = msg.yLocal;
  robot_pose.theta = msg.yaw+1.57; 

// std::cout << "robot_pose.theta:" << robot_pose.theta << "tf::getYaw(tf::createQuaternionMsgFromYaw(robot_pose.theta)):" << tf::getYaw(tf::createQuaternionMsgFromYaw(robot_pose.theta)) << std::endl;

// std::cout << "localgoal x:" << robot_pose_test.x  << " y subgoal:" << robot_pose_test.y << " heading:" << robot_pose_test.theta << std::endl;

  msg_pf_stat.header=msg.header;
}

void RRT::initPoseSub(){
  poseSubscriber = nodeHandle.subscribe<icars_2d_map_manager::Status>("/FLUENCE/map_manager/Status", 1, &RRT::poseCallback, this);
}

void RRT::goalCallback(const icars_2d_map_manager::Path msg){



for(int i=0;i<msg.points.size();i++)
{
  local_goal[i].x = msg.points[i].x;
  local_goal[i].y = msg.points[i].y;
  local_goal[i].theta = msg.points[i].yaw;
}
  global_goal.x = msg.xGlobalOrig;
  global_goal.y = msg.yGlobalOrig;
  global_goal.theta = msg.origHeading;

  goal_received = true;

}
  
void RRT::initGoalSub(){
  goalSubscriber = nodeHandle.subscribe<icars_2d_map_manager::Path>("/FLUENCE/map_manager/Path", 1, &RRT::goalCallback, this);
}


void RRT::lookaheadCallback(const std_msgs::Int8 msg){
nextWayPoint = msg.data;
}
  
void RRT::initlookaheadSub(){
  lookaheadSubscriber = nodeHandle.subscribe<std_msgs::Int8>("lookaheadpoint_feedback", 1, &RRT::lookaheadCallback, this);

}
//end of subscribers initializations and callbacks
