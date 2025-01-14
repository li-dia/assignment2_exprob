#include "../include/my_action.h"

#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>
#include <string>
#include <iostream>
#include <cstdlib>



NamedWaypoints wps[] = {
        {"wp0", -7.0, 1.5},
        {"wp1", -3.0, -8.0},
        {"wp2", 6.0, 2.0},
        {"wp3", 7.0, -5.0},
        {"wp4", 0.0, 3.0}
    };

namespace KCL_rosplan{

	// class constructor
	MyActionInterface::MyActionInterface(ros::NodeHandle &nh) {
	// here the initialization
	
	marker_id_sub = nh.subscribe("/marker_id", 1, &MyActionInterface::markerIDCallback, this);
        marker_pos_sub = nh.subscribe("/marker_center", 1, &MyActionInterface::markerCenterCallback, this);
        vel_cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        
        
        // Initialize variables
        MarkerCenterX = 0.0;
        MarkerCenterY = 0.0;
        CameraWidth = 320.0;
        Threshold = 20.0;
        Offset = 0.0;
        MarkerID = 0;
        IDSmall = INT_MAX; // Initialize to maximum integer value
        
	}

	void MyActionInterface::markerCenterCallback(const geometry_msgs::Point::ConstPtr& msg) {
          // Update the MarkerCenterX and MarkerCenterY variables with the x-coordinate and y-coordinate from the message.
          MarkerCenterX = msg->x;
          MarkerCenterY = msg->y;
	}
	    
	    
	 void MyActionInterface::markerIDCallback(const std_msgs::Int32::ConstPtr& msg) {
		// Update the MarkerCenterX and MarkerCenterY variables with the x-coordinate and y-coordinate from the message.
		MarkerID = msg->data;
		}
	
	bool MyActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
	
		actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
		move_base_msgs::MoveBaseGoal goal;
		ac.waitForServer();
		
		goal.target_pose.header.frame_id = "map";
		goal.target_pose.pose.orientation.w = 1.0;
		
		if(msg->name == "move_to"){
		std::cout << "Going from " << msg->parameters[1].value << " to " << msg->parameters[2].value << std::endl;
		
		if (msg->parameters[2].value == "wp0"){
			goal.target_pose.pose.position.x = -7.0;
			goal.target_pose.pose.position.y = 1.5;
			
		}
		else if(msg->parameters[2].value == "wp1"){
			goal.target_pose.pose.position.x = -3.0;
			goal.target_pose.pose.position.y = -8.0;
			
		}
		else if (msg->parameters[2].value == "wp2"){
			goal.target_pose.pose.position.x = 6.0;
			goal.target_pose.pose.position.y = 2.0;
			
		}
		else if (msg->parameters[2].value == "wp3"){
			goal.target_pose.pose.position.x = 7.0;
			goal.target_pose.pose.position.y = -5.0;
			
		}
		else if (msg->parameters[2].value == "wp4"){
			goal.target_pose.pose.position.x = 0.0;
			goal.target_pose.pose.position.y = 3.0;
			
		}
		ac.sendGoal(goal);
		ac.waitForResult();
		ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
		
		}
		else if(msg->name == "detect_marker"){
		   std::cout << "Detecting the marker's ID" << std::endl;
			
		   while(true){
			// detecting the marker as soon as it has been captured by the camera
			Offset = std::abs(MarkerCenterX - CameraWidth); 
			
			// Stop the robot from rotating
			geometry_msgs::Twist twist;
			twist.angular.z = 0.5;
                	   
                	// Publish the angular speed
                	vel_cmd_pub.publish(twist);
                	
			if(Offset <= Threshold){
			   // The marker with id " MarkerID " is found
			   std::cout << "marker with an id is found: "<< MarkerID << std::endl;
			   if (MarkerID < IDSmall ){
			      // The least marker ID is 
			      IDSmall = MarkerID;
			      // The waypoint of the marker that has the smallest ID so far
			      LeastPos = msg->parameters[1].value; 
			       
			       
			      // Set the values on the parameter server
			      ros::param::set("/IDSmall", IDSmall);
			      ros::param::set("/LeastPos", LeastPos);
			      std::cout << "Updated and stored parameters: IDSmall=" << IDSmall << ", LeastPos=" << LeastPos << std::endl;
			   }
			   else{
			      // Print something or do nothing 
			      std::cout << "No need to update the least marker ID " << IDSmall << " At the position : " << LeastPos <<  std::endl;
			   }
			   // Stop the robot from rotating
			   std::cout << "Stop the robot from rotating " << IDSmall<<std::endl;
			   twist.angular.z = 0.0;
                	   
                	   // Publish the angular speed
                	   vel_cmd_pub.publish(twist);
                	   
                	   break; // Exit the while loop
			}
			
		   }

		}
		else if(msg->name == "go_to_least_id"){
		int smallest_id;
		std::string least_pos;

		// Retrieve the stored values
		if (ros::param::get("/IDSmall", smallest_id) && ros::param::get("/LeastPos", least_pos)) {
		    std::cout << "Retrieved parameters: IDSmall=" << smallest_id 
			      << ", LeastPos=" << least_pos << std::endl;

		    // Update the member variables
		    IDSmall = smallest_id;
		    LeastPos = least_pos;

		    // Use the retrieved values to navigate
		    for (const auto& wp : wps) {
			if (wp.name == LeastPos) {
			    goal.target_pose.pose.position.x = wp.x;
			    goal.target_pose.pose.position.y = wp.y;
			    break;
			}
		    }
		    ac.sendGoal(goal);
		    ac.waitForResult();
		} else {
		    std::cerr << "Error: Parameters /IDSmall or /LeastPos are not set!" << std::endl;
		}

		}
	       return true;
	}
    }
int main(int argc, char **argv) {

	ros::init(argc, argv, "assignment2_exprob", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	static KCL_rosplan::MyActionInterface my_aci(nh);
	my_aci.runActionInterface();
	return 0;
}
