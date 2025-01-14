#ifndef MY_ACTION_H
#define MY_ACTION_H

#include <ros/ros.h>
#include "rosplan_action_interface/RPActionInterface.h"
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

// Structure that represents the way point and its x,y position
struct NamedWaypoints {
    std::string name;
    double x;
    double y;
};


namespace KCL_rosplan {

    class MyActionInterface : public RPActionInterface {
        private:
            // Subscribers
            ros::Subscriber marker_id_sub;
            ros::Subscriber marker_pos_sub;
            ros::Subscriber camera_info_sub;

            // Publishers
            ros::Publisher vel_cmd_pub;

            // Variables for marker detection and navigation
            double MarkerCenterX;
            double MarkerCenterY;
            double CameraWidth;
            double Threshold;
            double Offset;
            int MarkerID;
            

        public:
            /* Constructor */
            MyActionInterface(ros::NodeHandle& nh);

            /* Listen to and process action_dispatch topic */
            bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);

            /* Callback methods */
            void markerCenterCallback(const geometry_msgs::Point::ConstPtr& msg);
            void markerIDCallback(const std_msgs::Int32::ConstPtr& msg);
            void cameraInfoCallback(const std_msgs::Int32::ConstPtr& msg);
            int IDSmall;
            std::string LeastPos;
    };
}

#endif

