#include "WaypointNavigation.hpp"
#include <iostream>

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>

using namespace waypoint_navigation_lib;

class Path {
  public:
    bool set = false;
    double tol_pos;
    double tol_head;

    double w;    
    double x;
    double y;
    double z;
    double yaw;

    std::vector<base::Waypoint> trajectory;
    std::vector<base::Waypoint*> ptrajectory;

    void PathCallback(geometry_msgs::PoseArray msg);
};

//  CREAR CLASE
base::Pose robotPose;
WaypointNavigation pathTracker;
Eigen::AngleAxisd robotRot;

void talker (proxy_library::MotionCommand mc);
void chatterCallback(geometry_msgs::Twist pos);


void Path::PathCallback(geometry_msgs::PoseArray msg)
{
    int size = msg.poses.size();
    trajectory.resize(size);
    ptrajectory.resize(size);

    for (size_t i =0; i < size; i++)
    {
        trajectory.at(i).position = Eigen::Vector3d(msg.poses.at(i).position.x, msg.poses.at(i).position.y, msg.poses.at(i).position.z);

        x = msg.poses.at(i).orientation.x;
        y = msg.poses.at(i).orientation.y;
        z = msg.poses.at(i).orientation.z;
        w = msg.poses.at(i).orientation.w;

        yaw = atan2(-2*x*y + 2*w*z, +w*w +x*x -y*y -z*z);
        trajectory.at(i).heading = yaw;

        trajectory.at(i).tol_position = tol_pos;
        trajectory.at(i).tol_heading = tol_head;
        ptrajectory.at(i) = &trajectory.at(i);
    }
    set = true;
}

void chatterCallback(geometry_msgs::Twist pos)
{
    robotPose.position = Eigen::Vector3d(pos.linear.x, pos.linear.y, pos.linear.z);
    robotRot = Eigen::AngleAxisd(pos.angular.z, Eigen::Vector3d::UnitZ());
    robotPose.orientation = Eigen::Quaterniond(robotRot);
    pathTracker.setPose(robotPose);
}


void talker (proxy_library::MotionCommand mc, ros::Publisher chatter_pub)
{
    geometry_msgs::Twist msg;
    msg.linear.x = mc.m_speed_ms;
    msg.angular.z = mc.m_turnRate_rads;

    chatter_pub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navegation");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::Subscriber sub = n.subscribe("pos", 1000, chatterCallback);
    ros::Rate loop_rate(10);

    Path path;
    std::cout << "Position tolerance?";
    std::cin >> path.tol_pos;

    std::cout << "Heading tolerance?";
    std::cin >> path.tol_head;

    ros::Subscriber sub2 = n.subscribe<geometry_msgs::PoseArray>("WayPoints", 1, &Path::PathCallback, &path);
  
    //WaypointNavigation pathTracker; --> Global                
    base::Waypoint lpoint;
    proxy_library::MotionCommand mc;

    // = = = Robot = = =
    // base::Pose robotPose;  --> Global
    robotPose.orientation.setIdentity();
    robotPose.position = Eigen::Vector3d(0, 0, 0);
    pathTracker.setPose(robotPose);

    while (ros::ok)
    {
        while (ros::ok and path.set==false)
        {   ros::spinOnce();    }
        
        pathTracker.setTrajectory(path.ptrajectory);
        std::cout << "Trajectory set" << std::endl;
        pathTracker.setNavigationState(DRIVING);

        while (pathTracker.getNavigationState() != TARGET_REACHED and ros::ok())
        {
            pathTracker.update(mc);
            talker (mc,chatter_pub);
            std::cout << "tv = " << mc.m_speed_ms;
            std::cout << ", rv = " << mc.m_turnRate_rads << std::endl;

             std::cout << "Robot = (" << robotPose.position.x() << "," 
                                 << robotPose.position.y() << ","
                                 << robotPose.position.z() << ")"
                  << "yaw = " << robotPose.getYaw() * 180 / M_PI << "deg." << std::endl;

            ros::spinOnce();
            loop_rate.sleep();
        }
        path.set = false;
    }
    
  return 0;
}
