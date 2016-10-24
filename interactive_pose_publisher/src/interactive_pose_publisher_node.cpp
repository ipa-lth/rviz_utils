/****************************************************************
*
* Copyright (c) 2014
*
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA)
*
* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
* Description: This node provides a simple interactive marker to publish poses in tf and under a topic.
*
* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
* Author: Frank Naegele, email:frank.naegele@ipa.fraunhofer.de
*
* Date of creation: Jan 2014
*
* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*         * Redistributions of source code must retain the above copyright
*         notice, this list of conditions and the following disclaimer.
*         * Redistributions in binary form must reproduce the above copyright
*         notice, this list of conditions and the following disclaimer in the
*         documentation and/or other materials provided with the distribution.
*         * Neither the name of the Fraunhofer Institute for Manufacturing
*         Engineering and Automation (IPA) nor the names of its
*         contributors may be used to endorse or promote products derived from
*         this software without specific prior written permission.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License LGPL as
* published by the Free Software Foundation, either version 3 of the
* License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License LGPL along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*
****************************************************************/


// ROS //
//
#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <visualization_msgs/Marker.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include "interactive_pose_publisher/SetToFrame.h"

#include <dynamic_reconfigure/server.h>
#include <interactive_pose_publisher/interactive_pose_publisherConfig.h>

#include <fstream>

// Global Variables //
//
std::string             g_parent_frame;
std::string             g_child_frame;
std::string             g_marker_name;
std::string             g_topic_name;

boost::mutex            g_mutex;

tf::StampedTransform    g_transform;

interactive_markers::InteractiveMarkerServer * g_server = NULL;
tf::TransformBroadcaster * g_broadcaster = NULL;
tf::TransformListener * g_listener = NULL;

interactive_pose_publisher::interactive_pose_publisherConfig g_config;


/*! Feedback from interactive marker.
*
* Saves the feedback into the global variable g_transform.
*/
void
    processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback )
{
    boost::mutex::scoped_lock lock(g_mutex);


    // Print //
    //
    //ROS_INFO_STREAM( feedback->marker_name << " is now at " << feedback->pose.position.x << ", " << feedback->pose.position.y << ", " << feedback->pose.position.z );


    // Convert message to tf pose //
    //
    tf::Pose pose;
    tf::poseMsgToTF(feedback->pose, pose);

    g_transform.setBasis(tf::Matrix3x3(pose.getRotation()));
    g_transform.setOrigin(pose.getOrigin());
}


/*! Initializes the marker
*
*/
bool
    start()
{
    boost::mutex::scoped_lock lock(g_mutex);
    ros::NodeHandle n;

    // Interactive marker //
    //
    visualization_msgs::InteractiveMarker int_marker;

    int_marker.header.frame_id = g_parent_frame;
    int_marker.description = g_marker_name;
    int_marker.scale = g_config.marker_size;
    int_marker.name = g_marker_name;

    tf::Vector3 origin = g_transform.getOrigin();
    int_marker.pose.position.x = origin.x();
    int_marker.pose.position.y = origin.y();
    int_marker.pose.position.z = origin.z();

    tf::Quaternion quat = g_transform.getRotation();
    int_marker.pose.orientation.x = quat.x();
    int_marker.pose.orientation.y = quat.y();
    int_marker.pose.orientation.z = quat.z();
    int_marker.pose.orientation.w = quat.w();


    // Modify control: Arrows //
    //
    visualization_msgs::InteractiveMarkerControl marker_control;

    marker_control.always_visible = true;

    marker_control.orientation.w = 1;
    marker_control.orientation.x = 1;
    marker_control.orientation.y = 0;
    marker_control.orientation.z = 0;
    marker_control.name = "move_x";
    marker_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(marker_control);
    marker_control.name = "rotate_x";
    marker_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(marker_control);

    marker_control.orientation.w = 1;
    marker_control.orientation.x = 0;
    marker_control.orientation.y = 1;
    marker_control.orientation.z = 0;
    marker_control.name = "move_z";
    marker_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(marker_control);
    marker_control.name = "rotate_z";
    marker_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(marker_control);

    marker_control.orientation.w = 1;
    marker_control.orientation.x = 0;
    marker_control.orientation.y = 0;
    marker_control.orientation.z = 1;
    marker_control.name = "move_y";
    marker_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(marker_control);
    marker_control.name = "rotate_y";
    marker_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(marker_control);


    // Update //
    //
    g_server->insert(int_marker);
    g_server->setCallback(int_marker.name, &processFeedback);

    g_server->applyChanges();

    return true;
}



/*! Removes the marker
*
*/
bool
    stop()
{
    boost::mutex::scoped_lock lock(g_mutex);

    g_server->erase("interactive_marker");
    g_server->applyChanges();

    return true;
}


/*! Set the transform to a tf frame given by its name.
*
*/
bool
    setFrame(interactive_pose_publisher::SetToFrame::Request & req, interactive_pose_publisher::SetToFrame::Response & res)
{
    tf::StampedTransform transform;
    try
    {
        g_listener->lookupTransform(g_parent_frame, req.frame, ros::Time(0), transform);
    }
    catch(...)
    {
        ROS_ERROR("Transformation not found!");
        return false;
    }


    // Save //
    //
    stop();
    {
        boost::mutex::scoped_lock lock(g_mutex);
        g_transform = transform;
    }
    start();

    return true;
}

void dynamic_reconfigure_cb(interactive_pose_publisher::interactive_pose_publisherConfig &config, uint32_t level)
{
    if (config.save_pose)
    {
        //char resolved_path[100];
        //realpath(config.save_path.c_str(), resolved_path);

        // Only absolute path so far..
        ROS_WARN("Writing current pose to %s", config.save_path.c_str());

        std::ofstream file(config.save_path.c_str());
        std::stringstream ss;
        ss << "initial_pose:\n";
        ss << "  position:\n";
        ss << "    x: " << g_transform.getOrigin().getX() << "\n";
        ss << "    y: " << g_transform.getOrigin().getY() << "\n";
        ss << "    z: " << g_transform.getOrigin().getZ() << "\n";
        ss << "  orientation:\n";
        ss << "    x: " << g_transform.getRotation().x() << "\n";
        ss << "    y: " << g_transform.getRotation().y() << "\n";
        ss << "    z: " << g_transform.getRotation().z() << "\n";
        ss << "    w: " << g_transform.getRotation().w() << "\n";
        ss << "interactive_pose_topic: " << g_topic_name <<"\n";
        ss << "interactive_pose_parent: " << g_parent_frame << "\n";
        ss << "interactive_pose_child: " << g_child_frame << "\n";

        file << ss.str();
        config.save_pose = false;
    }


    if (level & 0x1)
    {
        stop();
        {
            boost::mutex::scoped_lock lock(g_mutex);
            g_config = interactive_pose_publisher::interactive_pose_publisherConfig(config);
        }
        start();
    }
}

/*! Main function
*
*/
int main( int argc, char** argv )
{
    // ROS //
    //
    ros::init(argc, argv, "interactive_pose_publisher");
    ros::NodeHandle node;
    ros::NodeHandle privateNode("~");
    ros::Rate r(10);


    // Parameters //
    //
    privateNode.param<std::string>("interactive_pose_topic", g_topic_name, "interactive_pose");

    privateNode.param<std::string>("interactive_pose_parent", g_parent_frame, "/world");
    privateNode.param<std::string>("interactive_pose_child", g_child_frame, "/marker");

    privateNode.param<std::string>("interactive_marker_name", g_marker_name, ros::this_node::getName()); // multiple markers mustn't have the same names, so default is node name


    // tf Broadcaster //
    //
    g_server = new interactive_markers::InteractiveMarkerServer("interactive_marker_server","",false);
    g_broadcaster = new tf::TransformBroadcaster();
    g_listener = new tf::TransformListener();


    // Pose Publisher //
    //
    ros::Publisher pub = node.advertise<geometry_msgs::PoseStamped>(g_topic_name, 10);
    ros::ServiceServer service_set_frame = node.advertiseService("interactive_pose_publisher_setFrame", setFrame);


    // Transformation //
    //
    geometry_msgs::Pose initial_pose;

    privateNode.param<double>("initial_pose/position/x",initial_pose.position.x, 0);
    privateNode.param<double>("initial_pose/position/y",initial_pose.position.y, 0);
    privateNode.param<double>("initial_pose/position/z",initial_pose.position.z, 0);
    privateNode.param<double>("initial_pose/orientation/x",initial_pose.orientation.x, 0);
    privateNode.param<double>("initial_pose/orientation/y",initial_pose.orientation.y, 0);
    privateNode.param<double>("initial_pose/orientation/z",initial_pose.orientation.z, 0);
    privateNode.param<double>("initial_pose/orientation/w",initial_pose.orientation.w, 1);

    g_transform.setOrigin(tf::Vector3(initial_pose.position.x,initial_pose.position.y,initial_pose.position.z));
    g_transform.setRotation(tf::Quaternion(initial_pose.orientation.x,initial_pose.orientation.y,initial_pose.orientation.z,initial_pose.orientation.w));

    //g_transform.setBasis(tf::Matrix3x3(initial_pose.position));
    //g_transform.setOrigin(initial_pose.getOrigin());
    //g_transform.setIdentity();

    // Dynamic reconfigure //
    //
    dynamic_reconfigure::Server<interactive_pose_publisher::interactive_pose_publisherConfig> server;
    dynamic_reconfigure::Server<interactive_pose_publisher::interactive_pose_publisherConfig>::CallbackType f;

    f = boost::bind(&dynamic_reconfigure_cb, _1, _2);
    server.setCallback(f);


    // Display Marker //
    //
    start();


    // LOOP //
    //
    while (ros::ok())
    {
        ros::spinOnce();


        // Broadcast Frame //
        //
        {
            boost::mutex::scoped_lock lock(g_mutex);


            // tf Broadcast //
            //
            g_broadcaster->sendTransform(tf::StampedTransform(g_transform, ros::Time::now(), g_parent_frame, g_child_frame ));


            // Pose Publisher //
            //
            geometry_msgs::Pose pose;
            tf::poseTFToMsg(g_transform, pose);

            geometry_msgs::PoseStamped poseStamped;
            poseStamped.header.stamp = ros::Time::now();
            poseStamped.header.frame_id = g_parent_frame;
            poseStamped.pose = pose;

            pub.publish(poseStamped);
        }

        r.sleep();
    }


    // Remove Marker //
    //
    stop();
}

//

