#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>
#include <actionlib/client/simple_action_client.h>
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "squirrel_manipulation_msgs/PushAction.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "mongodb_store/message_store.h"
#include "geometry_msgs/PoseStamped.h"
#include "squirrel_interface_manipulation/RPPushAction.h"

/* The implementation of RPPushAction.h */
namespace KCL_rosplan {

	/* constructor */
	RPPushAction::RPPushAction(ros::NodeHandle &nh, std::string &actionserver, bool simulate)
	 : message_store(nh), action_client(actionserver, true), simulate_client("/move_base", true), simulate_(simulate) {
		
		// create the action client
		if(!simulate) {
			ROS_INFO("KCL: (PushAction) waiting for action server to start on %s", actionserver.c_str());
			action_client.waitForServer();
		} else {
			ROS_INFO("KCL: (PushAction) waiting for action server to start on /move_base");
			simulate_client.waitForServer();
		}

		// create the action feedback publisher
		action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
	}

	/* action dispatch callback */
	void RPPushAction::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		// ignore non-goto-waypoint actions
		if(0!=msg->name.compare("push_object")) return;

		ROS_INFO("KCL: (PushAction) action recieved");

		// get waypoint ID from action dispatch
		std::string wpID, objectID;
		bool foundWP = false;
		bool foundObject = false;
		for(size_t i=0; i<msg->parameters.size(); i++) {
			if(0==msg->parameters[i].key.compare("to")) {
				wpID = msg->parameters[i].value;
				foundWP = true;
			}
			if(0==msg->parameters[i].key.compare("ob")) {
				objectID = msg->parameters[i].value;
				foundObject = true;
			}
		}
		if(!foundWP || !foundObject) {
			ROS_INFO("KCL: (PushAction) aborting action dispatch; malformed parameters");
			return;
		}
		
		// get pose from message store
		std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > results;
		if(message_store.queryNamed<geometry_msgs::PoseStamped>(wpID, results)) {
			if(results.size()<1) {
				ROS_INFO("KCL: (PushAction) aborting action dispatch; no matching wpID %s", wpID.c_str());
				return;
			}
			if(results.size()>1)
				ROS_INFO("KCL: (PushAction) multiple waypoints share the same wpID");


			if(!simulate_) {
				// dispatch Push action
				squirrel_manipulation_msgs::PushGoal goal;
				goal.pose.position.x = results[0]->pose.position.x;
				goal.pose.position.y = results[0]->pose.position.y;
				goal.pose.orientation.w = 1;
				goal.object_id = objectID;
				action_client.sendGoal(goal);
			} else {
				// dispatch MoveBase action
				move_base_msgs::MoveBaseGoal goal;
				geometry_msgs::PoseStamped &pose = *results[0];
				goal.target_pose = pose;
				goal.target_pose.pose.orientation.w = 1;
				simulate_client.sendGoal(goal);
			}

			// publish feedback (enabled)
			rosplan_dispatch_msgs::ActionFeedback fb;
			fb.action_id = msg->action_id;
			fb.status = "action enabled";
			action_feedback_pub.publish(fb);

			
			bool finished_before_timeout = false;
			if(!simulate_) {
				finished_before_timeout = action_client.waitForResult(ros::Duration(msg->duration));
			} else {
				finished_before_timeout = simulate_client.waitForResult(ros::Duration(msg->duration));
			}

			if (finished_before_timeout) {

				if(!simulate_) {
					actionlib::SimpleClientGoalState state = action_client.getState();
					ROS_INFO("KCL: (PushAction) action finished: %s", state.toString().c_str());
				} else {
					actionlib::SimpleClientGoalState state = simulate_client.getState();
					ROS_INFO("KCL: (PushAction) action finished: %s", state.toString().c_str());
				}
				
				// publish feedback (achieved)
				rosplan_dispatch_msgs::ActionFeedback fb;
				fb.action_id = msg->action_id;
				fb.status = "action achieved";
				action_feedback_pub.publish(fb);

			} else {

				// publish feedback (failed)
				rosplan_dispatch_msgs::ActionFeedback fb;
				fb.action_id = msg->action_id;
				fb.status = "action failed";
				action_feedback_pub.publish(fb);

				ROS_INFO("KCL: (PushAction) action timed out");

			}

		} else ROS_INFO("KCL: (PushAction) aborting action dispatch; query to sceneDB failed");
	}
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_interface_pushaction");
		ros::NodeHandle nh;

		bool simulate = true;
		// nh.getParam("simulate", simulate);

		std::string actionserver;
		nh.param("action_server", actionserver, std::string("/push"));

		// create PDDL action subscriber
		KCL_rosplan::RPPushAction rppa(nh, actionserver, simulate);
	
		// listen for action dispatch
		ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPPushAction::dispatchCallback, &rppa);
		ROS_INFO("KCL: (PushAction) Ready to receive");

		ros::spin();
		return 0;
	}
