#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>
#include <actionlib/client/simple_action_client.h>
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "squirrel_object_perception_msgs/LookForObjectsAction.h"
#include "squirrel_interface_perception/RPPerceptionAction.h"
#include "squirrel_planning_knowledge_msgs/AddObjectService.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "mongodb_store/message_store.h"
#include "geometry_msgs/PoseStamped.h"

/* The implementation of RPMoveBase.h */
namespace KCL_rosplan {

	/* constructor */
	RPPerceptionAction::RPPerceptionAction(ros::NodeHandle &nh, std::string &actionserver, bool simulate)
	 : message_store(nh), action_client(actionserver, true), movebase_client("/move_base", true), simulate_(simulate) {
		
		// create the action client
		ROS_INFO("KCL: (MoveBase) waiting for action server to start on /move_base");
		movebase_client.waitForServer();

		if(!simulate) {
			// create the action clients
			ROS_INFO("KCL: (PerceptionAction) waiting for action server to start on %s", actionserver.c_str());
			action_client.waitForServer();
		} else {
			// create add object client
			add_object_client = nh.serviceClient<squirrel_planning_knowledge_msgs::AddObjectService>("/kcl_rosplan/add_object");
		}
		
		// create the action feedback publisher
		action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
	}

	/* action dispatch callback; parameters (?v - robot ?wp - waypoint) */
	void RPPerceptionAction::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		// ignore non-goto-waypoint actions
		if(0!=msg->name.compare("explore_waypoint")) return;

		ROS_INFO("KCL: (PerceptionAction) action recieved");

		// get waypoint ID from action dispatch
		std::string wpID;
		bool found = false;
		for(size_t i=0; i<msg->parameters.size(); i++) {
			if(0==msg->parameters[i].key.compare("wp")) {
				wpID = msg->parameters[i].value;
				found = true;
			}
		}
		if(!found) {
			ROS_INFO("KCL: (PerceptionAction) aborting action dispatch; malformed parameters");
			return;
		}

		// get pose from message store
		std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > results;
		if(message_store.queryNamed<geometry_msgs::PoseStamped>(wpID, results)) {
			if(results.size()<1) {
				ROS_INFO("KCL: (MoveBase) aborting action dispatch; no matching wpID %s", wpID.c_str());
				publishFeedback(msg->action_id,"action failed");
				return;
			}
			if(results.size()>1)
				ROS_ERROR("KCL: (MoveBase) multiple waypoints share the same wpID");
		} else {
			// publish feedback (failed)
			publishFeedback(msg->action_id,"action failed");
			return;
		}

		if(!simulate_) {
			// dispatch Perception action
			squirrel_object_perception_msgs::LookForObjectsGoal goal;
			goal.look_for_object = squirrel_object_perception_msgs::LookForObjectsGoal::EXPLORE;
			action_client.sendGoal(goal);
		}

		// publish feedback (enabled)
		publishFeedback(msg->action_id,"action enabled");

		if(!simulate_) {

			bool success = true;
			for(int i=0; i<3; i++) {

				// rotate to angle
				ROS_INFO("KCL: (PerceptionAction) rotate to angle: %f", (i*2.7));

				// dispatch MoveBase action
				move_base_msgs::MoveBaseGoal goal;
				geometry_msgs::PoseStamped &pose = *results[0];
				goal.target_pose = pose;
				goal.target_pose.pose.orientation.w = (i*0.33);
				goal.target_pose.pose.orientation.x = 0;
				goal.target_pose.pose.orientation.y = 0;
				goal.target_pose.pose.orientation.z = sqrt(1-(i*0.33)*(i*0.33));
				movebase_client.sendGoal(goal);

				bool rotatedToAngle = movebase_client.waitForResult(ros::Duration(5*msg->duration));
				if(!rotatedToAngle) success = false;

				// observe
				bool finished_before_timeout = action_client.waitForResult(ros::Duration(msg->duration));
				actionlib::SimpleClientGoalState state = action_client.getState();
				if(!finished_before_timeout) success = false;
				ROS_INFO("KCL: (PerceptionAction) observe (%i) finished: %s", i, state.toString().c_str());
			}

			if (success) {
				ROS_INFO("KCL: (PerceptionAction) action complete");
				publishFeedback(msg->action_id, "action achieved");
			} else {
				ROS_INFO("KCL: (PerceptionAction) action timed out");
				publishFeedback(msg->action_id, "action failed");
			}

		} else {

			// ask user if there is a new object
			int i;
			std::cout << "Is there an object? (y/N)\n";
			std::cout << "> ";
			std::cin >> i;
			if('y' == i) {
				geometry_msgs::PoseStamped &pose = *results[0];
				squirrel_planning_knowledge_msgs::AddObjectService aos;
				aos.request.id = "simulated_object";
				aos.request.category = "ghost";
				aos.request.pose = pose;
				if(add_object_client.call(aos))
					ROS_INFO("KCL: (PerceptionAction) added object");
				else
					ROS_INFO("KCL: (PerceptionAction) Failed to add object");
			} else {
				
			}

			// publish feedback (achieved)
			ROS_INFO("KCL: (PerceptionAction) simulated action finished");
			publishFeedback(msg->action_id, "action achieved");
		}
	}

	void RPPerceptionAction::publishFeedback(int action_id, std::string feedback) {
		// publish feedback
		rosplan_dispatch_msgs::ActionFeedback fb;
		fb.action_id = action_id;
		fb.status = feedback;
		action_feedback_pub.publish(fb);
	}
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_interface_perception");
		ros::NodeHandle nh;

		bool simulate;
		nh.getParam("simulate", simulate);
		//nh.getParam("simulate_perception", simulate);

		std::string actionserver;
		nh.param("action_server", actionserver, std::string("/look_for_objects"));

		// create PDDL action subscriber
		KCL_rosplan::RPPerceptionAction rppa(nh, actionserver, simulate);
	
		// listen for action dispatch
		ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPPerceptionAction::dispatchCallback, &rppa);
		ROS_INFO("KCL: (PerceptionAction) Ready to receive");

		ros::spin();
		return 0;
	}
