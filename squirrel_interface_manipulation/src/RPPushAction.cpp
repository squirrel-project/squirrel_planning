#include "squirrel_interface_manipulation/RPPushAction.h"

/* The implementation of RPPushAction.h */
namespace KCL_rosplan {

	/* constructor */
	RPPushAction::RPPushAction(ros::NodeHandle &nh, std::string &pushactionserver, std::string &smashactionserver, bool simulate)
	 : message_store(nh), push_action_client(pushactionserver, true), smash_action_client(smashactionserver, true), 
	   simulate_client("/move_base", true), simulate_(simulate) {

		// create the push action client
		if(!simulate) {
			ROS_INFO("KCL: (PushAction) waiting for action server to start on %s", pushactionserver.c_str());
			push_action_client.waitForServer();
			ROS_INFO("KCL: (PushAction) waiting for action server to start on %s", smashactionserver.c_str());
			smash_action_client.waitForServer();
		} else {
			ROS_INFO("KCL: (PushAction) waiting for action server to start on /move_base");
			simulate_client.waitForServer();
		}

		// create the action feedback publisher
		action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
	}

	/* action dispatch callback */
	void RPPushAction::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		// ignore non-push actions
		if(0==msg->name.compare("push_object")) dispatchPushAction(msg);
		if(0==msg->name.compare("smash_clutter")) dispatchSmashAction(msg);
	}

	/* push action dispatch */
	void RPPushAction::dispatchSmashAction(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		ROS_INFO("KCL: (PushAction) smash action recieved");

		// get waypoint ID from action dispatch
		std::string wpID;
		bool foundWP = false;
		for(size_t i=0; i<msg->parameters.size(); i++) {
			if(0==msg->parameters[i].key.compare("wp")) {
				wpID = msg->parameters[i].value;
				foundWP = true;
			}
		}
		if(!foundWP) {
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
				squirrel_manipulation_msgs::SmashGoal goal;
				goal.pose.position.x = results[0]->pose.position.x;
				goal.pose.position.y = results[0]->pose.position.y;
				goal.pose.position.z = 0.0;
				tf::Quaternion quat(tf::Vector3(0., 0., 1.), M_PI);
				goal.pose.orientation.w = quat.w();
				goal.pose.orientation.x = quat.x();
				goal.pose.orientation.y = quat.y();
				goal.pose.orientation.z = quat.z();
				goal.radius.data = 0.3f;
				smash_action_client.sendGoal(goal);

			} else {
				// dispatch MoveBase action
				move_base_msgs::MoveBaseGoal goal;
				geometry_msgs::PoseStamped &pose = *results[0];
				goal.target_pose = pose;
				simulate_client.sendGoal(goal);
			}

			// publish feedback (enabled)
			rosplan_dispatch_msgs::ActionFeedback fb;
			fb.action_id = msg->action_id;
			fb.status = "action enabled";
			action_feedback_pub.publish(fb);

			
			bool finished_before_timeout = false;
			if(!simulate_) {
				finished_before_timeout = smash_action_client.waitForResult(ros::Duration(msg->duration));
			} else {
				finished_before_timeout = simulate_client.waitForResult(ros::Duration(msg->duration));
			}

			if (finished_before_timeout) {

				if(!simulate_) {
					actionlib::SimpleClientGoalState state = smash_action_client.getState();
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

	/* push action dispatch */
	void RPPushAction::dispatchPushAction(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		ROS_INFO("KCL: (PushAction) push action recieved");

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
				goal.pose.position.z = 0.0;
				goal.pose.orientation.x = 0.0;
				goal.pose.orientation.y = 0.0;
				goal.pose.orientation.z = 0.0;
				goal.pose.orientation.w = 1;
				goal.object_id ="lump"; // objectID;
				push_action_client.sendGoal(goal);
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
				finished_before_timeout = push_action_client.waitForResult(ros::Duration(msg->duration));
			} else {
				finished_before_timeout = simulate_client.waitForResult(ros::Duration(msg->duration));
			}

			if (finished_before_timeout) {

				if(!simulate_) {
					actionlib::SimpleClientGoalState state = push_action_client.getState();
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

		bool simulate = false;
		nh.getParam("simulate_pushing", simulate);

		std::string pushactionserver, smashactionserver;
		nh.param("push_action_server", pushactionserver, std::string("/push"));
		nh.param("smash_action_server", smashactionserver, std::string("/smash"));

		// create PDDL action subscriber
		KCL_rosplan::RPPushAction rppa(nh, pushactionserver, smashactionserver, simulate);
	
		// listen for action dispatch
		ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPPushAction::dispatchCallback, &rppa);
		ROS_INFO("KCL: (PushAction) Ready to receive");

		ros::spin();
		return 0;
	}
