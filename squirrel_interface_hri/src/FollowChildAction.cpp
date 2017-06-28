#include "squirrel_hri_knowledge/FollowChildAction.h"
#include <std_srvs/Empty.h>

namespace KCL_rosplan {

	/* constructor */
FollowChildAction::FollowChildAction(ros::NodeHandle &nh, const std::string& follow_child_action_name, const std::vector<geometry_msgs::Point>& cd)
	 : message_store(nh), action_client(follow_child_action_name, true), child_destinations(cd)
{
	knowledgeInterface = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
	action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);

	ROS_INFO("KCL: (FollowChildAction) waiting for follow_child_action action server to start on %s", follow_child_action_name.c_str());
	action_client.waitForServer();
	ROS_INFO("KCL: (FollowChildAction) action server started.");
	sound_pub_ = nh.advertise<std_msgs::String>("/expression", 1, true);
	clear_cost_map_client = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
}

void FollowChildAction::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
{
		// ignore other actions
		if(0!=msg->name.compare("follow_child")) return;

		ROS_INFO("KCL: (FollowChildAction) action recieved");
		

		// publish feedback (enabled)
		rosplan_dispatch_msgs::ActionFeedback fb;
		fb.action_id = msg->action_id;
		fb.status = "action enabled";
		action_feedback_pub.publish(fb);

		
		// get waypoint ID from action dispatch
		std::string childID;
		bool found = false;
		for(size_t i=0; i<msg->parameters.size(); i++) {
			if(0==msg->parameters[i].key.compare("c")) {
				childID = msg->parameters[i].value;
				found = true;
			}
		}
		if(!found) {
			ROS_INFO("KCL: (FollowChildAction) aborting action dispatch; malformed parameters");
			return;
		}
		
		// dispatch MoveBase action
		squirrel_hri_msgs::FollowChildGoal goal;
		goal.child_id_to_follow.data = childID.c_str();
		goal.time_standing_still = 10;
		goal.target_locations = child_destinations;
		
		action_client.sendGoal(goal);

		std_msgs::String sound_command;
		sound_command.data = "HERE_HERE";
		sound_pub_.publish(sound_command);

		bool finished_before_timeout = action_client.waitForResult();
		if (finished_before_timeout) {

			actionlib::SimpleClientGoalState state = action_client.getState();
			ROS_INFO("KCL: (FollowChildAction) action finished: %s", state.toString().c_str());

			if(state != actionlib::SimpleClientGoalState::SUCCEEDED) {

				// publish feedback (achieved)
				fb.action_id = msg->action_id;
				fb.status = "action failed";
				action_feedback_pub.publish(fb);
				return;
			}
			else
			{
				// Add the child's location to the knowledge base.
				squirrel_hri_msgs::FollowChildResultConstPtr result = action_client.getResult();
				geometry_msgs::PoseStamped child_location = result->final_location;

				if (last_stored_id != "")
				{
					if (message_store.deleteID(last_stored_id))
					{
						ROS_INFO("KCL: (FollowChildAction) Deleted previous child pose with id: %s.", last_stored_id.c_str());
					}
				}

				last_stored_id = message_store.insertNamed("child_location", child_location);
				message_store.updateNamed("child_location", child_location);
				ROS_INFO("KCL: (FollowChildAction) Child is at (%f,%f,%f,)", child_location.pose.position.x, child_location.pose.position.y, child_location.pose.position.z);
			}
		} else {
			// timed out (failed)
			action_client.cancelAllGoals();
			ROS_INFO("KCL: (FollowChildAction) action timed out");
			
			fb.action_id = msg->action_id;
			fb.status = "action failed";
			action_feedback_pub.publish(fb);
			
			return;
		}

		sound_command.data = "CHEERING";
		sound_pub_.publish(sound_command);

		std_srvs::Empty dummy;
		clear_cost_map_client.call(dummy);
		
		// publish feedback (achieved)
		fb.action_id = msg->action_id;
		fb.status = "action achieved";
		action_feedback_pub.publish(fb);
}

};

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

	ros::init(argc, argv, "rosplan_follow_child_action");
	ros::NodeHandle nh;

	// create PDDL action subscriber
	std::vector<geometry_msgs::Point> child_destinations;
	{
	geometry_msgs::Point p;
	p.x = -1.03;
	p.y = 1.72;
	p.z = 0;
	child_destinations.push_back(p);
	}
	{
	geometry_msgs::Point p;
	p.x = 2.17;
	p.y = -2.06;
	p.z = 0;
	child_destinations.push_back(p);
	}
	{
	geometry_msgs::Point p;
	p.x = -1.31;
	p.y = -2.12;
	p.z = 0;
	child_destinations.push_back(p);
	}
	KCL_rosplan::FollowChildAction fca(nh, "/squirrel_follow_child_node", child_destinations);

	// listen for action dispatch
	ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::FollowChildAction::dispatchCallback, &fca);
	ROS_INFO("KCL: (FollowChildAction) Ready to receive");


	ros::spin();
	return 0;
}

