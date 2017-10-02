#include "squirrel_hri_knowledge/PerformSocialBehaviour.h"

namespace KCL_rosplan {

	/* constructor */
PerformSocialBehaviour::PerformSocialBehaviour(ros::NodeHandle &nh)
	 : message_store(nh), arousal_threshold(0.25f)
{
	knowledgeInterface = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
	action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
	
	nh.getParam("arousal_threshold", arousal_threshold);
}

bool PerformSocialBehaviour::gotoLocation(const geometry_msgs::PoseStamped& location)
{
	ROS_INFO("KCL: (GotoViewWaypointPDDLAction) waiting for move_base action server to start");
	action_client.waitForServer();

	std::cout << "KCL: (GotoViewWaypointPDDLAction) Goto (" << location.pose.position.x << ", " << location.pose.position.y << ", " << location.pose.position.z << ")." << std::endl;
	
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose = location;
	action_client.sendGoal(goal);

	bool finished_before_timeout = action_client.waitForResult();
	if (finished_before_timeout) {

		actionlib::SimpleClientGoalState state = action_client.getState();
		ROS_INFO("KCL: (GotoViewWaypointPDDLAction) action finished: %s", state.toString().c_str());

		if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
			return true;

		}
	} else {
		// timed out (failed)
		action_client.cancelAllGoals();
		ROS_INFO("KCL: (GotoViewWaypointPDDLAction) action timed out");
	}
	return false;
}


void PerformSocialBehaviour::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
{
		// ignore other actions
		if(0!=msg->name.compare("perform_social_behaviour")) return;

		ROS_INFO("KCL: (PerformSocialBehaviour) action recieved");
		
		// publish feedback (enabled)
		rosplan_dispatch_msgs::ActionFeedback fb;
		fb.action_id = msg->action_id;
		fb.status = "action enabled";
		action_feedback_pub.publish(fb);
		
		// Get the latest arausal level.
		std::vector< boost::shared_ptr<squirrel_vad_msgs::vad> > results;
		message_store.queryNamed<squirrel_vad_msgs::vad>("vad", results);
		
		if (results.empty())
		{
			ROS_ERROR("KCL: (perform_social_behaviour) Could not receive the arausal level from the message store.");
			exit(1);
		}
		
		// Depending on the level we perform different social behaviour.
		float arousal = results[0];
		
		// No arousal.
		if (arousal == 0)
		{
			// Do nothing for a bit...
			ros::Duration(10)::sleep();
			
			// Then look at the children.
			// @todo Talk to Bajo.
		}
		// Low arousal.
		else if (arousal < arousal_threshold)
		{
			// Go to some random location.
		}
		// High arousal.
		else
		{
			// Make some noise!
			
		}
		
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
			ROS_INFO("KCL: (PerformSocialBehaviour) aborting action dispatch; malformed parameters");
			return;
		}
		
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

	ros::init(argc, argv, "rosplan_perform_social_behaviour");
	ros::NodeHandle nh;
	
	// listen for action dispatch
	ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::PerformSocialBehaviour::dispatchCallback, &fca);
	ROS_INFO("KCL: (PerformSocialBehaviour) Ready to receive");

	ros::spin();
	return 0;
}

