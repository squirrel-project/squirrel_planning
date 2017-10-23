#include "squirrel_hri_knowledge/PerformSocialBehaviour.h"
#include <squirrel_vad_msgs/vad.h>
#include <std_msgs/ColorRGBA.h>
#include <squirrel_hri_msgs/Expression.h>

namespace KCL_rosplan {

	/* constructor */
PerformSocialBehaviour::PerformSocialBehaviour(ros::NodeHandle &nh, const std::string& move_base_action_name)
	 : message_store(nh), arousal_threshold(0.25f), action_client(move_base_action_name)
{
	knowledgeInterface = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
	action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
	
	lights_pub_ = nh.advertise<std_msgs::ColorRGBA>("/light/command", 1, true);
	expression_pub_ = nh.advertise<std_msgs::String>("/expression", 1, true);
	
	nh.getParam("arousal_threshold", arousal_threshold);
	ROS_INFO("KCL: (PerformSocialBehaviour) Threshold set at %f.", arousal_threshold);
}

void PerformSocialBehaviour::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
{
		// ignore other actions
		if (msg->name != "accomodate-distress" &&
		    msg->name != "improve-distress" &&
		    msg->name != "accomodate-sadness" &&
		    msg->name != "improve-sadness" &&
		    msg->name != "improve-boredom" &&
		    msg->name != "maintain-happyness" &&
		    msg->name != "improve-introvert" &&
		    msg->name != "reciprocal-behaviour")
		{
			return;
		}

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
		float arousal = results[0]->energy;
		
		std_msgs::ColorRGBA color_command;
		
		// No arousal.
		if (arousal == -1.0)
		{
			// Do nothing for a bit...
			ros::Duration(10).sleep();
			
			// Then look at the children.
			// @todo Talk to Bajo.
			
			float red = 25;
			float green = 240;
			float blue = 50;
			ROS_INFO("KCL: (ShowLightsAction) Parsed the lights, showing (%f, %f, %f)", red, green, blue);
			
			color_command.r = red;
			color_command.g = green;
			color_command.b = blue;
			lights_pub_.publish(color_command);
			
			std_msgs::String sound_command;
			sound_command.data = squirrel_hri_msgs::Expression::HELLO;
			expression_pub_.publish(sound_command);
		}
		// Low arousal.
		else if (arousal < arousal_threshold)
		{
			// Go to some random location.
			float red = 25;
			float green = 40;
			float blue = 250;
			ROS_INFO("KCL: (ShowLightsAction) Parsed the lights, showing (%f, %f, %f)", red, green, blue);
			
			color_command.r = red;
			color_command.g = green;
			color_command.b = blue;
			lights_pub_.publish(color_command);
			
			std_msgs::String sound_command;
			sound_command.data = squirrel_hri_msgs::Expression::GOODBYE;
			expression_pub_.publish(sound_command);
		}
		// High arousal.
		else
		{
			// Make some noise!
			float red = 255;
			float green = 24;
			float blue = 50;
			ROS_INFO("KCL: (ShowLightsAction) Parsed the lights, showing (%f, %f, %f)", red, green, blue);
			
			color_command.r = red;
			color_command.g = green;
			color_command.b = blue;
			lights_pub_.publish(color_command);
			
			std_msgs::String sound_command;
			sound_command.data = squirrel_hri_msgs::Expression::CHEERING;
			expression_pub_.publish(sound_command);
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
	KCL_rosplan::	PerformSocialBehaviour psb(nh, "/move");
	ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::PerformSocialBehaviour::dispatchCallback, &psb);
	ROS_INFO("KCL: (PerformSocialBehaviour) Ready to receive");

	ros::spin();
	return 0;
}

