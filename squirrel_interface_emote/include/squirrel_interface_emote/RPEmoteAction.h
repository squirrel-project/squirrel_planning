#include <ros/ros.h>
#include <vector>
#include <squirrel_speech_msgs/RecognizedCommand.h>
#include <rosplan_dispatch_msgs/ActionDispatch.h>
#include <squirrel_vad_msgs/RecognisedResult.h>

#ifndef SQUIRREL_INTERFACE_EMOTE_RPEMOTEACTION_H
#define SQUIRREL_INTERFACE_EMOTE_RPEMOTEACTION_H

/**
 * This file defines the RPEmoteAction class.
 * RPEmoteAction is used by SQUIRREL to interpret the robot doing a little dance and emitting sound..
 * PDDL action "emote" makes Kenny say something and do a little wiggle. The knowledge base is unaffected
 * as it is only for the children's benefit and does not change the internal state of the robot (e.g.
 * this piece of kit is now 'happy' or 'dissapointed').
 */
namespace KCL_rosplan {
	
	class RPEmoteAction
	{

	private:
		ros::NodeHandle* node_handle_;
		
		ros::ServiceClient update_knowledge_client_;
		ros::ServiceClient get_instance_client_;
		ros::ServiceClient get_attribute_client_;
		
		ros::Subscriber dispatch_sub_;
		ros::Subscriber arousal_sub_;
		
		ros::Publisher action_feedback_pub_;
		ros::Publisher sound_pub_;
		ros::Publisher wiggle_pub_;

		float last_registered_arousal_;
		
	public:

		/* constructor */
		RPEmoteAction(ros::NodeHandle &nh);
		
		void dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);

		void registerArousal(const squirrel_vad_msgs::RecognisedResult::ConstPtr& msg);
	};
}
#endif
