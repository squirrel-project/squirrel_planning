#include <ros/ros.h>
#include <vector>
#include <squirrel_speech_msgs/RecognizedCommand.h>

#ifndef SQUIRREL_INTERFACE_SPEECH_RPSPEECHACTION_H
#define SQUIRREL_INTERFACE_SPEECH_RPSPEECHACTION_H

/**
 * This file defines the RPSpeechAction class.
 * RPSpeechAction is used by SQUIRREL to interpret speech.
 * PDDL "observe-has_spoken" observations actions check the knowledge base to see if 
 * a command has been spoken. We currently only care about the commands:
 * "gehe" - which is a command for Kenny to observe the dinosaur.
 * "links" - notifies Kenny that it is the next child's turn.
 */
namespace KCL_rosplan {
	
	class RPSpeechAction
	{

	private:
		ros::NodeHandle* node_handle_;
		
		ros::ServiceClient update_knowledge_client_;
		ros::ServiceClient get_instance_client_;
		ros::ServiceClient get_attribute_client_;
		ros::Publisher sound_pub_;
		
		ros::Subscriber command_stream_;                // Receive commands from the kids.
		std::vector<squirrel_speech_msgs::RecognizedCommand> active_commands_;   // All active commands that have been issued recently.

		void updateKnowledgeBase(const squirrel_speech_msgs::RecognizedCommand& command, bool remove);
		
	public:

		/* constructor */
		RPSpeechAction(ros::NodeHandle &nh);
		
		void processSpeechCommand(const squirrel_speech_msgs::RecognizedCommand::ConstPtr& msg);
		
		void purgeOldCommands();
	};
}
#endif
