#ifndef SQUIRREL_INTERFACE_SPEECH_VADSPEECHACTION_H
#define SQUIRREL_INTERFACE_SPEECH_VADSPEECHACTION_H

#include <ros/ros.h>
#include <squirrel_vad_msgs/vad.h>
#include <mongodb_store/message_store.h>

/**
 * VAD is an interface for registering the child's emotional state based on speech.
 * This class captures the topic's output and registers the child's emotional state
 * in the knowledge base.
 */
namespace KCL_rosplan {
	
	class VADSpeechAction
	{

	private:
		ros::NodeHandle* node_handle_;
		
		mongodb_store::MessageStoreProxy message_store_;
		ros::Subscriber command_stream_;
		ros::Publisher sound_pub_;
	public:

		/* constructor */
		VADSpeechAction(ros::NodeHandle &nh);
		
		void processVADSpeech(const squirrel_vad_msgs::vad::ConstPtr& msg);
	};
}

#endif

