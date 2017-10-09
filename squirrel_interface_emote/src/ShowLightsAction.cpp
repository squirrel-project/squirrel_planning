#include <algorithm>

#include "squirrel_interface_emote/ShowLightsAction.h"

#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>
#include <rosplan_knowledge_msgs/GetInstanceService.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_dispatch_msgs/ActionFeedback.h>

#include <std_msgs/ColorRGBA.h>

/* The implementation of ShowLightsAction.h */
namespace KCL_rosplan {

	/* constructor */
	ShowLightsAction::ShowLightsAction(ros::NodeHandle &nh)
		: node_handle_(&nh)
	{
		// knowledge interface
		update_knowledge_client_ = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
		get_instance_client_ = nh.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_current_instances");
		get_attribute_client_ = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_knowledge");
		action_feedback_pub_ = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
		
		dispatch_sub_ = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::ShowLightsAction::dispatchCallback, this);
		
		lights_pub_ = nh.advertise<std_msgs::ColorRGBA>("/light/command", 1, true);
	}
	
	void ShowLightsAction::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	{
		std::string normalised_action_name = msg->name;
		std::transform(normalised_action_name.begin(), normalised_action_name.end(), normalised_action_name.begin(), tolower);
		
		// Check if this action is to be handled by this class.
		if (normalised_action_name != "show_lights" || msg->parameters.size() != 2)
		{
			return;
		}
		
		ROS_INFO("KCL: (ShowLightsAction) Process the action: %s", normalised_action_name.c_str());

		
		// Report this action is enabled and completed successfully.
		rosplan_dispatch_msgs::ActionFeedback fb;
		fb.action_id = msg->action_id;
		fb.status = "action enabled";
		action_feedback_pub_.publish(fb);
		
		// Update the domain.
		const std::string& robot = msg->parameters[0].value;
		const std::string& child = msg->parameters[1].value;
		
		ROS_INFO("KCL: (ShowLightsAction) Process the action: %s, %s: show lights to %s", normalised_action_name.c_str(), robot.c_str(), child.c_str());

		
		// Show the light.
	
		std_msgs::ColorRGBA color_command;
		
		/*
		std::size_t red_pos = light.find('_', std::string("show_lights_").size());
		std::size_t green_pos = light.find('_', red_pos);
		
		float red = ::atof(light.substr(std::string("show_lights_").size(), red_pos).c_str());
		float green = ::atof(light.substr(red_pos + 1, green_pos).c_str());
		float blue = ::atof(light.substr(green_pos + 1).c_str());
		*/
		float red = 25;
		float green = 240;
		float blue = 50;
		ROS_INFO("KCL: (ShowLightsAction) Parsed the lights, showing (%f, %f, %f)", red, green, blue);
		
		color_command.r = red;
		color_command.g = green;
		color_command.b = blue;
		lights_pub_.publish(color_command);
		
		ros::spinOnce();
		
		fb.action_id = msg->action_id;
		fb.status = "action achieved";
		action_feedback_pub_.publish(fb);
	}
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_interface_show_lights");
		ros::NodeHandle nh;

		// create PDDL action subscriber
		KCL_rosplan::ShowLightsAction rpea(nh);
	
		ROS_INFO("KCL: (ShowLightsAction) Ready to receive");

		ros::spin();
		return 0;
	}
