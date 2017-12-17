#include "NeedBattery.h"
#include "squirrel_planning_execution/ConfigReader.h"

#include <actionlib/client/simple_action_client.h>

#include "rosplan_dispatch_msgs/PlanAction.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"

#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/GetInstanceService.h"
#include "rosplan_knowledge_msgs/GetAttributeService.h"
#include "rosplan_knowledge_msgs/GenerateProblemService.h"
#include "rosplan_knowledge_msgs/KnowledgeQueryService.h"
#include "rosplan_dispatch_msgs/PlanningService.h"

namespace KCL_rosplan
{
	
NeedBattery::NeedBattery(ros::NodeHandle& nh)
	: nh_(nh), message_store_(nh)
{
	// knowledge interface
	update_knowledge_client_ = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
	query_knowledge_client_ = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeQueryService>("/kcl_rosplan/query_knowledge_base");
	
	get_instance_client_ = nh.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_current_instances");
	get_attribute_client_ = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_knowledge");
	
	
	// Read and process the configuration file.
	std::string config_file;
	nh.getParam("/scenario_setup_file", config_file);
	
	ConfigReader reader(nh);
	reader.readConfigurationFile(config_file, message_store_);
	
	// Initialise the knowledge base and MongoDB for this problem.
	initialiseKnowledgeBase();
	initialiseGoal();
}

void NeedBattery::initialiseKnowledgeBase()
{
	rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
	// Set kenny at it's starting waypoint.
	{
		rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
		knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
		knowledge_item.instance_type = "robot";
		knowledge_item.instance_name = "robot";
		
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client_.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the robot robot to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (BehaviourAndEmotion) Added robot to the knowledge base.");
	}
	{
		rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
		knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		knowledge_item.attribute_name = "robot_at";
		knowledge_item.is_negative = false;
		
		diagnostic_msgs::KeyValue kv;
		kv.key = "v";
		kv.value = "robot";
		knowledge_item.values.push_back(kv);
		
		kv.key = "wp";
		kv.value = "kenny_waypoint";
		knowledge_item.values.push_back(kv);
		
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client_.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the fact (robot_at robot kenny_waypoint) to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (BehaviourAndEmotion) Added the fact (robot_at robot kenny_waypoint) to the knowledge base.");
	}
	
	// Robot is not busy initially.
	{
		rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
		knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		knowledge_item.attribute_name = "not_busy";
		knowledge_item.is_negative = false;
		
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client_.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the fact (not_busy) to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (BehaviourAndEmotion) Added the fact (not_busy) to the knowledge base.");
	}
	
	// Robot is not flashing lights initially.
	{
		rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
		knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		knowledge_item.attribute_name = "not_flashing_lights";
		knowledge_item.is_negative = false;
		
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client_.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the fact (not_flashing_lights) to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (BehaviourAndEmotion) Added the fact (not_flashing_lights) to the knowledge base.");
	}
	
	// Robot is not playing sounds initially.
	{
		rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
		knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		knowledge_item.attribute_name = "not_playing_sound";
		knowledge_item.is_negative = false;
		
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client_.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the fact (not_playing_sound) to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (BehaviourAndEmotion) Added the fact (not_playing_sound) to the knowledge base.");
	}
	
	// Robot is not gazing initially.
	{
		rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
		knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		knowledge_item.attribute_name = "not_gazing";
		knowledge_item.is_negative = false;
		
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client_.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the fact (not_gazing) to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (BehaviourAndEmotion) Added the fact (not_gazing) to the knowledge base.");
	}
	
	// Gripper is empty.
	{
		rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
		knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		knowledge_item.attribute_name = "gripper_empty";
		knowledge_item.is_negative = false;
		
		diagnostic_msgs::KeyValue kv;
		kv.key = "v";
		kv.value = "robot";
		knowledge_item.values.push_back(kv);
		
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client_.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the fact (gripper_empty robot) to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (BehaviourAndEmotion) Added the fact (gripper_empty robot) to the knowledge base.");
	}
	
	// Set initial power.
	{
		rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
		diagnostic_msgs::KeyValue kv;
		kv.key = "v";
		kv.value = "robot";
		knowledge_item.values.push_back(kv);
		
		// Pleasure.
		knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FUNCTION;
		knowledge_item.attribute_name = "power";
		knowledge_item.function_value = 4.5f;
		
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client_.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the function (power %s) = %f to the knowledge base.", kv.value.c_str(), knowledge_item.function_value);
			exit(-1);
		}
		ROS_INFO("KCL: (BehaviourAndEmotion) Added (power %s) = %f to the knowledge base.", kv.value.c_str(), knowledge_item.function_value);
	}
}

void NeedBattery::initialiseGoal()
{
	rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
	// Set kenny at it's starting waypoint.
	{
		rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
		knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		knowledge_item.attribute_name = "child_has_oxygen";
		knowledge_item.is_negative = false;
		
		diagnostic_msgs::KeyValue kv;
		kv.key = "c";
		kv.value = "c1";
		knowledge_item.values.push_back(kv);
		
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client_.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the fact (child_has_oxygen c1) to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (BehaviourAndEmotion) Added the fact (child_has_oxygen c1) to the knowledge base.");
	}
}

};

int main(int argc, char **argv) {

	ros::init(argc, argv, "need_battery");
	ros::NodeHandle nh;

	// create PDDL action subscriber
	KCL_rosplan::NeedBattery need_battery(nh);
	
	// Start the planning process.
	std::string data_path;
	nh.getParam("/data_path", data_path);
	
	std::string planner_path;
	nh.getParam("/planner_path", planner_path);
	
	std::string domain_path;
	nh.getParam("/rosplan/domain_path", domain_path);
	
	std::stringstream ss;
	ss << data_path << "robot_behaviour_problem.pddl";
	std::string problem_path = ss.str();
	
	std::string planner_command;
	nh.getParam("/squirrel_planning_execution/planner_command", planner_command);
	
	rosplan_dispatch_msgs::PlanGoal psrv;
	psrv.domain_path = domain_path;
	psrv.problem_path = problem_path;
	psrv.data_path = data_path;
	psrv.planner_command = planner_command;
	psrv.start_action_id = 0;

	ROS_INFO("KCL: (RPSquirrelRecursion) Start plan action");
	actionlib::SimpleActionClient<rosplan_dispatch_msgs::PlanAction> plan_action_client("/kcl_rosplan/start_planning", true);

	plan_action_client.waitForServer();
	ROS_INFO("KCL: (RPSquirrelRecursion) Start planning server found");
	
	// send goal
	plan_action_client.sendGoal(psrv);
	ROS_INFO("KCL: (RPSquirrelRecursion) Goal sent");
	
	ros::spin();
	return 0;
}
