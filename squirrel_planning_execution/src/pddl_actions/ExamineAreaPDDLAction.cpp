#include <map>
#include <vector>
#include <iostream>
#include <sstream>

#include <std_msgs/Int8.h>

#include <rosplan_dispatch_msgs/ActionFeedback.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_knowledge_msgs/GetInstanceService.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>

#include "squirrel_planning_execution/ContingentStrategicClassifyPDDLGenerator.h"

#include "ExamineAreaPDDLAction.h"
#include "PlannerInstance.h"



namespace KCL_rosplan {

	std::string ExamineAreaPDDLAction::g_action_name = "examine_area";
	
	ExamineAreaPDDLAction::ExamineAreaPDDLAction(ros::NodeHandle& node_handle)
		: node_handle_(&node_handle)
	{
		update_knowledge_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
		
		// create the action feedback publisher
		action_feedback_pub_ = node_handle.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
		
		get_instance_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_current_instances");
		get_attribute_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_knowledge");
		
		dispatch_sub_ = node_handle.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::ExamineAreaPDDLAction::dispatchCallback, this);
		
		node_handle.getParam("/squirrel_planning_execution/simulated", is_simulated_);
	}
	
	ExamineAreaPDDLAction::~ExamineAreaPDDLAction()
	{
		
	}
	
	/*---------------------------*/
	/* strategic action callback */
	/*---------------------------*/

	void ExamineAreaPDDLAction::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	{
		rosplan_dispatch_msgs::ActionDispatch normalised_action_dispatch = *msg;
		std::string action_name = msg->name;
		std::transform(action_name.begin(), action_name.end(), action_name.begin(), tolower);
		normalised_action_dispatch.name = action_name;
		
		// Ignore actions that do not correspond to g_action_name.
		if (g_action_name != action_name)
		{
			return;
		}

		bool actionAchieved = false;
		
		ROS_INFO("KCL: (ExamineAreaPDDLAction) action recieved %s", action_name.c_str());
		
		PlannerInstance& planner_instance = PlannerInstance::createInstance(*node_handle_);
		
		// Lets start the planning process.
		std::string data_path;
		node_handle_->getParam("/data_path", data_path);
		
		std::string planner_path;
		node_handle_->getParam("/planner_path", planner_path);
		
		std::stringstream ss;
		ss << data_path << action_name << "_domain-nt.pddl";
		std::string domain_name = ss.str();
		
		ss.str(std::string());
		ss << data_path << action_name << "_problem.pddl";
		std::string problem_name = ss.str();
		
		ss.str(std::string());
		ss << "timeout 180 " << planner_path << "ff -o DOMAIN -f PROBLEM";
		std::string planner_command = ss.str();
		
		// Before calling the planner we create the domain so it can be parsed.
		if (!createDomain())
		{
			ROS_ERROR("KCL: (ExamineAreaPDDLAction) failed to produce a domain at %s for action name %s.", domain_name.c_str(), action_name.c_str());
			return;
		}
		
		planner_instance.startPlanner(domain_name, problem_name, data_path, planner_command);
		
		// publish feedback (enabled)
		rosplan_dispatch_msgs::ActionFeedback fb;
		fb.action_id = msg->action_id;
		fb.status = "action enabled";
		action_feedback_pub_.publish(fb);

		// wait for action to finish
		ros::Rate loop_rate(1);
		while (ros::ok() && (planner_instance.getState() == actionlib::SimpleClientGoalState::ACTIVE || planner_instance.getState() == actionlib::SimpleClientGoalState::PENDING)) {
			ros::spinOnce();
			loop_rate.sleep();
		}

		actionlib::SimpleClientGoalState state = planner_instance.getState();
		ROS_INFO("KCL: (ExamineAreaPDDLAction) action finished: %s, %s", action_name.c_str(), state.toString().c_str());

		if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			// Update the knowledge base with what has been achieved.
			const std::string& robot = msg->parameters[0].value;
			const std::string& area = msg->parameters[1].value;
			
			ROS_INFO("KCL: (ExamineAreaPDDLAction) Process the action: %s, Examine %s by %s", action_name.c_str(), area.c_str(), robot.c_str());
			
			// Remove the old knowledge.
			rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
			knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
			rosplan_knowledge_msgs::KnowledgeItem kenny_knowledge;
			kenny_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			kenny_knowledge.attribute_name = "examined";
			kenny_knowledge.is_negative = false;
			
			diagnostic_msgs::KeyValue kv;
			kv.key = "a";
			kv.value = area;
			kenny_knowledge.values.push_back(kv);
			
			knowledge_update_service.request.knowledge = kenny_knowledge;
			if (!update_knowledge_client_.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (ExamineAreaPDDLAction) Could not add the (examined %s) predicate to the knowledge base.", area.c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (ExamineAreaPDDLAction) Added the action (examined %s) predicate to the knowledge base.", area.c_str());
			kenny_knowledge.values.clear();
			
			// publish feedback (achieved)
			rosplan_dispatch_msgs::ActionFeedback fb;
			fb.action_id = msg->action_id;
			fb.status = "action achieved";
			action_feedback_pub_.publish(fb);
		}
		else
		{
			// publish feedback (failed)
			rosplan_dispatch_msgs::ActionFeedback fb;
			fb.action_id = msg->action_id;
			fb.status = "action failed";
			action_feedback_pub_.publish(fb);
		}
	}
	
	/*--------------------*/
	/* problem generation */
	/*--------------------*/
	
	bool ExamineAreaPDDLAction::createDomain()
	{
		ROS_INFO("KCL: (ExamineAreaPDDLAction) Create domain for action %s.", g_action_name.c_str());
		// Lets start the planning process.
		std::string data_path;
		node_handle_->getParam("/data_path", data_path);

		std::stringstream ss;

		ss << g_action_name << "_domain-nt.pddl";
		std::string domain_name = ss.str();
		ss.str(std::string());

		ss << data_path << domain_name;
		std::string domain_path = ss.str();		
		ss.str(std::string());

		ss << g_action_name << "_problem.pddl";
		std::string problem_name = ss.str();
		ss.str(std::string());

		ss << data_path << problem_name;
		std::string problem_path = ss.str();
		ss.str(std::string());
		
		// Fetch all the objects.
		rosplan_knowledge_msgs::GetAttributeService get_attribute;
		get_attribute.request.predicate_name = "object_at";
		if (!get_attribute_client_.call(get_attribute)) {
			ROS_ERROR("KCL: (ExamineAreaPDDLAction) Failed to recieve the attributes of the predicate 'object_at'");
			return false;
		}
		
		std::map<std::string, std::string> object_to_location_mappings;
		std::map<std::string, std::vector<std::string> > near_waypoint_mappings;
		int max_objects = 0;
		for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = get_attribute.response.attributes.begin(); ci != get_attribute.response.attributes.end(); ++ci) {

			max_objects++;
			if(max_objects > 3) break;

			const rosplan_knowledge_msgs::KnowledgeItem& knowledge_item = *ci;
			std::string object_predicate;
			std::string location_predicate;
			for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = knowledge_item.values.begin(); ci != knowledge_item.values.end(); ++ci) {
				const diagnostic_msgs::KeyValue& key_value = *ci;
				if ("o" == key_value.key) {
					object_predicate = key_value.value;
				}
				
				if ("wp" == key_value.key) {
					location_predicate = key_value.value;
				}
			}
			
			object_to_location_mappings[object_predicate] = location_predicate;
			
			// Find waypoints that are near this waypoint, these waypoints are used by the 
			// robot to pickup or push this object.
			rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
			knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
			
			std::vector<std::string> near_waypoints;
			for (unsigned int i = 0; i < 1; ++i)
			{
				rosplan_knowledge_msgs::KnowledgeItem kenny_knowledge;
				kenny_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
				std::stringstream ss;
				ss << "near_" << location_predicate << "_" << i;
				
				near_waypoints.push_back(ss.str());
				
				kenny_knowledge.instance_type = "waypoint";
				kenny_knowledge.instance_name = ss.str();
				
				knowledge_update_service.request.knowledge = kenny_knowledge;
				if (!update_knowledge_client_.call(knowledge_update_service)) {
					ROS_ERROR("KCL: (ExamineAreaPDDLAction) Could not add the waypoint %s to the knowledge base.", ss.str().c_str());
					exit(-1);
				}
				ROS_INFO("KCL: (ExamineAreaPDDLAction) Added %s to the knowledge base.", ss.str().c_str());
				
				kenny_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
				kenny_knowledge.attribute_name = "near";
				kenny_knowledge.is_negative = false;
				diagnostic_msgs::KeyValue kv;
				kv.key = "wp1";
				kv.value = ss.str();
				kenny_knowledge.values.push_back(kv);
				kv.key = "wp2";
				kv.value = location_predicate;
				kenny_knowledge.values.push_back(kv);
				knowledge_update_service.request.knowledge = kenny_knowledge;
				if (!update_knowledge_client_.call(knowledge_update_service)) {
					ROS_ERROR("KCL: (ExamineAreaPDDLAction) Could not add the fact (near %s %s) to the knowledge base.", ss.str().c_str(), location_predicate.c_str());
					exit(-1);
				}
				ROS_INFO("KCL: (ExamineAreaPDDLAction) Added (near %s %s) to the knowledge base.", ss.str().c_str(), location_predicate.c_str());
				kenny_knowledge.values.clear();
			}
			near_waypoint_mappings[location_predicate] = near_waypoints;
		}
		std_msgs::Int8 nr_objects;
		nr_objects.data = object_to_location_mappings.size();
		ROS_INFO("KCL: (ExamineAreaPDDLAction) Found %d objects to eximine.", nr_objects.data);
		
		// Get the location of kenny.
		get_attribute.request.predicate_name = "robot_at";
		if (!get_attribute_client_.call(get_attribute)) {// || get_attribute.response.attributes.size() != 3) {
			ROS_ERROR("KCL: (ExamineAreaPDDLAction) Failed to recieve the attributes of the predicate 'robot_at'");
			return false;
		}
		
		std::string robot_location;
		for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = get_attribute.response.attributes[0].values.begin(); ci != get_attribute.response.attributes[0].values.end(); ++ci) {
			const diagnostic_msgs::KeyValue& knowledge_item = *ci;
			
			ROS_INFO("KCL: (ExamineAreaPDDLAction) Process robot_at attribute: %s %s", knowledge_item.key.c_str(), knowledge_item.value.c_str());
			
			if ("wp" == knowledge_item.key) {
				robot_location = knowledge_item.value;
			}
		}
		
		if ("" == robot_location) {
			ROS_ERROR("KCL: (ExamineAreaPDDLAction) Failed to recieve the location of Kenny");
			return false;
		}
		
		ROS_INFO("KCL: (ExamineAreaPDDLAction) Kenny is at waypoint: %s", robot_location.c_str());
		
		// Check which objects have already been classified.
		get_attribute.request.predicate_name = "is_of_type";
		if (!get_attribute_client_.call(get_attribute)) {
			ROS_ERROR("KCL: (ExamineAreaPDDLAction) Failed to recieve the attributes of the predicate 'is_of_type'");
			return false;
		}
		
		for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = get_attribute.response.attributes.begin(); ci != get_attribute.response.attributes.end(); ++ci) {
			const rosplan_knowledge_msgs::KnowledgeItem& knowledge_item = *ci;
			std::string object_predicate;
			std::string type_predicate;
			for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = knowledge_item.values.begin(); ci != knowledge_item.values.end(); ++ci) {
				const diagnostic_msgs::KeyValue& key_value = *ci;
				if ("o" == key_value.key) {
					object_predicate = key_value.value;
				}
				
				if ("t" == key_value.key) {
					type_predicate = key_value.value;
				}
			}
			
			if (type_predicate != "unknown")
			{
				object_to_location_mappings.erase(object_predicate);
				ROS_INFO("KCL: (ExamineAreaPDDLAction) No need to classify %s it is of type %s", object_predicate.c_str(), type_predicate.c_str());
			}
		}
		
		if (object_to_location_mappings.empty())
		{
			ROS_INFO("KCL: (ExamineAreaPDDLAction) All objects are all ready classified (or we found none!)");
		}
		else
		{
			ContingentStrategicClassifyPDDLGenerator::createPDDL(data_path, domain_name, problem_name, robot_location, object_to_location_mappings, near_waypoint_mappings, 3);
		}
		return true;
	}
};
