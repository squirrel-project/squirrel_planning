#include <map>
#include <vector>
#include <iostream>
#include <sstream>

#include <std_msgs/Int8.h>

#include <rosplan_dispatch_msgs/ActionFeedback.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_knowledge_msgs/GetInstanceService.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>

#include <squirrel_planning_execution/ContingentStrategicClassifyPDDLGenerator.h>
#include <squirrel_planning_execution/KnowledgeBase.h>

#include "ExamineAreaPDDLAction.h"
#include "PlannerInstance.h"



namespace KCL_rosplan {

	std::string ExamineAreaPDDLAction::g_action_name = "examine_area";
	
	ExamineAreaPDDLAction::ExamineAreaPDDLAction(ros::NodeHandle& node_handle, KCL_rosplan::KnowledgeBase& kb)
		: node_handle_(&node_handle), knowledge_base_(&kb)
	{
		// create the action feedback publisher
		action_feedback_pub_ = node_handle.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
		
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
		objects_to_examine_.clear();

		bool actionAchieved = false;
		
		ROS_INFO("KCL: (ExamineAreaPDDLAction) action recieved %s", action_name.c_str());
		ros::ServiceServer pddl_generation_service = node_handle_->advertiseService("/kcl_rosplan/generate_planning_problem", &ExamineAreaPDDLAction::generatePDDLProblemFile, this);

		PlannerInstance& planner_instance = PlannerInstance::createInstance(*node_handle_, "ff", false);
		
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
		//
		// Before calling the planner we create the domain so it can be parsed.
		if (!createPDDL())
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
//		pddl_generation_service.shutdown();


		actionlib::SimpleClientGoalState state = planner_instance.getState();
		ROS_INFO("KCL: (ExamineAreaPDDLAction) action finished: %s, %s", action_name.c_str(), state.toString().c_str());

		if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			// Update the knowledge base with what has been achieved.
			ROS_INFO("KCL: (ExamineAreaPDDLAction) Process the action: %s, Examined the room.", action_name.c_str());
			
			std::map<std::string, std::string> parameters;
			if (!knowledge_base_->addFact("examined_room", parameters, true, KCL_rosplan::KnowledgeBase::KB_ADD_KNOWLEDGE))
			{
				ROS_INFO("KCL: (ExamineAreaPDDLAction) Failed to add examined_room to the knowledge base!");
				exit(-1);
			}
			
			// publish feedback (achieved)
			rosplan_dispatch_msgs::ActionFeedback fb;
			fb.action_id = msg->action_id;
			fb.status = "action achieved";
			action_feedback_pub_.publish(fb);
		}
		else
		{
			// Remove all the old knowledge, all lumps, etc.
			for (std::set<std::string>::const_iterator ci = objects_to_examine_.begin(); ci != objects_to_examine_.end(); ++ci)
			{
				const std::string& object = *ci;
				ROS_INFO("KCL: (ExamineAreaPDDLAction) Process object: %s.", object.c_str());
				knowledge_base_->removeInstance("object", object);
			}

			std::vector<rosplan_knowledge_msgs::KnowledgeItem> facts;
			knowledge_base_->getFacts(facts, "object_at");
			for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = facts.begin(); ci != facts.end(); ++ci)
			{
				const rosplan_knowledge_msgs::KnowledgeItem& ki = *ci;
				std::string s = knowledge_base_->toString(ki);
				ROS_INFO("KCL: (ExamineAreaPDDLAction) Process fact: %s.", s.c_str());
				ROS_INFO("KCL: (ExamineAreaPDDLAction) Compare %s.", ki.values[0].value.c_str());
				if (objects_to_examine_.count(ki.values[0].value) != 0)
				{
					knowledge_base_->removeFact(ki, KCL_rosplan::KnowledgeBase::KB_REMOVE_KNOWLEDGE);
				}
			}

			// publish feedback (aborted, do not try again!)
			rosplan_dispatch_msgs::ActionFeedback fb;
			fb.action_id = msg->action_id;
			fb.status = "action aborted";
			action_feedback_pub_.publish(fb);
		}
	}
	
	/*--------------------*/
	/* problem generation */
	/*--------------------*/
	
	bool ExamineAreaPDDLAction::generatePDDLProblemFile(rosplan_knowledge_msgs::GenerateProblemService::Request &req, rosplan_knowledge_msgs::GenerateProblemService::Response &res)
	{
		return true;
	}

	bool ExamineAreaPDDLAction::createPDDL()
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
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> objects;
        if (!knowledge_base_->getFacts(objects, "object_at"))
        {
			ROS_ERROR("KCL: (ExamineAreaPDDLAction) Failed to recieve the attributes of the predicate 'object_at'");
			return false;
        }
        /*
		rosplan_knowledge_msgs::GetAttributeService get_attribute;
		get_attribute.request.predicate_name = "object_at";
		if (!get_attribute_client_.call(get_attribute)) {
			ROS_ERROR("KCL: (ExamineAreaPDDLAction) Failed to recieve the attributes of the predicate 'object_at'");
			return false;
		}
        */
		
		std::map<std::string, std::string> object_to_location_mappings;
		std::map<std::string, std::vector<std::string> > near_waypoint_mappings;
		int max_objects = 0;
		for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci) {

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
			if (objects_to_examine_.count(object_predicate) != 0) continue;

			max_objects++;
			object_to_location_mappings[object_predicate] = location_predicate;
           
            /** TEST
			// Find waypoints that are near this waypoint, these waypoints are used by the 
			// robot to pickup or push this object.
			std::vector<std::string> near_waypoints;
			for (unsigned int i = 0; i < 1; ++i)
			{
				std::stringstream ss;
				ss << "near_" << location_predicate << "_" << i;
				near_waypoints.push_back(ss.str());
				
				if (!knowledge_base_->addInstance("waypoint", ss.str()))
				{
					ROS_ERROR("KCL: (ExamineAreaPDDLAction) Could not add the waypoint %s to the knowledge base.", ss.str().c_str());
					exit(-1);
				}
				
				std::map<std::string, std::string> parameters;
				parameters["wp1"] = ss.str();
				parameters["wp2"] = location_predicate;
				if (!knowledge_base_->addFact("near", parameters, true, KCL_rosplan::KnowledgeBase::KB_ADD_KNOWLEDGE))
				{
					ROS_ERROR("KCL: (ExamineAreaPDDLAction) Could not add the fact (near %s %s) to the knowledge base.", ss.str().c_str(), location_predicate.c_str());
					exit(-1);
				}
			}
			near_waypoint_mappings[location_predicate] = near_waypoints;
            */
		}
		std_msgs::Int8 nr_objects;
		nr_objects.data = object_to_location_mappings.size();
		ROS_INFO("KCL: (ExamineAreaPDDLAction) Found %d objects to eximine.", nr_objects.data);

		if (object_to_location_mappings.size() == 0) return false;
		
		// Get the location of kenny.
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> robot_locations;
        if (!knowledge_base_->getFacts(robot_locations, "robot_at") || robot_locations.size() != 1)
        {
			ROS_ERROR("KCL: (ExamineAreaPDDLAction) Failed to recieve the attributes of the predicate 'robot_at'");
			return false;
        }
        /*
		get_attribute.request.predicate_name = "robot_at";
		if (!get_attribute_client_.call(get_attribute)) {// || get_attribute.response.attributes.size() != 3) {
			ROS_ERROR("KCL: (ExamineAreaPDDLAction) Failed to recieve the attributes of the predicate 'robot_at'");
			return false;
		}
        */
		
		std::string robot_location;
		for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = robot_locations[0].values.begin(); ci != robot_locations[0].values.end(); ++ci) {
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
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> type_associations;
        if (!knowledge_base_->getFacts(type_associations, "is_of_type"))
        {
			ROS_ERROR("KCL: (ExamineAreaPDDLAction) Failed to recieve the attributes of the predicate 'is_of_type'");
			return false;
        }
        /*
		get_attribute.request.predicate_name = "is_of_type";
		if (!get_attribute_client_.call(get_attribute)) {
			ROS_ERROR("KCL: (ExamineAreaPDDLAction) Failed to recieve the attributes of the predicate 'is_of_type'");
			return false;
		}
	    */	
		for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = type_associations.begin(); ci != type_associations.end(); ++ci) {
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
			return false;
		}
		else
		{
			for (std::map<std::string, std::string>::const_iterator ci = object_to_location_mappings.begin(); ci != object_to_location_mappings.end(); ++ci)
			{
				objects_to_examine_.insert(ci->first);
			}

			ContingentStrategicClassifyPDDLGenerator::createPDDL(data_path, domain_name, problem_name, robot_location, object_to_location_mappings, near_waypoint_mappings, 3);
		}
		return true;
	}
};
