#include <map>
#include <vector>
#include <iostream>
#include <sstream>

#include <std_msgs/Int8.h>
#include <std_msgs/ColorRGBA.h>

#include <rosplan_planning_system/PDDLProblemGenerator.h>

#include <rosplan_dispatch_msgs/ActionFeedback.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_knowledge_msgs/GetInstanceService.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>

#include <squirrel_object_perception_msgs/SceneObject.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/tf.h>

#include "squirrel_planning_execution/ClassicalTidyPDDLGenerator.h"

#include "TidyAreaPDDLAction.h"
#include "PlannerInstance.h"



namespace KCL_rosplan {

	std::string TidyAreaPDDLAction::g_action_name = "tidy_area";
	
	TidyAreaPDDLAction::TidyAreaPDDLAction(ros::NodeHandle& node_handle)
		: node_handle_(&node_handle), message_store_(node_handle)
	{
		update_knowledge_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
		
		// create the action feedback publisher
		action_feedback_pub_ = node_handle.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
		
		get_instance_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_current_instances");
		get_attribute_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_knowledge");
		
		dispatch_sub_ = node_handle.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::TidyAreaPDDLAction::dispatchCallback, this);
		
		node_handle.getParam("/squirrel_planning_execution/simulated", is_simulated_);
	}
	
	
	TidyAreaPDDLAction::~TidyAreaPDDLAction()
	{
		
	}
	
	/*---------------------------*/
	/* strategic action callback */
	/*---------------------------*/

	void TidyAreaPDDLAction::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
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
		
		ROS_INFO("KCL: (TidyAreaPDDLAction) action recieved %s", action_name.c_str());
		
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
			ROS_ERROR("KCL: (TidyAreaPDDLAction) failed to produce a domain at %s for action name %s.", domain_name.c_str(), action_name.c_str());
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
		ROS_INFO("KCL: (TidyAreaPDDLAction) action finished: %s, %s", action_name.c_str(), state.toString().c_str());

		if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
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
	
	bool TidyAreaPDDLAction::createDomain()
	{
		ROS_INFO("KCL: (TidyAreaPDDLAction) Create domain for action %s.", g_action_name.c_str());
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
		
		// Get all the objects in the knowledge base that are in this area. 
		// TODO For now we assume there is only one area, so all objects in the knowledge base are relevant (unless already tidied).
		rosplan_knowledge_msgs::GetAttributeService get_attribute;
		
		// Get the location of the boxes.
		// (box_at ?b - box ?wp - waypoint)
		get_attribute.request.predicate_name = "box_at";
		if (!get_attribute_client_.call(get_attribute)) {
			ROS_ERROR("KCL: (TidyAreaPDDLAction) Failed to recieve the attributes of the predicate 'box_at'");
			return false;
		}
		
		std::map<std::string, std::string> box_to_location_mapping;
		std::map<std::string, std::vector<std::string> > near_box_location_mapping;
		std::map<std::string, geometry_msgs::Pose> box_to_pose_mapping;
		std::map<std::string, geometry_msgs::Pose> type_to_box_pose_mapping;
		for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = get_attribute.response.attributes.begin(); ci != get_attribute.response.attributes.end(); ++ci) {
			const rosplan_knowledge_msgs::KnowledgeItem& knowledge_item = *ci;
			std::string box_predicate;
			std::string box_location_predicate;
			for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = knowledge_item.values.begin(); ci != knowledge_item.values.end(); ++ci) {
				const diagnostic_msgs::KeyValue& key_value = *ci;
				if ("b" == key_value.key) {
					box_predicate = key_value.value;
				} else if ("wp" == key_value.key) {
					box_location_predicate = key_value.value;
				}
			}
			
			// Get the actual location of this box.
			std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > results;
			
			std::stringstream ss;
			ss << "near_" << box_location_predicate;
			
			if (!is_simulated_)
			{
				if(message_store_.queryNamed<geometry_msgs::PoseStamped>(box_location_predicate, results))
				{
					if (results.size() < 1)
					{
						ROS_ERROR("KCL: (TidyAreaPDDLAction) aborting waypoint request; no matching wpID %s", box_location_predicate.c_str());
						exit(-1);
					}
				} else {
					ROS_ERROR("KCL: (TidyAreaPDDLAction) could not query message store to fetch waypoint pose");
					exit(-1);
				}
				const geometry_msgs::PoseStamped &box_wp = *results[0];
				box_to_pose_mapping[box_predicate] = box_wp.pose;
				
				// Create a waypoint 44 cm from this box at a random angle.
				float angle = ((float)rand() / (float)RAND_MAX) * 360.0f;
				tf::Vector3 v(0.44f, 0.0f, 0.0f);
				v.rotate(tf::Vector3(0, 0, 1), angle);
				v += tf::Vector3(box_wp.pose.position.x, box_wp.pose.position.y, 0.0f);
				
				tf::Quaternion v_rotation(tf::Vector3(0, 0, 1), angle + 180.0f);
				
				// Store this location in the knowledge base.
				geometry_msgs::PoseStamped near_pose;
				near_pose.header.seq = 0;
				near_pose.header.stamp = ros::Time::now();
				near_pose.header.frame_id = "/map";
				near_pose.pose.position.x = v.x();
				near_pose.pose.position.y = v.y();
				near_pose.pose.position.z = 0.0f;
				
				near_pose.pose.orientation.x = v_rotation.x();
				near_pose.pose.orientation.y = v_rotation.y();
				near_pose.pose.orientation.z = v_rotation.z();
				near_pose.pose.orientation.w = v_rotation.w();
				
				std::string near_waypoint_mongodb_id(message_store_.insertNamed(ss.str(), near_pose));
			}
			
			box_to_location_mapping[box_predicate] = box_location_predicate;
			
			std::vector<std::string> waypoints_near_box;
			
			rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
			knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
			
			rosplan_knowledge_msgs::KnowledgeItem kenny_knowledge;
			kenny_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
			
			waypoints_near_box.push_back(ss.str());
			
			kenny_knowledge.instance_type = "waypoint";
			kenny_knowledge.instance_name = ss.str();
			
			knowledge_update_service.request.knowledge = kenny_knowledge;
			if (!update_knowledge_client_.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (TidyAreaPDDLAction) Could not add the waypoint %s to the knowledge base.", ss.str().c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (TidyAreaPDDLAction) Added %s to the knowledge base.", ss.str().c_str());
			
			kenny_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			kenny_knowledge.attribute_name = "near_for_dropping";
			kenny_knowledge.is_negative = false;
			diagnostic_msgs::KeyValue kv;
			kv.key = "wp1";
			kv.value = ss.str();
			kenny_knowledge.values.push_back(kv);
			kv.key = "wp2";
			kv.value = box_location_predicate;
			kenny_knowledge.values.push_back(kv);
			knowledge_update_service.request.knowledge = kenny_knowledge;
			if (!update_knowledge_client_.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (TidyAreaPDDLAction) Could not add the fact (near %s %s) to the knowledge base.", ss.str().c_str(), box_location_predicate.c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (TidyAreaPDDLAction) Added (near %s %s) to the knowledge base.", ss.str().c_str(), box_location_predicate.c_str());
			kenny_knowledge.values.clear();

			near_box_location_mapping[box_location_predicate] = waypoints_near_box;
		}
		
		// Figure out which types of objects fit in each box.
		// (can_fit_inside ?t - type ?b - box)
		get_attribute.request.predicate_name = "can_fit_inside";
		if (!get_attribute_client_.call(get_attribute)) {
			ROS_ERROR("KCL: (TidyAreaPDDLAction) Failed to recieve the attributes of the predicate 'box_at'");
			return false;
		}
		
		std::map<std::string, std::string> box_to_type_mapping;
		for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = get_attribute.response.attributes.begin(); ci != get_attribute.response.attributes.end(); ++ci) {
			const rosplan_knowledge_msgs::KnowledgeItem& knowledge_item = *ci;
			std::string box_predicate;
			std::string type_predicate;
			for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = knowledge_item.values.begin(); ci != knowledge_item.values.end(); ++ci) {
				const diagnostic_msgs::KeyValue& key_value = *ci;
				if ("b" == key_value.key) {
					box_predicate = key_value.value;
				} else if ("t" == key_value.key) {
					type_predicate = key_value.value;
				}
			}
			
			box_to_type_mapping[box_predicate] = type_predicate;
			type_to_box_pose_mapping[type_predicate] = box_to_pose_mapping[box_predicate];
		}
		
		// Get the location of kenny.
		// (robot_at ?v - robot ?wp - waypoint)
		get_attribute.request.predicate_name = "robot_at";
		if (!get_attribute_client_.call(get_attribute)) {
			ROS_ERROR("KCL: (TidyAreaPDDLAction) Failed to recieve the attributes of the predicate 'robot_at'");
			return false;
		}
		
		std::string robot_location;
		for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = get_attribute.response.attributes[0].values.begin(); ci != get_attribute.response.attributes[0].values.end(); ++ci) {
			const diagnostic_msgs::KeyValue& knowledge_item = *ci;
			
			ROS_INFO("KCL: (TidyAreaPDDLAction) Process robot_at attribute: %s %s", knowledge_item.key.c_str(), knowledge_item.value.c_str());
			
			if ("wp" == knowledge_item.key) {
				robot_location = knowledge_item.value;
			}
		}
		
		if ("" == robot_location) {
			ROS_ERROR("KCL: (TidyAreaPDDLAction) Failed to recieve the location of Kenny");
			return false;
		}
		
		ROS_INFO("KCL: (TidyAreaPDDLAction) Kenny is at waypoint: %s", robot_location.c_str());
		
		// Get the location of the objects.
		// (object_at ?o - object ?wp - location)
		get_attribute.request.predicate_name = "object_at";
		if (!get_attribute_client_.call(get_attribute)) {
			ROS_ERROR("KCL: (TidyAreaPDDLAction) Failed to recieve the attributes of the predicate 'object_at'");
			return false;
		}
		
		// Create a mapping of each object to its location.
		std::map<std::string, std::string> object_to_location_mapping;
		std::map<std::string, std::vector<std::string> > grasping_waypoint_mappings;
		std::map<std::string, std::vector<std::string> > pushing_waypoint_mappings;
		std::map<std::string, geometry_msgs::Pose> object_to_pose_mapping;
		for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = get_attribute.response.attributes.begin(); ci != get_attribute.response.attributes.end(); ++ci) {
			const rosplan_knowledge_msgs::KnowledgeItem& knowledge_item = *ci;
			std::string object_predicate;
			std::string object_location_predicate;
			for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = knowledge_item.values.begin(); ci != knowledge_item.values.end(); ++ci) {
				const diagnostic_msgs::KeyValue& key_value = *ci;
				if ("o" == key_value.key) {
					object_predicate = key_value.value;
				} else if ("wp" == key_value.key) {
					object_location_predicate = key_value.value;
				}
			}
			
			if (object_predicate != "" && object_location_predicate != "")
			{
				object_to_location_mapping[object_predicate] = object_location_predicate;
			}
			
			if (!is_simulated_)
			{
				// Get the actual location of this object.
				std::vector< boost::shared_ptr<squirrel_object_perception_msgs::SceneObject> > results;
				if(message_store_.queryNamed<squirrel_object_perception_msgs::SceneObject>(object_predicate, results))
				{
					if (results.size() < 1)
					{
						ROS_ERROR("KCL: (TidyAreaPDDLAction) aborting waypoint request; no matching obID %s", object_predicate.c_str());
						exit(-1);
					}
				} else {
					ROS_ERROR("KCL: (TidyAreaPDDLAction) could not query message store to fetch object pose");
					exit(-1);
				}
				const squirrel_object_perception_msgs::SceneObject &obj = *results[0];
				const geometry_msgs::Pose& obj_pose = obj.pose;
				object_to_pose_mapping[object_predicate] = obj_pose;
			}
		}
		
		// Filter those objects that are already tidied.
		// (tidy ?o - object)
		get_attribute.request.predicate_name = "tidy";
		if (!get_attribute_client_.call(get_attribute)) {
			ROS_ERROR("KCL: (TidyAreaPDDLAction) Failed to recieve the attributes of the predicate 'tidy'");
			return false;
		}
		
		for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = get_attribute.response.attributes.begin(); ci != get_attribute.response.attributes.end(); ++ci) {
			const rosplan_knowledge_msgs::KnowledgeItem& knowledge_item = *ci;
			for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = knowledge_item.values.begin(); ci != knowledge_item.values.end(); ++ci) {
				const diagnostic_msgs::KeyValue& key_value = *ci;
				if ("o" == key_value.key) {
					object_to_location_mapping.erase(key_value.value);
					break;
				}
			}
		}
		
		std_msgs::Int8 nr_untidied_objects;
		nr_untidied_objects.data = object_to_location_mapping.size();
		ROS_INFO("KCL: (TidyAreaPDDLAction) Found %d untidied objects.", nr_untidied_objects.data);
		
		// Fetch the types of the untidied objects.
		// (is_of_type ?o - object ?t -type)
		get_attribute.request.predicate_name = "is_of_type";
		if (!get_attribute_client_.call(get_attribute)) {
			ROS_ERROR("KCL: (TidyAreaPDDLAction) Failed to recieve the attributes of the predicate 'is_of_type'");
			return false;
		}
		
		std::map<std::string, std::string> object_to_type_mapping;
		for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = get_attribute.response.attributes.begin(); ci != get_attribute.response.attributes.end(); ++ci) {
			const rosplan_knowledge_msgs::KnowledgeItem& knowledge_item = *ci;
			std::string type_predicate;
			std::string object_predicate;
			for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = knowledge_item.values.begin(); ci != knowledge_item.values.end(); ++ci) {
				const diagnostic_msgs::KeyValue& key_value = *ci;
				if ("o" == key_value.key && object_to_location_mapping.count(key_value.value) == 1) {
					object_predicate = key_value.value;
				} else if ("t" == key_value.key) {
					type_predicate = key_value.value;
				}
			}
			
			if ("" != object_predicate)
			{
				object_to_type_mapping[object_predicate] = type_predicate;
				
				std::cout << " ************** MAP: " << object_predicate << " ->> " << type_predicate << std::endl;
				
				geometry_msgs::Pose obj_pose = object_to_pose_mapping[object_predicate];
				
				/**
					* Create a waypoint for grasping.
					*/
				rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
				knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
				
				rosplan_knowledge_msgs::KnowledgeItem kenny_knowledge;
				kenny_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
				std::stringstream ss;
				ss << "near_for_grasping_" << object_predicate;
				
				std::vector<std::string> grasping_waypoints;
				grasping_waypoints.push_back(ss.str());
				
				kenny_knowledge.instance_type = "waypoint";
				kenny_knowledge.instance_name = ss.str();
				
				knowledge_update_service.request.knowledge = kenny_knowledge;
				if (!update_knowledge_client_.call(knowledge_update_service)) {
					ROS_ERROR("KCL: (TidyAreaPDDLAction) Could not add the waypoint %s to the knowledge base.", ss.str().c_str());
					exit(-1);
				}
				ROS_INFO("KCL: (TidyAreaPDDLAction) Added %s to the knowledge base.", ss.str().c_str());
				
				if (!is_simulated_)
				{
					// Create a waypoint 43 cm from this box at a random angle.
					float angle = ((float)rand() / (float)RAND_MAX) * 360.0f;
					tf::Vector3 v(0.43f, 0.0f, 0.0f);
					v.rotate(tf::Vector3(0, 0, 1), angle);
					v += tf::Vector3(obj_pose.position.x, obj_pose.position.y, 0.0f);
					
					tf::Quaternion v_rotation(tf::Vector3(0, 0, 1), angle + 180.0f);
					
					// Store this location in the knowledge base.
					geometry_msgs::PoseStamped near_pose;
					near_pose.header.seq = 0;
					near_pose.header.stamp = ros::Time::now();
					near_pose.header.frame_id = "/map";
					near_pose.pose.position.x = v.x();
					near_pose.pose.position.y = v.y();
					near_pose.pose.position.z = 0.0f;
					
					near_pose.pose.orientation.x = v_rotation.x();
					near_pose.pose.orientation.y = v_rotation.y();
					near_pose.pose.orientation.z = v_rotation.z();
					near_pose.pose.orientation.w = v_rotation.w();
					
					std::string near_waypoint_mongodb_id(message_store_.insertNamed(ss.str(), near_pose));
				}
				
				kenny_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
				kenny_knowledge.attribute_name = "near_for_grasping";
				kenny_knowledge.is_negative = false;
				diagnostic_msgs::KeyValue kv;
				kv.key = "wp1";
				kv.value = ss.str();
				kenny_knowledge.values.push_back(kv);
				kv.key = "wp2";
				kv.value = object_to_location_mapping[object_predicate];
				kenny_knowledge.values.push_back(kv);
				knowledge_update_service.request.knowledge = kenny_knowledge;
				if (!update_knowledge_client_.call(knowledge_update_service)) {
					ROS_ERROR("KCL: (TidyAreaPDDLAction) Could not add the fact (near %s %s) to the knowledge base.", ss.str().c_str(), object_predicate.c_str());
					exit(-1);
				}
				ROS_INFO("KCL: (TidyAreaPDDLAction) Added (near %s %s) to the knowledge base.", ss.str().c_str(), object_predicate.c_str());
				kenny_knowledge.values.clear();

				grasping_waypoint_mappings[object_to_location_mapping[object_predicate]] = grasping_waypoints;
				
				/**
					* Create a waypoing for pushing.
					*/
				ss.str(std::string());
				ss << "near_for_pushing_" << object_predicate;
				
				if (!is_simulated_)
				{
					const geometry_msgs::Pose& box_pose = type_to_box_pose_mapping[type_predicate];
					
					tf::Vector3 object_pose_v3(obj_pose.position.x, obj_pose.position.y, obj_pose.position.z);
					tf::Vector3 box_pose_v3(box_pose.position.x, box_pose.position.y, box_pose.position.z); 
					tf::Vector3 behind_robot = object_pose_v3 - box_pose_v3;
					behind_robot.normalize();
					behind_robot = object_pose_v3 + behind_robot * 0.4f;
					
					// Make the angle such that is faces the object.
					float angle = behind_robot.angle(object_pose_v3);
					
					// Create a waypoint 40 cm 'behind' the robot in relation to the box.
					tf::Quaternion behind_robot_rotation(tf::Vector3(0, 0, 1), angle);
					
					// Store this location in the knowledge base.
					geometry_msgs::PoseStamped near_pose;
					near_pose.header.seq = 0;
					near_pose.header.stamp = ros::Time::now();
					near_pose.header.frame_id = "/map";
					near_pose.pose.position.x = behind_robot.x();
					near_pose.pose.position.y = behind_robot.y();
					near_pose.pose.position.z = 0.0f;
					
					near_pose.pose.orientation.x = behind_robot_rotation.x();
					near_pose.pose.orientation.y = behind_robot_rotation.y();
					near_pose.pose.orientation.z = behind_robot_rotation.z();
					near_pose.pose.orientation.w = behind_robot_rotation.w();
					
					std::string behind_robot_waypoint_mongodb_id(message_store_.insertNamed(ss.str(), near_pose));
				}
				std::vector<std::string> pushing_waypoints;
				pushing_waypoints.push_back(ss.str());
				
				kenny_knowledge.instance_type = "waypoint";
				kenny_knowledge.instance_name = ss.str();
				
				knowledge_update_service.request.knowledge = kenny_knowledge;
				if (!update_knowledge_client_.call(knowledge_update_service)) {
					ROS_ERROR("KCL: (TidyAreaPDDLAction) Could not add the waypoint %s to the knowledge base.", ss.str().c_str());
					exit(-1);
				}
				ROS_INFO("KCL: (TidyAreaPDDLAction) Added %s to the knowledge base.", ss.str().c_str());
				
				kenny_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
				kenny_knowledge.attribute_name = "near_for_grasping";
				kenny_knowledge.is_negative = false;

				kv.key = "wp1";
				kv.value = ss.str();
				kenny_knowledge.values.push_back(kv);
				kv.key = "wp2";
				kv.value = object_to_location_mapping[object_predicate];
				kenny_knowledge.values.push_back(kv);
				knowledge_update_service.request.knowledge = kenny_knowledge;
				if (!update_knowledge_client_.call(knowledge_update_service)) {
					ROS_ERROR("KCL: (TidyAreaPDDLAction) Could not add the fact (near %s %s) to the knowledge base.", ss.str().c_str(), object_predicate.c_str());
					exit(-1);
				}
				ROS_INFO("KCL: (TidyAreaPDDLAction) Added (near %s %s) to the knowledge base.", ss.str().c_str(), object_predicate.c_str());
				kenny_knowledge.values.clear();

				pushing_waypoint_mappings[object_to_location_mapping[object_predicate]] = pushing_waypoints;
			}
		}
		
		// For all object that do not have a type (i.e. were not classified) we add the 'unknown' type.
		for (std::map<std::string, std::string>::const_iterator ci = object_to_location_mapping.begin(); ci != object_to_location_mapping.end(); ++ci)
		{
			const std::string& object_name = (*ci).first;
			
			std::cout << " ************** COULD NOT FIND TYPE OF: " << object_name << std::endl;
			if (object_to_type_mapping.find(object_name) == object_to_location_mapping.end())
			{
				object_to_type_mapping[object_name] = "unknown";
			}
		}
		
		ClassicalTidyPDDLGenerator::createPDDL(data_path, domain_name, problem_name, robot_location, object_to_location_mapping, grasping_waypoint_mappings, pushing_waypoint_mappings, object_to_type_mapping, box_to_location_mapping, box_to_type_mapping, near_box_location_mapping);
	}
};
