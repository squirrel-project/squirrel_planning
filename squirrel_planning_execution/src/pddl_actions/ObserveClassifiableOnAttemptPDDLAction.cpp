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
#include <rosplan_knowledge_msgs/KnowledgeQueryService.h>

#include <squirrel_object_perception_msgs/SceneObject.h>
#include <squirrel_waypoint_msgs/ExamineWaypoint.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/tf.h>

#include "squirrel_planning_execution/ContingentTacticalClassifyPDDLGenerator.h"

#include "ObserveClassifiableOnAttemptPDDLAction.h"
#include "PlannerInstance.h"

namespace KCL_rosplan {

	std::string ObserveClassifiableOnAttemptPDDLAction::g_action_name = "observe-classifiable_on_attempt";
	
	ObserveClassifiableOnAttemptPDDLAction::ObserveClassifiableOnAttemptPDDLAction(ros::NodeHandle& node_handle)
		: node_handle_(&node_handle), message_store_(node_handle)
	{
		update_knowledge_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
		
		// create the action feedback publisher
		action_feedback_pub_ = node_handle.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
		
		get_instance_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_current_instances");
		get_attribute_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_knowledge");
		
		dispatch_sub_ = node_handle.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::ObserveClassifiableOnAttemptPDDLAction::dispatchCallback, this);
		
		query_knowledge_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::KnowledgeQueryService>("/kcl_rosplan/query_knowledge_base");
		
		std::string classifyTopic("/squirrel_perception_examine_waypoint");
		node_handle.param("squirrel_perception_classify_waypoint_service_topic", classifyTopic, classifyTopic);
		classify_object_waypoint_client_ = node_handle.serviceClient<squirrel_waypoint_msgs::ExamineWaypoint>(classifyTopic);
		
		node_handle.getParam("/squirrel_planning_execution/simulated", is_simulated_);
	}
	
	ObserveClassifiableOnAttemptPDDLAction::~ObserveClassifiableOnAttemptPDDLAction()
	{
		
	}
	
	/*---------------------------*/
	/* strategic action callback */
	/*---------------------------*/

	void ObserveClassifiableOnAttemptPDDLAction::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
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
		
		ROS_INFO("KCL: (ObserveClassifiableOnAttemptPDDLAction) action recieved %s", action_name.c_str());
		
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
		if (!createDomain(msg->parameters[0].value))
		{
			ROS_ERROR("KCL: (ObserveClassifiableOnAttemptPDDLAction) failed to produce a domain at %s for action name %s.", domain_name.c_str(), action_name.c_str());
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
		ROS_INFO("KCL: (ObserveClassifiableOnAttemptPDDLAction) action finished: %s, %s", action_name.c_str(), state.toString().c_str());

		if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			// Update the domain.
			const std::string& object = msg->parameters[0].value;
			const std::string& counter = msg->parameters[1].value;
			
			// Add the new knowledge.
			rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
			knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
			rosplan_knowledge_msgs::KnowledgeItem kenny_knowledge;
			kenny_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			kenny_knowledge.attribute_name = "classifiable_on_attempt";
			
			// Check if this object has been classified or not.
			rosplan_knowledge_msgs::GetInstanceService get_instance;
			get_instance.request.type_name = "waypoint";
			
			if (!get_instance_client_.call(get_instance))
			{
				ROS_ERROR("KCL: (ObserveClassifiableOnAttemptPDDLAction) Could not get the instances of type 'waypoint'.");
				exit(1);
			}
			
			rosplan_knowledge_msgs::KnowledgeQueryService knowledge_query;
			
			// Find if this object has been classified at any location.
			for (std::vector<std::string>::const_iterator ci = get_instance.response.instances.begin(); ci != get_instance.response.instances.end(); ++ci)
			{
				const std::string& wp1 = *ci;
				for (std::vector<std::string>::const_iterator ci = get_instance.response.instances.begin(); ci != get_instance.response.instances.end(); ++ci)
				{
					const std::string& wp2 = *ci;
					
					rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
					knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
					knowledge_item.attribute_name = "classifiable_from";
					
					diagnostic_msgs::KeyValue kv;
					kv.key = "from";
					kv.value = wp1;
					knowledge_item.values.push_back(kv);
					
					kv.key = "view";
					kv.value = wp2;
					knowledge_item.values.push_back(kv);
					
					kv.key = "o";
					kv.value = object;
					knowledge_item.values.push_back(kv);
					knowledge_item.is_negative = false;
					
					knowledge_query.request.knowledge.push_back(knowledge_item);
					ROS_INFO("KCL: (ObserveClassifiableOnAttemptPDDLAction) Check if (classifiable_from %s %s %s) is true.", wp1.c_str(), wp2.c_str(), object.c_str());
				}
			}
			
			// Check if any of these facts are true.
			if (!query_knowledge_client_.call(knowledge_query))
			{
				ROS_ERROR("KCL: (ObserveClassifiableOnAttemptPDDLAction) Could not call the query knowledge server.");
				exit(1);
			}
			
			bool object_is_classified = false;
			for (std::vector<unsigned char>::const_iterator ci = knowledge_query.response.results.begin(); ci != knowledge_query.response.results.end(); ci++)
			{
				if (*ci == 1)
				{
					object_is_classified = true;
					break;
				}
			}
			
			kenny_knowledge.is_negative = !object_is_classified;
			if (kenny_knowledge.is_negative)
			{
				ROS_INFO("KCL: (ObserveClassifiableOnAttemptPDDLAction) %s was not classified on the %sth attempt.", object.c_str(), counter.c_str());
			}
			else
			{
				ROS_INFO("KCL: (ObserveClassifiableOnAttemptPDDLAction) %s was classified on the %sth attempt!", object.c_str(), counter.c_str());
			}
			
			diagnostic_msgs::KeyValue kv;
			kv.key = "o";
			kv.value = object;
			kenny_knowledge.values.push_back(kv);
			
			kv.key = "c";
			kv.value = counter;
			kenny_knowledge.values.push_back(kv);
			
			knowledge_update_service.request.knowledge = kenny_knowledge;
			if (!update_knowledge_client_.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (ObserveClassifiableOnAttemptPDDLAction) Could not add the classifiable_on_attempt predicate to the knowledge base.");
				exit(-1);
			}
			ROS_INFO("KCL: (ObserveClassifiableOnAttemptPDDLAction) Added classifiable_on_attempt predicate to the knowledge base.");
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
	
	bool ObserveClassifiableOnAttemptPDDLAction::createDomain(const std::string& object)
	{
		ROS_INFO("KCL: (ObserveClassifiableOnAttemptPDDLAction) Create domain for action %s.", g_action_name.c_str());
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
		
		// Find the object that needs to be classified.
		ROS_INFO("KCL: (ObserveClassifiableOnAttemptPDDLAction) %s.", g_action_name.c_str());
		std::string object_name = object;
		std::transform(object_name.begin(), object_name.end(), object_name.begin(), tolower);
		
		ROS_INFO("KCL: (ObserveClassifiableOnAttemptPDDLAction) Object name is: %s", object_name.c_str());
		
		// Get the location of the object.
		// (object_at ?o - object ?wp - location)
		rosplan_knowledge_msgs::GetAttributeService get_attribute;
		get_attribute.request.predicate_name = "object_at";
		if (!get_attribute_client_.call(get_attribute)) {
			ROS_ERROR("KCL: (ObserveClassifiableOnAttemptPDDLAction) Failed to recieve the attributes of the predicate 'object_at'");
			return false;
		}
		
		std::string object_location;
		bool found_object_location = false;
		for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = get_attribute.response.attributes.begin(); ci != get_attribute.response.attributes.end(); ++ci) {
			const rosplan_knowledge_msgs::KnowledgeItem& knowledge_item = *ci;
			ROS_INFO("KCL: (ObserveClassifiableOnAttemptPDDLAction) %s", knowledge_item.attribute_name.c_str());
			for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = knowledge_item.values.begin(); ci != knowledge_item.values.end(); ++ci) {
				const diagnostic_msgs::KeyValue& key_value = *ci;
				if ("o" == key_value.key && object_name == key_value.value) {
					found_object_location = true;
				}
				
				if ("wp" == key_value.key) {
					object_location = key_value.value;
				}
				
				ROS_INFO("KCL: (ObserveClassifiableOnAttemptPDDLAction) %s -> %s.", key_value.key.c_str(), key_value.value.c_str());
			}
			
			if (found_object_location) {
				break;
			}
		}
		
		if (!found_object_location) {
			ROS_ERROR("KCL: (ObserveClassifiableOnAttemptPDDLAction) Failed to recieve the location of the object %s", object_name.c_str());
			return false;
		}
		
		ROS_INFO("KCL: (ObserveClassifiableOnAttemptPDDLAction) Object location is: %s", object_location.c_str());
		
		squirrel_waypoint_msgs::ExamineWaypoint getTaskPose;
		if (!is_simulated_)
		{
			// fetch position of object from message store
			std::vector< boost::shared_ptr<squirrel_object_perception_msgs::SceneObject> > results;
			if(message_store_.queryNamed<squirrel_object_perception_msgs::SceneObject>(object_name, results)) {

				if(results.size()<1) {
					ROS_ERROR("KCL: (ObserveClassifiableOnAttemptPDDLAction) aborting waypoint request; no matching obID %s", object_name.c_str());
					return false;
				}
			} else {
				ROS_ERROR("KCL: (ObserveClassifiableOnAttemptPDDLAction) could not query message store to fetch object pose of %s", object_name.c_str());
				return false;
			}

			// request classification waypoints for object
			squirrel_object_perception_msgs::SceneObject &obj = *results[0];
			
			getTaskPose.request.object_pose.header = obj.header;
			getTaskPose.request.object_pose.pose = obj.pose;
			if (!classify_object_waypoint_client_.call(getTaskPose)) {
				ROS_ERROR("KCL: (ObserveClassifiableOnAttemptPDDLAction) Failed to recieve classification waypoints for %s.", object_name.c_str());
				return false;
			}

			std_msgs::Int8 debug_pose_number;
			debug_pose_number.data = getTaskPose.response.poses.size();
			ROS_INFO("KCL: (ObserveClassifiableOnAttemptPDDLAction) Found %d observation poses", debug_pose_number.data);
		}
		else
		{
			for (unsigned int i = 0; i < 4; ++i)
			{
				geometry_msgs::PoseWithCovarianceStamped pwcs;
				getTaskPose.response.poses.push_back(pwcs);
			}
		}

		// Add all the waypoints to the knowledge base.
		std::vector<std::string> observation_location_predicates;
		for(int i=0;i<getTaskPose.response.poses.size(); i++) {
			
			ss.str(std::string());
			ss << object_name << "_observation_wp" << i;
			
			ROS_INFO("KCL: (ObserveClassifiableOnAttemptPDDLAction) Process observation pose: %s", ss.str().c_str());
			
			rosplan_knowledge_msgs::KnowledgeUpdateService updateSrv;
			updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
			updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
			updateSrv.request.knowledge.instance_type = "waypoint";
			updateSrv.request.knowledge.instance_name = ss.str();
			update_knowledge_client_.call(updateSrv);
			
			// Store the waypoint in mongo DB.
			if (!is_simulated_)
			{
				const geometry_msgs::PoseWithCovarianceStamped& pwcs = getTaskPose.response.poses[i];
				geometry_msgs::PoseStamped ps;
				ps.header= pwcs.header;
				ps.header.frame_id = "/map";
				ps.pose = pwcs.pose.pose;
				std::string near_waypoint_mongodb_id(message_store_.insertNamed(ss.str(), ps));
			}
			
			observation_location_predicates.push_back(ss.str());
		}
		
		// Add a special observation waypoint.
		observation_location_predicates.push_back("nowhere");
		
		// Get the location of kenny.
		get_attribute.request.predicate_name = "robot_at";
		if (!get_attribute_client_.call(get_attribute)) {// || get_attribute.response.attributes.size() != 3) {
			ROS_ERROR("KCL: (ObserveClassifiableOnAttemptPDDLAction) Failed to recieve the attributes of the predicate 'robot_at'");
			return false;
		}
		
		std::string robot_location;
		for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = get_attribute.response.attributes[0].values.begin(); ci != get_attribute.response.attributes[0].values.end(); ++ci) {
			const diagnostic_msgs::KeyValue& knowledge_item = *ci;
			
			ROS_INFO("KCL: (ObserveClassifiableOnAttemptPDDLAction) Process robot_at attribute: %s %s", knowledge_item.key.c_str(), knowledge_item.value.c_str());
			
			if ("wp" == knowledge_item.key) {
				robot_location = knowledge_item.value;
			}
		}
		
		if ("" == robot_location) {
			ROS_ERROR("KCL: (ObserveClassifiableOnAttemptPDDLAction) Failed to recieve the location of Kenny");
			return false;
		}
		
		ROS_INFO("KCL: (ObserveClassifiableOnAttemptPDDLAction) Kenny is at waypoint: %s", robot_location.c_str());
		
		ContingentTacticalClassifyPDDLGenerator::createPDDL(data_path, domain_name, problem_name, robot_location, observation_location_predicates, object_name, object_location);
		return true;
	}
};
