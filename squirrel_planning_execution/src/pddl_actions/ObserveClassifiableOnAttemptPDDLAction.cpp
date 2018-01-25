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
#include "squirrel_planning_execution/KnowledgeBase.h"

#include "ObserveClassifiableOnAttemptPDDLAction.h"
#include "PlannerInstance.h"

namespace KCL_rosplan {

	std::string ObserveClassifiableOnAttemptPDDLAction::g_action_name = "observe-classifiable_on_attempt";
	
	ObserveClassifiableOnAttemptPDDLAction::ObserveClassifiableOnAttemptPDDLAction(ros::NodeHandle& node_handle)
		: node_handle_(&node_handle), message_store_(node_handle), knowledge_base_(node_handle, message_store_)
	{
		// create the action feedback publisher
		action_feedback_pub_ = node_handle.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
		dispatch_sub_ = node_handle.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::ObserveClassifiableOnAttemptPDDLAction::dispatchCallback, this);
		
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
		
		// Before calling the planner we create the domain so it can be parsed.
		if (!createDomain(msg->parameters[0].value))
		{
			ROS_ERROR("KCL: (ObserveClassifiableOnAttemptPDDLAction) failed to produce a domain at %s for action name %s.", domain_name.c_str(), action_name.c_str());

			rosplan_dispatch_msgs::ActionFeedback fb;
			fb.action_id = msg->action_id;
			fb.status = "action failed";
			action_feedback_pub_.publish(fb);

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
			
			std::vector<std::string> waypoints;
			knowledge_base_.getInstances(waypoints, "waypoint");
			bool object_is_classified = false;
			
			// Find if this object has been classified at any location.
			for (std::vector<std::string>::const_iterator ci = waypoints.begin(); ci != waypoints.end(); ++ci)
			{
				const std::string& wp1 = *ci;
				for (std::vector<std::string>::const_iterator ci = waypoints.begin(); ci != waypoints.end(); ++ci)
				{
					const std::string& wp2 = *ci;
					std::map<std::string, std::string> parameters;
					parameters["from"] = wp1;
					parameters["view"] = wp2;
					parameters["o"] = object;
					if (knowledge_base_.isFactTrue("classifiable_from", parameters, true))
					{
						ROS_INFO("KCL: (ObserveClassifiableOnAttemptPDDLAction) %s (at %s) is classifiable from %s.", object.c_str(), wp1.c_str(), wp2.c_str());
						object_is_classified = true;
						break;
					}
				}
				if (object_is_classified) break;
			}

			std::map<std::string, std::string> parameters;
			parameters["o"] = object;
			parameters["c"] = counter;
			if (!knowledge_base_.addFact("classifiable_on_attempt", parameters, object_is_classified, KnowledgeBase::KB_ADD_KNOWLEDGE))
			{
				ROS_ERROR("KCL: (ObserveClassifiableOnAttemptPDDLAction) Could not add the classifiable_on_attempt predicate to the knowledge base.");
				exit(-1);
			}

			// Make this object classified.
			parameters.clear();
			parameters["o"] = object;
			if (!knowledge_base_.addFact("examined", parameters, true, KnowledgeBase::KB_ADD_KNOWLEDGE))
			{
					ROS_ERROR("KCL: (ObserveClassifiableOnAttemptPDDLAction) Could not add (examined %s) to the knowledge base.", object.c_str());
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
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> objects;
        if (!knowledge_base_.getFacts(objects, "object_at"))
        {
			ROS_ERROR("KCL: (ObserveClassifiableOnAttemptPDDLAction) Failed to recieve the attributes of the predicate 'object_at'");
			return false;
		}
		
		std::string object_location;
		bool found_object_location = false;
		for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci) {
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

				// Remove this object, as we cannot get to it!
				std::map<std::string, std::string> parameters;
				parameters["o"] = object_name;
				parameters["wp"] = object_location;
				if (!knowledge_base_.removeFact("object_at", parameters, true, KnowledgeBase::KB_REMOVE_KNOWLEDGE))
				{
				ROS_ERROR("KCL: (ObserveClassifiableOnAttemptPDDLAction) Failed to remove (object_at %s %s) from the knowledge base!", object_name.c_str(), object_location.c_str());
					exit(-1);
				}
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
			knowledge_base_.addInstance("waypoint", ss.str());

			std::map<std::string, std::string> parameters;
			parameters["wp1"] = ss.str();
			parameters["wp2"] = object_location;
			knowledge_base_.addFact("near", parameters, true, KnowledgeBase::KB_ADD_KNOWLEDGE);
			
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
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> robot_locations;
		if (!knowledge_base_.getFacts(robot_locations, "robot_at") || robot_locations.size() != 1)
		{
			ROS_ERROR("KCL: (ObserveClassifiableOnAttemptPDDLAction) Failed to recieve the attributes of the predicate 'robot_at'");
			return false;
		}
		
		std::string robot_location;
		for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = robot_locations[0].values.begin(); ci != robot_locations[0].values.end(); ++ci) {
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
