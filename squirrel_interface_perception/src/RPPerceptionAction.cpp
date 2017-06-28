#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Quaternion.h>
#include <actionlib/client/simple_action_client.h>

#include <std_msgs/Float64MultiArray.h>
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "squirrel_object_perception_msgs/LookForObjectsAction.h"
#include "squirrel_object_perception_msgs/RecognizeObjectsAction.h"
#include "squirrel_object_perception_msgs/FindDynamicObjects.h"
#include "squirrel_interface_perception/RPPerceptionAction.h"
#include "squirrel_planning_knowledge_msgs/AddObjectService.h"
#include "mongodb_store/message_store.h"
#include "geometry_msgs/PoseStamped.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/GetInstanceService.h"
#include "rosplan_knowledge_msgs/KnowledgeQueryService.h"
#include "squirrel_object_perception_msgs/Recognize.h"

/* The implementation of RPMoveBase.h */
namespace KCL_rosplan {

	/* constructor */
	RPPerceptionAction::RPPerceptionAction(ros::NodeHandle &nh, const std::string &actionserver, const std::string& recogniseserver)
	 : message_store(nh), examine_action_client(actionserver, true), recognise_action_client(recogniseserver, true), ptpActionClient("/joint_ptp", true)
	{

		// create the action clients
		ROS_INFO("KCL: (PerceptionAction) waiting for action server to start on %s", actionserver.c_str());
		examine_action_client.waitForServer();
		ROS_INFO("KCL: (PerceptionAction) action server found!");

		ROS_INFO("KCL: (PerceptionAction) waiting for recognision server to start on %s", recogniseserver.c_str());
		recognise_action_client.waitForServer();
		ROS_INFO("KCL: (PerceptionAction) action server found!");


		ROS_INFO("KCL: (PerceptionAction) waiting for ptp server to start...");
		ptpActionClient.waitForServer();
		ROS_INFO("KCL: (PerceptionAction) ptp server found!");
		
		// create the action feedback publisher
		action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
		find_dynamic_objects_client = nh.serviceClient<squirrel_object_perception_msgs::FindDynamicObjects>("/squirrel_find_dynamic_objects");
		get_instance_client = nh.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_current_instances");

		examine_action_service = nh.serviceClient<squirrel_object_perception_msgs::Recognize>("/squirrel_recognizer/squirrel_recognize_objects");
		knowledge_query_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeQueryService>("/kcl_rosplan/query_knowledge_base");

		joint_state_sub = nh.subscribe("/real/robotino/joint_control/get_state", 10, &RPPerceptionAction::jointCallback, this);
	}

	/* action dispatch callback; parameters (?v - robot ?wp - waypoint) */
	void RPPerceptionAction::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	{
		// ignore non-perception actions
		if(msg->name == "explore_waypoint") exploreAction(msg);
		if(msg->name == "observe-classifiable_from") examineAction(msg);
		if(msg->name == "look_at_object") lookAtObject(msg);
		if(msg->name == "examine_object") examineObject(msg);
		if(msg->name == "examine_object_in_hand") examineObjectInHandAction(msg);
	}

	
	/* action dispatch callback; explore action */
	void RPPerceptionAction::exploreAction(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	{
		ROS_INFO("KCL: (PerceptionAction) explore action recieved");

		// get waypoint ID from action dispatch
		std::string explored_waypoint;
		bool foundWP = false;
		for(size_t i=0; i<msg->parameters.size(); i++) {
			if(0==msg->parameters[i].key.compare("wp")) {
				explored_waypoint = msg->parameters[i].value;
				foundWP = true;
			}
		}
		if(!foundWP) {
			ROS_INFO("KCL: (PerceptionAction) aborting action dispatch; malformed parameters");
			return;
		}

		// report this action is enabled
		publishFeedback(msg->action_id,"action enabled");

		// find dynamic objects
		squirrel_object_perception_msgs::FindDynamicObjects fdSrv;
		if (!find_dynamic_objects_client.call(fdSrv)) {
			ROS_ERROR("KCL: (PerceptionAction) Could not call the find_dynamic_objects service.");
		}

		// add all new objects
		std::vector<squirrel_object_perception_msgs::SceneObject>::const_iterator ci = fdSrv.response.dynamic_objects_added.begin();
		for (; ci != fdSrv.response.dynamic_objects_added.end(); ++ci) {
			squirrel_object_perception_msgs::SceneObject so = (*ci);
			addObject(so);
		}

		// update all new objects
		ci = fdSrv.response.dynamic_objects_updated.begin();
		for (; ci != fdSrv.response.dynamic_objects_updated.end(); ++ci) {
			squirrel_object_perception_msgs::SceneObject so = (*ci);
			updateObject(so, explored_waypoint);
		}

		// remove ghost objects
		ci = fdSrv.response.dynamic_objects_removed.begin();
		for (; ci != fdSrv.response.dynamic_objects_removed.end(); ++ci) {
			squirrel_object_perception_msgs::SceneObject so = (*ci);
			removeObject(so);
		}

		// add the new knowledge
		rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		rosplan_knowledge_msgs::KnowledgeItem knowledge;
		knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		knowledge.attribute_name = "explored";
		diagnostic_msgs::KeyValue kv;
		kv.key = "wp";
		kv.value = explored_waypoint;
		knowledge.values.push_back(kv);

		knowledge_update_service.request.knowledge = knowledge;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (PerceptionAction) Could not add the explored predicate to the knowledge base.");
		}

		// report this action is achieved
		publishFeedback(msg->action_id,"action achieved");
	}

	/**
	 * examine action dispatch callback;
	 * parameters ()
	 */
	void RPPerceptionAction::lookAtObject(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	{
		ROS_INFO("KCL: (PerceptionAction) explore action recieved");

		// get object ID from action dispatch
		std::string object_id;
		bool foundObject = false;
		for(size_t i=0; i<msg->parameters.size(); i++) {
			if(0==msg->parameters[i].key.compare("o")) {
				object_id = msg->parameters[i].value;
				foundObject = true;
			}
		}
		if(!foundObject) {
			ROS_INFO("KCL: (PerceptionAction) aborting action dispatch; malformed parameters");
			return;
		}

		// publish feedback (enabled)
		publishFeedback(msg->action_id,"action enabled");

		// Get the object pose.
		std::stringstream ss;
		ss << object_id << "_wp";
		std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > results;
		if(message_store.queryNamed<geometry_msgs::PoseStamped>(ss.str(), results)) {
			if(results.size()<1) {
				ROS_ERROR("KCL: (PerceptionAction) aborting waypoint request; no matching object wp %s", ss.str().c_str());
				publishFeedback(msg->action_id, "action failed");
				return;
			}
		} else {
			ROS_ERROR("KCL: (PerceptionAction) could not query message store to fetch object wp %s", ss.str().c_str());
			publishFeedback(msg->action_id, "action failed");
			return;
		}

		// request manipulation waypoints for object
		geometry_msgs::PoseStamped &object_wp = *results[0];

		ROS_INFO("KCL: (PerceptionAction) waiting for recognizer action server to start");
		recognise_action_client.waitForServer();
		ROS_INFO("KCL: (PerceptionAction) action server started!");


		squirrel_object_perception_msgs::RecognizeObjectsGoal perceptionGoal;
		perceptionGoal.look_for_object = squirrel_object_perception_msgs::RecognizeObjectsGoal::EXPLORE;
		perceptionGoal.look_at_pose = object_wp;
		recognise_action_client.sendGoal(perceptionGoal);

		ROS_INFO("KCL: (PerceptionAction) goal sent, waiting for result.");

		recognise_action_client.waitForResult();
		actionlib::SimpleClientGoalState state = recognise_action_client.getState();
		bool success =  (state == actionlib::SimpleClientGoalState::SUCCEEDED) && recognise_action_client.getResult()->objects_added.size() + recognise_action_client.getResult()->objects_updated.size() > 0;
		ROS_INFO("KCL: (PerceptionAction) check object finished: %s", state.toString().c_str());

		if (success) {

			ROS_INFO("KCL: (PerceptionAction) Found %zd objects!", (recognise_action_client.getResult()->objects_added.size() + recognise_action_client.getResult()->objects_updated.size()));
			for (std::vector<squirrel_object_perception_msgs::SceneObject>::const_iterator ci = recognise_action_client.getResult()->objects_added.begin(); ci != recognise_action_client.getResult()->objects_added.end(); ++ci)
			{
				squirrel_object_perception_msgs::SceneObject so = *ci;
				ROS_INFO("KCL: (PerceptionAction) ADD: %s (%s).", so.id.c_str(), so.category.c_str());

				if (recognise_action_client.getResult()->used_wizard)
				{
					// Set the data from what we have stored as default values.
					ROS_INFO("KCL: (PerceptionAction) Used the wizard, using default pose instead.");
					so.pose = object_wp.pose;
					so.bounding_cylinder.height = 0.2;
				}
				so.header.frame_id = "/map";
				so.header.stamp = ros::Time::now();
				so.id = object_id;
				so.category = object_id;
				addObject(so);
			}
		} else if (state != actionlib::SimpleClientGoalState::SUCCEEDED)  {
			ROS_INFO("KCL: (PerceptionAction) action failed");
			publishFeedback(msg->action_id, "action failed");
			return;
		} else {
			ROS_ERROR("KCL: (PerceptionAction) No objects returned!");
			publishFeedback(msg->action_id, "action failed");
			return;
		}
		// publish feedback
		ROS_INFO("KCL: (PerceptionAction) action complete");
		publishFeedback(msg->action_id, "action achieved");
	}

	/**
	 * examine action dispatch callback;
	 * parameters ()
	 */
	void RPPerceptionAction::examineObject(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	{

		ROS_INFO("KCL: (PerceptionAction) explore action recieved");

		// publish feedback (enabled)
		publishFeedback(msg->action_id,"action enabled");

		ROS_INFO("KCL: (PerceptionAction) waiting for recognizer action server to start");
		recognise_action_client.waitForServer();
		ROS_INFO("KCL: (PerceptionAction) action server started!");

		// Locate the location of the robot.
		tf::StampedTransform transform;
		tf::TransformListener tfl;
		try {
			tfl.waitForTransform("/map","/base_link", ros::Time::now(), ros::Duration(1.0));
			tfl.lookupTransform("/map", "/base_link", ros::Time(0), transform);
		} catch ( tf::TransformException& ex ) {
			ROS_ERROR("KCL: (PerceptionAction) Error find the transform between /map and /base_link.");
			publishFeedback(msg->action_id, "action failed");
			return;
		}
		
		std::string closest_box;
		geometry_msgs::PoseStamped closest_box_pose;
		float min_distance_from_robot = std::numeric_limits<float>::max();
		
		// Get all boxes and their poses, pick the one that is closest.
		rosplan_knowledge_msgs::GetInstanceService getInstances;
		getInstances.request.type_name = "box";
		if (!get_instance_client.call(getInstances)) {
			ROS_ERROR("KCL: (PerceptionAction) Failed to get all the box instances.");
			publishFeedback(msg->action_id, "action failed");
			return;
		}

		ROS_INFO("KCL: (PerceptionAction) Received all the box instances %zd.", getInstances.response.instances.size());
		for (std::vector<std::string>::const_iterator ci = getInstances.response.instances.begin(); ci != getInstances.response.instances.end(); ++ci)
		{
			// fetch position of the box from message store
			std::stringstream ss;
			ss << *ci << "_location";
			std::string box_loc = ss.str();

			std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > results;
			if(message_store.queryNamed<geometry_msgs::PoseStamped>(box_loc, results)) {
				if(results.size()<1) {
					ROS_ERROR("KCL: (PerceptionAction) aborting waypoint request; no matching boxID %s", box_loc.c_str());
					publishFeedback(msg->action_id, "action failed");
					return;
				}
			} else {
				ROS_ERROR("KCL: (PerceptionAction) could not query message store to fetch box pose %s", box_loc.c_str());
				publishFeedback(msg->action_id, "action failed");
				return;
			}

			// request manipulation waypoints for object
			geometry_msgs::PoseStamped &box_pose = *results[0];
			float distance = (box_pose.pose.position.x - transform.getOrigin().getX()) * (box_pose.pose.position.x - transform.getOrigin().getX()) +
			                 (box_pose.pose.position.y - transform.getOrigin().getY()) * (box_pose.pose.position.y - transform.getOrigin().getY());
			
			if (distance < min_distance_from_robot)
			{
				min_distance_from_robot = distance;
				closest_box = *ci;
				closest_box_pose = box_pose;
			}
		}

		squirrel_object_perception_msgs::RecognizeObjectsGoal perceptionGoal;
		perceptionGoal.look_for_object = squirrel_object_perception_msgs::RecognizeObjectsGoal::EXPLORE;
		perceptionGoal.look_at_pose = closest_box_pose;
		recognise_action_client.sendGoal(perceptionGoal);

		ROS_INFO("KCL: (PerceptionAction) goal sent, waiting for result.");

		recognise_action_client.waitForResult();
		actionlib::SimpleClientGoalState state = recognise_action_client.getState();
		bool success =  (state == actionlib::SimpleClientGoalState::SUCCEEDED) && recognise_action_client.getResult()->objects_added.size() + recognise_action_client.getResult()->objects_updated.size() > 0;
		ROS_INFO("KCL: (PerceptionAction) check object finished: %s", state.toString().c_str());

		if (success) {

			ROS_INFO("KCL: (PerceptionAction) Found %zd objects!", (recognise_action_client.getResult()->objects_added.size() + recognise_action_client.getResult()->objects_updated.size()));
			for (std::vector<squirrel_object_perception_msgs::SceneObject>::const_iterator ci = recognise_action_client.getResult()->objects_added.begin(); ci != recognise_action_client.getResult()->objects_added.end(); ++ci)
			{
				squirrel_object_perception_msgs::SceneObject so = *ci;
				ROS_INFO("KCL: (PerceptionAction) ADD: %s (%s).", so.id.c_str(), so.category.c_str());
				so.id = so.category;
				addObject(so);
			}
		} else if (state != actionlib::SimpleClientGoalState::SUCCEEDED)  {
			ROS_INFO("KCL: (PerceptionAction) action failed");
			publishFeedback(msg->action_id, "action failed");
			return;
		} else {
			ROS_ERROR("KCL: (PerceptionAction) No objects returned!");
			publishFeedback(msg->action_id, "action failed");
			return;
		}
		// publish feedback
		ROS_INFO("KCL: (PerceptionAction) action complete");
		publishFeedback(msg->action_id, "action achieved");
	}

	/**
	 * examine action dispatch callback;
	 * parameters (?from ?view - waypoint ?o - object ?v - robot  ?l ?l2 - level ?kb - knowledgebase)
	 */
	void RPPerceptionAction::examineAction(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	{

		ROS_INFO("KCL: (PerceptionAction) explore action recieved");

		// get waypoint ID from action dispatch
		std::string objectID, wpID, fromID;
		bool foundObject = false;
		for(size_t i=0; i<msg->parameters.size(); i++) {
			if(0==msg->parameters[i].key.compare("view"))
				wpID = msg->parameters[i].value;
			if(0==msg->parameters[i].key.compare("from"))
				fromID = msg->parameters[i].value;
			if(0==msg->parameters[i].key.compare("o")) {
				objectID = msg->parameters[i].value;
				foundObject = true;
			}
		}
		if(!foundObject) {
			ROS_INFO("KCL: (PerceptionAction) aborting action dispatch; malformed parameters");
			return;
		}

		// publish feedback (enabled)
		publishFeedback(msg->action_id,"action enabled");

		ROS_INFO("KCL: (PerceptionAction) waiting for recognizer action server to start");
		examine_action_client.waitForServer();
		ROS_INFO("KCL: (PerceptionAction) action server started!");

		squirrel_object_perception_msgs::LookForObjectsGoal perceptionGoal;
		perceptionGoal.look_for_object = squirrel_object_perception_msgs::LookForObjectsGoal::EXPLORE;
		perceptionGoal.id = objectID;
		examine_action_client.sendGoal(perceptionGoal);

		ROS_INFO("KCL: (PerceptionAction) goal sent, waiting for result.");

		examine_action_client.waitForResult();
		actionlib::SimpleClientGoalState state = examine_action_client.getState();
		bool success =  (state == actionlib::SimpleClientGoalState::SUCCEEDED) && examine_action_client.getResult()->objects_added.size() + examine_action_client.getResult()->objects_updated.size() > 0;
		ROS_INFO("KCL: (PerceptionAction) check object finished: %s", state.toString().c_str());

		// update classifiable_from in the knowledge base .
		rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
		knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		knowledge_item.attribute_name = "classifiable_from";
		knowledge_item.is_negative = !success;

		diagnostic_msgs::KeyValue kv;
		kv.key = "from";
		kv.value = fromID;
		knowledge_item.values.push_back(kv);
	
		kv.key = "view";
		kv.value = wpID;
		knowledge_item.values.push_back(kv);
	
		kv.key = "o";
		kv.value = objectID;
		knowledge_item.values.push_back(kv);

		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (ClassifyObjectPDDLAction) Could not add the classifiable_from predicate to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (ClassifyObjectPDDLAction) Added %s (classifiable_from %s %s %s) to the knowledge base.", knowledge_item.is_negative ? "NOT" : "", fromID.c_str(), wpID.c_str(), objectID.c_str());
       
		// Remove the opposite option from the knowledge base.
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
		knowledge_item.is_negative = !knowledge_item.is_negative;
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (ClassifyObjectPDDLAction) Could not remove the classifiable_from predicate to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (ClassifyObjectPDDLAction) Removed %s (classifiable_from %s %s %s) to the knowledge base.", knowledge_item.is_negative ? "NOT" : "", fromID.c_str(), wpID.c_str(), objectID.c_str());
	
		knowledge_item.values.clear();

		if (success) {

			ROS_INFO("KCL: (PerceptionAction) Found %zd objects!", (examine_action_client.getResult()->objects_added.size() + examine_action_client.getResult()->objects_updated.size()));
			for (std::vector<squirrel_object_perception_msgs::SceneObject>::const_iterator ci = examine_action_client.getResult()->objects_added.begin(); ci != examine_action_client.getResult()->objects_added.end(); ++ci)
			{
				ROS_INFO("KCL: (PerceptionAction) ADD: %s (%s).", ci->id.c_str(), ci->category.c_str());
			}
			for (std::vector<squirrel_object_perception_msgs::SceneObject>::const_iterator ci = examine_action_client.getResult()->objects_updated.begin(); ci != examine_action_client.getResult()->objects_updated.end(); ++ci)
			{
				ROS_INFO("KCL: (PerceptionAction) UPDATE: %s (%s).", ci->id.c_str(), ci->category.c_str());
			}

			if (examine_action_client.getResult()->objects_added.size() > 0)
			{
				updateType(objectID, examine_action_client.getResult()->objects_added[0].category);
			}

			if (examine_action_client.getResult()->objects_updated.size() > 0)
			{
				updateType(objectID, examine_action_client.getResult()->objects_updated[0].category);
			}

			// add all new objects
			std::vector<squirrel_object_perception_msgs::SceneObject>::const_iterator ci = examine_action_client.getResult()->objects_added.begin();
			for (; ci != examine_action_client.getResult()->objects_added.end(); ++ci) {
				squirrel_object_perception_msgs::SceneObject so = (*ci);
				addObject(so);
			}

			// update all new objects
			ci = examine_action_client.getResult()->objects_updated.begin();
			for (; ci != examine_action_client.getResult()->objects_updated.end(); ++ci) {
				squirrel_object_perception_msgs::SceneObject so = (*ci);
				updateObject(so, wpID);
			}

		} else if (state != actionlib::SimpleClientGoalState::SUCCEEDED)  {
			ROS_INFO("KCL: (PerceptionAction) action failed");
			publishFeedback(msg->action_id, "action failed");
			return;
		}
		// publish feedback
		ROS_INFO("KCL: (PerceptionAction) action complete");
		publishFeedback(msg->action_id, "action achieved");
	}

	/**
	 * examine action dispatch callback;
	 * parameters (?o - object ?v - robot)
	 */
	void RPPerceptionAction::examineObjectInHandAction(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	{

		ROS_INFO("KCL: (PerceptionAction) examine object in hand action recieved");

		// get waypoint ID from action dispatch
		std::string objectID;
		for(size_t i=0; i<msg->parameters.size(); i++) {
			if(0==msg->parameters[i].key.compare("o")) {
				objectID = msg->parameters[i].value;
			}
		}
		if(objectID == "") {
			ROS_INFO("KCL: (PerceptionAction) aborting action dispatch; malformed parameters");
			return;
		}

		// publish feedback (enabled)
		publishFeedback(msg->action_id,"action enabled");

		ROS_INFO("KCL: (PerceptionAction) waiting for recognizer action server to start");
		examine_action_client.waitForServer();
		ROS_INFO("KCL: (PerceptionAction) action server started!");

		if (!extendArm())
		{
			ROS_ERROR("KCL: (PerceptionAction) failed to extends the arm.");
			publishFeedback(msg->action_id,"action failed");
			return;
		}

		squirrel_object_perception_msgs::LookForObjectsGoal perceptionGoal;
		perceptionGoal.look_for_object = squirrel_object_perception_msgs::LookForObjectsGoal::EXPLORE;
		perceptionGoal.id = objectID;
		examine_action_client.sendGoal(perceptionGoal);

		ROS_INFO("KCL: (PerceptionAction) goal sent, waiting for result.");

		examine_action_client.waitForResult();
		actionlib::SimpleClientGoalState state = examine_action_client.getState();
		bool success =  (state == actionlib::SimpleClientGoalState::SUCCEEDED) && examine_action_client.getResult()->objects_added.size() + examine_action_client.getResult()->objects_updated.size() > 0;
		ROS_INFO("KCL: (PerceptionAction) check object finished: %s", state.toString().c_str());

		if (success) {

			ROS_INFO("KCL: (PerceptionAction) Found %zd objects!", (examine_action_client.getResult()->objects_added.size() + examine_action_client.getResult()->objects_updated.size()));
			for (std::vector<squirrel_object_perception_msgs::SceneObject>::const_iterator ci = examine_action_client.getResult()->objects_added.begin(); ci != examine_action_client.getResult()->objects_added.end(); ++ci)
			{
				ROS_INFO("KCL: (PerceptionAction) ADD: %s (%s).", ci->id.c_str(), ci->category.c_str());
			}
			for (std::vector<squirrel_object_perception_msgs::SceneObject>::const_iterator ci = examine_action_client.getResult()->objects_updated.begin(); ci != examine_action_client.getResult()->objects_updated.end(); ++ci)
			{
				ROS_INFO("KCL: (PerceptionAction) UPDATE: %s (%s).", ci->id.c_str(), ci->category.c_str());
			}

			if (examine_action_client.getResult()->objects_added.size() > 0)
			{
				updateType(objectID, examine_action_client.getResult()->objects_added[0].category);
			}

			if (examine_action_client.getResult()->objects_updated.size() > 0)
			{
				updateType(objectID, examine_action_client.getResult()->objects_updated[0].category);
			}
		} else if (state != actionlib::SimpleClientGoalState::SUCCEEDED)  {
			ROS_INFO("KCL: (PerceptionAction) action failed");
			publishFeedback(msg->action_id, "action failed");
			return;
		}

		if (!retractArm())
		{
			ROS_ERROR("KCL: (PerceptionAction) failed to retract the arm.");
			publishFeedback(msg->action_id,"action failed");
			return;
		}
		// publish feedback
		ROS_INFO("KCL: (PerceptionAction) action complete");
		publishFeedback(msg->action_id, "action achieved");
	}

	void RPPerceptionAction::publishFeedback(int action_id, std::string feedback) {
		// publish feedback
		rosplan_dispatch_msgs::ActionFeedback fb;
		fb.action_id = action_id;
		fb.status = feedback;
		action_feedback_pub.publish(fb);
	}
	
	void RPPerceptionAction::updateType(const std::string& object_id, const std::string& object_rec_name)
	{
		ROS_INFO("KCL: (PerceptionAction) Update where %s belongs.", object_id.c_str());
		
		// is_of_type fact
		rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
		
		diagnostic_msgs::KeyValue kv;
		
		// First we get all the possible types of toys that we can encounter.
		rosplan_knowledge_msgs::GetInstanceService getInstances;
		getInstances.request.type_name = "box";
		if (!get_instance_client.call(getInstances)) {
			ROS_ERROR("KCL: (PerceptionAction) Failed to get all the box instances.");
			return;
		}
		ROS_INFO("KCL: (PerceptionAction) Received %zd box instances.", getInstances.response.instances.size());
		
		std::string found_box;
		for (std::vector<std::string>::const_iterator ci = getInstances.response.instances.begin(); ci != getInstances.response.instances.end(); ++ci)
		{
			const std::string& box = *ci;
			rosplan_knowledge_msgs::KnowledgeQueryService knowledge_query;
			rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
			
			knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			knowledge_item.attribute_name = "belongs_in";
			kv.key = "o";
			kv.value = object_rec_name;
			knowledge_item.values.push_back(kv);
			kv.key = "b";
			kv.value = box;
			knowledge_item.values.push_back(kv);
			knowledge_item.is_negative = false;

			// Check if this fact is true.
			knowledge_query.request.knowledge.push_back(knowledge_item);

			if (!knowledge_query_client.call(knowledge_query))
			{
				ROS_INFO("KCL: (PerceptionAction) Could not query the knowledge base.");
				exit(1);
			}

			if (knowledge_query.response.results[0] != 1)
			{
				ROS_INFO("KCL: (PerceptionAction) %s does not belong in %s", object_rec_name.c_str(), box.c_str());
			}
			else
			{
				found_box = box;
				ROS_INFO("KCL: (PerceptionAction) %s belongs in %s", object_rec_name.c_str(), box.c_str());
			}
		}
		
		// Add new type, if necessary.
		if (found_box != "")
		{
			for (std::vector<std::string>::const_iterator ci = getInstances.response.instances.begin(); ci != getInstances.response.instances.end(); ++ci)
			{
				const std::string& box = *ci;

				// Make if of that type.
				knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
				knowledge_update_service.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
				knowledge_update_service.request.knowledge.attribute_name = "belongs_in";
				kv.key = "o";
				kv.value = object_id;
				knowledge_update_service.request.knowledge.values.push_back(kv);
				kv.key = "b";
				kv.value = box;
				knowledge_update_service.request.knowledge.values.push_back(kv);
				knowledge_update_service.request.knowledge.is_negative = box != found_box;

				if (!update_knowledge_client.call(knowledge_update_service)) {
					ROS_ERROR("KCL: (PerceptionAction) Could not add belongs_in predicate to the knowledge base.");
				}
				ROS_ERROR("KCL: (PerceptionAction) Add %s (belongs_in %s %s) predicate to the knowledge base.", knowledge_update_service.request.knowledge.is_negative ? "NOT" : "", object_id.c_str(), box.c_str());
				knowledge_update_service.request.knowledge.values.clear();

				knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
				knowledge_update_service.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
				knowledge_update_service.request.knowledge.attribute_name = "belongs_in";
				kv.key = "o";
				kv.value = object_id;
				knowledge_update_service.request.knowledge.values.push_back(kv);
				kv.key = "b";
				kv.value = box;
				knowledge_update_service.request.knowledge.values.push_back(kv);
				knowledge_update_service.request.knowledge.is_negative = box == found_box;

				if (!update_knowledge_client.call(knowledge_update_service)) {
					ROS_ERROR("KCL: (PerceptionAction) Could not add belongs_in predicate to the knowledge base.");
				}
				ROS_ERROR("KCL: (PerceptionAction) Remove %s (belongs_in %s %s) predicate from the knowledge base.", knowledge_update_service.request.knowledge.is_negative ? "NOT" : "", object_id.c_str(), box.c_str());
				knowledge_update_service.request.knowledge.values.clear();
			}
		}
	}
	
	void RPPerceptionAction::addObject(squirrel_object_perception_msgs::SceneObject &object) {

		std::stringstream wpid;
		wpid << "waypoint_" << object.id;
		std::string wpName(wpid.str());

		updateObject(object, wpName);

	}

	void RPPerceptionAction::updateObject(squirrel_object_perception_msgs::SceneObject &object, std::string newWaypoint) {

		// add the new object
		rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		knowledge_update_service.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
		knowledge_update_service.request.knowledge.instance_type = "object";
		knowledge_update_service.request.knowledge.instance_name = object.id;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (PerceptionAction) Could not add the object %s to the knowledge base.", object.id.c_str());
		}
		ROS_ERROR("KCL: (PerceptionAction) Added the object %s to the knowledge base.", object.id.c_str());
		

		// add the new object's waypoint
		knowledge_update_service.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
		knowledge_update_service.request.knowledge.instance_type = "waypoint";
		knowledge_update_service.request.knowledge.instance_name = newWaypoint;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (PerceptionAction) Could not add the object %s to the knowledge base.", newWaypoint.c_str());
		}
		ROS_ERROR("KCL: (PerceptionAction) Added the waypoint %s to the knowledge base.", newWaypoint.c_str());

		// object_at fact	
		knowledge_update_service.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		knowledge_update_service.request.knowledge.attribute_name = "object_at";
		knowledge_update_service.request.knowledge.is_negative = false;
		diagnostic_msgs::KeyValue kv;
		kv.key = "o";
		kv.value = object.id;
		knowledge_update_service.request.knowledge.values.push_back(kv);
		kv.key = "wp";
		kv.value = newWaypoint;
		knowledge_update_service.request.knowledge.values.push_back(kv);
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (PerceptionAction) Could not add object_at predicate to the knowledge base.");
		}

		// Add the opposite to the knowledge base.

		//data
		geometry_msgs::PoseStamped ps;
		ps.header = object.header;
		ps.pose = object.pose;
		db_name_map[newWaypoint] = message_store.insertNamed(newWaypoint, ps);
		db_name_map[object.id] = message_store.insertNamed(object.id, object);
	}

	void RPPerceptionAction::removeObject(squirrel_object_perception_msgs::SceneObject &object) {

			rosplan_knowledge_msgs::KnowledgeUpdateService updateSrv;
			updateSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
			updateSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
			updateSrv.request.knowledge.instance_type = "object";
			updateSrv.request.knowledge.instance_name = object.id;
			update_knowledge_client.call(updateSrv);

			//data
			message_store.deleteID(db_name_map[object.id]);
	}

	void RPPerceptionAction::waitForArm(const std_msgs::Float64MultiArray& goal_state, float error)
	{
		ros::Rate loop_rate(1);
		while (ros::ok())
		{
			ros::spinOnce();
			loop_rate.sleep();

			// Check whether we have reached the goal location yet.
			bool done = true;
			if (goal_state.data.size() != last_joint_state.position.size())
			{
				ROS_INFO("KCL (RPGraspAction) Goal state and last_joint_state don't have the same sized array! %zd %zd", goal_state.data.size(), last_joint_state.position.size());
			}
			ROS_INFO("KCL (RPGraspAction) Goal state and last_joint_state have the same sized array! %zd %zd", goal_state.data.size(), last_joint_state.position.size());
			for (unsigned int i = 3; i < std::min(goal_state.data.size(), last_joint_state.position.size()); ++i)
			{


				if (i > goal_state.data.size()) continue;
				if (std::abs(goal_state.data[i] - last_joint_state.position[i]) > error)
				{
					std::cout << i << std::endl;
					ROS_INFO("KCL (RPPerceptionAction) Joint #%zd is %f off target, not done yet!", i, std::abs(goal_state.data[i] - last_joint_state.position[i]));
					done = false;
					break;
				}
			}
			if (done) break;
		}
	}

	/**
	 * Arm manipulation.
	 */
	bool RPPerceptionAction::extendArm()
	{
		ROS_INFO("KCL: (RPPerceptionAction) Extend arm\n");
		std_msgs::Float64MultiArray data_arm;
		data_arm.data = last_joint_state.position;
		data_arm.data[3] = 1.5;
		data_arm.data[4] = 0.86;
		data_arm.data[5] = 0;
		data_arm.data[6] = -1.6;
		data_arm.data[7] = -1.8;

		squirrel_manipulation_msgs::JointPtpActionGoal armEndGoal;
		armEndGoal.goal.joints = data_arm;

		ptpActionClient.sendGoal(armEndGoal.goal);
		ROS_INFO("KCL: (RPPerceptionAction) Goal sent\n");
		ptpActionClient.waitForResult(ros::Duration(30.0));
		ROS_INFO("KCL: (RPPerceptionAction) Waiting form arm to finish moving...\n");
		sleep(1.0);
		ptpActionClient.sendGoal(armEndGoal.goal);

		waitForArm(data_arm, 0.05f);
		return true;

		if (ptpActionClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
			ROS_INFO("KCL: (RPPerceptionAction) Arm moved \n");
			return true;
		}else{
			ROS_ERROR("KCL: (RPPerceptionAction) Arm FAILED to move! \n");
			return false;
		}
	}

	bool RPPerceptionAction::retractArm()
	{
		ROS_INFO("KCL: (RPPerceptionAction) Retract arm\n");
		std_msgs::Float64MultiArray data_arm;
		data_arm.data = last_joint_state.position;
		data_arm.data[3] = 0.7;
		data_arm.data[4] = 1.6;
		data_arm.data[5] = 0;
		data_arm.data[6] = -1.7;
		data_arm.data[7] = -1.8;

		squirrel_manipulation_msgs::JointPtpActionGoal armEndGoal;
		armEndGoal.goal.joints = data_arm;


		ptpActionClient.sendGoal(armEndGoal.goal);
		ROS_INFO("KCL: (RPPerceptionAction) Goal sent\n");
		ptpActionClient.waitForResult(ros::Duration(30.0));
		ROS_INFO("KCL: (RPPerceptionAction) Waiting form arm to finish moving...\n");
		sleep(1.0);
		ptpActionClient.sendGoal(armEndGoal.goal);

		waitForArm(data_arm, 0.05f);
		return true;

		if (ptpActionClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
			ROS_INFO("KCL: (RPPerceptionAction) Arm moved \n");
			return true;
		}else{
			ROS_ERROR("KCL: (RPPerceptionAction) Arm FAILED to move! \n");
			return false;
		}
	}

	void RPPerceptionAction::jointCallback(const sensor_msgs::JointStateConstPtr& msg)
	{
		last_joint_state = *msg;
	}
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_interface_perception");
		ros::NodeHandle nh;

		std::string actionserver, recogniseserver;
		nh.param("action_server", actionserver, std::string("/squirrel_look_for_objects_in_hand"));
		nh.param("recognise_server", recogniseserver, std::string("/squirrel_recognize_objects"));

		// create PDDL action subscriber
		KCL_rosplan::RPPerceptionAction rppa(nh, actionserver, recogniseserver);
	
		// listen for action dispatch
		ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPPerceptionAction::dispatchCallback, &rppa);
		ROS_INFO("KCL: (PerceptionAction) Ready to receive");

		while(ros::ok() && ros::master::check()){ros::spinOnce();}
		return 0;
	}

