#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Quaternion.h>
#include <actionlib/client/simple_action_client.h>
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "squirrel_object_perception_msgs/LookForObjectsAction.h"
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
	RPPerceptionAction::RPPerceptionAction(ros::NodeHandle &nh, std::string &actionserver)
	 : message_store(nh), examine_action_client(actionserver, true)
	{

		// create the action clients
		ROS_INFO("KCL: (PerceptionAction) waiting for action server to start on %s", actionserver.c_str());
		examine_action_client.waitForServer();
		ROS_INFO("KCL: (PerceptionAction) action server found!");
		
		// create the action feedback publisher
		action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
		find_dynamic_objects_client = nh.serviceClient<squirrel_object_perception_msgs::FindDynamicObjects>("/squirrel_find_dynamic_objects");
		get_instance_client = nh.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_current_instances");

		examine_action_service = nh.serviceClient<squirrel_object_perception_msgs::Recognize>("/squirrel_recognizer/squirrel_recognize_objects");
		knowledge_query_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeQueryService>("/kcl_rosplan/query_knowledge_base");
	}

	/* action dispatch callback; parameters (?v - robot ?wp - waypoint) */
	void RPPerceptionAction::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	{
		// ignore non-perception actions
		if(msg->name == "explore_waypoint") exploreAction(msg);
		if(msg->name == "observe-classifiable_from") examineAction(msg);
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
			}
		}
	}
	
	void RPPerceptionAction::addObject(squirrel_object_perception_msgs::SceneObject &object) {

		std::stringstream wpid;
		wpid << "waypoint_" << object.id;
		std::string wpName(wpid.str());

		// add the new object
		rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		knowledge_update_service.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
		knowledge_update_service.request.knowledge.instance_type = "object";
		knowledge_update_service.request.knowledge.instance_name = object.id;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (PerceptionAction) Could not add the object %s to the knowledge base.", object.id.c_str());
		}

		// add the new object's waypoint
		knowledge_update_service.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
		knowledge_update_service.request.knowledge.instance_type = "waypoint";
		knowledge_update_service.request.knowledge.instance_name = wpName;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (PerceptionAction) Could not add the object %s to the knowledge base.", wpName.c_str());
		}

		// object_at fact	
		knowledge_update_service.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		knowledge_update_service.request.knowledge.attribute_name = "object_at";
		knowledge_update_service.request.knowledge.is_negative = false;
		diagnostic_msgs::KeyValue kv;
		kv.key = "o";
		kv.value = object.id;
		knowledge_update_service.request.knowledge.values.push_back(kv);
		kv.key = "wp";
		kv.value = wpName;
		knowledge_update_service.request.knowledge.values.push_back(kv);
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (PerceptionAction) Could not add object_at predicate to the knowledge base.");
		}

/*
		// is_of_type fact	
		knowledge_update_service.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		knowledge_update_service.request.knowledge.attribute_name = "is_of_type";
		knowledge_update_service.request.knowledge.values.pop_back();
		kv.key = "t";
                kv.value = "dinosaur";
		knowledge_update_service.request.knowledge.values.push_back(kv);
		knowledge_update_service.request.knowledge.is_negative = object.category.find("dinosaur") == std::string::npos;

		if (knowledge_update_service.request.knowledge.is_negative)
		{
			ROS_INFO("KCL: (PerceptionAction) %s is NOT a dinosaur", object.id.c_str());
		}
		else
		{
			ROS_INFO("KCL: (PerceptionAction) %s IS a dinosaur", object.id.c_str());
		}
*/
/*
        // TODO remove this after demo
        if(object.category.find("dinosaur") != std::string::npos)
                kv.value = "dinosaur";
        else kv.value = object.category;
*/
		
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (PerceptionAction) Could not add is_of_type predicate to the knowledge base.");
		}

		// Add the opposite to the knowledge base.

		//data
		geometry_msgs::PoseStamped ps;
		ps.header = object.header;
		ps.pose = object.pose;
		db_name_map[wpName] = message_store.insertNamed(wpName, ps);
		db_name_map[object.id] = message_store.insertNamed(object.id, object);
	}

	void RPPerceptionAction::updateObject(squirrel_object_perception_msgs::SceneObject &object, std::string newWaypoint) {

		std::stringstream wpid;
		wpid << "waypoint_" << object.id;
		std::string wpName(wpid.str());

		//data
		geometry_msgs::PoseStamped ps;
		ps.header = object.header;
		ps.pose = object.pose;
		db_name_map[wpName] = message_store.insertNamed(wpName, ps);
		db_name_map[object.id] = message_store.updateNamed(object.id, object);
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

} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_interface_perception");
		ros::NodeHandle nh;

		std::string actionserver;
		nh.param("action_server", actionserver, std::string("/squirrel_look_for_objects_in_hand"));

		// create PDDL action subscriber
		KCL_rosplan::RPPerceptionAction rppa(nh, actionserver);
	
		// listen for action dispatch
		ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPPerceptionAction::dispatchCallback, &rppa);
		ROS_INFO("KCL: (PerceptionAction) Ready to receive");

		while(ros::ok() && ros::master::check()){ros::spinOnce();}
		return 0;
	}

