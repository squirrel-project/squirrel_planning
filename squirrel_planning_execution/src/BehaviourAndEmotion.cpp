#include <std_msgs/Int8.h>

#include <ros/ros.h>
#include <map>
#include <algorithm>
#include <string>
#include <sstream>

#include <tf/tf.h>
#include <squirrel_vad_msgs/vad.h>
#include "squirrel_planning_execution/BehaviourAndEmotion.h"
#include "pddl_actions/ShedKnowledgePDDLAction.h"
#include "pddl_actions/FinaliseClassificationPDDLAction.h"
#include "pddl_actions/PlannerInstance.h"
#include "pddl_actions/ExamineAreaPDDLAction.h"
#include "pddl_actions/ExploreAreaPDDLAction.h"
#include "pddl_actions/ObserveClassifiableOnAttemptPDDLAction.h"
#include "pddl_actions/TidyAreaPDDLAction.h"

#include "squirrel_object_perception_msgs/BCylinder.h"
#include "squirrel_object_perception_msgs/SceneObject.h"

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>


/* The implementation of BehaviourAndEmotion.h */
namespace KCL_rosplan {

	/*-------------*/
	/* constructor */
	/*-------------*/

	BehaviourAndEmotion::BehaviourAndEmotion(ros::NodeHandle &nh, const std::string& vis_pub_topic)
		: node_handle(&nh), message_store(nh), initial_problem_generated(false), simulated(true)
	{
		// knowledge interface
		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
		query_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeQueryService>("/kcl_rosplan/query_knowledge_base");
		
		// create the action feedback publisher
		//action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
		
		get_instance_client = nh.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_current_instances");
		get_attribute_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_knowledge");
		
		vis_pub = nh.advertise<visualization_msgs::Marker>(vis_pub_topic, 0 );
		//std::string classifyTopic("/squirrel_perception_examine_waypoint");
		//nh.param("squirrel_perception_classify_waypoint_service_topic", classifyTopic, classifyTopic);
		//classify_object_waypoint_client = nh.serviceClient<squirrel_waypoint_msgs::ExamineWaypoint>(classifyTopic);
		
		//pddl_generation_service = nh.advertiseService("/kcl_rosplan/generate_planning_problem", &KCL_rosplan::BehaviourAndEmotion::generatePDDLProblemFile, this);

		//nh.getParam("/squirrel_planning_execution/simulated", simulated);
		
		//setupSimulation();
	}
	
	void BehaviourAndEmotion::sendMarker(const geometry_msgs::Pose& pose, const std::string& name, float size)
	{
		static int id = 0;
		visualization_msgs::Marker marker;
		marker.header.frame_id = "/map";
		marker.header.stamp = ros::Time();
		marker.ns = name;
		marker.id = id++;
		marker.type = visualization_msgs::Marker::CYLINDER;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose = pose;
		marker.scale.x = size;
		marker.scale.y = size;
		marker.scale.z = size;
		marker.color.a = 1.0; // Don't forget to set the alpha!
		marker.color.r = 0.75;
		marker.color.g = 0.0;
		marker.color.b = 0.75;
		//only if using a MESH_RESOURCE marker type:
		vis_pub.publish( marker );
	}
	
	void BehaviourAndEmotion::tokenise(const std::string& s, std::vector<std::string>& tokens)
	{
		size_t current;
		size_t next = -1;

		do
		{
			current = next + 1;
			next = s.find_first_of(" ", current);
			tokens.push_back(s.substr(current, next - current));
		} 
		while (next != std::string::npos);
	}

	
	void BehaviourAndEmotion::setupSimulation(const std::string& config_file)
	{
		ROS_INFO("KCL: (BehaviourAndEmotion) Load scenarion from file: %s.\n", config_file.c_str());
		std::ifstream f(config_file.c_str());
		std::string line;

		rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;

		if (f.is_open())
		{
			while (getline(f, line))
			{

				std::cout << line << std::endl;
				if (line.size() == 0) continue;

				std::vector<std::string> tokens;
				tokenise(line, tokens);

				// Boxes.
				if (line[0] == 'b')
				{
					if (tokens.size() != 4)
					{
						ROS_ERROR("KCL (BehaviourAndEmotion) Malformed line, expected b BOX_NAME (f,f,f) (f,f,f). Read %s\n", line.c_str());
						exit(0);
					}
					
					knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
					
					std::string box_predicate = tokens[1];
					geometry_msgs::Pose box_location = transformToPose(tokens[2]);
					geometry_msgs::Pose near_box = transformToPose(tokens[3]);
					sendMarker(box_location, box_predicate, 0.25f);
					{
						std::stringstream ss;
						ss << "near_" << box_predicate;
						sendMarker(near_box, ss.str(), 0.1f);
					}
					
					// Add the box predicate to the knowledge base.
					rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
					
					knowledge_item.instance_type = "box";
					knowledge_item.instance_name = box_predicate;
					
					knowledge_update_service.request.knowledge = knowledge_item;knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
					if (!update_knowledge_client.call(knowledge_update_service)) {
						ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the box %s to the knowledge base.", box_predicate.c_str());
						exit(-1);
					}
					ROS_INFO("KCL: (BehaviourAndEmotion) Added %s to the knowledge base.", box_predicate.c_str());

					// Add waypoints for these boxes.
					knowledge_item.instance_type = "waypoint";
					std::stringstream ss;
					ss << box_predicate << "_location";
					knowledge_item.instance_name = ss.str();
					
					knowledge_update_service.request.knowledge = knowledge_item;
					if (!update_knowledge_client.call(knowledge_update_service)) {
						ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the waypoint %s to the knowledge base.", ss.str().c_str());
						exit(-1);
					}
					ROS_INFO("KCL: (BehaviourAndEmotion) Added %s to the knowledge base.", ss.str().c_str());

					// Set the actual location of the box waypoints in message store.
					{
					geometry_msgs::PoseStamped pose;
					pose.header.seq = 0;
					pose.header.stamp = ros::Time::now();
					pose.header.frame_id = "/map";
					pose.pose = box_location;
					
					pose.pose.orientation.x = 0.0f;
					pose.pose.orientation.y = 0.0f;
					pose.pose.orientation.z = 0.0f;
					pose.pose.orientation.w = 1.0f;
					std::string near_waypoint_mongodb_id3(message_store.insertNamed(ss.str(), pose));
					ROS_INFO("KCL: (BehaviourAndEmotion) Added %s to the knowledge base.", ss.str().c_str());
					}
					
					// Link the boxes to these waypoints.
					knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
					knowledge_item.attribute_name = "box_at";
					knowledge_item.is_negative = false;
					
					diagnostic_msgs::KeyValue kv;
					kv.key = "b";
					kv.value = box_predicate;
					knowledge_item.values.push_back(kv);
					
					kv.key = "wp";
					kv.value = ss.str();
					knowledge_item.values.push_back(kv);
					
					knowledge_update_service.request.knowledge = knowledge_item;
					if (!update_knowledge_client.call(knowledge_update_service)) {
						ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the fact (box_at %s %s) to the knowledge base.", box_predicate.c_str(), ss.str().c_str());
						exit(-1);
					}
					ROS_INFO("KCL: (BehaviourAndEmotion) Added the fact (box_at %s %s) to the knowledge base.", box_predicate.c_str(), ss.str().c_str());
					knowledge_item.values.clear();
					
					knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
					knowledge_item.instance_type = "waypoint";
					ss.str(std::string());
					ss << "near_" << box_predicate;
					knowledge_item.instance_name = ss.str();
					
					knowledge_update_service.request.knowledge = knowledge_item;
					if (!update_knowledge_client.call(knowledge_update_service)) {
						ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the waypoint %s to the knowledge base.", ss.str().c_str());
						exit(-1);
					}
					{
					geometry_msgs::PoseStamped pose;
					pose.header.seq = 0;
					pose.header.stamp = ros::Time::now();
					pose.header.frame_id = "/map";
					pose.pose = near_box;
					
					float angle = atan2(box_location.position.y - near_box.position.y, box_location.position.x - near_box.position.x);
					pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
	//				pose.pose.orientation.x = 0.0f;
	//				pose.pose.orientation.y = 0.0f;
	//				pose.pose.orientation.z = 1.0f;
	//				pose.pose.orientation.w = angle;
					std::string near_waypoint_mongodb_id3(message_store.insertNamed(ss.str(), pose));
					ROS_INFO("KCL: (BehaviourAndEmotion) Added %s to the knowledge base.", ss.str().c_str());
					}
				}
				// toys.
				else if (line[0] == 't')
				{
					if (tokens.size() != 4)
					{
						ROS_ERROR("KCL (BehaviourAndEmotion) Malformed line, expected t OBJECT_NAME (f,f,f) (f,f,f). Read %s\n", line.c_str());
						exit(0);
					}
					
					knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
					
					std::string toy_predicate = tokens[1];
					geometry_msgs::Pose toy_location = transformToPose(tokens[2]);
					geometry_msgs::Pose near_toy = transformToPose(tokens[3]);
					sendMarker(toy_location, toy_predicate, 0.25f);
					{
						std::stringstream ss;
						ss << "near_" << toy_predicate;
						sendMarker(near_toy, ss.str(), 0.1f);
					}
					
					// Add the box predicate to the knowledge base.
					rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
					
					knowledge_item.instance_type = "object";
					knowledge_item.instance_name = toy_predicate;
					
					knowledge_update_service.request.knowledge = knowledge_item;
					if (!update_knowledge_client.call(knowledge_update_service)) {
						ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the object %s to the knowledge base.", toy_predicate.c_str());
						exit(-1);
					}
					ROS_INFO("KCL: (BehaviourAndEmotion) Added %s to the knowledge base.", toy_predicate.c_str());

					// Add waypoints for these boxes.
					knowledge_item.instance_type = "waypoint";
					std::stringstream ss;
					ss << toy_predicate << "_location";
					knowledge_item.instance_name = ss.str();
					
					knowledge_update_service.request.knowledge = knowledge_item;
					if (!update_knowledge_client.call(knowledge_update_service)) {
						ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the waypoint %s to the knowledge base.", ss.str().c_str());
						exit(-1);
					}
					ROS_INFO("KCL: (BehaviourAndEmotion) Added %s to the knowledge base.", ss.str().c_str());

					// Set the actual location of the toy waypoints in message store.
					{
					geometry_msgs::PoseStamped pose;
					pose.header.seq = 0;
					pose.header.stamp = ros::Time::now();
					pose.header.frame_id = "/map";
					pose.pose = toy_location;
					
					pose.pose.orientation.x = 0.0f;
					pose.pose.orientation.y = 0.0f;
					pose.pose.orientation.z = 0.0f;
					pose.pose.orientation.w = 1.0f;
					std::string near_waypoint_mongodb_id3(message_store.insertNamed(ss.str(), pose));
					ROS_INFO("KCL: (BehaviourAndEmotion) Added %s to the knowledge base.", ss.str().c_str());
					}
					
					// Link the boxes to these waypoints.
					knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
					knowledge_item.attribute_name = "object_at";
					knowledge_item.is_negative = false;
					
					diagnostic_msgs::KeyValue kv;
					kv.key = "o";
					kv.value = toy_predicate;
					knowledge_item.values.push_back(kv);
					
					kv.key = "wp";
					kv.value = ss.str();
					knowledge_item.values.push_back(kv);
					
					knowledge_update_service.request.knowledge = knowledge_item;
					if (!update_knowledge_client.call(knowledge_update_service)) {
						ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the fact (object_at %s %s) to the knowledge base.", toy_predicate.c_str(), ss.str().c_str());
						exit(-1);
					}
					ROS_INFO("KCL: (BehaviourAndEmotion) Added the fact (object_at %s %s) to the knowledge base.", toy_predicate.c_str(), ss.str().c_str());
					knowledge_item.values.clear();
					
					knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
					knowledge_item.instance_type = "waypoint";
					ss.str(std::string());
					ss << "near_" << toy_predicate;
					knowledge_item.instance_name = ss.str();
					
					knowledge_update_service.request.knowledge = knowledge_item;
					if (!update_knowledge_client.call(knowledge_update_service)) {
						ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the waypoint %s to the knowledge base.", ss.str().c_str());
						exit(-1);
					}
					{
					geometry_msgs::PoseStamped pose;
					pose.header.seq = 0;
					pose.header.stamp = ros::Time::now();
					pose.header.frame_id = "/map";
					pose.pose = near_toy;
					
					float angle = atan2(toy_location.position.y - near_toy.position.y, toy_location.position.x - near_toy.position.x);
					pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
	//				pose.pose.orientation.x = 0.0f;
	//				pose.pose.orientation.y = 0.0f;
	//				pose.pose.orientation.z = 1.0f;
	//				pose.pose.orientation.w = angle;
					std::string near_waypoint_mongodb_id3(message_store.insertNamed(ss.str(), pose));
					ROS_INFO("KCL: (BehaviourAndEmotion) Added %s to the knowledge base.", ss.str().c_str());
					}
				}
				// Waypoints
				else if (line[0] == 'w')
				{
					if (tokens.size() != 3)
					{
						ROS_ERROR("KCL (BehaviourAndEmotion) Malformed line, expected w WAYPOINT_NAME (f,f,f). Read %s\n", line.c_str());
						exit(0);
					}
					
					knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
					
					std::string wp_predicate = tokens[1];
					geometry_msgs::Pose wp_location = transformToPose(tokens[2]);
					sendMarker(wp_location, wp_predicate, 0.25f);
					
					// Add the toy predicate to the knowledge base.
					rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
					
					knowledge_item.instance_type = "waypoint";
					knowledge_item.instance_name = wp_predicate;
					
					knowledge_update_service.request.knowledge = knowledge_item;
					knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
					if (!update_knowledge_client.call(knowledge_update_service)) {
						ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the waypoint %s to the knowledge base.", wp_predicate.c_str());
						exit(-1);
					}
					ROS_INFO("KCL: (BehaviourAndEmotion) Added %s to the knowledge base.", wp_predicate.c_str());

					// Set the actual location of the toy waypoints in message store.
					{
					geometry_msgs::PoseStamped pose;
					pose.header.seq = 0;
					pose.header.stamp = ros::Time::now();
					pose.header.frame_id = "/map";
					pose.pose = wp_location;
					
					pose.pose.orientation.x = 0.0f;
					pose.pose.orientation.y = 0.0f;
					pose.pose.orientation.z = 0.0f;
					pose.pose.orientation.w = 1.0f;
					std::string near_waypoint_mongodb_id3(message_store.insertNamed(wp_predicate, pose));
					ROS_INFO("KCL: (BehaviourAndEmotion) Added %s to the knowledge base.", wp_predicate.c_str());
					}
				}
				// child predicates
				else if (line[0] == 'c')
				{
					if (tokens.size() != 6)
					{
						ROS_ERROR("KCL (BehaviourAndEmotion) Malformed line, expected c CHILD_NAME P A D R. Read %s\n", line.c_str());
						exit(0);
					}
					
					knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
					
					std::string child_predicate = tokens[1];
					float pleasure = ::atof(tokens[2].c_str());
					float arousal = ::atof(tokens[3].c_str());
					float dominance = ::atof(tokens[4].c_str());
					float reciprocity = ::atof(tokens[5].c_str());
					
					// Add the toy predicate to the knowledge base.
					rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
					
					knowledge_item.instance_type = "child";
					knowledge_item.instance_name = child_predicate;
					
					knowledge_update_service.request.knowledge = knowledge_item;
					knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
					if (!update_knowledge_client.call(knowledge_update_service)) {
						ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the child %s to the knowledge base.", child_predicate.c_str());
						exit(-1);
					}
					ROS_INFO("KCL: (BehaviourAndEmotion) Added %s to the knowledge base.", child_predicate.c_str());
					
					// Setup the functions.
					diagnostic_msgs::KeyValue kv;
					kv.key = "c";
					kv.value = child_predicate;
					knowledge_item.values.push_back(kv);
					
					// Pleasure.
					knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FUNCTION;
					knowledge_item.attribute_name = "pleasure";
					knowledge_item.function_value = pleasure;
					
					knowledge_update_service.request.knowledge = knowledge_item;
					if (!update_knowledge_client.call(knowledge_update_service)) {
						ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the function (pleasure %s) to the knowledge base.", child_predicate.c_str());
						exit(-1);
					}
					ROS_INFO("KCL: (BehaviourAndEmotion) Added (pleasure %s) to the knowledge base.", child_predicate.c_str());
					
					// Arousal.
					knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FUNCTION;
					knowledge_item.attribute_name = "arousal";
					knowledge_item.function_value = arousal;
					
					knowledge_update_service.request.knowledge = knowledge_item;
					if (!update_knowledge_client.call(knowledge_update_service)) {
						ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the function (arousal %s) to the knowledge base.", child_predicate.c_str());
						exit(-1);
					}
					ROS_INFO("KCL: (BehaviourAndEmotion) Added (arousal %s) to the knowledge base.", child_predicate.c_str());
					
					// Domain.
					knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FUNCTION;
					knowledge_item.attribute_name = "dominance";
					knowledge_item.function_value = dominance;
					
					knowledge_update_service.request.knowledge = knowledge_item;
					if (!update_knowledge_client.call(knowledge_update_service)) {
						ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the function (dominance %s) to the knowledge base.", child_predicate.c_str());
						exit(-1);
					}
					ROS_INFO("KCL: (BehaviourAndEmotion) Added (dominance %s) to the knowledge base.", child_predicate.c_str());
					
					// Reciprocal.
					knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FUNCTION;
					knowledge_item.attribute_name = "reciprocal";
					knowledge_item.function_value = reciprocity;
					
					knowledge_update_service.request.knowledge = knowledge_item;
					if (!update_knowledge_client.call(knowledge_update_service)) {
						ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the function (reciprocal %s) to the knowledge base.", child_predicate.c_str());
						exit(-1);
					}
					ROS_INFO("KCL: (BehaviourAndEmotion) Added (reciprocal %s) to the knowledge base.", child_predicate.c_str());
				}
				// belongs_in predicates
				else if (line[0] == 'm')
				{
					if (tokens.size() != 3)
					{
						ROS_ERROR("KCL (BehaviourAndEmotion) Malformed line, expected m OBJECT_ID BOX_NAME. Read %s\n", line.c_str());
						exit(0);
					}
					
					knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
					
					std::string object_id = tokens[1];
					std::string box_name = tokens[2];
					
					rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
					knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
					knowledge_item.attribute_name = "in_box";
					knowledge_item.is_negative = false;
					
					diagnostic_msgs::KeyValue kv;
					kv.key = "b";
					kv.value = box_name;
					knowledge_item.values.push_back(kv);
					
					kv.key = "o";
					kv.value = object_id;
					knowledge_item.values.push_back(kv);
					
					knowledge_update_service.request.knowledge = knowledge_item;
					if (!update_knowledge_client.call(knowledge_update_service)) {
						ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the goal (in_box %s %s) to the knowledge base.", knowledge_item.values[0].value.c_str(),  object_id.c_str());
						exit(-1);
					}
					ROS_INFO("KCL: (BehaviourAndEmotion) Added the goal (in_box %s %s) to the knowledge base.", knowledge_item.values[0].value.c_str(),  object_id.c_str());
				}
			}
		}

		// Set kenny at it's starting waypoint.
		{
			rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
			knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
			knowledge_item.instance_type = "robot";
			knowledge_item.instance_name = "robot";
			
			knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
			
			knowledge_update_service.request.knowledge = knowledge_item;
			if (!update_knowledge_client.call(knowledge_update_service)) {
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
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the fact (robot_at robot kenny_waypoint) to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (BehaviourAndEmotion) Added the fact (robot_at robot kenny_waypoint) to the knowledge base.");
		}
		
		// Add a dummy type -- just necessary to keep it consistent with previous domains.
		{
			rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
			knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
			knowledge_item.instance_type = "type";
			knowledge_item.instance_name = "dummy";
			
			knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
			
			knowledge_update_service.request.knowledge = knowledge_item;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the type dummy to the knowledge base.");
				exit(-1);
			}
			ROS_INFO("KCL: (BehaviourAndEmotion) Added dummy to the knowledge base.");
		}
		
		// Robot is not busy initially.
		{
		rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
		knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		knowledge_item.attribute_name = "not_busy";
		knowledge_item.is_negative = false;
		
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the fact (not_busy) to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (BehaviourAndEmotion) Added the fact (not_busy) to the knowledge base.");
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
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the fact (gripper_empty robot) to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (BehaviourAndEmotion) Added the fact (gripper_empty robot) to the knowledge base.");
		}
		f.close();
		
		// Set the initial arrousal level.
		squirrel_vad_msgs::vad vad;
		vad.header.seq = 0;
		vad.header.stamp = ros::Time::now();
		vad.header.frame_id = "/map";
		vad.energy = 0.5;
		vad.duration = 10.0;
		message_store.insertNamed<squirrel_vad_msgs::vad>("vad", vad);
	}
	
	// Expect s to be "(f,f,f)"
	geometry_msgs::Pose BehaviourAndEmotion::transformToPose(const std::string& s)
	{
		std::cout << "Tranform to pose: " << s << std::endl;
		geometry_msgs::Pose p;
		int first_break = s.find(',');
		int second_break = s.find(',', first_break + 1);

		p.position.x = ::atof(s.substr(1, first_break - 1).c_str());
		p.position.y = ::atof(s.substr(first_break + 1, second_break - (first_break + 1)).c_str());
		p.position.z = ::atof(s.substr(second_break + 1, s.size() - (second_break + 2)).c_str());

		return p;
	}
	/*
	void BehaviourAndEmotion::setupSimulation()
	{
		// We will make some fictional objects and associated waypoints.
		rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		
		// Create some toys.
		std::vector<std::string> toys;
		toys.push_back("toy1");
		toys.push_back("toy2");
		//toys.push_back("toy3");
		
		for (std::vector<std::string>::const_iterator ci = toys.begin(); ci != toys.end(); ++ci)
		{
			const std::string& toy_predicate = *ci;
			rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
			knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
			knowledge_item.instance_type = "object";
			knowledge_item.instance_name = toy_predicate;
			
			knowledge_update_service.request.knowledge = knowledge_item;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the object %s to the knowledge base.", toy_predicate.c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (BehaviourAndEmotion) Added %s to the knowledge base.", toy_predicate.c_str());
			
			
			// Add waypoints for these boxes.
			knowledge_item.instance_type = "waypoint";
			std::stringstream ss;
			ss << toy_predicate << "_location";
			knowledge_item.instance_name = ss.str();
			
			knowledge_update_service.request.knowledge = knowledge_item;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the waypoint %s to the knowledge base.", ss.str().c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (BehaviourAndEmotion) Added %s to the knowledge base.", ss.str().c_str());
			
			// Link the boxes to these waypoints.
			knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			knowledge_item.attribute_name = "object_at";
			knowledge_item.is_negative = false;
			
			diagnostic_msgs::KeyValue kv;
			kv.key = "o";
			kv.value = toy_predicate;
			knowledge_item.values.push_back(kv);
			
			kv.key = "wp";
			kv.value = ss.str();
			knowledge_item.values.push_back(kv);
			
			knowledge_update_service.request.knowledge = knowledge_item;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the fact (object_at %s %s) to the knowledge base.", toy_predicate.c_str(), ss.str().c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (BehaviourAndEmotion) Added the fact (object_at %s %s) to the knowledge base.", toy_predicate.c_str(), ss.str().c_str());
			knowledge_item.values.clear();
			
			knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
			knowledge_item.instance_type = "waypoint";
			ss.str(std::string());
			ss << "near_" << toy_predicate;
			knowledge_item.instance_name = ss.str();
			
			knowledge_update_service.request.knowledge = knowledge_item;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the waypoint %s to the knowledge base.", ss.str().c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (BehaviourAndEmotion) Added %s to the knowledge base.", ss.str().c_str());
			
			// Make all but one object reachable.
			if (ci == toys.begin())
			{
				continue;
			}
			
			knowledge_item.values.clear();
			
			// Link the boxes to these waypoints.
			knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			knowledge_item.attribute_name = "reachable";
			knowledge_item.is_negative = false;
			
			kv.key = "o";
			kv.value = toy_predicate;
			knowledge_item.values.push_back(kv);
			
			knowledge_update_service.request.knowledge = knowledge_item;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the fact (reachable %s) to the knowledge base.", toy_predicate.c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (BehaviourAndEmotion) Added the fact (reachable %s) to the knowledge base.", toy_predicate.c_str());
			knowledge_item.values.clear();
		}
		
		// Add the boxes.
		std::vector<std::string> boxes;
		boxes.push_back("box1");
		boxes.push_back("box2");
		
		for (std::vector<std::string>::const_iterator ci = boxes.begin(); ci != boxes.end(); ++ci)
		{
			const std::string& box_predicate = *ci;
			rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
			
			knowledge_item.instance_type = "box";
			knowledge_item.instance_name = box_predicate;
			
			knowledge_update_service.request.knowledge = knowledge_item;knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the box %s to the knowledge base.", box_predicate.c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (BehaviourAndEmotion) Added %s to the knowledge base.", box_predicate.c_str());
			
			// Add waypoints for these boxes.
			knowledge_item.instance_type = "waypoint";
			std::stringstream ss;
			ss << box_predicate << "_location";
			knowledge_item.instance_name = ss.str();
			
			knowledge_update_service.request.knowledge = knowledge_item;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the waypoint %s to the knowledge base.", ss.str().c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (BehaviourAndEmotion) Added %s to the knowledge base.", ss.str().c_str());
			
			// Link the boxes to these waypoints.
			knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			knowledge_item.attribute_name = "box_at";
			knowledge_item.is_negative = false;
			
			diagnostic_msgs::KeyValue kv;
			kv.key = "b";
			kv.value = box_predicate;
			knowledge_item.values.push_back(kv);
			
			kv.key = "wp";
			kv.value = ss.str();
			knowledge_item.values.push_back(kv);
			
			knowledge_update_service.request.knowledge = knowledge_item;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the fact (box_at %s %s) to the knowledge base.", box_predicate.c_str(), ss.str().c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (BehaviourAndEmotion) Added the fact (box_at %s %s) to the knowledge base.", box_predicate.c_str(), ss.str().c_str());
			knowledge_item.values.clear();
			
			knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
			knowledge_item.instance_type = "waypoint";
			ss.str(std::string());
			ss << "near_" << box_predicate;
			knowledge_item.instance_name = ss.str();
			
			knowledge_update_service.request.knowledge = knowledge_item;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the waypoint %s to the knowledge base.", ss.str().c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (BehaviourAndEmotion) Added %s to the knowledge base.", ss.str().c_str());
		}
		
		// Add the waypoints.
		std::vector<std::string> waypoints;
		waypoints.push_back("kenny_waypoint");
		waypoints.push_back("pickup_waypoint");
		waypoints.push_back("child_waypoint");
		
		for (std::vector<std::string>::const_iterator ci = waypoints.begin(); ci != waypoints.end(); ++ci)
		{
			const std::string& waypoint_predicate = *ci;
			rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
			knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
			knowledge_item.instance_type = "waypoint";
			knowledge_item.instance_name = waypoint_predicate;
			
			knowledge_update_service.request.knowledge = knowledge_item;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the waypoint %s to the knowledge base.", waypoint_predicate.c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (BehaviourAndEmotion) Added %s to the knowledge base.", waypoint_predicate.c_str());
		}
		
		// Add a token type (not really used).
		{
			rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
			knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
			knowledge_item.instance_type = "type";
			knowledge_item.instance_name = "dummy_type";
			
			knowledge_update_service.request.knowledge = knowledge_item;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the type %s to the knowledge base.", knowledge_item.instance_name.c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (BehaviourAndEmotion) Added the type %s to the knowledge base.", knowledge_item.instance_name.c_str());
		}
		
		// Introduce kenny.
		rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
		knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
		knowledge_item.instance_type = "robot";
		knowledge_item.instance_name = "kenny";
		
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the robot %s to the knowledge base.", knowledge_item.instance_name.c_str());
			exit(-1);
		}
		ROS_INFO("KCL: (BehaviourAndEmotion) Added %s to the knowledge base.", knowledge_item.instance_name.c_str());
		
		// Set kenny at it's starting waypoint.
		knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		knowledge_item.attribute_name = "robot_at";
		knowledge_item.is_negative = false;
		
		diagnostic_msgs::KeyValue kv;
		kv.key = "v";
		kv.value = "kenny";
		knowledge_item.values.push_back(kv);
		
		kv.key = "wp";
		kv.value = "kenny_waypoint";
		knowledge_item.values.push_back(kv);
		
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the fact (robot_at kenny kenny_waypoint) to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (BehaviourAndEmotion) Added the fact (robot_at kenny kenny_waypoint) to the knowledge base.");
		knowledge_item.values.clear();
		
		// Set the robot to not being busy initially.
		knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		knowledge_item.attribute_name = "not_busy";
		knowledge_item.is_negative = false;
		
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the fact (not_busy) to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (BehaviourAndEmotion) Added the fact (not_busy) to the knowledge base.");
		knowledge_item.values.clear();
		
		// Make the gripper empty.
		knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		knowledge_item.attribute_name = "gripper_empty";
		knowledge_item.is_negative = false;
		
		kv.key = "v";
		kv.value = "kenny";
		knowledge_item.values.push_back(kv);
		
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the fact (gripper_empty kenny) to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (BehaviourAndEmotion) Added the fact (gripper_empty kenny) to the knowledge base.");
		knowledge_item.values.clear();
		
		**
		 * Setup the children.
		 *
		std::vector<std::string> children;
		children.push_back("c1");
		children.push_back("c2");
		children.push_back("c3");
		
		for (std::vector<std::string>::const_iterator ci = children.begin(); ci != children.end(); ++ci)
		{
			const std::string& child_predicate = *ci;
			rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
			knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
			knowledge_item.instance_type = "child";
			knowledge_item.instance_name = child_predicate;
			
			knowledge_update_service.request.knowledge = knowledge_item;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the child %s to the knowledge base.", child_predicate.c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (BehaviourAndEmotion) Added %s to the knowledge base.", child_predicate.c_str());
			
			// Setup the functions.
			diagnostic_msgs::KeyValue kv;
			kv.key = "c";
			kv.value = child_predicate;
			knowledge_item.values.push_back(kv);
			
			// Pleasure.
			knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FUNCTION;
			knowledge_item.attribute_name = "pleasure";
			knowledge_item.function_value = 0.1;
			
			knowledge_update_service.request.knowledge = knowledge_item;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the function (pleasure %s) to the knowledge base.", child_predicate.c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (BehaviourAndEmotion) Added (pleasure %s) to the knowledge base.", child_predicate.c_str());
			
			// Arousal.
			knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FUNCTION;
			knowledge_item.attribute_name = "arousal";
			knowledge_item.function_value = 0.1;
			
			knowledge_update_service.request.knowledge = knowledge_item;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the function (arousal %s) to the knowledge base.", child_predicate.c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (BehaviourAndEmotion) Added (arousal %s) to the knowledge base.", child_predicate.c_str());
			
			// Domain.
			knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FUNCTION;
			knowledge_item.attribute_name = "dominance";
			knowledge_item.function_value = 0.1;
			
			knowledge_update_service.request.knowledge = knowledge_item;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the function (dominance %s) to the knowledge base.", child_predicate.c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (BehaviourAndEmotion) Added (dominance %s) to the knowledge base.", child_predicate.c_str());
			
			// Reciprocal.
			knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FUNCTION;
			knowledge_item.attribute_name = "reciprocal";
			knowledge_item.function_value = 0.1;
			
			knowledge_update_service.request.knowledge = knowledge_item;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the function (reciprocal %s) to the knowledge base.", child_predicate.c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (BehaviourAndEmotion) Added (reciprocal %s) to the knowledge base.", child_predicate.c_str());
		}
		
		/**
		 * Setup the goals.
		 *
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
		for (std::vector<std::string>::const_iterator ci = toys.begin(); ci != toys.end(); ++ci)
		{
			const std::string& toy_predicate = *ci;
			rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
			knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			knowledge_item.attribute_name = "in_box";
			knowledge_item.is_negative = false;
			
			// Select a random box.
			kv.key = "b";
			kv.value = boxes[rand() % boxes.size()];
			knowledge_item.values.push_back(kv);
			
			kv.key = "o";
			kv.value = toy_predicate;
			knowledge_item.values.push_back(kv);
			
			knowledge_update_service.request.knowledge = knowledge_item;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (BehaviourAndEmotion) Could not add the goal (in_box %s %s) to the knowledge base.", knowledge_item.values[0].value.c_str(),  toy_predicate.c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (BehaviourAndEmotion) Added the goal (in_box %s %s) to the knowledge base.", knowledge_item.values[0].value.c_str(),  toy_predicate.c_str());
		}
	}
	*/
	/*--------------------*/
	/* problem generation */
	/*--------------------*/

	/**
	 * Generate a contingent problem.
	 */
	//bool BehaviourAndEmotion::generatePDDLProblemFile(rosplan_knowledge_msgs::GenerateProblemService::Request &req, rosplan_knowledge_msgs::GenerateProblemService::Response &res) {
	//	return true;
	//}
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/
	/*
	void initMongoDBData(mongodb_store::MessageStoreProxy& message_store)
	{
		// Create a Scene Object for the toy.
		squirrel_object_perception_msgs::SceneObject scene_object;
		scene_object.id = "toy1";
		scene_object.category = "unknown";
		
		geometry_msgs::PoseStamped object_pose;
		object_pose.header.seq = 0;
		object_pose.header.stamp = ros::Time::now();
		object_pose.header.frame_id = "/map";
		object_pose.pose.position.x = -0.08f;
		object_pose.pose.position.y = -3.88f;
		object_pose.pose.position.z = 0.0f;
		
		object_pose.pose.orientation.x = 0.0f;
		object_pose.pose.orientation.y = 0.0f;
		object_pose.pose.orientation.z = 0.0f;
		object_pose.pose.orientation.w = 1.0f;
		scene_object.pose = object_pose.pose;
		
		squirrel_object_perception_msgs::BCylinder c;
		c.diameter = 0.3f;
		c.height = 0.2f;
		scene_object.bounding_cylinder = c;
		std::string near_waypoint_mongodb_id(message_store.insertNamed("toy1", scene_object));
		
		// Create the location of both boxes.
		geometry_msgs::PoseStamped box1_pose;
		box1_pose.header.seq = 0;
		box1_pose.header.stamp = ros::Time::now();
		box1_pose.header.frame_id = "/map";
		box1_pose.pose.position.x = 4.09f;
		box1_pose.pose.position.y = -3.88f;
		box1_pose.pose.position.z = 0.0f;
		
		box1_pose.pose.orientation.x = 0.0f;
		box1_pose.pose.orientation.y = 0.0f;
		box1_pose.pose.orientation.z = 0.0f;
		box1_pose.pose.orientation.w = 1.0f;
		std::string near_waypoint_mongodb_id2(message_store.insertNamed("box1", box1_pose));
		
		geometry_msgs::PoseStamped box2_pose;
		box2_pose.header.seq = 0;
		box2_pose.header.stamp = ros::Time::now();
		box2_pose.header.frame_id = "/map";
		box2_pose.pose.position.x = 3.71f;
		box2_pose.pose.position.y = -7.55f;
		box2_pose.pose.position.z = 0.0f;
		
		box2_pose.pose.orientation.x = 0.0f;
		box2_pose.pose.orientation.y = 0.0f;
		box2_pose.pose.orientation.z = 0.0f;
		box2_pose.pose.orientation.w = 1.0f;
		std::string near_waypoint_mongodb_id3(message_store.insertNamed("box2", box2_pose));
		{
		geometry_msgs::PoseStamped pose;
		pose.header.seq = 0;
		pose.header.stamp = ros::Time::now();
		pose.header.frame_id = "/map";
		pose.pose.position.x = -0.243f;
		pose.pose.position.y = 4.43f;
		pose.pose.position.z = 0.0f;
		
		pose.pose.orientation.x = 0.0f;
		pose.pose.orientation.y = 0.0f;
		pose.pose.orientation.z = 0.0f;
		pose.pose.orientation.w = 1.0f;
		std::string near_waypoint_mongodb_id3(message_store.insertNamed("box1_location", box1_pose));
		}
		
		{
		geometry_msgs::PoseStamped pose;
		pose.header.seq = 0;
		pose.header.stamp = ros::Time::now();
		pose.header.frame_id = "/map";
		pose.pose.position.x = -0.243f;
		pose.pose.position.y = 4.03f;
		pose.pose.position.z = 0.0f;
		
		pose.pose.orientation.x = 0.0f;
		pose.pose.orientation.y = 0.0f;
		pose.pose.orientation.z = 0.0f;
		pose.pose.orientation.w = 1.0f;
		std::string near_waypoint_mongodb_id3(message_store.insertNamed("near_box1", box1_pose));
		}
		
		{
		geometry_msgs::PoseStamped pose;
		pose.header.seq = 0;
		pose.header.stamp = ros::Time::now();
		pose.header.frame_id = "/map";
		pose.pose.position.x = -1.71f;
		pose.pose.position.y = 1.5f;
		pose.pose.position.z = 0.0f;
		
		pose.pose.orientation.x = 0.0f;
		pose.pose.orientation.y = 0.0f;
		pose.pose.orientation.z = 0.0f;
		pose.pose.orientation.w = 1.0f;
		std::string near_waypoint_mongodb_id3(message_store.insertNamed("box2_location", box2_pose));
		}
		
		{
		geometry_msgs::PoseStamped pose;
		pose.header.seq = 0;
		pose.header.stamp = ros::Time::now();
		pose.header.frame_id = "/map";
		pose.pose.position.x = -1.71f;
		pose.pose.position.y = 1.3f;
		pose.pose.position.z = 0.0f;
		
		pose.pose.orientation.x = 0.0f;
		pose.pose.orientation.y = 0.0f;
		pose.pose.orientation.z = 0.0f;
		pose.pose.orientation.w = 1.0f;
		std::string near_waypoint_mongodb_id3(message_store.insertNamed("near_box2", box2_pose));
		}
		
		{
		geometry_msgs::PoseStamped pose;
		pose.header.seq = 0;
		pose.header.stamp = ros::Time::now();
		pose.header.frame_id = "/map";
		pose.pose.position.x = -1.71f;
		pose.pose.position.y = 1.5f;
		pose.pose.position.z = 0.0f;
		
		pose.pose.orientation.x = 0.0f;
		pose.pose.orientation.y = 0.0f;
		pose.pose.orientation.z = 0.0f;
		pose.pose.orientation.w = 1.0f;
		std::string near_waypoint_mongodb_id3(message_store.insertNamed("box2_location", box2_pose));
		}
		
		{
		geometry_msgs::PoseStamped pose;
		pose.header.seq = 0;
		pose.header.stamp = ros::Time::now();
		pose.header.frame_id = "/map";
		pose.pose.position.x = 2.41f;
		pose.pose.position.y = 0.292f;
		pose.pose.position.z = 0.0f;
		
		pose.pose.orientation.x = 0.0f;
		pose.pose.orientation.y = 0.0f;
		pose.pose.orientation.z = 0.0f;
		pose.pose.orientation.w = 1.0f;
		std::string near_waypoint_mongodb_id3(message_store.insertNamed("near_child1", pose));
		}
		
		{
		geometry_msgs::PoseStamped pose;
		pose.header.seq = 0;
		pose.header.stamp = ros::Time::now();
		pose.header.frame_id = "/map";
		pose.pose.position.x = 2.41f;
		pose.pose.position.y = 0.292f;
		pose.pose.position.z = 0.0f;
		
		pose.pose.orientation.x = 0.0f;
		pose.pose.orientation.y = 0.0f;
		pose.pose.orientation.z = 0.0f;
		pose.pose.orientation.w = 1.0f;
		std::string near_waypoint_mongodb_id3(message_store.insertNamed("child1_location", pose));
		}
		
		// Set the initial arrousal level.
		squirrel_vad_msgs::vad vad;
		vad.header.seq = 0;
		vad.header.stamp = ros::Time::now();
		vad.header.frame_id = "/map";
		vad.energy = 0.5;
		vad.duration = 10.0;
		message_store.insertNamed<squirrel_vad_msgs::vad>("vad", vad);
	}
*/
	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_interface_BehaviourAndEmotion");
		ros::NodeHandle nh;

		std::string config_file;
		nh.getParam("/scenario_setup_file", config_file);
	
		
		// create PDDL action subscriber
		KCL_rosplan::BehaviourAndEmotion rpsr(nh, "visualization_marker");
		rpsr.setupSimulation(config_file);
		// Setup the environment.
		///initMongoDBData(rpsr.getMessageStore());
		
		// Lets start the planning process.
		std::string data_path;
		nh.getParam("/data_path", data_path);
		ROS_INFO("KCL: (BehaviourAndEmotion) Data path: %s.", data_path.c_str());
		
		std::string planner_path;
		nh.getParam("/planner_path", planner_path);
		ROS_INFO("KCL: (BehaviourAndEmotion) Planner path: %s.", planner_path.c_str());
		
		std::stringstream ss;
		ss << data_path << "strategic-behaviour-domain.pddl";
		std::string domain_path = ss.str();
		
		ss.str(std::string());
		ss << data_path << "strategic-behaviour-problem.pddl";
		std::string problem_path = ss.str();
		
		std::string planner_command;
		nh.getParam("/planner_command", planner_command);
		ROS_INFO("KCL: (BehaviourAndEmotion) Planning command: %s.", planner_command.c_str());
		
		rosplan_dispatch_msgs::PlanGoal psrv;
		psrv.domain_path = domain_path;
		psrv.problem_path = problem_path;
		psrv.data_path = data_path;
		psrv.planner_command = planner_command;
		psrv.start_action_id = 0;

		ROS_INFO("KCL: (BehaviourAndEmotion) Start plan action");
		actionlib::SimpleActionClient<rosplan_dispatch_msgs::PlanAction> plan_action_client("/kcl_rosplan/start_planning", true);

		plan_action_client.waitForServer();
		ROS_INFO("KCL: (BehaviourAndEmotion) Start planning server found");
		
		// send goal
		plan_action_client.sendGoal(psrv);
		ROS_INFO("KCL: (BehaviourAndEmotion) Goal sent");
		
		ros::spin();
		return 0;
	}
