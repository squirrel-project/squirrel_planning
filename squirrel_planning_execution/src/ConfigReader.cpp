#include "squirrel_planning_execution/ConfigReader.h"
#include <vector>
#include <fstream>
#include <iostream>

#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/GetInstanceService.h"
#include "rosplan_knowledge_msgs/GetAttributeService.h"
#include "rosplan_knowledge_msgs/GenerateProblemService.h"
#include "rosplan_knowledge_msgs/KnowledgeQueryService.h"

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

namespace KCL_rosplan
{
ConfigReader::ConfigReader(ros::NodeHandle &nh)
{
	// knowledge interface
	update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
	query_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeQueryService>("/kcl_rosplan/query_knowledge_base");
	
	get_instance_client = nh.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_current_instances");
	get_attribute_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_knowledge");
}

void ConfigReader::tokenise(const std::string& s, std::vector<std::string>& tokens)
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


// Expect s to be "(f,f,f)"
geometry_msgs::Pose ConfigReader::transformToPose(const std::string& s)
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

void ConfigReader::sendMarker(const geometry_msgs::Pose& pose, const std::string& name, float size)
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

bool ConfigReader::readConfigurationFile(const std::string& config_file, mongodb_store::MessageStoreProxy& ms)
{
	ROS_INFO("KCL: (ConfigReader) Load scenarion from file: %s.\n", config_file.c_str());
	std::ifstream f(config_file.c_str());
	std::string line;
	bool success = true;

	rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;

	if (f.is_open())
	{
		while (getline(f, line))
		{
			std::cout << line << std::endl;
			if (line.size() == 0) continue;

			std::vector<std::string> tokens;
			tokenise(line, tokens);

			if (line[0] == 'b')
			{
				if (!processBox(tokens, ms, line)) success = false;
			}
			else if (line[0] == 't')
			{
				if (!processToy(tokens, ms, line)) success = false;
			}
			else if (line[0] == 'w')
			{
				if (!processWaypoint(tokens, ms, line)) success = false;
			}
			else if (line[0] == 'c')
			{
				if (!processChild(tokens, ms, line)) success = false;
			}
			else if (line[0] == 'm')
			{
				if (!processObjectToBoxMapping(tokens, ms, line)) success = false;
			}
			else if (line[0] != '#')
			{
				ROS_ERROR("KCL (ConfigReader) Unknown initial token: %s\n", tokens[0].c_str());
				success = false;
			}
		}
	}
	f.close();
	return success;
}

bool ConfigReader::processBox(const std::vector<std::string>& tokens, mongodb_store::MessageStoreProxy& ms, const std::string& line)
{
	if (tokens.size() != 4)
	{
		ROS_ERROR("KCL (ConfigReader) Malformed line, expected b BOX_NAME (f,f,f) (f,f,f). Read %s\n", line.c_str());
		return false;
	}
	
	rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
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
		ROS_ERROR("KCL: (ConfigReader) Could not add the box %s to the knowledge base.", box_predicate.c_str());
		return false;
	}
	ROS_INFO("KCL: (ConfigReader) Added %s to the knowledge base.", box_predicate.c_str());

	// Add waypoints for these boxes.
	knowledge_item.instance_type = "waypoint";
	std::stringstream ss;
	ss << box_predicate << "_location";
	knowledge_item.instance_name = ss.str();
	
	knowledge_update_service.request.knowledge = knowledge_item;
	if (!update_knowledge_client.call(knowledge_update_service)) {
		ROS_ERROR("KCL: (ConfigReader) Could not add the waypoint %s to the knowledge base.", ss.str().c_str());
		return false;
	}
	ROS_INFO("KCL: (ConfigReader) Added %s to the knowledge base.", ss.str().c_str());

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
	std::string near_waypoint_mongodb_id3(ms.insertNamed(ss.str(), pose));
	ROS_INFO("KCL: (ConfigReader) Added %s to the knowledge base.", ss.str().c_str());
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
		ROS_ERROR("KCL: (ConfigReader) Could not add the fact (box_at %s %s) to the knowledge base.", box_predicate.c_str(), ss.str().c_str());
		return false;
	}
	ROS_INFO("KCL: (ConfigReader) Added the fact (box_at %s %s) to the knowledge base.", box_predicate.c_str(), ss.str().c_str());
	knowledge_item.values.clear();
	
	knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
	knowledge_item.instance_type = "waypoint";
	ss.str(std::string());
	ss << "near_" << box_predicate;
	knowledge_item.instance_name = ss.str();
	
	knowledge_update_service.request.knowledge = knowledge_item;
	if (!update_knowledge_client.call(knowledge_update_service)) {
		ROS_ERROR("KCL: (ConfigReader) Could not add the waypoint %s to the knowledge base.", ss.str().c_str());
		return false;
	}
	{
	geometry_msgs::PoseStamped pose;
	pose.header.seq = 0;
	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = "/map";
	pose.pose = near_box;
	
	float angle = atan2(box_location.position.y - near_box.position.y, box_location.position.x - near_box.position.x);
	pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
	std::string near_waypoint_mongodb_id3(ms.insertNamed(ss.str(), pose));
	ROS_INFO("KCL: (ConfigReader) Added %s to the knowledge base.", ss.str().c_str());
	}
	return true;
}

bool ConfigReader::processToy(const std::vector<std::string>& tokens, mongodb_store::MessageStoreProxy& ms, const std::string& line)
{
	if (tokens.size() != 5)
	{
		ROS_ERROR("KCL (ConfigReader) Malformed line, expected t OBJECT_NAME OBJECT_TYPE (f,f,f) (f,f,f). Read %s\n", line.c_str());
		return false;
	}
	
	rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
	knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
	
	std::string toy_predicate = tokens[1];
	std::string type_predicate = tokens[2];
	geometry_msgs::Pose toy_location = transformToPose(tokens[3]);
	geometry_msgs::Pose near_toy = transformToPose(tokens[4]);
	sendMarker(toy_location, toy_predicate, 0.25f);
	{
		std::stringstream ss;
		ss << "near_" << toy_predicate;
		sendMarker(near_toy, ss.str(), 0.1f);
	}
	
	// Add the box predicate to the knowledge base.
	rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
	knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
	
	knowledge_item.instance_type = "object";
	knowledge_item.instance_name = toy_predicate;
	
	knowledge_update_service.request.knowledge = knowledge_item;
	if (!update_knowledge_client.call(knowledge_update_service)) {
		ROS_ERROR("KCL: (ConfigReader) Could not add the object %s to the knowledge base.", toy_predicate.c_str());
		return false;
	}
	ROS_INFO("KCL: (ConfigReader) Added %s to the knowledge base.", toy_predicate.c_str());
	
	knowledge_item.instance_type = "type";
	knowledge_item.instance_name = type_predicate;
	
	knowledge_update_service.request.knowledge = knowledge_item;
	if (!update_knowledge_client.call(knowledge_update_service)) {
		ROS_ERROR("KCL: (ConfigReader) Could not add the type %s to the knowledge base.", type_predicate.c_str());
		return false;
	}
	ROS_INFO("KCL: (ConfigReader) Added %s to the knowledge base.", type_predicate.c_str());

	// Add waypoints for these boxes.
	knowledge_item.instance_type = "waypoint";
	std::stringstream ss;
	ss << toy_predicate << "_location";
	knowledge_item.instance_name = ss.str();
	
	knowledge_update_service.request.knowledge = knowledge_item;
	if (!update_knowledge_client.call(knowledge_update_service)) {
		ROS_ERROR("KCL: (ConfigReader) Could not add the waypoint %s to the knowledge base.", ss.str().c_str());
		return false;
	}
	ROS_INFO("KCL: (ConfigReader) Added %s to the knowledge base.", ss.str().c_str());

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
	std::string near_waypoint_mongodb_id3(ms.insertNamed(ss.str(), pose));
	ROS_INFO("KCL: (ConfigReader) Added %s to the knowledge base.", ss.str().c_str());
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
		ROS_ERROR("KCL: (ConfigReader) Could not add the fact (object_at %s %s) to the knowledge base.", toy_predicate.c_str(), ss.str().c_str());
		return false;
	}
	ROS_INFO("KCL: (ConfigReader) Added the fact (object_at %s %s) to the knowledge base.", toy_predicate.c_str(), ss.str().c_str());
	knowledge_item.values.clear();
	
	knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
	knowledge_item.instance_type = "waypoint";
	ss.str(std::string());
	ss << "near_" << toy_predicate;
	knowledge_item.instance_name = ss.str();
	
	knowledge_update_service.request.knowledge = knowledge_item;
	if (!update_knowledge_client.call(knowledge_update_service)) {
		ROS_ERROR("KCL: (ConfigReader) Could not add the waypoint %s to the knowledge base.", ss.str().c_str());
		return false;
	}
	{
	geometry_msgs::PoseStamped pose;
	pose.header.seq = 0;
	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = "/map";
	pose.pose = near_toy;
	
	float angle = atan2(toy_location.position.y - near_toy.position.y, toy_location.position.x - near_toy.position.x);
	pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
	std::string near_waypoint_mongodb_id3(ms.insertNamed(ss.str(), pose));
	ROS_INFO("KCL: (ConfigReader) Added %s to the knowledge base.", ss.str().c_str());
	}
	return true;
}

bool ConfigReader::processWaypoint(const std::vector<std::string>& tokens, mongodb_store::MessageStoreProxy& ms, const std::string& line)
{
	if (tokens.size() != 3)
		{
			ROS_ERROR("KCL (ConfigReader) Malformed line, expected w WAYPOINT_NAME (f,f,f). Read %s\n", line.c_str());
			return false;
		}
		
		rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
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
			ROS_ERROR("KCL: (ConfigReader) Could not add the waypoint %s to the knowledge base.", wp_predicate.c_str());
			return false;
		}
		ROS_INFO("KCL: (ConfigReader) Added %s to the knowledge base.", wp_predicate.c_str());

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
		std::string near_waypoint_mongodb_id3(ms.insertNamed(wp_predicate, pose));
		ROS_INFO("KCL: (ConfigReader) Added %s to the knowledge base.", wp_predicate.c_str());
		}
		return true;
}

bool ConfigReader::processChild(const std::vector<std::string>& tokens, mongodb_store::MessageStoreProxy& ms, const std::string& line)
{
	if (tokens.size() != 6)
	{
		ROS_ERROR("KCL (ConfigReader) Malformed line, expected c CHILD_NAME P A D R. Read %s\n", line.c_str());
		return false;
	}
	
	rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
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
		ROS_ERROR("KCL: (ConfigReader) Could not add the child %s to the knowledge base.", child_predicate.c_str());
		return false;
	}
	ROS_INFO("KCL: (ConfigReader) Added %s to the knowledge base.", child_predicate.c_str());
	
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
		ROS_ERROR("KCL: (ConfigReader) Could not add the function (pleasure %s) to the knowledge base.", child_predicate.c_str());
		return false;
	}
	ROS_INFO("KCL: (ConfigReader) Added (pleasure %s) to the knowledge base.", child_predicate.c_str());
	
	// Arousal.
	knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FUNCTION;
	knowledge_item.attribute_name = "arousal";
	knowledge_item.function_value = arousal;
	
	knowledge_update_service.request.knowledge = knowledge_item;
	if (!update_knowledge_client.call(knowledge_update_service)) {
		ROS_ERROR("KCL: (ConfigReader) Could not add the function (arousal %s) to the knowledge base.", child_predicate.c_str());
		return false;
	}
	ROS_INFO("KCL: (ConfigReader) Added (arousal %s) to the knowledge base.", child_predicate.c_str());
	
	// Domain.
	knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FUNCTION;
	knowledge_item.attribute_name = "dominance";
	knowledge_item.function_value = dominance;
	
	knowledge_update_service.request.knowledge = knowledge_item;
	if (!update_knowledge_client.call(knowledge_update_service)) {
		ROS_ERROR("KCL: (ConfigReader) Could not add the function (dominance %s) to the knowledge base.", child_predicate.c_str());
		return false;
	}
	ROS_INFO("KCL: (ConfigReader) Added (dominance %s) to the knowledge base.", child_predicate.c_str());
	
	// Reciprocal.
	knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FUNCTION;
	knowledge_item.attribute_name = "reciprocal";
	knowledge_item.function_value = reciprocity;
	
	knowledge_update_service.request.knowledge = knowledge_item;
	if (!update_knowledge_client.call(knowledge_update_service)) {
		ROS_ERROR("KCL: (ConfigReader) Could not add the function (reciprocal %s) to the knowledge base.", child_predicate.c_str());
		return false;
	}
	ROS_INFO("KCL: (ConfigReader) Added (reciprocal %s) to the knowledge base.", child_predicate.c_str());
	return true;
}

bool ConfigReader::processObjectToBoxMapping(const std::vector<std::string>& tokens, mongodb_store::MessageStoreProxy& ms, const std::string& line)
{
	if (tokens.size() != 3)
	{
		ROS_ERROR("KCL (ConfigReader) Malformed line, expected m OBJECT_ID BOX_NAME. Read %s\n", line.c_str());
		return false;
	}
	
	rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
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
		ROS_ERROR("KCL: (ConfigReader) Could not add the goal (in_box %s %s) to the knowledge base.", knowledge_item.values[0].value.c_str(),  object_id.c_str());
		return false;
	}
	ROS_INFO("KCL: (ConfigReader) Added the goal (in_box %s %s) to the knowledge base.", knowledge_item.values[0].value.c_str(),  object_id.c_str());
	return true;
}

}
