#include "squirrel_planning_execution/ConfigReader.h"
#include <vector>
#include <fstream>
#include <iostream>
#include <map>

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
ConfigReader::ConfigReader(ros::NodeHandle &nh, mongodb_store::MessageStoreProxy& ms)
	: node_handle(&nh), knowledge_base_(nh, ms), message_store_(&ms)
{
	// knowledge interface
	update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
	query_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeQueryService>("/kcl_rosplan/query_knowledge_base");
	
	get_instance_client = nh.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_current_instances");
	get_attribute_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_knowledge");
	
	vis_pub = nh.advertise<visualization_msgs::Marker>("/config_viz", 1, false);
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

bool ConfigReader::readConfigurationFile(const std::string& config_file)
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
				if (!processBox(tokens, line)) success = false;
			}
			else if (line[0] == 't')
			{
				if (!processToy(tokens, line)) success = false;
			}
			else if (line[0] == 'w')
			{
				if (!processWaypoint(tokens, line)) success = false;
			}
			else if (line[0] == 'c')
			{
				if (!processChild(tokens, line)) success = false;
			}
			else if (line[0] == 'm')
			{
				if (!processObjectToBoxMapping(tokens, line)) success = false;
			}
			else if (line[0] == 'f')
			{
				if (!processFunction(tokens, line)) success = false;
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

bool ConfigReader::processBox(const std::vector<std::string>& tokens, const std::string& line)
{
	if (tokens.size() != 4)
	{
		ROS_ERROR("KCL (ConfigReader) Malformed line, expected b BOX_NAME (f,f,f) (f,f,f). Read %s\n", line.c_str());
		return false;
	}
	
	// Add the box.
	std::string box_predicate = tokens[1];
	geometry_msgs::Pose box_location = transformToPose(tokens[2]);
	geometry_msgs::Pose near_box = transformToPose(tokens[3]);
	sendMarker(box_location, box_predicate, 0.25f);
	{
		//std::stringstream ss;
		//ss << "near_" << box_predicate;
		sendMarker(near_box, "near_" + box_predicate, 0.1f);
	}
	
	knowledge_base_.addInstance("box", box_predicate);
	
	// Add waypoints for the boxes.
	//std::stringstream ss;
	//ss << box_predicate << "_location";
	knowledge_base_.addInstance("waypoint", box_predicate + "_location");
	
	// Set the actual location of the box waypoints in message store.
	geometry_msgs::PoseStamped pose;
	pose.header.seq = 0;
	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = "/map";
	pose.pose = box_location;
	
	pose.pose.orientation.x = 0.0f;
	pose.pose.orientation.y = 0.0f;
	pose.pose.orientation.z = 0.0f;
	pose.pose.orientation.w = 1.0f;
	std::string near_waypoint_mongodb_id3(message_store_->insertNamed(box_predicate + "_location", pose));
	ROS_INFO("KCL: (ConfigReader) Added %s to the knowledge base.", (box_predicate + "_location").c_str());
	
	std::map<std::string, std::string> parameters;
	parameters["b"] = box_predicate;
	parameters["wp"] = box_predicate + "_location";
	knowledge_base_.addFact("box_at", parameters, true, KnowledgeBase::KB_ADD_KNOWLEDGE);
	
	knowledge_base_.addInstance("waypoint", "near_" + box_predicate);

    parameters.clear();
    parameters["wp1"] = "near_" + box_predicate;
    parameters["wp2"] = box_predicate + "_location";
	knowledge_base_.addFact("near", parameters, true, KnowledgeBase::KB_ADD_KNOWLEDGE);
	
	pose.pose = near_box;
	
	float angle = atan2(box_location.position.y - near_box.position.y, box_location.position.x - near_box.position.x);
	pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
	std::string near_waypoint_mongodb_id4(message_store_->insertNamed("near_" + box_predicate, pose));
	
	ROS_INFO("KCL: (ConfigReader) Added %s to the knowledge base.", ("near_" + box_predicate).c_str());
	return true;
}

bool ConfigReader::processToy(const std::vector<std::string>& tokens, const std::string& line)
{
	if (tokens.size() != 5 && tokens.size() != 3)
	{
		ROS_ERROR("KCL (ConfigReader) Malformed line, expected t OBJECT_NAME OBJECT_TYPE (f,f,f) (f,f,f). Read %s\n", line.c_str());
		ROS_ERROR("KCL (ConfigReader) Malformed line, expected t OBJECT_NAME OBJECT_TYPE. Read %s\n", line.c_str());
		return false;
	}
	
	rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
	knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
	
	std::string toy_predicate = tokens[1];
	std::string type_predicate = tokens[2];

	if (!knowledge_base_.addInstance("object", toy_predicate)) return false;
	if (!knowledge_base_.addInstance("type", type_predicate)) return false;

	std::map<std::string, std::string> variables;
	variables["o"] = toy_predicate;
	variables["t"] = type_predicate;
	if (!knowledge_base_.addFact("is_of_type", variables, true, KnowledgeBase::KB_ADD_KNOWLEDGE)) return false;

    if (tokens.size() == 3) return true;

	geometry_msgs::Pose toy_location = transformToPose(tokens[3]);
	geometry_msgs::Pose near_toy = transformToPose(tokens[4]);
	sendMarker(toy_location, toy_predicate, 0.25f);
	sendMarker(near_toy, "near_" + toy_predicate, 0.1f);
	
	if (!knowledge_base_.addInstance("waypoint", toy_predicate + "_location")) return false;
	
    /*
	if (type_predicate == "battery")
	{
		std::map<std::string, std::string> variables;
		variables["o"] = toy_predicate;
		if (!knowledge_base_.addFact("battery_available", variables, true, KnowledgeBase::KB_ADD_KNOWLEDGE)) return false;
	}
    */
	
	
	// Set the actual location of the toy waypoints in message store.
	geometry_msgs::PoseStamped pose;
	pose.header.seq = 0;
	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = "/map";
	pose.pose = toy_location;
	
	pose.pose.orientation.x = 0.0f;
	pose.pose.orientation.y = 0.0f;
	pose.pose.orientation.z = 0.0f;
	pose.pose.orientation.w = 1.0f;
	std::string near_waypoint_mongodb_id3(message_store_->insertNamed(toy_predicate + "_location", pose));
	ROS_INFO("KCL: (ConfigReader) Added %s to the knowledge base.", (toy_predicate + "_location").c_str());
	
	variables.clear();
	variables["o"] = toy_predicate;
	variables["wp"] = toy_predicate + "_location";
	if (!knowledge_base_.addFact("object_at", variables, true, KnowledgeBase::KB_ADD_KNOWLEDGE)) return false;
	if (!knowledge_base_.addInstance("waypoint", "near_" + toy_predicate)) return false;
	
	pose.pose = near_toy;
	
	float angle = atan2(toy_location.position.y - near_toy.position.y, toy_location.position.x - near_toy.position.x);
	pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
	std::string near_waypoint_mongodb_id4(message_store_->insertNamed("near_" + toy_predicate, pose));
	ROS_INFO("KCL: (ConfigReader) Added %s to the knowledge base.", ("near_" + toy_predicate).c_str());
	return true;
}

bool ConfigReader::processWaypoint(const std::vector<std::string>& tokens, const std::string& line)
{
	if (tokens.size() != 3)
	{
		ROS_ERROR("KCL (ConfigReader) Malformed line, expected w WAYPOINT_NAME (f,f,f). Read %s\n", line.c_str());
		return false;
	}
	
	std::string wp_predicate = tokens[1];
	geometry_msgs::Pose wp_location = transformToPose(tokens[2]);
	sendMarker(wp_location, wp_predicate, 0.25f);
	
	if (!knowledge_base_.addInstance("waypoint", wp_predicate)) return false;
	
	// Set the actual location of the toy waypoints in message store.
	geometry_msgs::PoseStamped pose;
	pose.header.seq = 0;
	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = "/map";
	pose.pose = wp_location;
	
	pose.pose.orientation.x = 0.0f;
	pose.pose.orientation.y = 0.0f;
	pose.pose.orientation.z = 0.0f;
	pose.pose.orientation.w = 1.0f;
	std::string near_waypoint_mongodb_id3(message_store_->insertNamed(wp_predicate, pose));
	ROS_INFO("KCL: (ConfigReader) Added %s to the knowledge base.", wp_predicate.c_str());
	return true;
}

bool ConfigReader::processChild(const std::vector<std::string>& tokens, const std::string& line)
{
	if (tokens.size() != 6)
	{
		ROS_ERROR("KCL (ConfigReader) Malformed line, expected c CHILD_NAME P A D R. Read %s\n", line.c_str());
		return false;
	}
	
	std::string child_predicate = tokens[1];
	float pleasure = ::atof(tokens[2].c_str());
	float arousal = ::atof(tokens[3].c_str());
	float dominance = ::atof(tokens[4].c_str());
	float reciprocity = ::atof(tokens[5].c_str());
	
	if (!knowledge_base_.addInstance("child", child_predicate)) return false;
	
	std::map<std::string, std::string> variables;
	variables["c"] = child_predicate;
	if (!knowledge_base_.addFunction("pleasure", variables, pleasure, KnowledgeBase::KB_ADD_KNOWLEDGE) ||
	    !knowledge_base_.addFunction("arousal", variables, arousal, KnowledgeBase::KB_ADD_KNOWLEDGE) ||
	    !knowledge_base_.addFunction("dominance", variables, dominance, KnowledgeBase::KB_ADD_KNOWLEDGE) ||
	    !knowledge_base_.addFunction("reciprocal", variables, reciprocity, KnowledgeBase::KB_ADD_KNOWLEDGE))
	{
		return false;
	}
	return true;
}

bool ConfigReader::processObjectToBoxMapping(const std::vector<std::string>& tokens, const std::string& line)
{
	if (tokens.size() != 3)
	{
		ROS_ERROR("KCL (ConfigReader) Malformed line, expected m OBJECT_ID BOX_NAME. Read %s\n", line.c_str());
		return false;
	}
	
	std::string object_id = tokens[1];
	std::string box_name = tokens[2];
	
	std::map<std::string, std::string> parameters;
	parameters["b"] = box_name;
	parameters["o"] = object_id;
	return knowledge_base_.addFact("belongs_in", parameters, true, KnowledgeBase::KB_ADD_KNOWLEDGE);
	
	/*
	rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
	knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
	
	std::string object_id = tokens[1];
	std::string box_name = tokens[2];
	
	std::map<std::string, std::string> parameters;
	parameters["b"] = box_name;
	parameters["o"] = object_id;
	return knowledge_base_.addFact("in_box", parameters, true, KnowledgeBase::KB_ADD_GOAL);
	*/
}

bool ConfigReader::processFunction(const std::vector<std::string>& tokens, const std::string& line)
{
	if (tokens.size() != 3)
	{
		ROS_ERROR("KCL (ConfigReader) Malformed line, expected f FUNCTION VALUE. Read %s\n", line.c_str());
		return false;
	}
	
	std::map<std::string, std::string> variables;
	variables["r"] = "robot";
	return knowledge_base_.addFunction(tokens[1], variables, ::atof(tokens[2].c_str()), KnowledgeBase::KB_ADD_KNOWLEDGE);
}

}
