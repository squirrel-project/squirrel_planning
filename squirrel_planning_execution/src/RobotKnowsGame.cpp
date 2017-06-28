#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <tf/tf.h>

#include <actionlib/client/simple_action_client.h>

#include <geometry_msgs/PoseStamped.h>
#include <squirrel_object_perception_msgs/SceneObject.h>
#include <visualization_msgs/Marker.h>

#include <rosplan_dispatch_msgs/PlanAction.h>
#include <rosplan_dispatch_msgs/PlanGoal.h>
#include <rosplan_dispatch_msgs/ActionFeedback.h>
#include <rosplan_knowledge_msgs/GenerateProblemService.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>

#include "pddl_actions/ShedKnowledgePDDLAction.h"
#include "pddl_actions/FinaliseClassificationPDDLAction.h"
#include "pddl_actions/ExamineAreaPDDLAction.h"
#include "pddl_actions/ExploreAreaPDDLAction.h"
#include "pddl_actions/ObserveClassifiableOnAttemptPDDLAction.h"
#include "pddl_actions/TidyAreaPDDLAction.h"
#include "pddl_actions/PlannerInstance.h"
#include "pddl_actions/GotoViewWaypointPDDLAction.h"

ros::NodeHandle* nh;
ros::Publisher action_feedback_pub;


void tokenise(const std::string& s, std::vector<std::string>& tokens)
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
geometry_msgs::Pose transformToPose(const std::string& s)
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

void sendMarker(const geometry_msgs::Pose& pose, const std::string& name, ros::Publisher& vis_pub, float size)
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


void setupSimulation(const std::string& config_file, ros::ServiceClient& update_knowledge_client, mongodb_store::MessageStoreProxy& message_store, ros::Publisher& vis_pub)
{
	ROS_INFO("KCL: (RobotKnowsGame) Load scenarion from file: %s.\n", config_file.c_str());
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
					ROS_ERROR("KCL (RobotKnowsGame) Malformed line, expected b BOX_NAME (f,f,f) (f,f,f). Read %s\n", line.c_str());
					exit(0);
				}
				std::string box_predicate = tokens[1];
				geometry_msgs::Pose box_location = transformToPose(tokens[2]);
				geometry_msgs::Pose near_box = transformToPose(tokens[3]);
				sendMarker(box_location, box_predicate, vis_pub, 0.25f);
				{
					std::stringstream ss;
					ss << "near_" << box_predicate;
					sendMarker(near_box, ss.str(), vis_pub, 0.1f);
				}
				

				// Add the box predicate to the knowledge base.
				rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
				
				knowledge_item.instance_type = "box";
				knowledge_item.instance_name = box_predicate;
				
				knowledge_update_service.request.knowledge = knowledge_item;knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
				if (!update_knowledge_client.call(knowledge_update_service)) {
					ROS_ERROR("KCL: (RobotKnowsGame) Could not add the box %s to the knowledge base.", box_predicate.c_str());
					exit(-1);
				}
				ROS_INFO("KCL: (RobotKnowsGame) Added %s to the knowledge base.", box_predicate.c_str());

				// Add waypoints for these boxes.
				knowledge_item.instance_type = "waypoint";
				std::stringstream ss;
				ss << box_predicate << "_location";
				knowledge_item.instance_name = ss.str();
				
				knowledge_update_service.request.knowledge = knowledge_item;
				if (!update_knowledge_client.call(knowledge_update_service)) {
					ROS_ERROR("KCL: (RobotKnowsGame) Could not add the waypoint %s to the knowledge base.", ss.str().c_str());
					exit(-1);
				}
				ROS_INFO("KCL: (RobotKnowsGame) Added %s to the knowledge base.", ss.str().c_str());

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
				ROS_INFO("KCL: (RobotKnowsGame) Added %s to the knowledge base.", ss.str().c_str());
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
					ROS_ERROR("KCL: (RobotKnowsGame) Could not add the fact (box_at %s %s) to the knowledge base.", box_predicate.c_str(), ss.str().c_str());
					exit(-1);
				}
				ROS_INFO("KCL: (RobotKnowsGame) Added the fact (box_at %s %s) to the knowledge base.", box_predicate.c_str(), ss.str().c_str());
				knowledge_item.values.clear();
				
				knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
				knowledge_item.instance_type = "waypoint";
				ss.str(std::string());
				ss << "near_" << box_predicate;
				knowledge_item.instance_name = ss.str();
				
				knowledge_update_service.request.knowledge = knowledge_item;
				if (!update_knowledge_client.call(knowledge_update_service)) {
					ROS_ERROR("KCL: (RobotKnowsGame) Could not add the waypoint %s to the knowledge base.", ss.str().c_str());
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
				ROS_INFO("KCL: (RobotKnowsGame) Added %s to the knowledge base.", ss.str().c_str());
				}
			}
			else if (line[0] == 'w')
			{

				if (tokens.size() != 3)
				{
					ROS_ERROR("KCL (RobotKnowsGame) Malformed line, expected w WAYPOINT_NAME (f,f,f). Read %s\n", line.c_str());
					exit(0);
				}
				std::string waypoint_predicate = tokens[1];
				geometry_msgs::Pose waypoint_pose = transformToPose(tokens[2]);

				geometry_msgs::PoseStamped pose;
				pose.header.seq = 0;
				pose.header.stamp = ros::Time::now();
				pose.header.frame_id = "/map";
				pose.pose = waypoint_pose;
				
				pose.pose.orientation.x = 0.0f;
				pose.pose.orientation.y = 0.0f;
				pose.pose.orientation.z = 0.0f;
				pose.pose.orientation.w = 1.0f;
				std::string near_waypoint_mongodb_id3(message_store.insertNamed(waypoint_predicate, pose));

				rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
				knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
				knowledge_item.instance_type = "waypoint";
				knowledge_item.instance_name = waypoint_predicate;
				
				knowledge_update_service.request.knowledge = knowledge_item;
				if (!update_knowledge_client.call(knowledge_update_service)) {
					ROS_ERROR("KCL: (RobotKnowsGame) Could not add the waypoint %s to the knowledge base.", waypoint_predicate.c_str());
					exit(-1);
				}
				ROS_INFO("KCL: (RobotKnowsGame) Added %s to the knowledge base.", waypoint_predicate.c_str());
			}
			else if (line[0] == 't')
			{
				if (tokens.size() != 3)
				{
					ROS_ERROR("KCL (RobotKnowsGame) Malformed line, expected t TOY_NAME (f,f,f). Read %s\n", line.c_str());
					exit(0);
				}
				std::string toy_predicate = tokens[1];
				geometry_msgs::Pose toy_pose = transformToPose(tokens[2]);

				// Create a Scene Object for the toy.
				squirrel_object_perception_msgs::SceneObject scene_object;
				scene_object.id = toy_predicate;
				scene_object.category = "unknown";
				
				geometry_msgs::PoseStamped object_pose;
				object_pose.header.seq = 0;
				object_pose.header.stamp = ros::Time::now();
				object_pose.header.frame_id = "/map";
				object_pose.pose = toy_pose;
				
				object_pose.pose.orientation.x = 0.0f;
				object_pose.pose.orientation.y = 0.0f;
				object_pose.pose.orientation.z = 0.0f;
				object_pose.pose.orientation.w = 1.0f;
				scene_object.pose = object_pose.pose;
				
				std::string near_waypoint_mongodb_id(message_store.insertNamed(toy_predicate, scene_object));
				ROS_INFO("KCL: (RobotKnowsGame) Added %s to the knowledge base.", toy_predicate.c_str());
			}
			else if (line[0] == 'm')
			{
				if (tokens.size() != 3)
				{
					ROS_ERROR("KCL (RobotKnowsGame) Malformed line, expected m OBJECT_ID BOX_NAME. Read %s\n", line.c_str());
					exit(0);
				}
				std::string object_id = tokens[1];
				std::string box_name = tokens[2];

				rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
				knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
				knowledge_item.attribute_name = "belongs_in";
				knowledge_item.is_negative = false;
				
				diagnostic_msgs::KeyValue kv;
				kv.key = "o";
				kv.value = object_id;
				knowledge_item.values.push_back(kv);
				
				kv.key = "b";
				kv.value = box_name;
				knowledge_item.values.push_back(kv);
				
				knowledge_update_service.request.knowledge = knowledge_item;
				if (!update_knowledge_client.call(knowledge_update_service)) {
					ROS_ERROR("KCL: (RobotKnowsGame) Could not add the fact (belongs_in %s %s) to the knowledge base.", object_id.c_str(), box_name.c_str());
					exit(-1);
				}
				ROS_INFO("KCL: (RobotKnowsGame) Added the fact (belongs_in %s %s) to the knowledge base.", object_id.c_str(), box_name.c_str());
			}
		}
	}

	// Set kenny at it's starting waypoint.
	{
		rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
		knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
		knowledge_item.instance_type = "robot";
		knowledge_item.instance_name = "robot";
		
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (RobotKnowsGame) Could not add the robot robot to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (RobotKnowsGame) Added robot to the knowledge base.");
	}

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
	
	knowledge_update_service.request.knowledge = knowledge_item;
	if (!update_knowledge_client.call(knowledge_update_service)) {
		ROS_ERROR("KCL: (RobotKnowsGame) Could not add the fact (robot_at robot kenny_waypoint) to the knowledge base.");
		exit(-1);
	}
	ROS_INFO("KCL: (RobotKnowsGame) Added the fact (robot_at robot kenny_waypoint) to the knowledge base.");

	f.close();
}

void setupSimulation(ros::ServiceClient& update_knowledge_client)
{
	// We will make some fictional objects and associated waypoints.
	rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
	knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
	
	// Create some types for the toys.
	std::vector<std::string> toy_types;
	toy_types.push_back("dinosaur");
	toy_types.push_back("car");
	toy_types.push_back("unknown");
	
	for (std::vector<std::string>::const_iterator ci = toy_types.begin(); ci != toy_types.end(); ++ci)
	{
		const std::string& type_predicate = *ci;
		rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
		knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
		knowledge_item.instance_type = "type";
		knowledge_item.instance_name = type_predicate;
		
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (RobotKnowsGame) Could not add the type %s to the knowledge base.", type_predicate.c_str());
			exit(-1);
		}
		ROS_INFO("KCL: (RobotKnowsGame) Added %s to the knowledge base.", type_predicate.c_str());
	}

	// Map the id's known by the recogniser to the types we care about.
	std::map<std::string, std::string> recogniser_mapping;
	recogniser_mapping["green_dinosaur"] = "dinosaur";
	recogniser_mapping["yellow_dinosaur"] = "dinosaur";
	recogniser_mapping["girafe"] = "car";
	recogniser_mapping["dolphin"] = "car";

	for (std::map<std::string, std::string>::const_iterator ci = recogniser_mapping.begin(); ci != recogniser_mapping.end(); ++ci)
	{
		rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
		knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		knowledge_item.attribute_name = "is_of_type";
		knowledge_item.is_negative = false;
		
		diagnostic_msgs::KeyValue kv;
		kv.key = "o";
		kv.value = ci->first;
		knowledge_item.values.push_back(kv);
		
		kv.key = "t";
		kv.value = ci->second;
		knowledge_item.values.push_back(kv);
		
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (RobotKnowsGame) Could not add the fact (is_of_type %s %s) to the knowledge base.", ci->first.c_str(), ci->second.c_str());
			exit(-1);
		}
		ROS_INFO("KCL: (RobotKnowsGame) Added the fact (is_of_type %s %s) to the knowledge base.", ci->first.c_str(), ci->second.c_str());
	}
	
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
			ROS_ERROR("KCL: (RobotKnowsGame) Could not add the box %s to the knowledge base.", box_predicate.c_str());
			exit(-1);
		}
		ROS_INFO("KCL: (RobotKnowsGame) Added %s to the knowledge base.", box_predicate.c_str());
		
		// Add waypoints for these boxes.
		knowledge_item.instance_type = "waypoint";
		std::stringstream ss;
		ss << box_predicate << "_location";
		knowledge_item.instance_name = ss.str();
		
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (RobotKnowsGame) Could not add the waypoint %s to the knowledge base.", ss.str().c_str());
			exit(-1);
		}
		ROS_INFO("KCL: (RobotKnowsGame) Added %s to the knowledge base.", ss.str().c_str());
		
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
			ROS_ERROR("KCL: (RobotKnowsGame) Could not add the fact (box_at %s %s) to the knowledge base.", box_predicate.c_str(), ss.str().c_str());
			exit(-1);
		}
		ROS_INFO("KCL: (RobotKnowsGame) Added the fact (box_at %s %s) to the knowledge base.", box_predicate.c_str(), ss.str().c_str());
		knowledge_item.values.clear();
		
		knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
		knowledge_item.instance_type = "waypoint";
		ss.str(std::string());
		ss << "near_" << box_predicate;
		knowledge_item.instance_name = ss.str();
		
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (RobotKnowsGame) Could not add the waypoint %s to the knowledge base.", ss.str().c_str());
			exit(-1);
		}
		ROS_INFO("KCL: (RobotKnowsGame) Added %s to the knowledge base.", ss.str().c_str());
	}
	
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
			ROS_ERROR("KCL: (RobotKnowsGame) Could not add the waypoint %s to the knowledge base.", waypoint_predicate.c_str());
			exit(-1);
		}
		ROS_INFO("KCL: (RobotKnowsGame) Added %s to the knowledge base.", waypoint_predicate.c_str());
	}
	
	// Set kenny at it's starting waypoint.
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
	
	knowledge_update_service.request.knowledge = knowledge_item;
	if (!update_knowledge_client.call(knowledge_update_service)) {
		ROS_ERROR("KCL: (RobotKnowsGame) Could not add the fact (robot_at robot kenny_waypoint) to the knowledge base.");
		exit(-1);
	}
	ROS_INFO("KCL: (RobotKnowsGame) Added the fact (robot_at robot kenny_waypoint) to the knowledge base.");
	knowledge_item.values.clear();
}
/*-------------*/
/* Main method */
/*-------------*/

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
	object_pose.pose.position.x = -0.655f;
	object_pose.pose.position.y = 0.237f;
	object_pose.pose.position.z = 0.0f;
	
	object_pose.pose.orientation.x = 0.0f;
	object_pose.pose.orientation.y = 0.0f;
	object_pose.pose.orientation.z = 0.0f;
	object_pose.pose.orientation.w = 1.0f;
	scene_object.pose = object_pose.pose;
	
//	squirrel_object_perception_msgs::BCylinder c;
//	c.diameter = 0.3f;
//	c.height = 0.2f;
//	scene_object.bounding_cylinder = c;
	std::string near_waypoint_mongodb_id(message_store.insertNamed("toy1", scene_object));
	
	// Create the location of both boxes.
	geometry_msgs::PoseStamped box1_pose;
	box1_pose.header.seq = 0;
	box1_pose.header.stamp = ros::Time::now();
	box1_pose.header.frame_id = "/map";
	box1_pose.pose.position.x = -2.1f;
	box1_pose.pose.position.y = 0.738f;
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
	box2_pose.pose.position.x = 1.6f;
	box2_pose.pose.position.y = 1.4f;
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
	pose.pose.position.x = -2.1f;
	pose.pose.position.y = 0.738f;
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
	pose.pose.position.x = -1.85f;
	pose.pose.position.y = 1.02f;
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
	pose.pose.position.x = 1.6f;
	pose.pose.position.y = 1.4f;
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
	pose.pose.position.x = 1.74f;
	pose.pose.position.y = 1.62f;
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
	pose.pose.position.x = -0.41f;
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
	pose.pose.position.x = -0.46f;
	pose.pose.position.y = 0.234f;
	pose.pose.position.z = 0.0f;
	
	pose.pose.orientation.x = 0.0f;
	pose.pose.orientation.y = 0.0f;
	pose.pose.orientation.z = 0.0f;
	pose.pose.orientation.w = 1.0f;
	std::string near_waypoint_mongodb_id3(message_store.insertNamed("child_waypoint", pose));
	}

	{
	geometry_msgs::PoseStamped pose;
	pose.header.seq = 0;
	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = "/map";
	pose.pose.position.x = -0.655f;
	pose.pose.position.y = 0.237f;
	pose.pose.position.z = 0.0f;
	
	pose.pose.orientation.x = 0.0f;
	pose.pose.orientation.y = 0.0f;
	pose.pose.orientation.z = 0.0f;
	pose.pose.orientation.w = 1.0f;
	std::string near_waypoint_mongodb_id3(message_store.insertNamed("pickup_waypoint", pose));
	}
}

bool generatePDDLProblemFile(rosplan_knowledge_msgs::GenerateProblemService::Request &req, rosplan_knowledge_msgs::GenerateProblemService::Response &res) {
	return true;
}

void dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
{
	rosplan_dispatch_msgs::ActionDispatch normalised_action_dispatch = *msg;
	std::string action_name = msg->name;
	std::transform(action_name.begin(), action_name.end(), action_name.begin(), tolower);
	normalised_action_dispatch.name = action_name;
	
	// Ignore actions that do not correspond to g_action_name.
	if ("start_phase3" != action_name &&
	    "start_phase2" != action_name)
	{
		return;
	}

	bool actionAchieved = false;
	
	ROS_INFO("KCL: (ExamineAreaPDDLAction) action recieved %s", action_name.c_str());
	
	// publish feedback (enabled)
	rosplan_dispatch_msgs::ActionFeedback fb;
	fb.action_id = msg->action_id;
	fb.status = "action enabled";
	action_feedback_pub.publish(fb);
	
	KCL_rosplan::PlannerInstance& planner_instance = KCL_rosplan::PlannerInstance::createInstance(*nh);
	
	// Lets start the planning process.
	std::string data_path;
	nh->getParam("/data_path", data_path);
	
	std::string planner_path;
	nh->getParam("/planner_path", planner_path);
	
	std::stringstream ss;
	//ss << data_path << "tidy_room_domain-nt.pddl";
	ss << data_path << "template_robot_knows_domain.pddl";
	std::string domain_path = ss.str();
	
	ss.str(std::string());
	ss << data_path << "tidy_room_problem.pddl";
	std::string problem_path = ss.str();
	
	std::string planner_command;
	if ("start_phase3" == action_name)
	{
		nh->getParam("/squirrel_planning_execution/planner_command_phase3", planner_command);
	}
	else if ("start_phase2" == action_name)
	{
		nh->getParam("/squirrel_planning_execution/planner_command_phase2", planner_command);
	}
	
	planner_instance.startPlanner(domain_path, problem_path, data_path, planner_command);
	
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
		// publish feedback (achieved)
		rosplan_dispatch_msgs::ActionFeedback fb;
		fb.action_id = msg->action_id;
		fb.status = "action achieved";
		action_feedback_pub.publish(fb);
	}
	else
	{
		// publish feedback (failed)
		rosplan_dispatch_msgs::ActionFeedback fb;
		fb.action_id = msg->action_id;
		fb.status = "action failed";
		action_feedback_pub.publish(fb);
	}
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "rosplan_interface_RobotKnowsGame");
	nh = new ros::NodeHandle();

	// Visualisation.
	ros::Publisher vis_pub = nh->advertise<visualization_msgs::Marker>( "visualization_marker", 0 );


	// Overwrite the normal problem generator of ROSPlan.
	ros::ServiceServer pddl_generation_service = nh->advertiseService("/kcl_rosplan/generate_planning_problem", &generatePDDLProblemFile);
	
	// Initialise the knowledge base.
	ros::ServiceClient update_knowledge_client = nh->serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
	mongodb_store::MessageStoreProxy message_store(*nh);
	std::string config_file;
	nh->getParam("/scenario_setup_file", config_file);
	setupSimulation(config_file, update_knowledge_client, message_store, vis_pub);
	
	//setupSimulation(update_knowledge_client);
	//initMongoDBData(message_store);

	// Setup all the simulated actions.
	KCL_rosplan::ShedKnowledgePDDLAction shed_knowledge_action(*nh);
	KCL_rosplan::FinaliseClassificationPDDLAction finalise_classify_action(*nh);
	
	// Setup the recursive actions.
	KCL_rosplan::ExamineAreaPDDLAction examine_area_action(*nh);
	KCL_rosplan::ExploreAreaPDDLAction explore_area_action(*nh);
	KCL_rosplan::ObserveClassifiableOnAttemptPDDLAction observe_classifiable_on_attempt_action(*nh);
	KCL_rosplan::TidyAreaPDDLAction tidy_are_action(*nh);
	
	// Setup the actions exclusive to this domain.
	KCL_rosplan::GotoViewWaypointPDDLAction goto_view_waypoint_action(*nh, "/move_base");

	// We listen to the dispatcher, but we are only interested in the action 'start_phase3' which starts a 
	// new planning process where the robot follows the children.
	ros::Subscriber dispatch_sub_ = nh->subscribe("/kcl_rosplan/action_dispatch", 1000, &dispatchCallback);
	
	// Action feedback publisher.
	action_feedback_pub = nh->advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
	
	// Lets start the planning process.
	std::string data_path;
	nh->getParam("/data_path", data_path);
	
	std::string planner_path;
	nh->getParam("/planner_path", planner_path);
	
	std::stringstream ss;
	//ss << data_path << "tidy_room_domain-nt.pddl";
	ss << data_path << "template_robot_knows_domain.pddl";
	std::string domain_path = ss.str();
	
	ss.str(std::string());
	ss << data_path << "tidy_room_problem.pddl";
	std::string problem_path = ss.str();
	
	std::string planner_command;
	nh->getParam("/squirrel_planning_execution/planner_command", planner_command);
	
	rosplan_dispatch_msgs::PlanGoal psrv;
	psrv.domain_path = domain_path;
	psrv.problem_path = problem_path;
	psrv.data_path = data_path;
	psrv.planner_command = planner_command;
	psrv.start_action_id = 0;

	ROS_INFO("KCL: (RobotKnowsGame) Start plan action");
	actionlib::SimpleActionClient<rosplan_dispatch_msgs::PlanAction> plan_action_client("/kcl_rosplan/start_planning", true);

	plan_action_client.waitForServer();
	ROS_INFO("KCL: (RobotKnowsGame) Start planning server found");
	
	// send goal
	plan_action_client.sendGoal(psrv);
	ROS_INFO("KCL: (RobotKnowsGame) Goal sent");
	
	ros::spin();
	return 0;
}

