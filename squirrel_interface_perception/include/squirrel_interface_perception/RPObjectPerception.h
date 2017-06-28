#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include "squirrel_planning_knowledge_msgs/AddObjectService.h"
#include "squirrel_planning_knowledge_msgs/RemoveObjectService.h"
#include "squirrel_planning_knowledge_msgs/UpdateObjectService.h"
#include "mongodb_store/message_store.h"
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Quaternion.h>

#ifndef KCL_object_perception
#define KCL_object_perception

#include <map>

/**
 * This file defines the RPObjectPerception class.
 * RPObjectPerception is used to convert between real and
 * symbolic representations of toy objects for SQUIRREL.
 * Symbols are stored in the Knoweldge Base.
 * real data are stored in the SceneDB (implemented by mongoDB).
 */
namespace KCL_rosplan {

	class RPObjectPerception
	{

	private:
		
		std::string dataPath;

		// Scene database
		mongodb_store::MessageStoreProxy message_store;

		// Knowledge base
		ros::ServiceClient update_knowledge_client;

		// ROSPlan interface roadmap
		ros::ServiceClient add_waypoint_client;
		
		// Clients for object services.
		ros::ServiceServer add_object_service;
		ros::ServiceServer remove_object_service;
		ros::ServiceServer update_object_service;

		// Map ids passed by the caller to ids as stored in mongodb.
		std::multimap<std::string, std::string> mongo_id_mapping;


	public:

		/* constructor */
		RPObjectPerception(ros::NodeHandle &nh, std::string &dp);

		/* services */
		bool addObjects(squirrel_planning_knowledge_msgs::AddObjectService::Request &req, squirrel_planning_knowledge_msgs::AddObjectService::Response &res);
		bool removeObjects(squirrel_planning_knowledge_msgs::RemoveObjectService::Request &req, squirrel_planning_knowledge_msgs::RemoveObjectService::Response &res);
		bool updateObjects(squirrel_planning_knowledge_msgs::UpdateObjectService::Request &req, squirrel_planning_knowledge_msgs::UpdateObjectService::Response &res);
		bool lookAtObject(squirrel_planning_knowledge_msgs::UpdateObjectService::Request &req, squirrel_planning_knowledge_msgs::UpdateObjectService::Response &res);

	};
}
#endif
