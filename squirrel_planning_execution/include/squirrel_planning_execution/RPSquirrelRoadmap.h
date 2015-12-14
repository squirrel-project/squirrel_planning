#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "diagnostic_msgs/KeyValue.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#include "mongodb_store/message_store.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/GetInstanceService.h"
#include "squirrel_planning_knowledge_msgs/TaskPoseService.h"
#include "rosplan_knowledge_msgs/CreatePRM.h"
#include "rosplan_knowledge_msgs/AddWaypoint.h"
#include <tf/transform_datatypes.h>
#include <sstream>
#include <string>
#include <ctime>
#include <stdlib.h>

#ifndef KCL_squirrel_roadmap
#define KCL_squirrel_roadmap

namespace KCL_rosplan {

	struct Waypoint 
	{
		Waypoint(const std::string &id, double xCoord, double yCoord)
			: wpID(id), real_x(xCoord), real_y(yCoord) {}

		Waypoint()
			: wpID("wp_err"), real_x(0), real_y(0) {}
		
		float getDistance(const Waypoint& other) {
			return sqrt((real_x - other.real_x) * (real_x - other.real_x) + (real_y - other.real_y) * (real_y - other.real_y));
		}
		
		std::string wpID;
		double real_x;
		double real_y;
		std::vector<std::string> neighbours;
	};

	class RPSquirrelRoadmap
	{

	private:
		
		std::string data_path;
		std::string static_map_service;
		bool use_static_map;
		double occupancy_threshold;

		// Scene database
		mongodb_store::MessageStoreProxy message_store;

		// Knowledge base
		ros::ServiceClient update_knowledge_client;
		ros::ServiceClient get_instance_client;

		// map
		nav_msgs::OccupancyGrid cost_map;
		ros::ServiceClient map_client;

		// Roadmap
		std::map<std::string, Waypoint*> waypoints;
		std::map<std::string, std::string> db_name_map;
		
		// visualisation
		std::string fixed_frame;
		ros::Publisher waypoints_pub;
		void publishWaypointMarkerArray(ros::NodeHandle nh);
		void clearMarkerArrays(ros::NodeHandle nh);

		// waypoint request services
		ros::ServiceClient manipulation_client;

	public:

		/* constructor */
		RPSquirrelRoadmap(ros::NodeHandle &nh, std::string frame);

		/* service to (re)generate waypoints */
		bool generateRoadmap(rosplan_knowledge_msgs::CreatePRM::Request &req, rosplan_knowledge_msgs::CreatePRM::Response &res);
		void costMapCallback( const nav_msgs::OccupancyGridConstPtr& msg );
	};
}
#endif
