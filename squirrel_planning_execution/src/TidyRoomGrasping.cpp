#include "ros/ros.h"
#include "squirrel_planning_execution/TidyRoomGrasping.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <stdlib.h> 
#include <algorithm>
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h" 
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "mongodb_store/message_store.h"

#include <tf/transform_listener.h>
#include <tf/tf.h>
	
namespace KCL_rosplan {

	/* constructor */
	SimpleDemoExecutor::SimpleDemoExecutor(ros::NodeHandle &nh)
	 : message_store(nh), has_received_point_(false) {

		// access to the knowledge base.
		get_instance_client = nh.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_instances");
		get_attribute_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_instances_attributes");
		knowledge_update_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
		filter_publisher = nh.advertise<rosplan_knowledge_msgs::Filter>("/kcl_rosplan/mission_filter", 10, true);

		// PRM
		roadmap_service = nh.serviceClient<rosplan_knowledge_msgs::CreatePRM>("/kcl_rosplan/roadmap_server/request_waypoints");
		add_waypoint_client = nh.serviceClient<rosplan_knowledge_msgs::AddWaypoint>("/kcl_rosplan/roadmap_server/add_waypoint");

		// planner control.
		run_planner_client = nh.serviceClient<std_srvs::Empty>("/kcl_rosplan/planning_server");

		head_tilt_pub = nh.advertise<std_msgs::Float64>("/tilt_controller/command", 10, true);
		head_nod_pub = nh.advertise<std_msgs::String>("/expression", 10, true);
		head_down_angle = 0.6;
		head_up_angle = -0.3;
		count = 0;

	}
	
	void SimpleDemoExecutor::receivePointLocation(const geometry_msgs::PointStamped::ConstPtr& ptr) {
		count++;
		if(count>20) {
			received_point_ = *ptr;
			has_received_point_ = true;
		}
	}
	
	/* get point */
	void SimpleDemoExecutor::makePoint(std::string& wpID) {

		// tilt the head kinect up
		std_msgs::Float64 ht;
		ht.data = head_up_angle;
		head_tilt_pub.publish(ht);

		// Wait for a point to be published.
		ros::Rate r(10);
		has_received_point_ = false;
		while (!has_received_point_ && ros::ok()) {
			ros::spinOnce();
			r.sleep();
		}
		has_received_point_ = false;
		ROS_INFO("KCL: (SimpleDemo) Received point");

		// nod the head
		std_msgs::String exp;
		exp.data = "ok";
		head_nod_pub.publish(exp);
		ros::Rate nodRate(1);
		nodRate.sleep();

		// tilt the head kinect down
		ht.data = head_down_angle;
		head_tilt_pub.publish(ht);

		// convert point to pose
		geometry_msgs::PoseStamped pose_bl, pose;
		pose_bl.header.frame_id = "/kinect_depth_optical_frame";
		pose_bl.pose.position.x = received_point_.point.x;
		pose_bl.pose.position.y = received_point_.point.y;
		pose_bl.pose.position.z = received_point_.point.z;
		pose_bl.pose.orientation.x = 0;
		pose_bl.pose.orientation.y = 0;
		pose_bl.pose.orientation.z = 0;
		pose_bl.pose.orientation.w = 1;

		tf::TransformListener tfl;
		try {
			tfl.waitForTransform("/map","/kinect_depth_optical_frame", ros::Time::now(), ros::Duration(1.0));
			tfl.transformPose("/map", pose_bl, pose);
		} catch ( tf::TransformException& ex ) {
			ROS_ERROR("%s: error while transforming point %s", ros::this_node::getName().c_str(), ex.what());
			return;
		}

		// set goal pose
		goal_pose.header.frame_id = pose.header.frame_id;
		goal_pose.pose.position.x = pose.pose.position.x;
		goal_pose.pose.position.y = pose.pose.position.y;
		goal_pose.pose.position.z = pose.pose.position.z;
		goal_pose.pose.orientation.x = pose.pose.orientation.x;
		goal_pose.pose.orientation.y = pose.pose.orientation.y;
		goal_pose.pose.orientation.z = pose.pose.orientation.z;
		goal_pose.pose.orientation.w = pose.pose.orientation.w;

		// create new waypoint
		rosplan_knowledge_msgs::AddWaypoint addWPSrv;
		std::stringstream ss;
		ss << wpID;
		addWPSrv.request.id = ss.str();
		addWPSrv.request.waypoint = pose;
		addWPSrv.request.connecting_distance = 5;
		addWPSrv.request.occupancy_threshold = 20;
		if (!add_waypoint_client.call(addWPSrv)) {
			ROS_ERROR("KCL: (SimpleDemo) Failed to add a new waypoint for the object");
		}
		ROS_INFO("KCL: (SimpleDemo) Road map service returned");
	}

	void SimpleDemoExecutor::runDemo() {

		ros::NodeHandle nh;

		// clear the old filter
		rosplan_knowledge_msgs::Filter filterMessage;
		filterMessage.function = rosplan_knowledge_msgs::Filter::CLEAR;
		filter_publisher.publish(filterMessage);

		/* push the new filter
		filterMessage.function = rosplan_knowledge_msgs::Filter::ADD;
		rosplan_knowledge_msgs::KnowledgeItem object_filter;
		object_filter.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
		object_filter.instance_type = "object";
		filterMessage.knowledge_items.push_back(object_filter);
		rosplan_knowledge_msgs::KnowledgeItem waypoint_filter;
		waypoint_filter.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
		waypoint_filter.instance_type = "waypoint";
		filterMessage.knowledge_items.push_back(waypoint_filter);
		filter_publisher.publish(filterMessage);
		rosplan_knowledge_msgs::KnowledgeItem tidy_location;
		tidy_location.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		tidy_location.attribute_name = "tidy_location";
		filterMessage.knowledge_items.push_back(tidy_location);
		filter_publisher.publish(filterMessage);
		*/

		// run once
		ros::spinOnce();
		ROS_INFO("KCL: (SimpleDemo) Start the mission");
/*
		// Start by generating some waypoints.
		rosplan_knowledge_msgs::CreatePRM create_prm;
		create_prm.request.nr_waypoints = 1;
		create_prm.request.min_distance = 0.5;
		create_prm.request.casting_distance = 1.6;
		create_prm.request.connecting_distance = 5;
		create_prm.request.occupancy_threshold = 20;
		create_prm.request.total_attempts = 1000;
		if (!roadmap_service.call(create_prm)) {
			ROS_ERROR("KCL: (SimpleDemo) Failed to call the road map service.");
			return;
		}
		ROS_INFO("KCL: (SimpleDemo) Road map service returned.");
*/
		// get object point
		ROS_INFO("KCL: (SimpleDemo) Waiting for explore point.");
		std::string obWPID("explore_waypoint");
		makePoint(obWPID);

		// get goal point
		ROS_INFO("KCL: (SimpleDemo) Waiting for goal location point.");
		std::string goWPID("goal_waypoint");
		makePoint(goWPID);

		// Setup the explore goal.
		rosplan_knowledge_msgs::KnowledgeItem waypoint_goal;
		waypoint_goal.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		waypoint_goal.attribute_name = "explored";
		diagnostic_msgs::KeyValue kv;
		kv.key = "wp"; kv.value = obWPID;
		waypoint_goal.values.push_back(kv);

		rosplan_knowledge_msgs::KnowledgeUpdateService add_goal;
		add_goal.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
		add_goal.request.knowledge = waypoint_goal;
		if (!knowledge_update_client.call(add_goal)) {
			ROS_ERROR("KCL: (SimpleDemo) Could not add the goal to the knowledge base.");
			exit(-1);
		}

		// Run the planner.
		std_srvs::Empty dummy;
		if (!run_planner_client.call(dummy)) {
			ROS_ERROR("KCL: (SimpleDemo) Failed to run the planning system.");
			exit(-1);
		}
		ROS_INFO("KCL: (SimpleDemo) Planning system returned.");

		// get all objects
		rosplan_knowledge_msgs::GetInstanceService getInstances;
		getInstances.request.type_name = "object";		
		if (!get_instance_client.call(getInstances)) {
			ROS_ERROR("KCL: (SimpleDemo) Failed to get all the object instances.");
			return;
		}
		ROS_INFO("KCL: (SimpleDemo) Received all the object instances.");

		for (std::vector<std::string>::const_iterator ci = getInstances.response.instances.begin(); ci != getInstances.response.instances.end(); ++ci) {

			// add PREDICATE tidy_location
			rosplan_knowledge_msgs::KnowledgeUpdateService tlSrv;
			tlSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
			tlSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			tlSrv.request.knowledge.attribute_name = "tidy_location";
			diagnostic_msgs::KeyValue object;
			object.key = "o";
			object.value = (*ci);
			tlSrv.request.knowledge.values.push_back(object);
			diagnostic_msgs::KeyValue location;
			location.key = "wp";
			location.value = goWPID;
			tlSrv.request.knowledge.values.push_back(location);
			if (!knowledge_update_client.call(tlSrv))
				ROS_ERROR("KCL: (SimpleDemo) error adding knowledge");
		
			// Remove tidy_location_unknown for this object.
			rosplan_knowledge_msgs::KnowledgeUpdateService tidyLocationUnknownSrv;
			tidyLocationUnknownSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
			tidyLocationUnknownSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			tidyLocationUnknownSrv.request.knowledge.attribute_name = "tidy_location_unknown";
			object.key = "o";
			object.value = (*ci);
			tidyLocationUnknownSrv.request.knowledge.values.push_back(object);
			if (!knowledge_update_client.call(tidyLocationUnknownSrv)) 
				ROS_ERROR("KCL: (SimpleDemo) error removing tidy_location_unknown predicate");
/*
			// fetch position of object from message store
			std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > results;
			if(message_store.queryNamed<geometry_msgs::PoseStamped>(*ci, results)) {
				if(results.size()<1) {
					ROS_ERROR("KCL: (SimpleDemo) aborting pushing location; no matching obID %s", (*ci).c_str());
					return;
				}
				if(results.size()>1)
					ROS_ERROR("KCL: (SimpleDemo) multiple waypoints share the same wpID");
			} else {
				ROS_ERROR("KCL: (SimpleDemo) could not query message store to fetch object pose");
				return;
			}

			// calculate pushing pose for object and new point
			geometry_msgs::PoseStamped &objPose = *results[0];
			float d = sqrt(
				(objPose.pose.position.x - goal_pose.pose.position.x)*(objPose.pose.position.x - goal_pose.pose.position.x) +
				(objPose.pose.position.y - goal_pose.pose.position.y)*(objPose.pose.position.y - goal_pose.pose.position.y));
			geometry_msgs::PoseStamped pushingPose;
			pushingPose.header.frame_id = "/map";
			float startDistance = 0.30;
			pushingPose.pose.position.x = objPose.pose.position.x + startDistance*(objPose.pose.position.x - goal_pose.pose.position.x)/d;
			pushingPose.pose.position.y = objPose.pose.position.y + startDistance*(objPose.pose.position.y - goal_pose.pose.position.y)/d;

			float angle = atan2(goal_pose.pose.position.y - objPose.pose.position.y, goal_pose.pose.position.x - objPose.pose.position.x);
			if(isnan(angle)) angle = 0;

			pushingPose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);

			// add pushing location for this object
			rosplan_knowledge_msgs::AddWaypoint addWPSrv;
			std::stringstream ss_pp;
			ss_pp << "push_start_location_" << *ci;
			addWPSrv.request.id = ss_pp.str();
			addWPSrv.request.waypoint = pushingPose;
			addWPSrv.request.connecting_distance = 5;
			addWPSrv.request.occupancy_threshold = 20;
			if (!add_waypoint_client.call(addWPSrv)) {
				ROS_ERROR("KCL: (SimpleDemo) Failed to add a new waypoint for the object");
			}
			ROS_INFO("KCL: (SimpleDemo) Road map service returned");

			// add PREDICATE push_location
			rosplan_knowledge_msgs::KnowledgeUpdateService ppSrv;
			ppSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
			ppSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			ppSrv.request.knowledge.attribute_name = "push_location";
			ppSrv.request.knowledge.values.push_back(object);
			location.value = ss_pp.str();
			ppSrv.request.knowledge.values.push_back(location);
			if (!knowledge_update_client.call(ppSrv))
				ROS_ERROR("KCL: (SimpleDemo) error adding knowledge");
*/
			// generating some waypoints.
			rosplan_knowledge_msgs::CreatePRM create_prm;
			if (!roadmap_service.call(create_prm)) {
				ROS_ERROR("KCL: (SimpleDemo) Failed to call the road map service.");
				return;
			}
			ROS_INFO("KCL: (SimpleDemo) Road map service returned.");
		}

		// Run the planner agian.
		if (!run_planner_client.call(dummy)) {
			ROS_ERROR("KCL: (SimpleDemo) Failed to run the planning system.");
			exit(-1);
		}
		ROS_INFO("KCL: (SimpleDemo) Planning system returned.");
	}

} // close namespace

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

	ros::init(argc, argv, "rosplan_pointing_server");
	ros::NodeHandle nh;

	// create PDDL action subscriber
	KCL_rosplan::SimpleDemoExecutor sde(nh);

	// listen for pointing
	ros::Subscriber pointing_pose_sub = nh.subscribe("/squirrel_person_tracker/pointing_pose", 1, &KCL_rosplan::SimpleDemoExecutor::receivePointLocation, &sde);

	// listen for action dispatch
	ROS_INFO("KCL: (SimpleDemo) Ready to receive");

	sde.runDemo();
	return 0;
}



