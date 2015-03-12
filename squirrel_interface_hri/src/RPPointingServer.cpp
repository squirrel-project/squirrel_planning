#include "ros/ros.h"
#include "squirrel_hri_knowledge/RPPointingServer.h"
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

namespace KCL_rosplan {

	/* constructor */
	RPPointingServer::RPPointingServer(ros::NodeHandle &nh)
	 : message_store(nh), has_received_point_(false) {
		knowledgeInterface = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
		action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
		head_tilt_pub = nh.advertise<std_msgs::Float64>("/tilt_controller/command", 10, true);
		head_nod_pub = nh.advertise<std_msgs::String>("/expression", 10, true);
		head_down_angle = 0.6;
		head_up_angle = -0.3;
	}
	
	void RPPointingServer::receivePointLocation(const geometry_msgs::PointStamped::ConstPtr& ptr) {
		received_point_ = *ptr;
		has_received_point_ = true;
	}
	
	/* action dispatch callback; parameters (?ob - object) */
	void RPPointingServer::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		// ignore non-goto-waypoint actions
		if(0!=msg->name.compare("request_tidy")) return;

		ROS_INFO("KCL: (PointingServer) action recieved");

		// publish feedback (enabled)
		rosplan_dispatch_msgs::ActionFeedback fb;
		fb.action_id = msg->action_id;
		fb.status = "action enabled";
		action_feedback_pub.publish(fb);
		
		// get waypoint ID from action dispatch
		std::string obID;
		bool found = false;
		for(size_t i=0; i<msg->parameters.size(); i++) {
			if(0==msg->parameters[i].key.compare("ob")) {
				obID = msg->parameters[i].value;
				found = true;
			}
		}
		if(!found) {
			ROS_INFO("KCL: (PointingServer) aborting action dispatch; malformed parameters");
			return;
		}

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
		ROS_INFO("KCL: (PointingServer) Received point");

		// nod the head
		std_msgs::String exp;
		exp.data = "ok";
		head_nod_pub.publish(exp);
		ros::Rate nodRate(1);
		nodRate.sleep();

		// tilt the head kinect down
		ht.data = head_down_angle;
		head_tilt_pub.publish(ht);

		// Store the found point in the database.
		std::stringstream ss;
		ss << "point_location_" << obID;
		std::string id(message_store.insertNamed(ss.str(), received_point_));
		
		// Store it in the knowledge base.
		rosplan_knowledge_msgs::KnowledgeUpdateService wpSrv;
		wpSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		wpSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
		wpSrv.request.knowledge.instance_type = "waypoint";
		wpSrv.request.knowledge.instance_name = ss.str();
		if (!knowledgeInterface.call(wpSrv))
			ROS_ERROR("KCL: (PointingServer) error adding knowledge");
		
		rosplan_knowledge_msgs::KnowledgeUpdateService tlSrv;
		tlSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		tlSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::DOMAIN_ATTRIBUTE;
		tlSrv.request.knowledge.attribute_name = "tidy_location";
		diagnostic_msgs::KeyValue object;
		object.key = "o";
		object.value = obID;
		tlSrv.request.knowledge.values.push_back(object);
		diagnostic_msgs::KeyValue location;
		location.key = "wp";
		location.value = ss.str();
		tlSrv.request.knowledge.values.push_back(location);
		if (!knowledgeInterface.call(tlSrv))
			ROS_ERROR("KCL: (PointingServer) error adding knowledge");
		
		// Remove tidy_location_unknown for this object.
		rosplan_knowledge_msgs::KnowledgeUpdateService tidyLocationUnknownSrv;
		tidyLocationUnknownSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
		tidyLocationUnknownSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::DOMAIN_ATTRIBUTE;
		tidyLocationUnknownSrv.request.knowledge.attribute_name = "tidy_location_unknown";
		object.key = "o";
		object.value = obID;
		tidyLocationUnknownSrv.request.knowledge.values.push_back(object);
		if (!knowledgeInterface.call(tidyLocationUnknownSrv)) 
			ROS_ERROR("KCL: (PointingServer) error removing tidy_location_unknown predicate");
		
		// publish feedback (achieved)
		fb.action_id = msg->action_id;
		fb.status = "action achieved";
		action_feedback_pub.publish(fb);
	}

} // close namespace

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

	ros::init(argc, argv, "rosplan_pointing_server");
	ros::NodeHandle nh;

	// create PDDL action subscriber
	KCL_rosplan::RPPointingServer rpps(nh);

	// listen for pointing
	ros::Subscriber pointing_pose_sub = nh.subscribe("/squirrel_person_tracker/pointing_pose", 1, &KCL_rosplan::RPPointingServer::receivePointLocation, &rpps);

	// listen for action dispatch
	ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPPointingServer::dispatchCallback, &rpps);
	ROS_INFO("KCL: (PointingServer) Ready to receive");

	ros::spin();
	return 0;
}
