#include <map>
#include <vector>
#include <iostream>
#include <sstream>
//#include <boost/concept_check.hpp>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <std_msgs/Int8.h>
#include <std_msgs/ColorRGBA.h>

#include <rosplan_planning_system/PDDLProblemGenerator.h>

#include <rosplan_dispatch_msgs/ActionFeedback.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_knowledge_msgs/GetInstanceService.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>

#include <geometry_msgs/Pose.h>

#include "squirrel_planning_execution/ViewConeGenerator.h"
#include <squirrel_planning_execution/KnowledgeBase.h>

#include "ExploreAreaPDDLAction.h"
#include "PlannerInstance.h"



namespace KCL_rosplan {

	std::string ExploreAreaPDDLAction::g_action_name = "explore_area";
	
	ExploreAreaPDDLAction::ExploreAreaPDDLAction(ros::NodeHandle& node_handle, KCL_rosplan::KnowledgeBase& kb)
		: node_handle_(&node_handle), knowledge_base_(&kb), message_store_(node_handle), is_simulated_(false)
	{
		// create the action feedback publisher
		action_feedback_pub_ = node_handle.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
		
		get_instance_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_current_instances");
		get_attribute_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_knowledge");
		
		dispatch_sub_ = node_handle.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::ExploreAreaPDDLAction::dispatchCallback, this);
		
		std::string occupancyTopic("/map");
		node_handle.param("occupancy_topic", occupancyTopic, occupancyTopic);
		view_cone_generator_ = new ViewConeGenerator(node_handle, occupancyTopic);
		
		node_handle.getParam("/squirrel_planning_execution/simulated", is_simulated_);
	}
	
	ExploreAreaPDDLAction::~ExploreAreaPDDLAction()
	{
		
	}
	
	/*---------------------------*/
	/* strategic action callback */
	/*---------------------------*/

	void ExploreAreaPDDLAction::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	{
		rosplan_dispatch_msgs::ActionDispatch normalised_action_dispatch = *msg;
		std::string action_name = msg->name;
		std::transform(action_name.begin(), action_name.end(), action_name.begin(), tolower);
		normalised_action_dispatch.name = action_name;
		
		// Ignore actions that do not correspond to g_action_name.
		if (g_action_name != action_name)
		{
			return;
		}

		bool actionAchieved = false;
		
		ROS_INFO("KCL: (ExploreAreaPDDLAction) action recieved %s", action_name.c_str());
		
		PlannerInstance& planner_instance = PlannerInstance::createInstance(*node_handle_, "ff", true);
		
		// Lets start the planning process.
		std::string data_path;
		node_handle_->getParam("/data_path", data_path);
		
		std::string planner_path;
		node_handle_->getParam("/planner_path", planner_path);
		
		std::stringstream ss;
		ss << data_path << action_name << "_domain-nt.pddl";
		std::string domain_name = ss.str();
		
		ss.str(std::string());
		ss << data_path << action_name << "_problem.pddl";
		std::string problem_name = ss.str();
		
		ss.str(std::string());
		ss << "timeout 180 " << planner_path << "ff -o DOMAIN -f PROBLEM";
		std::string planner_command = ss.str();
		
		// Before calling the planner we create the domain so it can be parsed.
		if (!createDomain())
		{
			ROS_ERROR("KCL: (ExploreAreaPDDLAction) failed to produce a domain at %s for action name %s.", domain_name.c_str(), action_name.c_str());
			return;
		}
		
		planner_instance.startPlanner(domain_name, problem_name, data_path, planner_command);
		
		// publish feedback (enabled)
		rosplan_dispatch_msgs::ActionFeedback fb;
		fb.action_id = msg->action_id;
		fb.status = "action enabled";
		action_feedback_pub_.publish(fb);

		// wait for action to finish
		ros::Rate loop_rate(1);
		while (ros::ok() && (planner_instance.getState() == actionlib::SimpleClientGoalState::ACTIVE || planner_instance.getState() == actionlib::SimpleClientGoalState::PENDING)) {
			ros::spinOnce();
			loop_rate.sleep();
		}

		actionlib::SimpleClientGoalState state = planner_instance.getState();
		ROS_INFO("KCL: (ExploreAreaPDDLAction) action finished: %s, %s", action_name.c_str(), state.toString().c_str());
		
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> all_facts;
		if (!knowledge_base_->getFacts(all_facts, "explored"))
		{
			ROS_INFO("KCL: (ExploreAreaPDDLAction) Failed to get all actions!");
			exit(-1);
		}
		
		// Remove all explored facts, otherwise subsequent calls will fail.
		for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = all_facts.begin(); ci != all_facts.end(); ++ci)
		{
			const rosplan_knowledge_msgs::KnowledgeItem& ki = *ci;
			knowledge_base_->removeFact(ki, KnowledgeBase::KB_REMOVE_KNOWLEDGE);
		}
        
		// Cleanup any previous waypoints from MongoDB.
        for (std::map<std::string, std::string>::const_iterator ci = db_name_map_.begin(); ci != db_name_map_.end(); ++ci)
        {
			knowledge_base_->removeInstance("waypoint", ci->first);
			message_store_.deleteID(db_name_map_[ci->second]);
        }
		db_name_map_.clear();
		
		// Reset the robot back to the default waypoint.
		all_facts.clear();
		if (!knowledge_base_->getFacts(all_facts, "robot_at"))
		{
			ROS_INFO("KCL: (ExploreAreaPDDLAction) Failed to get all actions!");
			exit(-1);
		}
		for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = all_facts.begin(); ci != all_facts.end(); ++ci)
		{
			const rosplan_knowledge_msgs::KnowledgeItem& ki = *ci;
			knowledge_base_->removeFact(ki, KnowledgeBase::KB_REMOVE_KNOWLEDGE);
		}
		
		std::map<std::string, std::string> params;
		params["v"] = "robot";
		params["wp"] = "kenny_waypoint";
		if (!knowledge_base_->addFact("robot_at", params, true, KCL_rosplan::KnowledgeBase::KB_ADD_KNOWLEDGE))
		{
			ROS_INFO("KCL: (ExploreAreaPDDLAction) Failed to reset the waypoint of the robot!");
			exit(-1);
		}

		if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO("KCL: (ExploreAreaPDDLAction) Process the action: %s.", action_name.c_str());
			
			std::map<std::string, std::string> parameters;
			if (!knowledge_base_->addFact("explored_room", parameters, true, KCL_rosplan::KnowledgeBase::KB_ADD_KNOWLEDGE))
			{
				ROS_INFO("KCL: (ExploreAreaPDDLAction) Failed to add explored_room to the knowledge base!");
				exit(-1);
			}
			
			// publish feedback (achieved)
			rosplan_dispatch_msgs::ActionFeedback fb;
			fb.action_id = msg->action_id;
			fb.status = "action achieved";
			action_feedback_pub_.publish(fb);
		}
		else
		{
			// publish feedback (failed)
			rosplan_dispatch_msgs::ActionFeedback fb;
			fb.action_id = msg->action_id;
			fb.status = "action aborted";
			action_feedback_pub_.publish(fb);
		}
	}
	
	/*--------------------*/
	/* problem generation */
	/*--------------------*/
	
	bool ExploreAreaPDDLAction::createDomain()
	{
		ROS_INFO("KCL: (ExploreAreaPDDLAction) Create domain for action %s.", g_action_name.c_str());
		// Lets start the planning process.
		std::string data_path;
		node_handle_->getParam("/data_path", data_path);

		std::stringstream ss;

		ss << g_action_name << "_domain-nt.pddl";
		std::string domain_name = ss.str();
		ss.str(std::string());

		ss << data_path << domain_name;
		std::string domain_path = ss.str();		
		ss.str(std::string());

		ss << g_action_name << "_problem.pddl";
		std::string problem_name = ss.str();
		ss.str(std::string());

		ss << data_path << problem_name;
		std::string problem_path = ss.str();
		ss.str(std::string());
		
		//rviz things
		std::vector<geometry_msgs::Point> waypoints;
		std::vector<std_msgs::ColorRGBA> waypoint_colours;
		std::vector<geometry_msgs::Point> triangle_points;
		std::vector<std_msgs::ColorRGBA> triangle_colours;

		std::vector<geometry_msgs::Pose> view_poses;
		
		// If the name is 'current_area', then we create a bounding box around the robot to put 
		// view cones in.
		std::vector<tf::Vector3> bounding_box;
		if (!is_simulated_)
		{
			// Locate the location of the robot.
			tf::StampedTransform transform;
			tf::TransformListener tfl;
			try {
				tfl.waitForTransform("/map","/base_link", ros::Time::now(), ros::Duration(1.0));
				tfl.lookupTransform("/map", "/base_link", ros::Time(0), transform);
			} catch ( tf::TransformException& ex ) {
				std::cout << "Is supposed to be simulated! " << is_simulated_ << std::endl;
				ROS_ERROR("KCL: (ExploreAreaPDDLAction) Error find the transform between /map and /base_link.");
				return false;
			}
			
			tf::Vector3 p1(transform.getOrigin().getX() - 5.0f, transform.getOrigin().getY() - 5.0f, 0.00);
			tf::Vector3 p2(transform.getOrigin().getX() + 5.0f, transform.getOrigin().getY() - 5.0f, 0.00);
			tf::Vector3 p3(transform.getOrigin().getX() + 5.0f, transform.getOrigin().getY() + 5.0f, 0.00);
			tf::Vector3 p4(transform.getOrigin().getX() + 5.0f, transform.getOrigin().getY() - 5.0f, 0.00);
			bounding_box.push_back(p1);
			bounding_box.push_back(p3);
			bounding_box.push_back(p4);
			bounding_box.push_back(p2);
			view_cone_generator_->createViewCones(view_poses, bounding_box, 1, 5, 30.0f, 2.0f, 100, 0.5f);
		}
		else
		{
			view_poses.push_back(geometry_msgs::Pose());
			view_poses.push_back(geometry_msgs::Pose());
			view_poses.push_back(geometry_msgs::Pose());
			view_poses.push_back(geometry_msgs::Pose());
		}
		
		// Add these poses to the knowledge base.
		rosplan_knowledge_msgs::KnowledgeUpdateService add_waypoints_service;
		add_waypoints_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		
		unsigned int waypoint_number = 0;
		for (std::vector<geometry_msgs::Pose>::const_iterator ci = view_poses.begin(); ci != view_poses.end(); ++ci) {
			
			ss.str(std::string());
			ss << "explore_wp" << waypoint_number;
			
			if (!knowledge_base_->addInstance("waypoint", ss.str()))
			{
				ROS_ERROR("KCL: (ExploreAreaPDDLAction) Could not add an explore wayoint to the knowledge base.");
				exit(-1);
			}
			
			// add waypoint to MongoDB
			geometry_msgs::PoseStamped pose;
			pose.header.frame_id = "/map";
			pose.pose = *ci;
			std::string id(message_store_.insertNamed(ss.str(), pose));
			db_name_map_[ss.str()] = id;
			
			std::map<std::string, std::string> parameters;
			parameters["wp"] = ss.str();
			if (!knowledge_base_->addFact("explored", parameters, true, KCL_rosplan::KnowledgeBase::KB_ADD_GOAL))
			{
				ROS_ERROR("KCL: (ExploreAreaPDDLAction) Could not add the goal (explored %s) to the knowledge base.", ss.str().c_str());
				exit(-1);
			}
			++waypoint_number;
		}
		
		std_msgs::Int8 nr_waypoint_number_int8;
		nr_waypoint_number_int8.data = waypoint_number;
		ROS_INFO("KCL: (ExploreAreaPDDLAction) Added %d waypoints to the knowledge base.", nr_waypoint_number_int8.data);
		
		PlanningEnvironment planning_environment;
		planning_environment.parseDomain(domain_path);
		planning_environment.update(*node_handle_);
		PDDLProblemGenerator pddl_problem_generator;
		
		pddl_problem_generator.generatePDDLProblemFile(planning_environment, problem_path);
		return true;
	}
};

