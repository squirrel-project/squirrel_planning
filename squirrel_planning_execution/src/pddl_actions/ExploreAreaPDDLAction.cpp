#include <map>
#include <vector>
#include <iostream>
#include <sstream>

#include <std_msgs/Int8.h>
#include <std_msgs/ColorRGBA.h>

#include <rosplan_planning_system/PDDLProblemGenerator.h>

#include <rosplan_dispatch_msgs/ActionFeedback.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_knowledge_msgs/GetInstanceService.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>

#include <geometry_msgs/Pose.h>

#include "squirrel_planning_execution/ViewConeGenerator.h"

#include "ExploreAreaPDDLAction.h"
#include "PlannerInstance.h"



namespace KCL_rosplan {

	std::string ExploreAreaPDDLAction::g_action_name = "explore_area";
	
	ExploreAreaPDDLAction::ExploreAreaPDDLAction(ros::NodeHandle& node_handle)
		: node_handle_(&node_handle), message_store_(node_handle), is_simulated_(true)
	{
		update_knowledge_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
		
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
		
		PlannerInstance& planner_instance = PlannerInstance::createInstance(*node_handle_);
		
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

		if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			// Update the domain.
			const std::string& robot = msg->parameters[0].value;
			const std::string& area = msg->parameters[1].value;
			
			ROS_INFO("KCL: (ExploreAreaPDDLAction) Process the action: %s, Explore %s by %s", action_name.c_str(), area.c_str(), robot.c_str());
			
			// Remove the old knowledge.
			rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
			knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
			rosplan_knowledge_msgs::KnowledgeItem kenny_knowledge;
			kenny_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			kenny_knowledge.attribute_name = "explored";
			kenny_knowledge.is_negative = false;
			
			diagnostic_msgs::KeyValue kv;
			kv.key = "a";
			kv.value = area;
			kenny_knowledge.values.push_back(kv);
			
			knowledge_update_service.request.knowledge = kenny_knowledge;
			if (!update_knowledge_client_.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (ExploreAreaPDDLAction) Could not add the (explored %s) predicate to the knowledge base.", area.c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (ExploreAreaPDDLAction) Added the (explored %s) predicate to the knowledge base.", area.c_str());
			kenny_knowledge.values.clear();
			
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
			fb.status = "action failed";
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
		if (!is_simulated_ && false)
		{
			std::vector<tf::Vector3> bounding_box;
			tf::Vector3 p1(3.22, 4.36, 0.00);
			tf::Vector3 p2(-0.5, 4.07, 0.00);
			tf::Vector3 p3(3.49, 0.01, 0.00);
			tf::Vector3 p4(-0.35, -0.09, 0.00);
			bounding_box.push_back(p1);
			bounding_box.push_back(p3);
			bounding_box.push_back(p4);
			bounding_box.push_back(p2);
			view_cone_generator_->createViewCones(view_poses, bounding_box, 3, 5, 30.0f, 2.0f, 100, 0.35f);
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
			rosplan_knowledge_msgs::KnowledgeItem waypoint_knowledge;
			add_waypoints_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
			waypoint_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
			waypoint_knowledge.instance_type = "waypoint";
			waypoint_knowledge.instance_name = ss.str();
			add_waypoints_service.request.knowledge = waypoint_knowledge;
			if (!update_knowledge_client_.call(add_waypoints_service)) {
				ROS_ERROR("KCL: (ExploreAreaPDDLAction) Could not add an explore wayoint to the knowledge base.");
				exit(-1);
			}

			// add waypoint to MongoDB
			geometry_msgs::PoseStamped pose;
			pose.header.frame_id = "/map";
			pose.pose = *ci;
			std::string id(message_store_.insertNamed(ss.str(), pose));
			
			// Setup the goal.
			waypoint_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			waypoint_knowledge.attribute_name = "explored";
			waypoint_knowledge.is_negative = false;
			diagnostic_msgs::KeyValue kv;
			kv.key = "wp";
			kv.value = ss.str();
			waypoint_knowledge.values.push_back(kv);
			
			// Add the goal.
			add_waypoints_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
			add_waypoints_service.request.knowledge = waypoint_knowledge;
			if (!update_knowledge_client_.call(add_waypoints_service)) {
				ROS_ERROR("KCL: (ExploreAreaPDDLAction) Could not add the goal (explored %s) to the knowledge base.", ss.str().c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (ExploreAreaPDDLAction) Added the goal (explored %s) to the knowledge base.", ss.str().c_str());
			++waypoint_number;
		}

		// add initial state (robot_at)
		rosplan_knowledge_msgs::KnowledgeItem waypoint_knowledge;
		add_waypoints_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		waypoint_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
		waypoint_knowledge.instance_type = "waypoint";
		waypoint_knowledge.instance_name = "kenny_waypoint";
		add_waypoints_service.request.knowledge = waypoint_knowledge;
		if (!update_knowledge_client_.call(add_waypoints_service)) {
			ROS_ERROR("KCL: (ExploreAreaPDDLAction) Could not add an explore wayoint to the knowledge base.");
			exit(-1);
		}
/*		
		// Set the location of the robot.
		waypoint_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		waypoint_knowledge.attribute_name = "robot_at";
		waypoint_knowledge.is_negative = false;
		diagnostic_msgs::KeyValue kv;
		kv.key = "v";
		kv.value = "kenny";
		waypoint_knowledge.values.push_back(kv);
		kv.key = "wp";
		kv.value = "kenny_waypoint";
		waypoint_knowledge.values.push_back(kv);
		add_waypoints_service.request.knowledge = waypoint_knowledge;
		if (!update_knowledge_client_.call(add_waypoints_service)) {
			ROS_ERROR("KCL: (ExploreAreaPDDLAction) Could not add the fact (robot_at kenny room) to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (ExploreAreaPDDLAction) Added (robot_at kenny room) to the knowledge base.");
		waypoint_knowledge.values.clear();
*/
		
		
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
