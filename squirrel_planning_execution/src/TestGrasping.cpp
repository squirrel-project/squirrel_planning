/**
 * Class to test grasping on its own.
 */

#include <map>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <rosplan_knowledge_msgs/KnowledgeItem.h>
#include <rosplan_dispatch_msgs/PlanGoal.h>
#include <rosplan_dispatch_msgs/PlanAction.h>
#include <squirrel_object_perception_msgs/SceneObject.h>
#include <geometry_msgs/PoseStamped.h>
#include <squirrel_planning_execution/KnowledgeBase.h>
#include <squirrel_planning_execution/ConfigReader.h>

#include "pddl_actions/ExamineAreaPDDLAction.h"
#include "pddl_actions/ExploreAreaPDDLAction.h"
#include "pddl_actions/TidyAreaPDDLAction.h"
#include "pddl_actions/FinaliseClassificationPDDLAction.h"
#include "pddl_actions/ObserveClassifiableOnAttemptPDDLAction.h"
#include "pddl_actions/ClearObjectPDDLAction.h"
#include "pddl_actions/ShedKnowledgePDDLAction.h"
#include "pddl_actions/FinaliseClassificationPDDLAction.h"
#include "squirrel_object_perception_msgs/SceneObject.h"
#include "squirrel_object_perception_msgs/BCylinder.h"

void startPlanning(ros::NodeHandle& nh)
{
	// Start the planning process.
	std::string data_path;
	nh.getParam("/data_path", data_path);
	
	std::string planner_path;
	nh.getParam("/planner_path", planner_path);
	
	std::string domain_path;
	nh.getParam("/rosplan/domain_path", domain_path);
	
	std::stringstream ss;
	ss << data_path << "final-review_problem.pddl";
	std::string problem_path = ss.str();
	
	std::string planner_command;
	nh.getParam("/rosplan_planning_system/planner_command", planner_command);
	
	rosplan_dispatch_msgs::PlanGoal psrv;
	psrv.domain_path = domain_path;
	psrv.problem_path = problem_path;
	psrv.data_path = data_path;
	psrv.planner_command = planner_command;
	psrv.start_action_id = 0;

	ROS_INFO("KCL: (TestGrasping) Start plan action");
	actionlib::SimpleActionClient<rosplan_dispatch_msgs::PlanAction> plan_action_client("/kcl_rosplan/start_planning", true);

	plan_action_client.waitForServer();
	ROS_INFO("KCL: (TestGrasping) Start planning server found");
	
	// send goal
	plan_action_client.sendGoal(psrv);
	ROS_INFO("KCL: (TestGrasping) Goal sent");
	
	ros::Rate rate(1.0f);
	while (plan_action_client.getState() == actionlib::SimpleClientGoalState::ACTIVE ||
		   plan_action_client.getState() == actionlib::SimpleClientGoalState::PENDING)
	{
		rate.sleep();
		ros::spinOnce();
	}
}

void initialiseKnowledgeBase(KCL_rosplan::KnowledgeBase& kb, mongodb_store::MessageStoreProxy& message_store)
{
	kb.addInstance("robot", "robot");
	
	std::map<std::string, std::string> params;
	params["v"] = "robot";
	params["wp"] = "kenny_waypoint";
	kb.addFact("robot_at", params, true, KCL_rosplan::KnowledgeBase::KB_ADD_KNOWLEDGE);
    
	params.clear();
	params["v"] = "robot";
	kb.addFact("gripper_empty", params, true, KCL_rosplan::KnowledgeBase::KB_ADD_KNOWLEDGE);

    // For every object we need to define the bounding box.
    std::vector<std::string> objects;
    kb.getInstances(objects, "object");
    for (std::vector<std::string>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
    {
        const std::string& object = *ci;
	    ROS_INFO("KCL: (TestGrasping) Add a scene object with ID %s", object.c_str());
        squirrel_object_perception_msgs::SceneObject so;
        so.header.frame_id = "/map";
        so.header.stamp = ros::Time::now();
        so.id = object;
        so.category = object;

        std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > results;
        if (!message_store.queryNamed<geometry_msgs::PoseStamped>(object + "_location", results)) {
            ROS_ERROR("Could not find the location of %s.", std::string(object + "_location").c_str());
            exit(-1);
        }

        so.pose = results[0]->pose;
        so.pose.orientation.x = -0.707;
        so.pose.orientation.w = 0.707;
        so.bounding_cylinder.diameter = 0.1f;
        so.bounding_cylinder.height = 0.1f;
        message_store.insertNamed(so.id, so); 
    }
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "TestGraspingPlanning");
	ros::NodeHandle nh;
	
	// Setup the knowledge base.
	mongodb_store::MessageStoreProxy message_store(nh);
	KCL_rosplan::KnowledgeBase knowledge_base(nh, message_store);
	
	// Initialise the context for this planning task.
	std::string config_file;
	nh.getParam("/scenario_setup_file", config_file);
	
	KCL_rosplan::ConfigReader reader(nh, message_store);
	reader.readConfigurationFile(config_file);

	initialiseKnowledgeBase(knowledge_base, message_store);
	startPlanning(nh);
    ros::spin();
	
	return 0;
}

