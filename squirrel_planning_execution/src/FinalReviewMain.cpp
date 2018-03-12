/**
 * Main function for the final review.
 * 
 * It is a loop that selects what goals to plan for, givent the current state. Options are:
 * 
 * - Explore the room to find objects to identify.
 * - Examine objects.
 * - Tidy examined objects.
 * 
 * These can be used within a single planning problem. All the while the robot keeps track 
 * of its current battery power and frustration level. These influence the way the robot 
 * interacts with its surroundings. If the frustration is very high, then it will ask children 
 * for help when it comes to grasping tasks or identification tasks.
 * 
 * If the power level is very low then it will look for batteries and ask children to give it 
 * the battery.
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

// Note: Part of code in https://github.com/Morloth1274/squirrel_planning/blob/edith-exploration-2017/squirrel_planning_execution/src/FinalReview.cpp
bool setupLumps(KCL_rosplan::KnowledgeBase& kb, mongodb_store::MessageStoreProxy& message_store)
{
	std::vector< boost::shared_ptr<squirrel_object_perception_msgs::SceneObject> > sceneObjects_results;
	message_store.query<squirrel_object_perception_msgs::SceneObject>(sceneObjects_results);
	bool found_lumps = false;
	ROS_INFO("KCL: (SetupLumps) Found %lu lumps.", sceneObjects_results.size());

	// Check if this fact actually exists, or is a leftover (this is bad!).
	std::vector<rosplan_knowledge_msgs::KnowledgeItem> all_facts;
	kb.getFacts(all_facts, "object_at");

	for (std::vector< boost::shared_ptr<squirrel_object_perception_msgs::SceneObject> >::const_iterator ci = sceneObjects_results.begin(); ci != sceneObjects_results.end(); ++ci)
	{
		const squirrel_object_perception_msgs::SceneObject& lump = **ci;
		const std::string& lump_name = lump.id;
		
		ROS_INFO("KCL: (SetupLumps) Process the lump %s.", lump_name.c_str());

		bool exists = false;
		for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = all_facts.begin(); ci != all_facts.end(); ++ci)
		{
			const rosplan_knowledge_msgs::KnowledgeItem& ki = *ci;
			std::string s = kb.toString(ki);
			if (ki.values[0].value == lump_name)
			{
				ROS_INFO("KCL: (SetupLumps) Process lump is in the Knowledge base: %s.", s.c_str());
				exists = true;
				break;
			}
		}
		if (!exists) continue;

		std::map<std::string, std::string> parameters;
		parameters["o"] = lump_name;
		bool is_examined = kb.isFactTrue("examined", parameters, true);
		bool is_not_examined = kb.isFactTrue("examined", parameters, false);
		
		if (!is_examined || is_not_examined)
		{
			ROS_INFO("KCL: (SetupLumps) %s has not been examined yet, time to examine lumps!", lump_name.c_str());
			found_lumps = true;
			break;
		}
	}

	if (found_lumps)
	{
		std::map<std::string, std::string> parameters;
		kb.addFact("examined_room", parameters, true, KCL_rosplan::KnowledgeBase::KB_ADD_GOAL);
		if (!kb.removeFact("examined_room", parameters, true, KCL_rosplan::KnowledgeBase::KB_REMOVE_KNOWLEDGE))
		{
			ROS_ERROR("KCL: (SetupGoals) Failed to remove the predicate (examined_room)!");
			exit(-1);
		}
	}
	
	return found_lumps;
}

bool setupToysToTidy(KCL_rosplan::KnowledgeBase& kb, mongodb_store::MessageStoreProxy& message_store, const std::vector<rosplan_knowledge_msgs::KnowledgeItem>& all_facts)
{
	// Check if there are objects that need to be tidied.
	std::vector<std::string> objects_to_tidy;
	std::map<std::string, std::string> object_tidy_locations;
	for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = all_facts.begin(); ci != all_facts.end(); ++ci)
	{
		const rosplan_knowledge_msgs::KnowledgeItem& fact = *ci;
		if (fact.attribute_name == "object_at")
		{
			for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = fact.values.begin(); ci != fact.values.end(); ++ci)
			{
				const diagnostic_msgs::KeyValue& kv = *ci;
				if (kv.key == "o")
				{
					const std::string object = kv.value;
					ROS_INFO("KCL: (SetupGoals) Add the object %s to the list of objects to tidy.", object.c_str());
					objects_to_tidy.push_back(object);
				}
			}
		}
		else if (fact.attribute_name == "belongs_in")
		{
			std::string object;
			std::string box;
			
			for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = fact.values.begin(); ci != fact.values.end(); ++ci)
			{
				const diagnostic_msgs::KeyValue& kv = *ci;
				if (kv.key == "o")
				{
					object = kv.value;
				}
				else if (kv.key == "b")
				{
					box = kv.value;
				}
			}
			
			ROS_INFO("KCL: (SetupGoals) %s belongs in %s.", object.c_str(), box.c_str());
			object_tidy_locations[object] = box;
		}
	}
	
	// Create a goal for every object of which we know its tidy location.
	bool found_a_toy_to_tidy = false;
	for (std::vector<std::string>::const_iterator ci = objects_to_tidy.begin(); ci != objects_to_tidy.end(); ++ci)
	{
		const std::string& object = *ci;
		ROS_INFO("KCL: (SetupGoals) Do know where %s belongs?", object.c_str());
		if (object_tidy_locations.find(object) == object_tidy_locations.end())
		{
			ROS_INFO("KCL: (SetupGoals) Do not know where to tidy %s, ask someone for help!.", object.c_str());
			continue;
		}
		const std::string& box = object_tidy_locations[object];
		
		// Now create a goal for this object.
		std::map<std::string, std::string> parameters;
		parameters["b"] = box;
		parameters["o"] = object;
		if (!kb.addFact("in_box", parameters, true, KCL_rosplan::KnowledgeBase::KB_ADD_GOAL))
		{
			ROS_ERROR("KCL: (SetupGoals) Failed to add (in_box %s %s) as a goal!", object.c_str(), box.c_str());
			exit(-1);
		}
		
		found_a_toy_to_tidy = true;
	}
	return found_a_toy_to_tidy;
}

void setupGoals(KCL_rosplan::KnowledgeBase& kb, mongodb_store::MessageStoreProxy& message_store)
{
	kb.removeAllGoals();
	
	// Figure out what facts are true, so know what goals  to set for this iteration.
	std::vector<rosplan_knowledge_msgs::KnowledgeItem> all_facts;
	kb.getAllFacts(all_facts);
	
	bool found_a_toy_to_tidy = false;
	if (setupToysToTidy(kb, message_store, all_facts))
	{
		found_a_toy_to_tidy = true;
	}
	
	if (setupLumps(kb, message_store))
	{
		found_a_toy_to_tidy = true;
	}
	
	// If no toys were found to tidy and no lumps exist, then we need to explore the room.
	if (!found_a_toy_to_tidy)
	{
		std::map<std::string, std::string> parameters;
		if (!kb.removeFact("explored_room", parameters, true, KCL_rosplan::KnowledgeBase::KB_REMOVE_KNOWLEDGE))
		{
			ROS_ERROR("KCL: (SetupGoals) Failed to remove the predicate (explored_roomn)!");
			exit(-1);
		}

		if (!kb.removeFact("examined_room", parameters, true, KCL_rosplan::KnowledgeBase::KB_REMOVE_KNOWLEDGE))
		{
			ROS_ERROR("KCL: (SetupGoals) Failed to remove the predicate (examined_room)!");
			exit(-1);
		}
		
		if (!kb.addFact("explored_room", parameters, true, KCL_rosplan::KnowledgeBase::KB_ADD_GOAL))
		{
			ROS_ERROR("KCL: (SetupGoals) Failed to add (explored) as a goal!");
			exit(-1);
		}
	}
}

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

	ROS_INFO("KCL: (FinalReview) Start plan action");
	actionlib::SimpleActionClient<rosplan_dispatch_msgs::PlanAction> plan_action_client("/kcl_rosplan/start_planning", true);

	plan_action_client.waitForServer();
	ROS_INFO("KCL: (FinalReview) Start planning server found");
	
	// send goal
	plan_action_client.sendGoal(psrv);
	ROS_INFO("KCL: (FinalReview) Goal sent");
	
	ros::Rate rate(1.0f);
	while (plan_action_client.getState() == actionlib::SimpleClientGoalState::ACTIVE ||
		   plan_action_client.getState() == actionlib::SimpleClientGoalState::PENDING)
	{
		rate.sleep();
		ros::spinOnce();
	}
}

void initialiseKnowledgeBase(KCL_rosplan::KnowledgeBase& kb)
{
	kb.addInstance("robot", "robot");
	
	std::map<std::string, std::string> params;
	params["v"] = "robot";
	params["wp"] = "kenny_waypoint";
	kb.addFact("robot_at", params, true, KCL_rosplan::KnowledgeBase::KB_ADD_KNOWLEDGE);
	params.clear();
	
	kb.addFact("not_busy", params, true, KCL_rosplan::KnowledgeBase::KB_ADD_KNOWLEDGE);
	kb.addFact("not_flashing_lights", params, true, KCL_rosplan::KnowledgeBase::KB_ADD_KNOWLEDGE);
	kb.addFact("not_playing_sound", params, true, KCL_rosplan::KnowledgeBase::KB_ADD_KNOWLEDGE);
	kb.addFact("not_gazing", params, true, KCL_rosplan::KnowledgeBase::KB_ADD_KNOWLEDGE);
	
	params["v"] = "robot";
	kb.addFact("gripper_empty", params, true, KCL_rosplan::KnowledgeBase::KB_ADD_KNOWLEDGE);
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "FinalReviewPlanning");
	ros::NodeHandle nh;
	
	// Setup the knowledge base.
	mongodb_store::MessageStoreProxy message_store(nh);
	KCL_rosplan::KnowledgeBase knowledge_base(nh, message_store);
	
	
	// Initialise the context for this planning task.
	std::string config_file;
	nh.getParam("/scenario_setup_file", config_file);
	
	KCL_rosplan::ConfigReader reader(nh, message_store);
	reader.readConfigurationFile(config_file);
	
	// Create all PDDL actions that we might need during execution.
	KCL_rosplan::ExamineAreaPDDLAction examine_area_action(nh, knowledge_base);
	KCL_rosplan::ExploreAreaPDDLAction explore_area_action(nh, knowledge_base);
	KCL_rosplan::TidyAreaPDDLAction tidy_area_action(nh, knowledge_base);
	KCL_rosplan::FinaliseClassificationPDDLAction finalise_classification_action(nh);
	KCL_rosplan::ObserveClassifiableOnAttemptPDDLAction observe_classifiable_on_attempt_action(nh);
	KCL_rosplan::ClearObjectPDDLAction clear_object_action(nh);
	KCL_rosplan::ShedKnowledgePDDLAction shed_knowledge_action(nh);
	KCL_rosplan::FinaliseClassificationPDDLAction finalise_classification__action(nh);
	
	initialiseKnowledgeBase(knowledge_base);
	
	// Keep running forever.
	while (true)
	{
		setupGoals(knowledge_base, message_store);
		startPlanning(nh);
	}
	
	return 0;
}
