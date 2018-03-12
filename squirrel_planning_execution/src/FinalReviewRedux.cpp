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

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <rosplan_knowledge_msgs/KnowledgeItem.h>
#include <rosplan_dispatch_msgs/PlanGoal.h>
#include <rosplan_dispatch_msgs/PlanAction.h>
#include <squirrel_object_perception_msgs/SceneObject.h>
#include <squirrel_waypoint_msgs/ExamineWaypoint.h>

#include <squirrel_planning_execution/KnowledgeBase.h>
#include <squirrel_planning_execution/ConfigReader.h>
#include "squirrel_planning_execution/ViewConeGenerator.h"

//#include "pddl_actions/ExploreAreaPDDLAction.h"
#include "pddl_actions/AttemptToExamineObjectPDDLAction.h"

// All service clients.
ros::ServiceClient classify_object_waypoint_client;
KCL_rosplan::ViewConeGenerator* view_cone_generator_;

bool setupLumps(KCL_rosplan::KnowledgeBase& kb, mongodb_store::MessageStoreProxy& message_store)
{
	std::vector< boost::shared_ptr<squirrel_object_perception_msgs::SceneObject> > sceneObjects_results;
	message_store.query<squirrel_object_perception_msgs::SceneObject>(sceneObjects_results);
	
	unsigned int nr_lumps_found = 0;
	ROS_INFO("KCL: (SetupLumps) Found %lu lumps.", sceneObjects_results.size());
	
	// Check if this fact actually exists, or is a leftover (this is bad!).
	std::vector<rosplan_knowledge_msgs::KnowledgeItem> all_facts;
	kb.getFacts(all_facts, "object_at");

	for (std::vector< boost::shared_ptr<squirrel_object_perception_msgs::SceneObject> >::const_iterator ci = sceneObjects_results.begin(); ci != sceneObjects_results.end(); ++ci)
	{
		const squirrel_object_perception_msgs::SceneObject& lump = **ci;
		const std::string& lump_name = lump.id;
		std::string lump_waypoint;
		
		ROS_INFO("KCL: (SetupLumps) Process the lump %s.", lump_name.c_str());

		// Check if we know where this lump is, if not we cannot process it.
		for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = all_facts.begin(); ci != all_facts.end(); ++ci)
		{
			const rosplan_knowledge_msgs::KnowledgeItem& ki = *ci;
			std::string s = kb.toString(ki);
			std::string object_name;
			std::string object_waypoint;
			
			for (unsigned int i = 0; i < ki.values.size(); ++i)
			{
				if (ki.values[i].key == "o")
					object_name = ki.values[i].value;
				else if (ki.values[i].key == "wp")
					object_waypoint = ki.values[i].value;
			}
			
			if (object_name == lump_name)
			{
				ROS_INFO("KCL: (SetupLumps) Process lump is in the Knowledge base: %s.", s.c_str());
				lump_waypoint = object_waypoint;
				break;
			}
		}
		if (lump_waypoint == "") continue;

		// Check if this lump has already been examined, if so then we are done.
		std::map<std::string, std::string> parameters;
		parameters["o"] = lump_name;
		bool is_examined = kb.isFactTrue("examined", parameters, true);
		
		// If it is not examined, we create waypoints around the lump from where the lump can be examined.
		if (is_examined) continue;
		
		ROS_INFO("KCL: (SetupLumps) %s has not been examined yet, time to examine lumps!", lump_name.c_str());
		squirrel_waypoint_msgs::ExamineWaypoint getTaskPose;
	
		// fetch position of object from message store
		std::vector< boost::shared_ptr<squirrel_object_perception_msgs::SceneObject> > results;
		if(message_store.queryNamed<squirrel_object_perception_msgs::SceneObject>(lump_name, results)) {
			if(results.size()<1) {
				ROS_ERROR("KCL: (SetupLumps) aborting waypoint request; no matching obID %s", lump_name.c_str());
				return false;
			}
		} else {
			ROS_ERROR("KCL: (SetupLumps) could not query message store to fetch object pose of %s", lump_name.c_str());
			return false;
		}

		// request classification waypoints for object
		squirrel_object_perception_msgs::SceneObject &obj = *results[0];
		
		getTaskPose.request.object_pose.header = obj.header;
		getTaskPose.request.object_pose.pose = obj.pose;
		if (!classify_object_waypoint_client.call(getTaskPose))
		{
			ROS_ERROR("KCL: (SetupLumps) Failed to recieve classification waypoints for %s.", lump_name.c_str());

			// Remove this object, as we cannot get to it!
			std::map<std::string, std::string> parameters;
			parameters["o"] = lump_name;
			parameters["wp"] = lump_waypoint;
			if (!kb.removeFact("object_at", parameters, true, KCL_rosplan::KnowledgeBase::KB_REMOVE_KNOWLEDGE))
			{
				ROS_ERROR("KCL: (SetupLumps) Failed to remove (object_at %s %s) from the knowledge base!", lump_name.c_str(), lump_waypoint.c_str());
				exit(-1);
			}
		}

		ROS_INFO("KCL: (SetupLumps) Found %lu observation poses", getTaskPose.response.poses.size());

		// Add all the waypoints to the knowledge base.
		for(int i = 0; i < getTaskPose.response.poses.size(); i++)
		{
			std::stringstream ss;
			ss << lump_name << "_observation_wp" << i;
			
			ROS_INFO("KCL: (SetupLumps) Process observation pose: %s", ss.str().c_str());
			kb.addInstance("waypoint", ss.str());

			std::map<std::string, std::string> parameters;
			parameters["wp1"] = ss.str();
			parameters["wp2"] = lump_waypoint;
			kb.addFact("near", parameters, true, KCL_rosplan::KnowledgeBase::KB_ADD_KNOWLEDGE);
			
			const geometry_msgs::PoseWithCovarianceStamped& pwcs = getTaskPose.response.poses[i];
			geometry_msgs::PoseStamped ps;
			ps.header= pwcs.header;
			ps.header.frame_id = "/map";
			ps.pose = pwcs.pose.pose;
			std::string near_waypoint_mongodb_id(message_store.insertNamed(ss.str(), ps));
			
			// Add as goal to examine this object from each possible waypoint.
			parameters.clear();
			parameters["o"] = lump_name;
			parameters["wp"] = ss.str();
			kb.addFact("observed-from", parameters, true, KCL_rosplan::KnowledgeBase::KB_ADD_GOAL);
			++nr_lumps_found;

			if (nr_lumps_found > 5) return true;
		}
	}
	
	return nr_lumps_found > 0;
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

void setupViewCones(KCL_rosplan::KnowledgeBase& kb, mongodb_store::MessageStoreProxy& message_store)
{
	static std::map<std::string, std::string> db_name_map;
	std::vector<rosplan_knowledge_msgs::KnowledgeItem> all_facts;
	if (!kb.getFacts(all_facts, "explored"))
	{
		ROS_INFO("KCL: (ExploreAreaPDDLAction) Failed to get all actions!");
		exit(-1);
	}
	
	// Remove all explored facts, otherwise subsequent calls will fail.
	for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = all_facts.begin(); ci != all_facts.end(); ++ci)
	{
		const rosplan_knowledge_msgs::KnowledgeItem& ki = *ci;
		kb.removeFact(ki, KCL_rosplan::KnowledgeBase::KB_REMOVE_KNOWLEDGE);
	}
	
	// Cleanup any previous waypoints from MongoDB.
	for (std::map<std::string, std::string>::const_iterator ci = db_name_map.begin(); ci != db_name_map.end(); ++ci)
	{
		ROS_INFO("KCL: (SetupViewCones) Remove the waypoint: %s, MongoDB id: %s!", ci->first.c_str(), ci->second.c_str());
		kb.removeInstance("waypoint", ci->first);
		message_store.deleteID(db_name_map[ci->second]);
	}
	db_name_map.clear();
	
	// Reset the robot back to the default waypoint.
	all_facts.clear();
	if (!kb.getFacts(all_facts, "robot_at"))
	{
		ROS_INFO("KCL: (ExploreAreaPDDLAction) Failed to get all actions!");
		exit(-1);
	}
	for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = all_facts.begin(); ci != all_facts.end(); ++ci)
	{
		const rosplan_knowledge_msgs::KnowledgeItem& ki = *ci;
		kb.removeFact(ki, KCL_rosplan::KnowledgeBase::KB_REMOVE_KNOWLEDGE);
	}
	
	std::map<std::string, std::string> params;
	params["v"] = "robot";
	params["wp"] = "kenny_waypoint";
	if (!kb.addFact("robot_at", params, true, KCL_rosplan::KnowledgeBase::KB_ADD_KNOWLEDGE) ||
	    !kb.addInstance("waypoint", "kenny_waypoint"))
	{
		ROS_INFO("KCL: (ExploreAreaPDDLAction) Failed to reset the waypoint of the robot!");
		exit(-1);
	}

	// Locate the location of the robot.
	tf::StampedTransform transform;
	tf::TransformListener tfl;
	try {
		tfl.waitForTransform("/map","/base_link", ros::Time::now(), ros::Duration(1.0));
		tfl.lookupTransform("/map", "/base_link", ros::Time(0), transform);
	} catch ( tf::TransformException& ex ) {
		ROS_ERROR("KCL: (ExploreAreaPDDLAction) Error find the transform between /map and /base_link.");
		return;
	}
	geometry_msgs::PoseStamped robot_pose;
	robot_pose.header.frame_id = "/map";
	robot_pose.pose.position.x = transform.getOrigin().getX();
	robot_pose.pose.position.y = transform.getOrigin().getY();
	robot_pose.pose.position.z = transform.getOrigin().getZ();
	robot_pose.pose.orientation.x = transform.getRotation().getX();
	robot_pose.pose.orientation.y = transform.getRotation().getY();
	robot_pose.pose.orientation.z = transform.getRotation().getZ();
	robot_pose.pose.orientation.w = transform.getRotation().getW();
	std::string robot_id(message_store.insertNamed("kenny_waypoint", robot_pose));
	db_name_map["kenny_waypoint"] = robot_id;

	ROS_INFO("KCL: (setupViewCones) Store the location of kenny.");
	
	// Create the view cones.
	tf::Vector3 p1(transform.getOrigin().getX() - 5.0f, transform.getOrigin().getY() - 5.0f, 0.00);
	tf::Vector3 p2(transform.getOrigin().getX() + 5.0f, transform.getOrigin().getY() - 5.0f, 0.00);
	tf::Vector3 p3(transform.getOrigin().getX() + 5.0f, transform.getOrigin().getY() + 5.0f, 0.00);
	tf::Vector3 p4(transform.getOrigin().getX() + 5.0f, transform.getOrigin().getY() - 5.0f, 0.00);
	std::vector<tf::Vector3> bounding_box;
	bounding_box.push_back(p1);
	bounding_box.push_back(p3);
	bounding_box.push_back(p4);
	bounding_box.push_back(p2);
	std::vector<geometry_msgs::Pose> view_poses;
	view_cone_generator_->createViewCones(view_poses, bounding_box, 1, 5, 30.0f, 2.0f, 100, 0.5f);
	
	// Add these poses to the knowledge base.
	unsigned int waypoint_number = 0;
	for (std::vector<geometry_msgs::Pose>::const_iterator ci = view_poses.begin(); ci != view_poses.end(); ++ci) {
		
		std::stringstream ss;
		ss << "explore_wp" << waypoint_number;
		
		if (!kb.addInstance("waypoint", ss.str()))
		{
			ROS_ERROR("KCL: (ExploreAreaPDDLAction) Could not add an explore wayoint to the knowledge base.");
			exit(-1);
		}
		
		// add waypoint to MongoDB
		geometry_msgs::PoseStamped pose;
		pose.header.frame_id = "/map";
		pose.pose = *ci;
		std::string id(message_store.insertNamed(ss.str(), pose));
		db_name_map[ss.str()] = id;
		
		std::map<std::string, std::string> parameters;
		parameters["wp"] = ss.str();
		if (!kb.addFact("explored", parameters, true, KCL_rosplan::KnowledgeBase::KB_ADD_GOAL))
		{
			ROS_ERROR("KCL: (ExploreAreaPDDLAction) Could not add the goal (explored %s) to the knowledge base.", ss.str().c_str());
			exit(-1);
		}
		++waypoint_number;
	}
	
	ROS_INFO("KCL: (ExploreAreaPDDLAction) Added %u waypoints to the knowledge base.", waypoint_number);
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
		setupViewCones(kb, message_store);
	}

	// Update the distance between any pair of waypoints.
	std::vector<std::string> waypoints;
	kb.getInstances(waypoints, "waypoint");

	for (int i = 0; i < waypoints.size() - 1; ++i)
	{
		const std::string& wp = waypoints[i];
		std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > results;
		if (!message_store.queryNamed<geometry_msgs::PoseStamped>(wp, results) || results.size() < 1) continue;
		geometry_msgs::PoseStamped &wp_pose = *results[0];
		
		for (int j = i + 1; j < waypoints.size(); ++j)
		{
			const std::string& other_wp = waypoints[j];
			std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > other_results;
			if (!message_store.queryNamed<geometry_msgs::PoseStamped>(other_wp, other_results) || other_results.size() < 1) continue;
			geometry_msgs::PoseStamped &other_wp_pose = *other_results[0];

			float distance = sqrt((wp_pose.pose.position.x - other_wp_pose.pose.position.x) * (wp_pose.pose.position.x - other_wp_pose.pose.position.x) +
								  (wp_pose.pose.position.y - other_wp_pose.pose.position.y) * (wp_pose.pose.position.y - other_wp_pose.pose.position.y));
			std::map<std::string, std::string> parameters;
			parameters["wp1"] = wp;
			parameters["wp2"] = other_wp;
			kb.addFunction("distance", parameters, distance, KCL_rosplan::KnowledgeBase::KB_ADD_KNOWLEDGE);

			parameters["wp1"] = other_wp;
			parameters["wp2"] = wp;
			kb.addFunction("distance", parameters, distance, KCL_rosplan::KnowledgeBase::KB_ADD_KNOWLEDGE);
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
	ss << data_path << "final-review_redux_problem.pddl";
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
	
	// Setup all services.
	classify_object_waypoint_client = nh.serviceClient<squirrel_waypoint_msgs::ExamineWaypoint>("/squirrel_perception_examine_waypoint");
	
	std::string occupancyTopic("/map");
	nh.param("occupancy_topic", occupancyTopic, occupancyTopic);
	view_cone_generator_ = new KCL_rosplan::ViewConeGenerator(nh, occupancyTopic);

	// Setup the knowledge base.
	mongodb_store::MessageStoreProxy message_store(nh);
	KCL_rosplan::KnowledgeBase knowledge_base(nh, message_store);
	
	// Initialise the context for this planning task.
	std::string config_file;
	nh.getParam("/scenario_setup_file", config_file);
	
	KCL_rosplan::ConfigReader reader(nh, message_store);
	reader.readConfigurationFile(config_file);
	
	// Create all PDDL actions that we might need during execution.
//	KCL_rosplan::ExploreAreaPDDLAction explore_area_action(nh, knowledge_base);
	KCL_rosplan::AttemptToExamineObjectPDDLAction attempt_to_examine_action(nh, knowledge_base, message_store);
	
	initialiseKnowledgeBase(knowledge_base);
	
	// Keep running forever.
	while (true)
	{
		ros::spinOnce();
		setupGoals(knowledge_base, message_store);
		startPlanning(nh);
	}
	
	return 0;
}
