#include "squirrel_interface_manipulation/RPGraspAction.h"
#include <std_srvs/Empty.h>

/* The implementation of RPGraspAction.h */
namespace KCL_rosplan {

	/* constructor */
	RPGraspAction::RPGraspAction(ros::NodeHandle &nh, std::string &blindGraspActionServer)
	 : message_store(nh), blind_grasp_action_client(blindGraspActionServer, true), putDownActionClient("metahand_place_server", true), kclhandGraspActionClient("hand_controller/actuate_hand", true), ptpActionClient("/joint_ptp", true)

	{

		// Read the parameter if real placement is selected or only dropping
		nh.param("/squirrel_interface_manipulation/placement", do_placement, false);
		if (!do_placement)
		    ROS_WARN("KCL: (GraspAction) placement is not selected, only dropping objects");
		else
		    ROS_INFO("KCL: (GraspAction) placement is selected");

		ROS_INFO("KCL: (GraspAction) waiting for action server to start on %s", blindGraspActionServer.c_str());
		ROS_INFO("KCL: (GraspAction) action servers found");

		// Wait for the action servers.
        	kclhandGraspActionClient.waitForServer();
		putDownActionClient.waitForServer();
		ptpActionClient.waitForServer();
		ROS_INFO("KCL: (GraspAction) all action servers found!");
		

		// create the action feedback publisher
		action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);

		// create knowledge base link
		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");

		// listen to the joins.
		joint_state_sub = nh.subscribe("/real/robotino/joint_control/get_state", 10, &RPGraspAction::jointCallback, this);
		clear_cost_map_client = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
	}

	/* action dispatch callback */
	void RPGraspAction::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		ROS_INFO("Grasp server callback function for action name %s", msg->name.c_str());
		rosplan_dispatch_msgs::ActionFeedback fb;

		// ignore non-grasp actions
		if(0==msg->name.compare("pickup_object")) {

			publishFeedback(msg->action_id, "action enabled");
			if(dispatchBlindGraspAction(msg)) {
				publishFeedback(msg->action_id, "action achieved");
				return;
			}
			publishFeedback(msg->action_id, "action achieved");
		}

		// ignore non-drop objects
		else if(0==msg->name.compare("putdown_object") || 0==msg->name.compare("put_object_in_box")) {
			publishFeedback(msg->action_id, "action enabled");
			// Placement and dropping are toggled here depending on the parameter
			if (do_placement) {
				if(dispatchDropActionCorrect(msg)) {
					publishFeedback(msg->action_id, "action achieved");
					return;
				}
			}
			else
			{
				if(dispatchDropAction(msg)) {
					publishFeedback(msg->action_id, "action achieved");
					return;
				}
			}
			publishFeedback(msg->action_id, "action failed");
		}
		else if(msg->name == "drop_object") {
			publishFeedback(msg->action_id, "action enabled");
			if(dispatchDropAction(msg)) {
				publishFeedback(msg->action_id, "action achieved");
				return;
			}
		}
	}

	/* blind grasp action dispatch
		:parameters (?v - robot ?wp - waypoint ?o - object)
		:effect (and
				(not (holding ?v ?o))
				(gripper_empty ?v)
				(object_at ?o ?wp)
	*/
	bool RPGraspAction::dispatchDropAction(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

        ROS_INFO("KCL: (DropAction) dropping the object from the hand");

		// get object ID from action dispatch
		std::string robotID, objectID, wpID;
		bool foundObject = false;
		for(size_t i=0; i<msg->parameters.size(); i++) {
			if(0==msg->parameters[i].key.compare("wp"))
				wpID = msg->parameters[i].value;
			if(0==msg->parameters[i].key.compare("r"))
				robotID = msg->parameters[i].value;
	            if(0==msg->parameters[i].key.substr(0,1).compare("o")) {
				objectID = msg->parameters[i].value;
				foundObject = true;
			}
		}
		if(!foundObject) {
			ROS_INFO("KCL: (GraspAction) aborting action dispatch; malformed parameters");
			return false;
		}

		// Call action to open hand
		kclhand_control::ActuateHandActionGoal open_hand_goal;
		open_hand_goal.goal.command = 0;
		open_hand_goal.goal.force_limit = 1.0;
		kclhandGraspActionClient.sendGoal(open_hand_goal.goal);
		ROS_INFO("KCL: sent the goal, waiting for the result");
		kclhandGraspActionClient.waitForResult();
		actionlib::SimpleClientGoalState state = kclhandGraspActionClient.getState();
		sleep(1.0);
		kclhandGraspActionClient.sendGoal(open_hand_goal.goal);
		ROS_INFO("KCL: (DropAction) action finished: %s", state.toString().c_str());
		if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			// gripper_empty fact
			rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
			knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
			knowledge_update_service.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			knowledge_update_service.request.knowledge.attribute_name = "gripper_empty";
			diagnostic_msgs::KeyValue kv;
			kv.key = "v";
			kv.value = robotID;
			knowledge_update_service.request.knowledge.values.push_back(kv);
			if (!update_knowledge_client.call(knowledge_update_service)) {
                ROS_ERROR("KCL: (DropAction) Could not add gripper_empty predicate to the knowledge base.");
			}

			// holding fact
			knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
			knowledge_update_service.request.knowledge.attribute_name = "holding";
			kv.key = "o";
			kv.value = objectID;
			knowledge_update_service.request.knowledge.values.push_back(kv);
			if (!update_knowledge_client.call(knowledge_update_service)) {
                ROS_ERROR("KCL: (DropAction) Could not remove holding predicate from the knowledge base.");
			}

			// object_at fact
			knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
			knowledge_update_service.request.knowledge.attribute_name = "object_at";
			knowledge_update_service.request.knowledge.values.clear();
			kv.key = "o";
			kv.value = objectID;
			knowledge_update_service.request.knowledge.values.push_back(kv);
			kv.key = "wp";
			kv.value = wpID;
			knowledge_update_service.request.knowledge.values.push_back(kv);
			if (!update_knowledge_client.call(knowledge_update_service)) {
                ROS_ERROR("KCL: (DropAction) Could not remove object_at predicate to the knowledge base.");
			}
			
			return true;
		}
		return false;
	}

	bool RPGraspAction::dispatchDropActionCorrect(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

	        ROS_INFO("KCL: (DropActionCorrect) placing the object on the ground");

        	// get object ID from action dispatch
		std::string boxID, robotID, wpID, objectID;
		bool foundBox = false;
		for(size_t i=0; i<msg->parameters.size(); i++) {
			std::cout << msg->parameters[i].key << std::endl;
			if(0==msg->parameters[i].key.compare("b"))
			{
				boxID = msg->parameters[i].value;
				foundBox = true;
			}
			if (msg->parameters[i].key == "v")
				robotID = msg->parameters[i].value;
			if (msg->parameters[i].key == "o1")
				objectID = msg->parameters[i].value;
			if (msg->parameters[i].key == "wp")
				wpID = msg->parameters[i].value;
		}
		if(!foundBox) {
		ROS_INFO("KCL: (DropActionCorrect) aborting action dispatch; malformed parameters");
			return false;
		}

		// Set up action servers
		bool success = true;

		// Get the location of the box.
		std::stringstream ss;
		ss << boxID << "_location";
		std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > results;
		if(message_store.queryNamed<geometry_msgs::PoseStamped>(ss.str(), results)) {
			if(results.size()<1) {
				ROS_ERROR("KCL: (DropActionCorrect) aborting pushing location; no matching obID %s", ss.str().c_str());
				return false;
			}
			if(results.size()>1)
				ROS_ERROR("KCL: (DropActionCorrect) multiple waypoints share the same boxID");
		} else {
			ROS_ERROR("KCL: (DropActionCorrect) could not query message store to fetch object pose");
			return false;
		}

		// Get the near box location.
		ss.str(std::string());
		ss << "near_" << boxID;
		std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > near_results;
		if(message_store.queryNamed<geometry_msgs::PoseStamped>(ss.str(), near_results)) {
			if(results.size()<1) {
				ROS_ERROR("KCL: (DropActionCorrect) aborting pushing location; no matching obID %s", ss.str().c_str());
				return false;
			}
			if(results.size()>1)
				ROS_ERROR("KCL: (DropActionCorrect) multiple waypoints share the same boxID");
		} else {
			ROS_ERROR("KCL: (SimpleDemo) could not query message store to fetch object pose");
			return false;
		}

		// calculate pushing pose for object and new point
		geometry_msgs::PoseStamped &objPose = *results[0];
		geometry_msgs::PoseStamped &nearPose = *near_results[0];

		float angle = atan2(objPose.pose.position.y - nearPose.pose.position.y, objPose.pose.position.x - nearPose.pose.position.x);
		
		// Call action to move arm to put down location
		squirrel_manipulation_msgs::PutDownActionGoal put_down_goal;
		put_down_goal.goal.destination_id = "box";
		put_down_goal.goal.destPoseSE2.header.frame_id = "map";
		put_down_goal.goal.destPoseSE2.pose = results[0]->pose;
		put_down_goal.goal.destPoseSE2.pose.position.z = 0.2;
		put_down_goal.goal.destPoseSE2.pose.orientation = tf::createQuaternionMsgFromYaw(angle);

		// Call action
		putDownActionClient.sendGoal(put_down_goal.goal);
		putDownActionClient.waitForResult();
		sleep(1.0);
		putDownActionClient.sendGoal(put_down_goal.goal);
		actionlib::SimpleClientGoalState state_put_down = putDownActionClient.getState();
		ROS_INFO("KCL: (DropActionCorrect) action finished: %s", state_put_down.toString().c_str());

		//retractArm(); // moved to after open hand (for failure case)

		if (state_put_down != actionlib::SimpleClientGoalState::SUCCEEDED) {
			ROS_WARN("KCL (DropActionCorrect) failed to put down, but that is not catastrophic so proceeding with opening hand");
			// Call action to open hand
			kclhand_control::ActuateHandActionGoal open_hand_goal;
			open_hand_goal.goal.command = 0;
			open_hand_goal.goal.force_limit = 1.0;
			kclhandGraspActionClient.sendGoal(open_hand_goal.goal);
		
			// bool finished_before_timeout = false;
			kclhandGraspActionClient.waitForResult();
			actionlib::SimpleClientGoalState state = kclhandGraspActionClient.getState();
			ROS_INFO("KCL: (DropAction) action finished: %s", state.toString().c_str());
                
			if (state != actionlib::SimpleClientGoalState::SUCCEEDED)
				success = false;
		}

		retractArm();
		std_srvs::Empty dummy;
		clear_cost_map_client.call(dummy);

		if (success){
			// gripper_empty fact
			rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
			knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
			knowledge_update_service.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			knowledge_update_service.request.knowledge.attribute_name = "gripper_empty";
			diagnostic_msgs::KeyValue kv;
			kv.key = "v";
			kv.value = robotID;
			knowledge_update_service.request.knowledge.values.push_back(kv);
			if (!update_knowledge_client.call(knowledge_update_service)) {
		                ROS_ERROR("KCL: (DropActionCorrect) Could not add gripper_empty predicate to the knowledge base.");
			}

			// holding fact
			knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
			knowledge_update_service.request.knowledge.attribute_name = "holding";
			kv.key = "o";
			kv.value = objectID;
			knowledge_update_service.request.knowledge.values.push_back(kv);
			if (!update_knowledge_client.call(knowledge_update_service)) {
               			ROS_ERROR("KCL: (DropActionCorrect) Could not remove holding predicate from the knowledge base.");
			}

			// object_at fact
			knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
			knowledge_update_service.request.knowledge.attribute_name = "object_at";
			knowledge_update_service.request.knowledge.values.clear();
			kv.key = "o";
			kv.value = objectID;
			knowledge_update_service.request.knowledge.values.push_back(kv);
			kv.key = "wp";
			kv.value = wpID;
			knowledge_update_service.request.knowledge.values.push_back(kv);
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (DropActionCorrect) Could not remove object_at predicate to the knowledge base.");
			}
			
			return true;
		}
		return false;
	}

	void RPGraspAction::jointCallback(const sensor_msgs::JointStateConstPtr& msg)
	{
		last_joint_state = *msg;
	}

	void RPGraspAction::waitForArm(const std_msgs::Float64MultiArray& goal_state, float error)
	{
		ros::Rate loop_rate(1);
		while (ros::ok())
		{
			ros::spinOnce();
			loop_rate.sleep();

			// Check whether we have reached the goal location yet.
			bool done = true;
			if (goal_state.data.size() != last_joint_state.position.size())
			{
				ROS_ERROR("KCL (RPGraspAction) Goal state and last_joint_state don't have the same sized array! %zd %zd", goal_state.data.size(), last_joint_state.position.size());
			}
			ROS_INFO("KCL (RPGraspAction) Goal state and last_joint_state the same sized array! %zd %zd", goal_state.data.size(), last_joint_state.position.size());
			for (unsigned int i = 3; i < std::min(goal_state.data.size(), last_joint_state.position.size()); ++i)
			{
				if (i > goal_state.data.size()) continue;
				if (std::abs(goal_state.data[i] - last_joint_state.position[i]) > error)
				{
					ROS_INFO("KCL (RPGraspAction) Joint #%u is %f off target, not done yet!", i, std::abs(goal_state.data[i] - last_joint_state.position[i]));
					done = false;
					break;
				}
			}
			if (done) break;
		}
	}

	bool RPGraspAction::retractArm()
	{
		ROS_INFO("KCL: (RPPerceptionAction) Retract arm\n");
		std_msgs::Float64MultiArray data_arm;
		data_arm.data = last_joint_state.position;
		data_arm.data[3] = 0.7;
		data_arm.data[4] = 1.6;
		data_arm.data[5] = 0;
		data_arm.data[6] = -1.7;
		data_arm.data[7] = -1.8;

		squirrel_manipulation_msgs::JointPtpActionGoal armEndGoal;
		armEndGoal.goal.joints = data_arm;

		ptpActionClient.sendGoal(armEndGoal.goal);
		ROS_INFO("KCL: (RPGraspAction) Goal sent\n");
		ptpActionClient.waitForResult(ros::Duration(30.0));
		ROS_INFO("KCL: (RPGraspAction) Waiting form arm to finish moving...\n");
		sleep(1.0);
		ptpActionClient.sendGoal(armEndGoal.goal);

		waitForArm(data_arm, 0.05f);
		return true;

		if (ptpActionClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
			ROS_INFO("KCL: (RPGraspAction) Arm moved \n");
			return true;
		}else{
			ROS_ERROR("KCL: (RPGraspAction) Arm FAILED to move! \n");
			return false;
		}
	}


	/*
	 * blind grasp action dispatch
	 *	:parameters (?v - robot ?wp - waypoint ?o - object ?t - type)
	 *	:effect (and
	 *		(not (gripper_empty ?v))
	 *		(not (object_at ?o ?wp))
	 *		(holding ?v ?o)
	 */
	bool RPGraspAction::dispatchBlindGraspAction(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		ROS_INFO("KCL: (GraspAction) blind grasp action recieved");

		// get object ID from action dispatch
		std::string robotID;
		std::string objectID;
		bool foundObject = false;
		for(size_t i=0; i<msg->parameters.size(); i++) {
			if(0==msg->parameters[i].key.compare("v"))
				robotID = msg->parameters[i].value;
			if(0==msg->parameters[i].key.compare("o")) {
				objectID = msg->parameters[i].value;
				foundObject = true;
			}
		}
		if(!foundObject) {
			ROS_INFO("KCL: (GraspAction) aborting action dispatch; malformed parameters");
			return false;
		}
		
		// get details from message store
		std::vector< boost::shared_ptr<squirrel_object_perception_msgs::SceneObject> > results;
		if(message_store.queryNamed<squirrel_object_perception_msgs::SceneObject>(objectID, results)) {
			if(results.size()<1) {
				ROS_INFO("KCL: (GraspAction) aborting action dispatch; no matching objectID %s", objectID.c_str());
				return false;
			}
			if(results.size()>1)
				ROS_INFO("KCL: (GraspAction) multiple objects share the same objectID");

			// get pose in odom frame
			geometry_msgs::PoseStamped poseMap;
			poseMap.header = results[0]->header;
			poseMap.pose = results[0]->pose;
/*			tf::TransformListener tfl;
			try {
				tfl.waitForTransform("/odom", "/map", ros::Time::now(), ros::Duration(20.0));
				tfl.transformPose("/odom", poseMap, pose);
			} catch ( tf::TransformException& ex ) {
				ROS_ERROR("%s: error while transforming point: %s", ros::this_node::getName().c_str(), ex.what());
				return false;
			}
*/
			// Get the object pose.
			std::stringstream ss;
			ss << objectID << "_wp";
			std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > object_results;
			if(message_store.queryNamed<geometry_msgs::PoseStamped>(ss.str(), object_results)) {
				if(results.size()<1) {
					ROS_ERROR("KCL: (PerceptionAction) aborting waypoint request; no matching object wp %s", ss.str().c_str());
					publishFeedback(msg->action_id, "action failed");
					return false;
				}
			} else {
				ROS_ERROR("KCL: (PerceptionAction) could not query message store to fetch object wp %s", ss.str().c_str());
				publishFeedback(msg->action_id, "action failed");
				return false;
			}

			// request manipulation waypoints for object
			geometry_msgs::PoseStamped &object_wp = *object_results[0];

			// dispatch Grasp action
			squirrel_manipulation_msgs::BlindGraspGoal goal;
			goal.heap_center_pose = poseMap;
			goal.heap_center_pose_static = object_wp;
			goal.heap_bounding_cylinder = results[0]->bounding_cylinder;
			goal.heap_point_cloud = results[0]->cloud;
			actionlib::SimpleClientGoalState state = actionlib::SimpleClientGoalState::PENDING;
			
			while (ros::ok() && state != actionlib::SimpleClientGoalState::SUCCEEDED && state != actionlib::SimpleClientGoalState::ACTIVE)
			{
				blind_grasp_action_client.sendGoal(goal);
				ROS_INFO("KCL: (GraspAction) goal sent, waiting for result");
				
				// bool finished_before_timeout = false;
				blind_grasp_action_client.waitForResult(ros::Duration(150));
				state = blind_grasp_action_client.getState();

				
				ROS_INFO("KCL: (GraspAction) Returned state: %s (%s)", state.getText().c_str(), state.toString().c_str());
			}
			ROS_INFO("KCL: (GraspAction) action finished: %s", state.toString().c_str());
		
/**
 * TODO For the sorting demo we assume that this always fails!
 */	
			if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
/*

				// gripper_empty fact (remove)
				rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
				knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
				knowledge_update_service.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
				knowledge_update_service.request.knowledge.attribute_name = "gripper_empty";
				diagnostic_msgs::KeyValue kv;
				kv.key = "v";
				kv.value = robotID;
				knowledge_update_service.request.knowledge.values.push_back(kv);
				if (!update_knowledge_client.call(knowledge_update_service)) {
					ROS_ERROR("KCL: (PerceptionAction) Could not remove gripper_empty predicate from the knowledge base.");
				}

				// holding fact	
				knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
				knowledge_update_service.request.knowledge.attribute_name = "holding";
				kv.key = "o";
				kv.value = objectID;
				knowledge_update_service.request.knowledge.values.push_back(kv);
				if (!update_knowledge_client.call(knowledge_update_service)) {
					ROS_ERROR("KCL: (PerceptionAction) Could not add holding predicate to the knowledge base.");
				}

				// object_at fact (remove)
				knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
				knowledge_update_service.request.knowledge.attribute_name = "object_at";
				knowledge_update_service.request.knowledge.values.clear();
				kv.key = "o";
				kv.value = objectID;
				knowledge_update_service.request.knowledge.values.push_back(kv);
				if (!update_knowledge_client.call(knowledge_update_service)) {
					ROS_ERROR("KCL: (PerceptionAction) Could not remove object_at predicate from the knowledge base.");
				}
*/
				// holding fact	
				rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
				knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
				knowledge_update_service.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
				knowledge_update_service.request.knowledge.attribute_name = "holding";
				knowledge_update_service.request.knowledge.is_negative = true;

				diagnostic_msgs::KeyValue kv;
				kv.key = "v";
				kv.value = robotID;
				knowledge_update_service.request.knowledge.values.push_back(kv);
				kv.key = "o";
				kv.value = objectID;
				knowledge_update_service.request.knowledge.values.push_back(kv);
				if (!update_knowledge_client.call(knowledge_update_service)) {
					ROS_ERROR("KCL: (PerceptionAction) Could not add (not (holding %s %s)) predicate to the knowledge base.", robotID.c_str(), objectID.c_str());
				}
				std_srvs::Empty dummy;
				clear_cost_map_client.call(dummy);
			
				return true;

			}// else return false;

		} else {
			ROS_INFO("KCL: (GraspAction) aborting action dispatch; query to sceneDB failed");
			return false;
		}
		return true;
	}
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_interface_grasping");
		ros::NodeHandle nh;


		std::string GraspActionserver, blindGraspActionServer;
		nh.param("blind_grasp_action_server", blindGraspActionServer, std::string("/metahand_grasp_server"));

		// create PDDL action subscriber
		KCL_rosplan::RPGraspAction rpga(nh, blindGraspActionServer);
	
		// listen for action dispatch
		ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPGraspAction::dispatchCallback, &rpga);
		ROS_INFO("KCL: (GraspAction) Ready to receive");

		ros::spin();
		return 0;
	}
