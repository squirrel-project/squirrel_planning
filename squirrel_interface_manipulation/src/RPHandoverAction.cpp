#include "squirrel_interface_manipulation/RPHandoverAction.h"

/* The implementation of RPHandoverAction.h */
namespace KCL_rosplan {

	/* constructor */
	RPHandoverAction::RPHandoverAction(ros::NodeHandle& nh, const std::string& handoveractionserver)
	 : message_store(nh), handover_action_client(handoveractionserver, true)
	 {
		// create the push action client
		ROS_INFO("KCL: (HandoverAction) waiting for action server to start on %s", handoveractionserver.c_str());
		handover_action_client.waitForServer();

		// create the action feedback publisher
		action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);

		// create knowledge base link
		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
	}

	/* action dispatch callback */
	void RPHandoverAction::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		// ignore non-push actions
		if(0==msg->name.compare("give_object")) dispatchGiveAction(msg);
		if(0==msg->name.compare("take_object")) dispatchTakeAction(msg);
	}

	/*
	 * give action dispatch
	 */
	void RPHandoverAction::dispatchGiveAction(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		ROS_INFO("KCL: (HandoverAction) give action recieved");

		// get waypoint ID from action dispatch
		std::string robotID;
		std::string objectID;
		std::string childID;
		std::string waypointID;
		for(size_t i=0; i<msg->parameters.size(); i++) {
			if(0==msg->parameters[i].key.compare("v")) {
				robotID = msg->parameters[i].value;
			}
			if(0==msg->parameters[i].key.compare("o")) {
				objectID = msg->parameters[i].value;
			}
			if(0==msg->parameters[i].key.compare("c")) {
				childID = msg->parameters[i].value;
			}
			if(0==msg->parameters[i].key.compare("wp")) {
				waypointID = msg->parameters[i].value;
			}
		}
		if("" == robotID || "" == objectID || "" == childID || "" == waypointID)
		{
			ROS_ERROR("KCL: (HandoverAction) Give handover, aborting action dispatch; malformed parameters. robot=%s, object=%o, child=%s, waypoint=%s", robotID.c_str(), objectID.c_str(), childID.c_str(), waypointID.c_str());
			return;
		}
		
		// dispatch handover
		squirrel_manipulation_msgs::HandoverGoal goal;
		goal.action_type = "give";
		goal.handover_type = 0;
		handover_action_client.sendGoal(goal);

		// publish feedback (enabled)
		publishFeedback(msg->action_id, "action enabled");
		
		handover_action_client.waitForResult();

		actionlib::SimpleClientGoalState state = handover_action_client.getState();
		ROS_INFO("KCL: (HandoverAction) action finished: %s", state.toString().c_str());
						
		if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {

			rosplan_knowledge_msgs::KnowledgeUpdateService updatePredSrv;
			diagnostic_msgs::KeyValue kv;
			
			// delete (holding ?v ?o)
			kv.key = "v"; kv.value = robotID;
			updatePredSrv.request.knowledge.values.push_back(kv);
			kv.key = "o"; kv.value = objectID;
			updatePredSrv.request.knowledge.values.push_back(kv);
			
			updatePredSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			updatePredSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
			updatePredSrv.request.knowledge.attribute_name = "holding";
			update_knowledge_client.call(updatePredSrv);
			updatePredSrv.request.knowledge.values.clear();
			
			// add (gripper_empty ?v)
			kv.key = "v"; kv.value = robotID;
			updatePredSrv.request.knowledge.values.push_back(kv);
			
			updatePredSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			updatePredSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
			updatePredSrv.request.knowledge.attribute_name = "gripper_empty";
			update_knowledge_client.call(updatePredSrv);
			updatePredSrv.request.knowledge.values.clear();
			
			// add (child_is_holding ?c ?o)
			kv.key = "c"; kv.value = childID;
			updatePredSrv.request.knowledge.values.push_back(kv);
			kv.key = "o"; kv.value = objectID;
			updatePredSrv.request.knowledge.values.push_back(kv);
			
			updatePredSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
			updatePredSrv.request.knowledge.attribute_name = "child_is_holding";
			update_knowledge_client.call(updatePredSrv);
			
			// publish feedback (achieved)
			publishFeedback(msg->action_id, "action achieved");

		} else {

			// publish feedback (failed)
			ROS_ERROR("KCL: (HandoverAction) aborting action dispatch; could not call handover action lib server.");
			publishFeedback(msg->action_id, "action failed");
		}
	}
	
	void RPHandoverAction::dispatchTakeAction(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		ROS_INFO("KCL: (HandoverAction) take action recieved");

		// get waypoint ID from action dispatch
		std::string robotID;
		std::string objectID;
		std::string childID;
		std::string waypointID;
		for(size_t i=0; i<msg->parameters.size(); i++) {
			if(0==msg->parameters[i].key.compare("v")) {
				robotID = msg->parameters[i].value;
			}
			if(0==msg->parameters[i].key.compare("o")) {
				objectID = msg->parameters[i].value;
			}
			if(0==msg->parameters[i].key.compare("c")) {
				childID = msg->parameters[i].value;
			}
			if(0==msg->parameters[i].key.compare("wp")) {
				waypointID = msg->parameters[i].value;
			}
		}
		if("" == robotID || "" == objectID || "" == childID || "" == waypointID)
		{
			ROS_ERROR("KCL: (HandoverAction) Take handover, aborting action dispatch; malformed parameters. robot=%s, object=%o, child=%s, waypoit=%s", robotID.c_str(), objectID.c_str(), childID.c_str(), waypointID.c_str());
			return;
		}
		
		// dispatch handover
		squirrel_manipulation_msgs::HandoverGoal goal;
		goal.action_type = "take";
		goal.handover_type = 0;
		handover_action_client.sendGoal(goal);

		// publish feedback (enabled)
		publishFeedback(msg->action_id, "action enabled");
		
		handover_action_client.waitForResult();

		actionlib::SimpleClientGoalState state = handover_action_client.getState();
		ROS_INFO("KCL: (HandoverAction) action finished: %s", state.toString().c_str());
						
		if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {

			rosplan_knowledge_msgs::KnowledgeUpdateService updatePredSrv;
			diagnostic_msgs::KeyValue kv;
			
			// add (holding ?v ?o)
			kv.key = "v"; kv.value = robotID;
			updatePredSrv.request.knowledge.values.push_back(kv);
			kv.key = "o"; kv.value = objectID;
			updatePredSrv.request.knowledge.values.push_back(kv);
			
			updatePredSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			updatePredSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
			updatePredSrv.request.knowledge.attribute_name = "holding";
			update_knowledge_client.call(updatePredSrv);
			updatePredSrv.request.knowledge.values.clear();
			
			// remove (gripper_empty ?v)
			kv.key = "v"; kv.value = robotID;
			updatePredSrv.request.knowledge.values.push_back(kv);
			
			updatePredSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			updatePredSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
			updatePredSrv.request.knowledge.attribute_name = "gripper_empty";
			update_knowledge_client.call(updatePredSrv);
			updatePredSrv.request.knowledge.values.clear();
			
			// remove (child_is_holding ?c ?o)
			kv.key = "c"; kv.value = childID;
			updatePredSrv.request.knowledge.values.push_back(kv);
			kv.key = "o"; kv.value = objectID;
			updatePredSrv.request.knowledge.values.push_back(kv);
			
			updatePredSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
			updatePredSrv.request.knowledge.attribute_name = "child_is_holding";
			update_knowledge_client.call(updatePredSrv);
			
			// publish feedback (achieved)
			publishFeedback(msg->action_id, "action achieved");

		} else {

			// publish feedback (failed)
			ROS_ERROR("KCL: (HandoverAction) aborting action dispatch; could not call handover action lib server.");
			publishFeedback(msg->action_id, "action failed");
		}
	}
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_interface_handoveraction");
		ros::NodeHandle nh;

		std::string handoveractionserver;
		nh.param("handover_action_server", handoveractionserver, std::string("/handover"));

		// create PDDL action subscriber
		KCL_rosplan::RPHandoverAction rppa(nh, handoveractionserver);
	
		// listen for action dispatch
		ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPHandoverAction::dispatchCallback, &rppa);
		ROS_INFO("KCL: (HandoverAction) Ready to receive");

		ros::spin();
		return 0;
	}
