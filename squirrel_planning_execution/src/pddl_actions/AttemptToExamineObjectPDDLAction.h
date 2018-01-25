#ifndef SQUIRRELPLANNINGEXECUTION_PDDLACTIONS_ATTEMPTTOEXAMINEOBJECTPDDLACTION_H
#define SQUIRRELPLANNINGEXECUTION_PDDLACTIONS_ATTEMPTTOEXAMINEOBJECTPDDLACTION_H

#include <ros/ros.h>
#include <rosplan_dispatch_msgs/ActionDispatch.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <mongodb_store/message_store.h>

namespace KCL_rosplan
{
class KnowledgeBase;

/**
 * An instance of this class gets called whenever the PDDL action 'classify_object' (or variants thereof) is
 * dispatched. It is an action that makes the robot move to a certain location.
 */
class AttemptToExamineObjectPDDLAction
{
public:
	
	/**
	 * Constructor.
	 * @param node_handle An existing and initialised ros node handle.
	 * @param knowledge_base The interface to the knowledge base.
	 * @param message_store The message store.
	 */
	AttemptToExamineObjectPDDLAction(ros::NodeHandle& node_handle, KnowledgeBase& knowledge_base, mongodb_store::MessageStoreProxy& message_store);
	
	/**
	 * Destructor
	 */
	~AttemptToExamineObjectPDDLAction();
	
	/**
	 * Called when this action needs to be executed.
	 * @param msg The dispatch message sent by ROSPlan.
	 * @return True if the action was successfull, false otherwise.
	 */
	void dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	
private:
	
	/**
	 * Move to the given location.
	 */
	bool moveTo(const std::string& wp);
	
	KnowledgeBase* knowledge_base_;              // The knowledge base interface.
	ros::Publisher action_feedback_pub_;         // Publisher that communicates feedback to ROSPlan.
	ros::Publisher dispatch_pub_;                // Publisher of dummy actions.
	ros::Subscriber dispatch_sub_;               // Subscriber to the dispatch topic of ROSPlan.
	
	ros::ServiceClient clear_costmaps_client;    // Clear up the costmap before moving.
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client_;
	mongodb_store::MessageStoreProxy* message_store_;
};

};

#endif
