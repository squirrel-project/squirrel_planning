#ifndef SQUIRRELPLANNINGEXECUTION_PDDLACTIONS_EXPLOREWAYPOINTPDDLACTION_H
#define SQUIRRELPLANNINGEXECUTION_PDDLACTIONS_EXPLOREWAYPOINTPDDLACTION_H

#include <ros/ros.h>
#include <rosplan_dispatch_msgs/ActionDispatch.h>
#include <mongodb_store/message_store.h>

namespace KCL_rosplan
{
class KnowledgeBase;
/**
 * An instance of this class gets called whenever the PDDL action 'goto' (or variants thereof) is
 * dispatched. It is an action that makes the robot move to a certain location.
 */
class ExploreWaypointPDDLAction
{
public:
	
	/**
	 * Constructor.
	 * @param node_handle An existing and initialised ros node handle.
	 * @param message_store The message_store.
	 * @param knowledge_base The knowledge base interface.
	 */
	ExploreWaypointPDDLAction(ros::NodeHandle& node_handle, mongodb_store::MessageStoreProxy& message_store, KnowledgeBase& knowledge_base);
	
	/**
	 * Destructor
	 */
	~ExploreWaypointPDDLAction();
	
	/**
	 * Called when this action needs to be executed.
	 * @param msg The dispatch message sent by ROSPlan.
	 * @return True if the action was successfull, false otherwise.
	 */
	void dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	
private:
	mongodb_store::MessageStoreProxy* message_store_;
	KnowledgeBase* knowledge_base_;
	
	ros::Publisher action_feedback_pub_;         // Publisher that communicates feedback to ROSPlan.
	ros::Subscriber dispatch_sub_;               // Subscriber to the dispatch topic of ROSPlan.
};

};

#endif
