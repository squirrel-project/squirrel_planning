#ifndef SQUIRRELPLANNINGEXECUTION_PDDLACTIONS_TAKEOBJECTPDDLCOMMAND_H
#define SQUIRRELPLANNINGEXECUTION_PDDLACTIONS_TAKEOBJECTPDDLCOMMAND_H

#include <ros/ros.h>
#include <rosplan_dispatch_msgs/ActionDispatch.h>

namespace KCL_rosplan
{

/**
 * An instance of this class gets called whenever the PDDL action 'take_object' (or variants thereof) is
 * dispatched. It is an action take an object from a child.
 */
class TakeObjectPDDLAction
{
public:
	
	/**
	 * Constructor.
	 * @param node_handle An existing and initialised ros node handle.
	 */
	TakeObjectPDDLAction(ros::NodeHandle& node_handle);
	
	/**
	 * Destructor
	 */
	~TakeObjectPDDLAction();
	
	/**
	 * Called when this action needs to be executed.
	 * @param msg The dispatch message sent by ROSPlan.
	 * @return True if the action was successfull, false otherwise.
	 */
	void dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	
private:
	ros::ServiceClient update_knowledge_client_; // Service client to update the knowledge base.
	ros::ServiceClient get_instance_client_;     // Service client to get instances stored by ROSPlan.
	ros::ServiceClient get_attribute_client_;    // Service client to get attributes of instances stored by ROSPlan.
	ros::Publisher action_feedback_pub_;         // Publisher that communicates feedback to ROSPlan.
	ros::Subscriber dispatch_sub_;               // Subscriber to the dispatch topic of ROSPlan.
};

};

#endif
