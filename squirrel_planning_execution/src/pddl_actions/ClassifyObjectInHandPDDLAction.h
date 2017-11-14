#ifndef SQUIRRELPLANNINGEXECUTION_PDDLACTIONS_CLASSIFYEOBJETINHANDPDDLCOMMAND_H
#define SQUIRRELPLANNINGEXECUTION_PDDLACTIONS_CLASSIFYEOBJETINHANDPDDLCOMMAND_H

#include <ros/ros.h>
#include <rosplan_dispatch_msgs/ActionDispatch.h>

namespace KCL_rosplan
{

/**
 * An instance of this class gets called whenever the PDDL action 'classify_object_in_hand' (or variants thereof) is
 * dispatched. It is an action that examines an object that is grasped by the robot.
 */
class ClassifyObjectInHandPDDLAction
{
public:
	
	/**
	 * Constructor.
	 * @param node_handle An existing and initialised ros node handle.
	 * @param classification_probability A number between 0 and 1 that determines how likely it is to classify an object.
	 */
	ClassifyObjectInHandPDDLAction(ros::NodeHandle& node_handle, float classification_probability);
	
	/**
	 * Destructor
	 */
	~ClassifyObjectInHandPDDLAction();
	
	/**
	 * Called when this action needs to be executed.
	 * @param msg The dispatch message sent by ROSPlan.
	 * @return True if the action was successfull, false otherwise.
	 */
	void dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	
private:
	float classification_probability_;           // A number between 0 and 1 that determines how likely it is to classify successfully.
	ros::ServiceClient update_knowledge_client_; // Service client to update the knowledge base.
	ros::ServiceClient get_instance_client_;     // Service client to get instances stored by ROSPlan.
	ros::ServiceClient get_attribute_client_;    // Service client to get attributes of instances stored by ROSPlan.
	//ros::ServiceClient query_knowledge_client_;  // Service client to query the knowledge base.
	ros::Publisher action_feedback_pub_;         // Publisher that communicates feedback to ROSPlan.
	ros::Subscriber dispatch_sub_;               // Subscriber to the dispatch topic of ROSPlan.
	
	bool ask_user_input_;                        // If true the user is queried whether a classification action fails or succeeds.
};

};

#endif
