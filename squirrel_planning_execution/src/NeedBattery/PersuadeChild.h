#ifndef SQUIRREL_PLANNING_EXECUTION_PERSUADE_CHILD_H
#define SQUIRREL_PLANNING_EXECUTION_PERSUADE_CHILD_H

#include <ros/ros.h>
#include <mongodb_store/message_store.h>
#include <rosplan_dispatch_msgs/ActionDispatch.h>

/**
 * Class that deals with the persuade child action. It creates a behaviour for the robot and executes
 * its actions.
 */
namespace KCL_rosplan
{
class PersuadeChild
{
public:
	/**
	 * Constructor.
	 * @param nh The node handle.
	 */
	PersuadeChild(ros::NodeHandle& nh);
	
	/**
	 * Deconstruction.
	 */
	~PersuadeChild();
	
	/**
	 * Callback function for all actions that are dispatched.
	 * @param msg The action that is dispatched.
	 */
	void dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	
private:
	/**
	 * Update goals for this problem.
	 */
	bool initialiseGoal();
	
	ros::NodeHandle* node_handle_;
	mongodb_store::MessageStoreProxy message_store_;
	ros::Publisher action_feedback_pub_;
	ros::Subscriber dispatch_sub_;
	
	ros::ServiceClient update_knowledge_client_;
	ros::ServiceClient query_knowledge_client_;
	
	ros::ServiceClient get_instance_client_;
	ros::ServiceClient get_attribute_client_;
	ros::ServiceClient get_current_goals_client_;
};

};

#endif
