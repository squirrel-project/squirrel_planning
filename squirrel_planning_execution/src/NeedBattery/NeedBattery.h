#ifndef SQUIRREL_PLANNING_EXECUTION_NEED_BATTERY_H
#define SQUIRREL_PLANNING_EXECUTION_NEED_BATTERY_H

#include <ros/ros.h>
#include <mongodb_store/message_store.h>

/**
 * Main class for the SQUIRREL scenario where the robot needs to convince children to give it batteries.
 */
namespace KCL_rosplan
{
class NeedBattery
{
public:
	/**
	 * Constructor.
	 * @param nh The node handle.
	 */
	NeedBattery(ros::NodeHandle& nh);
	
private:
	/**
	 * Setup the initial facts for this problem.
	 */
	void initialiseKnowledgeBase();
	
	/**
	 * Set the goal.
	 */
	bool initialiseGoal();
	
	ros::NodeHandle* nh_;
	mongodb_store::MessageStoreProxy message_store_;
	
	ros::ServiceClient update_knowledge_client_;
	ros::ServiceClient query_knowledge_client_;
	
	ros::ServiceClient get_instance_client_;
	ros::ServiceClient get_attribute_client_;
	ros::ServiceClient get_current_goals_client_;
};

};

#endif
