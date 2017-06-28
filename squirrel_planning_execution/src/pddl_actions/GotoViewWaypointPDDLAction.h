#ifndef SQUIRRELPLANNINGEXECUTION_PDDLACTIONS_GOTOVIEWWAYPOINTPDDLCOMMAND_H
#define SQUIRRELPLANNINGEXECUTION_PDDLACTIONS_GOTOVIEWWAYPOINTPDDLCOMMAND_H

#include <ros/ros.h>
#include <rosplan_dispatch_msgs/ActionDispatch.h>
#include "actionlib/client/simple_action_client.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "mongodb_store/message_store.h"

namespace KCL_rosplan
{

/**
 * An instance of this class gets called whenever the PDDL action 'goto_view_waypoint' (or variants thereof) is
 * dispatched. It is an action that makes the robot move to a view waypoint of the nearest box.
 */
class GotoViewWaypointPDDLAction
{
public:
	
	/**
	 * Constructor.
	 * @param node_handle An existing and initialised ros node handle.
	 * @param actionserver The name of the move_base action server.
	 */
	GotoViewWaypointPDDLAction(ros::NodeHandle& node_handle, const std::string &actionserver);
	
	/**
	 * Destructor
	 */
	~GotoViewWaypointPDDLAction();
	
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
	
	mongodb_store::MessageStoreProxy message_store_; // Message store to lookup real data.
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client_; // Action client of move_base
	ros::ServiceClient clear_costmaps_client_;       // Message to clear the costmap.
};

};

#endif
