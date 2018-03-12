#ifndef SQUIRRELPLANNINGEXECUTION_PDDLACTIONS_EXAMINEAREAPDDLCOMMAND_H
#define SQUIRRELPLANNINGEXECUTION_PDDLACTIONS_EXAMINEAREAPDDLCOMMAND_H

#include <ros/ros.h>
#include <rosplan_dispatch_msgs/ActionDispatch.h>
#include <rosplan_knowledge_msgs/GenerateProblemService.h>

namespace KCL_rosplan
{
class KnowledgeBase;

/**
 * An instance of this class gets called whenever the PDDL action 'examine_area' (or variants thereof) is
 * dispatched. It is an action that makes the robot examine all objects in the given area.
 */
class ExamineAreaPDDLAction
{
public:
	
	/**
	 * Constructor.
	 * @param node_handle An existing and initialised ros node handle.
	 * @param kb The knowledge base that contains all facts.
	 */
	ExamineAreaPDDLAction(ros::NodeHandle& node_handle, KCL_rosplan::KnowledgeBase& kb);
	
	/**
	 * Destructor
	 */
	~ExamineAreaPDDLAction();
	
	/**
	 * Called when this action needs to be executed.
	 * @param msg The dispatch message sent by ROSPlan.
	 * @return True if the action was successfull, false otherwise.
	 */
	void dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	
private:
	
	/**
	 * Create a PDDL domain.
	 */
	bool generatePDDLProblemFile(rosplan_knowledge_msgs::GenerateProblemService::Request &req, rosplan_knowledge_msgs::GenerateProblemService::Response &res);

	bool createPDDL();
	
	static std::string g_action_name;			 // The action name as specified in PDDL files.
	
	ros::NodeHandle* node_handle_;				 // The ROS node.
	KCL_rosplan::KnowledgeBase* knowledge_base_; // The knowledge base.
	bool is_simulated_;							 // Whether this action is to be simulated.
	
	ros::Publisher action_feedback_pub_;		 // Publisher that communicates feedback to ROSPlan.
	ros::Subscriber dispatch_sub_;				 // Subscriber to the dispatch topic of ROSPlan.

	std::set<std::string> objects_to_examine_;	 // Objects that are examined this round.
};

};

#endif
