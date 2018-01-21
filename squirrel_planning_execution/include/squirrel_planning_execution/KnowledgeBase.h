#ifndef SQUIRREL_PLANNING_EXECUTION_KNOWLEDGEBASE_H
#define SQUIRREL_PLANNING_EXECUTION_KNOWLEDGEBASE_H

#include <string>
#include <boost/concept_check.hpp>

#include <ros/ros.h>
#include <mongodb_store/message_store.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>
#include <rosplan_knowledge_msgs/KnowledgeItem.h>

/**
 * Utility class that helps updating the knowledge base and getting information form the knowledge base.
 */
namespace KCL_rosplan
{
class KnowledgeBase
{
public:
	
	enum AddUpdateTarget {
		KB_ADD_GOAL = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL,
		KB_ADD_KNOWLEDGE = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE
	};
	
	enum RemoveUpdateTarget { 
		KB_REMOVE_GOAL = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_GOAL,
		KB_REMOVE_KNOWLEDGE = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE
	};
	
	/**
	 * Constructor.
	 * @param nh The node handle.
	 * @param message_store The message store proxy to the MongoDB database.
	 */
	KnowledgeBase(ros::NodeHandle& nh, mongodb_store::MessageStoreProxy& message_store);
	
	/**
	 * Add instance.
	 * @param type The type of the instance.
	 * @param name The name of the instance.
	 * @return True if the instance could be added, false otherwise.
	 */
	bool addInstance(const std::string& type, const std::string& name);
	
	/**
	 * Remove instance.
	 * @param type The type of the instance.
	 * @param name The name of the instance.
	 * @return True if the instance could be removed, false otherwise.
	 */
	bool removeInstance(const std::string& type, const std::string& name);
	
	/**
	 * Add a fact to the knowledge base.
	 * @param predicate The predicate of the new fact.
	 * @param parameters The parameters of the new fact, they need to match the parameters in the PDDL domain.
	 * @param is_true Whether the fact is true or false.
	 * @param target Determines whether this fact is a goal or regular knowledge.
	 * @return True if the fact could be added, false otherwise.
	 */
	bool addFact(const std::string& predicate, const std::map<std::string, std::string>& parameters, bool is_true, AddUpdateTarget target);
	
	/**
	 * Add a fact to the knowledge base.
	 * @param fact The fact to add.
	 * @param target Determines whether this fact is a goal or regular knowledge.
	 * @return True if the fact could be added, false otherwise.
	 */
	bool addFact(const rosplan_knowledge_msgs::KnowledgeItem& fact, AddUpdateTarget target);
	
	/**
	 * Remove a fact from the knowledge base.
	 * @param predicate The predicate of the fact to be removed.
	 * @param parameters The parameters of the fact to be removed, they need to match the parameters in the PDDL domain.
	 * @param is_true Whether the fact is true or false.
	 * @param target Determines whether this fact is a goal or regular knowledge.
	 * @return True if the fact could be removed, false otherwise.
	 */
	bool removeFact(const std::string& predicate, const std::map<std::string, std::string>& parameters, bool is_true, RemoveUpdateTarget target);
	
	/**
	 * Remove a fact from the knowledge base.
	 * @param fact The fact to remove.
	 * @param target Determines whether this fact is a goal or regular knowledge.
	 * @return True if the fact could be removed, false otherwise.
	 */
	bool removeFact(const rosplan_knowledge_msgs::KnowledgeItem& fact, RemoveUpdateTarget target);
	
	/**
	 * Add a function to the knowledge base.
	 * @param predicate The predicate of the new function.
	 * @param parameters The parameters of the new function, they need to match the parameters in the PDDL domain.
	 * @param value The value of this fluent.
	 * @param target Determines whether this fact is a goal or regular knowledge.
	 * @return True if the function could be added, false otherwise.
	 */
	bool addFunction(const std::string& predicate, const std::map<std::string, std::string>& parameters, float value, AddUpdateTarget target);
	
	/**
	 * Remove a function from the knowledge base.
	 * @param predicate The predicate of the function to be removed.
	 * @param parameters The parameters of the function to be removed, they need to match the parameters in the PDDL domain.
	 * @param target Determines whether this fact is a goal or regular knowledge.
	 * @return True if the function could be removed, false otherwise.
	 */
	bool removeFunction(const std::string& predicate, const std::map<std::string, std::string>& parameters, RemoveUpdateTarget target);
	
	/**
	 * Remove all goals.
	 */
	bool removeAllGoals();
	
	/**
	 * Get all facts that are in the knowledge base.
	 * @param store All retreived facts are added to this vector.
	 * @return True if the facts could be retreived, false if something went wrong.
	 */
	bool getAllFacts(std::vector<rosplan_knowledge_msgs::KnowledgeItem>& store);
	
	/**
	 * Check if a fact is true.
	 * @param predicate The predicate of the fact to be removed.
	 * @param parameters The parameters of the fact to be removed, they need to match the parameters in the PDDL domain.
	 * @param is_true Whether the fact is true or false.
	 */
	bool isFactTrue(const std::string& predicate, const std::map<std::string, std::string>& parameters, bool is_true);
	
	/**
	 * Convert a knowledge item to a string.
	 * @param knowledge_item The knowledge item to be converted into a string.
	 * @return The string that represents the knowledge item.
	 */
	std::string toString(const rosplan_knowledge_msgs::KnowledgeItem& knowledge_item) const;
	
private:
	
	/**
	 * Create a fact using the rosplan knowledge message.
	 * @param predicate The predicate of the new fact.
	 * @param parameters The parameters of the new fact, they need to match the parameters in the PDDL domain.
	 * @param is_true Whether the fact is true or false.
	 * @return The rosplan representation of a fact.
	 */
	rosplan_knowledge_msgs::KnowledgeItem createFact(const std::string& predicate, const std::map<std::string, std::string>& parameters, bool is_true);
	
	/**
	 * Create a function using the rosplan knowledge message.
	 * @param predicate The predicate of the new function.
	 * @param parameters The parameters of the new function, they need to match the parameters in the PDDL domain.
	 * @param value The value of this function.
	 * @return The rosplan representation of a function.
	 */
	rosplan_knowledge_msgs::KnowledgeItem createFunction(const std::string& predicate, const std::map<std::string, std::string>& parameters, float value);
	
	ros::NodeHandle* nh_; // The node handle.
	mongodb_store::MessageStoreProxy* message_store_; // The message store.
	
	// All services to modify and query the knowledge base.
	ros::ServiceClient update_knowledge_client_;
	ros::ServiceClient query_knowledge_client_;
	
	ros::ServiceClient get_domain_predicates_client_;
	ros::ServiceClient get_instance_client_;
	ros::ServiceClient get_attribute_client_;
	ros::ServiceClient get_current_goals_client_;
};
};

#endif
