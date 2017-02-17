#include <map>
#include <vector>
#include <string>
#include <algorithm> 
#include <functional> 
#include <cctype>
#include <locale>
#include <iostream>

#include <ros/ros.h>
#include "mongodb_store/message_store.h"

#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/GetInstanceService.h"
#include "rosplan_knowledge_msgs/GetAttributeService.h"
#include "rosplan_knowledge_msgs/GenerateProblemService.h"
#include "rosplan_knowledge_msgs/KnowledgeQueryService.h"

#include "squirrel_planning_execution/StringUtilityFunctions.h"

#ifndef SQUIRREL_PLANNING_EXECUTION_RECOMMENDER_SYSTEM_H
#define SQUIRREL_PLANNING_EXECUTION_RECOMMENDER_SYSTEM_H

/**
 * An utility class that reads / writes CVS files and invokes the recommender system. Its aim is 
 * to suggest a number of sensing actions that increases the robot's knowledge of the world.
 */
namespace KCL_rosplan {
	
	class Fact;
	
	/**
	 * A type.
	 */
	class Type
	{
	private:
		Type(const std::string& name, const Type* parent)
			: name_(name), parent_(parent)
		{
			
		}
		
		std::string name_;
		const Type* parent_;
		static std::map<std::string, const Type*> generated_types_;
	public:
		/**
		 * Get an object, all objects are stored in a global cache.
		 */
		static const Type& getType(const std::string& name, const Type* parent);
		
		/** @return The name. */
		inline const std::string& getName() const { return name_; }
		
		/** @return The parent. */
		inline const Type* getParent() const { return parent_; }
		
		/** 
		 * Check if @ref{type} is a super type of this type.
		 * @param type The type to check against.
		 * @return True if @ref{type} is a super type of this type, false otherwise.
		 */
		bool isChildOf(const Type& type) const
		{
			const Type* p = this;
			while (p != NULL)
			{
				if (p == &type)
				{
					return true;
				}
				p = p->parent_;
			}
			return false;
		}
		
		/**
		 * Delete all objects ever created.
		 */
		static void cleanup();
	};
	
	/**
	 * An object.
	 */
	class Object
	{
	private:
		Object(const std::string& name, const Type& type)
			: name_(name), type_(&type)
		{
			
		}
		
		std::string name_;
		const Type* type_;
		static std::map<std::string, const Object*> generated_objects_;
	public:
		/**
		 * Get an object, all objects are stored in a global cache.
		 */
		static const Object& getObject(const std::string& name, const Type& type);
		
		/**
		 * Get an object given the name, no new object is created!
		 */
		static const Object* getObject(const std::string& name);
		
		/** @return The name. */
		inline const std::string& getName() const { return name_; }
		
		/** @return The type. */
		inline const Type& getType() const { return *type_; }
		
		/**
		 * @return All objects of a certain type.
		 */
		static void getObjects(const Type& type, std::vector<const Object*>& objects);
		
		/**
		 * Delete all objects ever created.
		 */
		static void cleanup();
	};
	
	/**
	 * A predicate.
	 */
	class Predicate
	{
	private:
		Predicate(const std::string& name, const std::vector<const Type*>& types)
			: name_(name), types_(types)
		{
			
		}
		
		std::string name_;
		std::vector<const Type*> types_;
		static std::map<std::string, const Predicate*> generated_predicates_;
	public:
		/**
		 * Get a predicate, all predicates are stored in a global cache.
		 */
		static const Predicate& getPredicate(const std::string& name, const std::vector<const Type*>& types);
		
		/**
		 * Get a predicate, if a predicate with the given name does not exist, a new one is not generated!
		 */
		static const Predicate* getPredicate(const std::string& name);
		
		/** @return The name. */
		inline const std::string& getName() const { return name_; }
		
		/** @return The arity. */
		inline unsigned int getArity() const { return types_.size(); }
		
		/** @return The types. */
		inline const std::vector<const Type*>& getTypes() const { return types_; }
		
		/**
		 * Ground this predicate.
		 * @param facts All grounded facts.
		 */
		void ground(std::vector<const Fact*>& facts) const;
		
		/**
		 * Delete all objects ever created.
		 */
		static void cleanup();
	};
	
	/**
	 * A fact.
	 */
	class Fact
	{
	private:
		Fact(const Predicate& predicate, const std::vector<const Object*>& objects)
			: predicate_(&predicate), objects_(objects)
		{
			
		}
		
		const Predicate* predicate_;
		std::vector<const Object*> objects_;
		static std::map<std::pair<const Predicate*, std::vector<const Object*> >, const Fact*> generated_facts_;
	public:
		/**
		 * Get a fact, all facts are stored in a global cache.
		 */
		static const Fact& getFact(const Predicate& predicate, const std::vector<const Object*>& objects);
		
		/** @return The predicate. */
		inline const Predicate& getPredicate() const { return *predicate_; }
		
		/** @return The objects. */
		inline const std::vector<const Object*>& getObjects() const { return objects_; }
		
		/**
		 * Delete all objects ever created.
		 */
		static void cleanup();
	};
	
	std::ostream& operator<<(std::ostream& os, const Fact& fact);
	
	class RecommenderSystem
	{

	private:
		ros::NodeHandle* node_handle;
		//mongodb_store::MessageStoreProxy message_store;
		
		/* knowledge service clients */
		ros::ServiceClient update_knowledge_client;
		ros::ServiceClient get_instance_client;
		ros::ServiceClient get_attribute_client;
		ros::ServiceClient query_knowledge_client;
		
		/* Recommender system service. */
		ros::ServiceClient recommender_client;
		
		/* Variables set by the launch file. */
		std::string data_path;    // The data path where the input and output files are stored.
		std::string input_file;   // The file where we write the input file.
		std::string output_file;  // The file where we write the output file.
		std::string absolute_input_file; // data_path/input_file
		std::string absolute_output_file;// data_path/output_file
		
		/* CSV functions. */
		
		/**
		 * Write a CSV file given the set of objects, predicates, and a mapping from facts to their probability of being true. The 
		 * data path we write to is: data_path/output_file.
		 * @param objects The list of objects.
		 * @param predicates The list of predicates and their arity.
		 * @param weighted_facts The list of facts and their probabilities.
		 */
		void writeCSV(const std::vector<const Object*>& objects, const std::vector<const Predicate*>& predicates, const std::map<const Fact*, float>& weighted_facts);
		
		/**
		 * Read a CSV file and return the probabilities. The data path we read from is: data_path/input_file.
		 * @param results The map where the probabilities are stored in.
		 * @param predicates The predicates are not written by the recommender, so they need to be provided.
		 */
		void readCSV(std::map<const Fact*, float>& results, const std::vector<const Predicate*>& predicates);
		
	public:

		/* constructor */
		RecommenderSystem(ros::NodeHandle &nh);
		
		/* destructor */
		~RecommenderSystem();
		
		/**
		 * Get the best sensing actions to perform given a data set and a weighted list of facts we care about.
		 * @param objects The list of objects.
		 * @param predicates The list of predicates and their arity.
		 * @param weighted_facts The list of facts and their probabilities.
		 * @param interesting_facts The list of facts that we care to learn more about.
		 * @return A list of facts that give good information gain.
		 */
		std::vector<const Fact*> getBestSensingActions(const std::vector<const Object*>& objects, const std::vector<const Predicate*>& predicates, const std::map<const Fact*, float>& weighted_facts, const std::vector<const Fact*>& interesting_facts);
	};
}
#endif
