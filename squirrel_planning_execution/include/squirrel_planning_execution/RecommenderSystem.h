#include <map>
#include <vector>
#include <string>
#include <algorithm> 
#include <functional> 
#include <cctype>
#include <locale>
#include <iostream>

#include <ros/ros.h>
#include <mongodb_store/message_store.h>

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
		 * Create a type.
		 * @param name The name of the new type.
		 * @param paretn The parent of this type (if any).
		 */
		static const Type& createType(const std::string& name, const Type* parent);
		
		/**
		 * Get a type, all types are stored in a global cache.
		 * @param name The name of this type.
		 */
		static const Type& getType(const std::string& name);
		
		/**
		 * Get all types.
		 */
		static void getAllTypes(std::vector<const Type*>& all_types);
		
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
		 * Create an object, all objects are stored in a global cache.
		 * @param name The name of the object.
		 * @param type The type of object.
		 * @return The object that has been created.
		 */
		static const Object& createObject(const std::string& name, const Type& type);
		
		/**
		 * Get an object, all objects are stored in a global cache.
		 * @param name The name of the object.
		 * @return The object with the given name (or NULL if such an object does not exist).
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
	 * Fact prototype.
	 */
	struct FactProto : public std::binary_function<FactProto, FactProto, bool>
	{
		FactProto()
			: p_(NULL), is_false_(true)
		{

		}

		FactProto(const Predicate& p, const std::vector<const Object*>& objects, bool is_false)
			: p_(&p), objects_(objects), is_false_(is_false)
		{

		}

		const Predicate* p_;
		std::vector<const Object*> objects_;
		bool is_false_;

		std::string toString() const
		{
			std::stringstream ss;
			if (p_ != NULL)
				ss << p_->getName() << " ";
			for (std::vector<const Object*>::const_iterator ci = objects_.begin(); ci != objects_.end(); ++ci)
			{
				ss << (*ci)->getName() << " ";
			}
			ss << is_false_;
			return ss.str();
		}

		bool operator()(const FactProto& a, const FactProto& b) const
		{
			return a.toString() < b.toString();
		}
	};
	
	/**
	 * A fact.
	 */
	class Fact
	{
	private:
		Fact(const Predicate& predicate, const std::vector<const Object*>& objects, bool is_negative = false)
			: predicate_(&predicate), objects_(objects), is_negative_(is_negative)
		{
			
		}
		
		const Predicate* predicate_;
		std::vector<const Object*> objects_;
		bool is_negative_;
		static std::map<FactProto, const Fact*, FactProto> generated_facts_;
	public:
		
		/**
		 * Get all facts.
		 */
		static void getAllFacts(std::vector<const Fact*>& all_facts);
		
		/**
		 * Get a fact, all facts are stored in a global cache.
		 * @predicate The predicate.
		 * @objects The set of objects.
		 */
		static const Fact& getFact(const Predicate& predicate, const std::vector<const Object*>& objects, bool is_negative = false);
		
		/** @return The predicate. */
		inline const Predicate& getPredicate() const { return *predicate_; }
		
		/** @return The objects. */
		inline const std::vector<const Object*>& getObjects() const { return objects_; }
		
		/** @return Whether this fact is negative. */
		inline bool isNegative() const { return is_negative_; }
		
		/** @return The fact as a string as it would be represented in an PDDL domain. **/
		std::string getFullForm() const;
		
		/**
		 * Delete all objects ever created.
		 */
		static void cleanup();
	};
	
	std::ostream& operator<<(std::ostream& os, const Fact& fact);
	
	/**
	 * Class that gives the utility of a fact.
	 */
	class UtilityFact
	{
	private:
		const Fact* fact_; // The fact.
		float utility_;    // The utility of this fact.
	public:
		
		/**
		 * @param fact A fact.
		 * @param utility The utility of htis fact.
		 */
		UtilityFact(const Fact& fact, float utility)
			: fact_(&fact), utility_(utility)
		{
			
		}
		
		/**
		 * @return The fact.
		 */
		const Fact& getFact() const { return *fact_; }
		
		/**
		 * @return The utility.
		 */
		float getUtility() const { return utility_; }
		
		/**
		 * Compare the utility of @ref{lhs} to @ref{rhs}.
		 * @param lhs An instance of UtilityFact.
		 * @param rhs An instance of UtilityFact.
		 * @return true if @ref{lhs} has a lower utility than @ref{rhs}, false otherwise.
		 */
		bool operator() (const UtilityFact& lhs, const UtilityFact& rhs)
		{
			return lhs.utility_ < rhs.utility_;
		}
	};
	
	std::ostream& operator<<(std::ostream& os, const UtilityFact& utility_fact);
	
	/**
	 * Capture the order in which facts should be checked.
	 */
	struct FactObserveTree
	{
		FactObserveTree(const Fact& fact_to_observe)
			: fact_to_observe_(&fact_to_observe), true_branch_(NULL), false_branch_(NULL)
		{
			
		}
		
		bool contains(const Fact& fact) const
		{
			if (fact_to_observe_ == &fact)
				return true;
			
			if (true_branch_ != NULL && true_branch_->contains(fact))
				return true;
			
			if (false_branch_ != NULL && false_branch_->contains(fact))
				return true;
			return false;
		}
		
		bool contains(const Object& object) const
		{
			for (std::vector<const Object*>::const_iterator ci = fact_to_observe_->getObjects().begin(); ci != fact_to_observe_->getObjects().end(); ++ci)
			{
				if (&object == *ci)
					return true;
			}
			
			if (true_branch_ != NULL && true_branch_->contains(object))
				return true;
			
			if (false_branch_ != NULL && false_branch_->contains(object))
				return true;
			return false;
		}
		
		const Fact* fact_to_observe_;
		const FactObserveTree* true_branch_;
		const FactObserveTree* false_branch_;
	};
	
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
		ros::ServiceClient get_domain_type_client;
		ros::ServiceClient get_domain_attribute_client;
		
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
		
		/**
		 * Calculate the knowledge increase by performing @ref{sensing_action}.
		 * @param objects The list of objects.
		 * @param predicates The list of predicates and their arity.
		 * @param weighted_facts The list of facts and their probabilities.
		 * @param interesting_facts The list of facts that we care to learn more about.
		 * @param sensing_action The fact that is observed.
		 * @return The pair increase if @ref{sensing_action} is true, and if @ref{sensing_action} is false.
		 */
		std::pair<float, float> calculateKnowledgeIncrease(const std::vector<const Object*>& objects, const std::vector<const Predicate*>& predicates, std::map<const Fact*, float> weighted_facts, const std::vector<const Fact*>& interesting_facts, const Fact& sensing_action);

		/**
		 * Calculate the knowledge increase by performing @ref{sensing_action}.
		 * @param objects The list of objects.
		 * @param predicates The list of predicates and their arity.
		 * @param weighted_facts The list of facts and their probabilities.
		 * @return The output of the recommender.
		 */
		std::map<const Fact*, float> runRecommender(const std::vector<const Object*>& objects, const std::vector<const Predicate*>& predicates, const std::map<const Fact*, float>& weighted_facts);
		
		/**
		 * Calculate the knowldge value of the output of the recommemder.
		 * @param results The output values of the recommenderr.
		 * @param interesting_facts The list of facts that we care to learn more about.
		 * @return The total knowledge given by @ref{interesting_facts}.
		 */
		float calculateKnowledge(const std::map<const Fact*, float>& results, const std::vector<const Fact*>& interesting_facts) const;
		
		/**
		 * Initialise the recommender system.
		 */
		void initialise();
		
		/**
		 * Store the relationships between objects, in which order they need to be observed.
		 * @param new_observed_fact The newly observed fact.
		 * @param true_branch If true then we explore the true branch, otherwise we explore the false branch.
		 */
		//void updateKnowledgeBase(const Fact& last_observed_fact, const Fact& new_observed_fact, bool true_branch);
		
		/**
		 * Update @ref{weighted_facts_copy} by infering mutual exclusive pairs of actions and update their weights
		 * accordingly.
		 */
		void processMutualExclusive(std::map<const Fact*, float>& weighted_facts_copy, const std::vector<const Fact*>& interesting_facts);
		
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
		 * @param max_depth The maximum depth of the tree of facts to observe.
		 * @return A list of facts that give good information gain.
		 */
		std::vector<const Fact*> getBestSensingActions(const std::vector<const Object*>& objects, const std::vector<const Predicate*>& predicates, const std::map<const Fact*, float>& weighted_facts, const std::vector<const Fact*>& interesting_facts, unsigned int max_depth);
		
		/**
		 * Get the best sensing actions to perform given a data set and a weighted list of facts we care about and the last observed fact.
		 * @param node The current node, branches will be added to this node.
		 * @param last_sensed_fact The fact that was last changed.
		 * @param current_depth The current depth.
		 * @param max_depth The max depth before the algorithm terminates.
		 * @param objects The list of objects.
		 * @param predicates The list of predicates and their arity.
		 * @param weighted_facts The list of facts and their probabilities.
		 * @param interesting_facts The list of facts that we care to learn more about.
		 * @param max_depth The maximum depth of the tree of facts to observe.
		 */
		void callRecogniser(FactObserveTree& node, const Fact& last_sensed_fact, unsigned int current_depth, unsigned int max_depth, const std::vector<const Object*>& objects, const std::vector<const Predicate*>& predicates, const std::map<const Fact*, float>& weighted_facts, const std::vector<const Fact*>& interesting_facts);
		
		/**
		 * Visualise the knowledge we have right now.
		 * @param path The path where the output file will be saved.
		 * @param objects The list of objects
		 * @param predicates The list of predicates and their arity.
		 * @param all_facts All the grounded facts.
		 */
		void visualise(const std::string& path, const std::vector<const Object*>& objects, const std::vector<const Predicate*>& predicates, const std::vector<const KCL_rosplan::Fact*>& all_facts, const std::set<const KCL_rosplan::Fact*>& true_facts);
	};
}
#endif
