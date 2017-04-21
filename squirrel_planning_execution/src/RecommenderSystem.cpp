#include <sstream>
#include <fstream>
#include <queue>
#include <set>
#include <vector>
#include <sstream>
#include <time.h>
#include <tf/tf.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <actionlib/client/simple_action_client.h>
#include <rosplan_planning_system/PlanningEnvironment.h>

#include <rosplan_knowledge_msgs/DomainFormula.h>
#include <rosplan_knowledge_msgs/KnowledgeItem.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>
#include <rosplan_knowledge_msgs/GetInstanceService.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_knowledge_msgs/GenerateProblemService.h>
#include <rosplan_knowledge_msgs/KnowledgeQueryService.h>
#include <rosplan_knowledge_msgs/GetDomainTypeService.h>
#include <rosplan_knowledge_msgs/GetDomainAttributeService.h>
#include <rosplan_dispatch_msgs/PlanAction.h>

#include <rosplan_knowledge_msgs/GenerateProblemService.h>

#include "pddl_actions/ListenToFeedbackPDDLAction.h"
#include "pddl_actions/InspectObjectPDDLAction.h"
#include "squirrel_prediction_msgs/RecommendRelations.h"
#include "squirrel_planning_execution/PlanToSensePDDLGenerator.h"
#include "squirrel_planning_execution/PlanToAskPDDLGenerator.h"

#include "squirrel_planning_execution/RecommenderSystem.h"

//#define RECOMMENDER_SYSTEM_DEBUG
namespace KCL_rosplan {

	/**
	 * Types.
	 */
	std::map<std::string, const Type*> Type::generated_types_;
	
	const Type& Type::createType(const std::string& name, const Type* parent)
	{
		const Type* t = NULL;
		std::map<std::string, const Type*>::const_iterator mi = generated_types_.find(name);
		if (mi == generated_types_.end())
		{
			t = new Type(name, parent);
			generated_types_[name] = t;
		}
		else
		{
			t = (*mi).second;
		}
		
		// Sanity check, make sure the parents are the same.
		if (parent != t->parent_)
		{
			ROS_ERROR("The types for %s is %s, but the given parent is %s. Stopping!", name.c_str(), t->parent_ == NULL ? "" : t->parent_->getName().c_str(), parent == NULL ? "" : parent->getName().c_str());
			exit(-1);
		}
		ROS_INFO("KCL: (RecommenderSystem) Created the type %s (super type=%s)\n", name.c_str(), (parent == NULL ? "NULL" : parent->getName().c_str()));
		return *t;
	}
	
	const Type& Type::getType(const std::string& name)
	{
		const Type* t = NULL;
		std::map<std::string, const Type*>::const_iterator mi = generated_types_.find(name);
		if (mi == generated_types_.end())
		{
			ROS_ERROR("KCL: (RecommanderSystem) Could not find the type with the name %s.\n", name.c_str());
			exit(1);
		}
		return *(*mi).second;
	}
	
	void Type::getAllTypes(std::vector<const Type*>& all_types)
	{
		for (std::map<std::string, const Type*>::const_iterator ci = generated_types_.begin(); ci != generated_types_.end(); ++ci)
		{
			all_types.push_back(ci->second);
		}
	}
	
	void Type::cleanup()
	{
		for (std::map<std::string, const Type*>::iterator i = generated_types_.begin(); i != generated_types_.end(); ++i)
		{
			delete (*i).second;
		}
	}
	
	/**
	 * Objects.
	 */
	std::map<std::string, const Object*> Object::generated_objects_;
	
	const Object& Object::createObject(const std::string& name, const Type& type)
	{
		const Object* o = getObject(name);
		if (o == NULL)
		{
			o = new Object(name, type);
			generated_objects_[name] = o;
		}
		
		// Check if we need to update the type of object.
		if (type.isChildOf(o->getType()))
		{
			Object* no = const_cast<Object*>(o);
			no->type_ = &type;
		}
		// Saniny check.
		else if (!o->type_->isChildOf(type) && o->type_ != &type)
		{
			ROS_ERROR("Requested the object %s with type %s, but an existing object with the same name exists that has the type %s. Stopping!", name.c_str(), type.getName().c_str(), o->type_->getName().c_str());
			exit(-1);
		}
		ROS_INFO("KCL: (RecommenderSystem) Created the object %s (type=%s)\n", name.c_str(), type.getName().c_str());
		return *o;
	}
	
	const Object* Object::getObject(const std::string& name)
	{
		Object* o = NULL;
		std::map<std::string, const Object*>::const_iterator mi = generated_objects_.find(name);
		if (mi == generated_objects_.end())
		{
			return NULL;
			
		}
		return (*mi).second;
	}
	
	void Object::getObjects(const Type& type, std::vector<const Object*>& objects)
	{
		for (std::map<std::string, const Object*>::const_iterator ci = generated_objects_.begin(); ci != generated_objects_.end(); ++ci)
		{
			const Object* o = (*ci).second;
			if (type.isChildOf(*o->type_))
			{
				objects.push_back(o);
			}
		}
	}
	
	void Object::cleanup()
	{
		for (std::map<std::string, const Object*>::iterator i = generated_objects_.begin(); i != generated_objects_.end(); ++i)
		{
			delete (*i).second;
		}
	}
	
	/**
	 * Predicates.
	 */
	std::map<std::string, const Predicate*> Predicate::generated_predicates_;
	
	/**
	 * Get a predicate, all predicates are stored in a global cache.
	 */
	const Predicate& Predicate::getPredicate(const std::string& name, const std::vector<const Type*>& types)
	{
		const Predicate* p = getPredicate(name);
		if (p == NULL)
		{
			p = new Predicate(name, types);
			generated_predicates_[name] = p;
		}
		
		// Sanity check, it's illegal in PDDL to have predicates with the same name but seperate arity. The types
		// must also match
		if (types.size() != p->types_.size())
		{
			ROS_ERROR("Predicate detected with different arity! Predicate: %s, expected arity %lu but given arity %lu. Stopping!\n", name.c_str(), p->types_.size(), types.size());
			exit(-1);
			for (unsigned int i = 0; i < types.size(); ++i)
			{
				if (types[i] != p->types_[i])
				{
					ROS_ERROR("Predicate detected with different types! Predicate: %s, has type %s at index %ud but given type %s. Stopping!\n", name.c_str(), p->types_[i]->getName().c_str(), i, types[i]->getName().c_str());
					exit(-1);
				}
			}
		}
		return *p;
	}
	
	/**
	 * Get a predicate, if a predicate with the given name does not exist, a new one is not generated!
	 */
	const Predicate* Predicate::getPredicate(const std::string& name)
	{
		Predicate* p = NULL;
		std::map<std::string, const Predicate*>::const_iterator mi = generated_predicates_.find(name);
		if (mi == generated_predicates_.end())
		{
			return NULL;
		}
		
		return (*mi).second;
	}
	
	void Predicate::ground(std::vector<const Fact*>& facts) const
	{
		unsigned int counter[getArity()];
		std::vector<std::vector<const Object*> > objects;
		for (unsigned int i = 0; i < getArity(); ++i)
		{
			counter[i] = 0;
			
			std::vector<const Object*> objects_of_type;
			Object::getObjects(*types_[i], objects_of_type);
			objects.push_back(objects_of_type);
		}
		
		bool done = false;
		while (!done)
		{
			done = true;
			std::vector<const Object*> fact_objects;
			for (unsigned int i = 0; i < getArity(); ++i)
			{
				fact_objects.push_back(objects[i][counter[i]]);
			}
			facts.push_back(&Fact::getFact(*this, fact_objects));
			
			for (unsigned int i = 0; i < getArity(); ++i)
			{
				if (counter[i] + 1 == objects[i].size())
				{
					counter[i] = 0;
				}
				else 
				{
					counter[i] = counter[i] + 1;
					done = false;
					break;
				}
			}
		}
	}
	
	void Predicate::cleanup()
	{
		for (std::map<std::string, const Predicate*>::iterator i = generated_predicates_.begin(); i != generated_predicates_.end(); ++i)
		{
			delete (*i).second;
		}
	}
	
	/**
	 * Facts.
	 */
	std::map<FactProto, const Fact*, FactProto> Fact::generated_facts_;
	
	
	const Fact& Fact::getFact(const Predicate& predicate, const std::vector<const Object*>& objects, bool is_negative)
	{
		// Sanity check, make sure the number of objects matches the predicate's arity.
		if (objects.size() != predicate.getArity())
		{
			ROS_ERROR("We cannot create a fact with %lu facts, because the predicate %s has an arity of %u. Stopping!\n", objects.size(), predicate.getName().c_str(), predicate.getArity());
			exit(-1);
		}
		
		const Fact* f = NULL;
		FactProto proto(predicate, objects, is_negative);
		std::map<FactProto, const Fact*>::const_iterator mi = generated_facts_.find(proto);
		if (mi == generated_facts_.end())
		{
			f = new Fact(predicate, objects, is_negative);
			generated_facts_[proto] = f;
			return *f;
		}
		f = (*mi).second;
		return *f;
	}
	
	void Fact::getAllFacts(std::vector<const Fact*>& all_facts)
	{
		for (std::map<FactProto, const Fact*>::iterator i = generated_facts_.begin(); i != generated_facts_.end(); ++i)
		{
			all_facts.push_back(i->second);
		}
	}
	
	std::string Fact::getFullForm() const
	{
		std::stringstream ss;
		if (is_negative_)
			ss << "not ";
		ss << predicate_->getName();
		for (std::vector<const Object*>::const_iterator ci = objects_.begin(); ci != objects_.end(); ++ci)
		{
			const Object* o = *ci;
			ss << " " << o->getName();
		}
		return ss.str();
	}
	
	/**
	 * Delete all objects ever created.
	 */
	void Fact::cleanup()
	{
		for (std::map<FactProto, const Fact*>::iterator i = generated_facts_.begin(); i != generated_facts_.end(); ++i)
		{
			delete (*i).second;
		}
	}
	
	std::ostream& operator<<(std::ostream& os, const Fact& fact)
	{
		if (fact.isNegative())
			os << "(not ";
		os << "(" << fact.getPredicate().getName();
		for (std::vector<const Object*>::const_iterator ci = fact.getObjects().begin(); ci != fact.getObjects().end(); ++ci)
		{
			os << " " << (*ci)->getName();
		}
		os << ")";
		if (fact.isNegative())
			os << ")";
		return os;
	}
	
	std::ostream& operator<<(std::ostream& os, const UtilityFact& utility_fact)
	{
		os << "[" << utility_fact.getUtility() << "] " << utility_fact.getFact();
		return os;
	}
	
	/*-------------*/
	/* constructor */
	/*-------------*/
	
	RecommenderSystem::RecommenderSystem(ros::NodeHandle &nh)
		: node_handle(&nh)//, message_store(nh)
	{
		// knowledge interface
		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
		query_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeQueryService>("/kcl_rosplan/query_knowledge_base");
		
		get_instance_client = nh.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_current_instances");
		get_attribute_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_knowledge");
		
		get_domain_type_client = nh.serviceClient<rosplan_knowledge_msgs::GetDomainTypeService>("/kcl_rosplan/get_domain_types");
		get_domain_attribute_client = nh.serviceClient<rosplan_knowledge_msgs::GetDomainAttributeService>("/kcl_rosplan/get_domain_predicates");
		
		// Initialise the domain.
		//initialise();
		
		// Recommender interface.
		recommender_client = nh.serviceClient<squirrel_prediction_msgs::RecommendRelations>("/squirrel_relations_prediction");
		
		// Load file parameters.
		nh.getParam("/squirrel_planning_execution/recommender/data_path", data_path);
		nh.getParam("/squirrel_planning_execution/recommender/input_file", input_file);
		nh.getParam("/squirrel_planning_execution/recommender/output_file", output_file);
		
		std::stringstream ss;
		ss << data_path << "/" << input_file;
		absolute_input_file = ss.str();
		ss.str(std::string());
		ss << data_path << "/" << output_file;
		absolute_output_file = ss.str();
#ifdef RECOMMENDER_SYSTEM_DEBUG
		ROS_INFO("KCL: (RecommenderSystem) Running! Input file: %s; Output file: %s.\n",  absolute_input_file.c_str(), absolute_output_file.c_str());
#endif
	}
	
	RecommenderSystem::~RecommenderSystem()
	{
		Fact::cleanup();
		Predicate::cleanup();
		Object::cleanup();
	}
	
	void RecommenderSystem::initialise()
	{
		ROS_INFO("KCL: (RecommenderSystem) Initialise.\n");
		// Get all the types.
		rosplan_knowledge_msgs::GetDomainTypeService typeSrv;
		if (!get_domain_type_client.call(typeSrv))
		{
			ROS_ERROR("KCL: (RecommanderSystem) Failed to call the get_domain_type_client.\n");
			exit(1);
		}
		std::vector<std::string> types = typeSrv.response.types;
		std::vector<std::string> super_types = typeSrv.response.super_types;
		
		ROS_INFO("KCL: (RecommenderSystem) Found %zd types and %zd super types..\n", types.size(), super_types.size());
		
		// Create all the types.
		std::set<std::string> processed_types;
		while (processed_types.size() < types.size())
		{
			for (unsigned int i = 0; i < types.size(); ++i)
			{
				if ((super_types[i] == "" || processed_types.count(super_types[i]) == 1) && processed_types.count(types[i]) == 0)
				{
					Type::createType(types[i], &Type::getType(super_types[i]));
					processed_types.insert(types[i]);
				}
			}
		}
		
		std::vector<const Type*> all_types;
		Type::getAllTypes(all_types);
		for(std::vector<const Type*>::const_iterator ci = all_types.begin(); ci != all_types.end(); ++ci)
		{
			const Type* type = *ci;

			// Fetch all instances of this type.
			rosplan_knowledge_msgs::GetInstanceService instanceSrv;
			instanceSrv.request.type_name = type->getName();
			if (!get_instance_client.call(instanceSrv))
			{
				ROS_ERROR("KCL: (RecommanderSystem) Could not call the instance server.\n");
				exit(-1);
			}
			
			for(std::vector<std::string>::iterator iit = instanceSrv.response.instances.begin(); iit!=instanceSrv.response.instances.end(); iit++)
			{
				Object::createObject(*iit, *type);
			}
		}
		
		// Get all the predicates.
		rosplan_knowledge_msgs::GetDomainAttributeService predSrv;
		if (!get_domain_attribute_client.call(predSrv))
		{
			ROS_ERROR("Could not call the get_domain_attribute_client.\n");
			exit(1);
		}
		
		for (std::vector<rosplan_knowledge_msgs::DomainFormula>::const_iterator ci = predSrv.response.items.begin(); ci != predSrv.response.items.end(); ++ci)
		{
			const std::string& name = ci->name;
			const std::vector<diagnostic_msgs::KeyValue>& parameters = ci->typed_parameters;
			
			std::vector<const Type*> types;
			for (std::vector<diagnostic_msgs::KeyValue>::const_iterator p_ci = parameters.begin(); p_ci != parameters.end(); ++p_ci)
			{
				types.push_back(&Type::getType(p_ci->value));
			}
			
			// Create the predicate and ground it to get all the facts.
			const Predicate& predicate = Predicate::getPredicate(name, types);
			std::vector<const Fact*> grounded_facts;
			predicate.ground(grounded_facts);
		}
	}
	
	void RecommenderSystem::writeCSV(const std::vector<const Object*>& objects, const std::vector<const Predicate*>& predicates, const std::map<const Fact*, float>& weighted_facts)
	{
#ifdef RECOMMENDER_SYSTEM_DEBUG
		ROS_INFO("KCL: (RecommenderSystem) Write CSV.\n");
#endif
		std::ofstream ofs;
		ofs.open(absolute_input_file.c_str());
		
		// Write header.
		ofs << "'object1','object2'";
		for (std::vector<const Predicate*>::const_iterator ci = predicates.begin(); ci != predicates.end(); ++ci)
		{
			ofs << ",'" << (*ci)->getName() << "'";
		}
		
		// Recommender specific.
		ofs<< ",'second_is_box'";
		
		ofs << "," << std::endl;
		
		for (std::vector<const Object*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
		{
			const Object* o1 = *ci;
			for (std::vector<const Object*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
			{
				const Object* o2 = *ci;
				ofs << "'" << o1->getName() << "'" << ",'" << o2->getName() << "'";
				
				for (std::vector<const Predicate*>::const_iterator ci = predicates.begin(); ci != predicates.end(); ++ci)
				{
					const Predicate* predicate = *ci;
					
					// Try to create a fact with this predicate and the given objects. If this is impossible, mark it as
					// a '0' and move on.
					std::vector<const Object*> objects;
					if (predicate->getArity() > 0)
					{
						if (o1->getType().isChildOf(*predicate->getTypes()[0]))
						{
							objects.push_back(o1);
						}
						else
						{
							ofs <<",1";
							continue;
						}
					}
					if (predicate->getArity() > 1)
					{
						if (o2->getType().isChildOf(*predicate->getTypes()[1]))
						{
							objects.push_back(o2);
						}
						else
						{
							ofs <<",1";
							continue;
						}
					}
					
					// A fact can be created given these objects.
					const Fact& fact = Fact::getFact(*predicate, objects);
					std::map<const Fact*, float>::const_iterator mi = weighted_facts.find(&fact);
					
					// Check if we know the probability of this fact being true, otherwise leave it empty.
					if (mi == weighted_facts.end())
					{
						ofs << ",1";
					}
					else
					{
						if (mi->second == 0)
							ofs << ",";
						else
							ofs << "," << mi->second;
					}
				}
				if (o2->getType().getName() == "box")
				{
					ofs << ",2";
				}
				else
				{
					ofs << ",1";
				}
				ofs << "," << std::endl;
			}
		}
		ofs.close();
#ifdef RECOMMENDER_SYSTEM_DEBUG
		ROS_INFO("KCL: (RecommenderSystem) Finished writing CVS.\n");
#endif
	}
		
	void RecommenderSystem::readCSV(std::map<const Fact*, float>& results, const std::vector<const Predicate*>& predicates)
	{
#ifdef RECOMMENDER_SYSTEM_DEBUG
		ROS_INFO("KCL: (RecommenderSystem) Read CSV.\n");
#endif
		// Read the header.
		std::ifstream file(absolute_output_file.c_str());
		std::string line;
		unsigned int line_nr = 0;
		unsigned int max_arity = 0;
		
		// Read the header.
		//std::vector<const Predicate*> predicates;
		if (std::getline(file, line))
		{
#ifdef RECOMMENDER_SYSTEM_DEBUG
			ROS_INFO("Process the line: %s\n", line.c_str());
#endif
			std::vector<std::string> tokens = split(line, ',');
			
			// Skip the first few tokens as they refer to the object names.
			for (unsigned int i = 0; i < tokens.size(); ++i)
			{
				if (tokens[i].size() == 0)
				{
					continue;
				}
				
				const std::string& name = tokens[i].substr(1, tokens[i].size() - 2);
#ifdef RECOMMENDER_SYSTEM_DEBUG
				ROS_INFO("Process the token: %s\n", name.c_str());
#endif
				std::string name_trimmed = trimCopy(name);
				if (name_trimmed.substr(0, 6) == "object" && name_trimmed.size() > 6 && 
				   (::atoi(name_trimmed.substr(6).c_str()) != 0 || name_trimmed == "object0"))
				{
					++max_arity;
#ifdef RECOMMENDER_SYSTEM_DEBUG
					ROS_INFO("Object found %s.\n", name_trimmed.c_str());
#endif
					continue;
				}
				
				//const Predicate* p = Predicate::getPredicate(name_trimmed);
				//if (p == NULL)
				//{
				//	ROS_ERROR("Could not find the predicate: %s. Stopping!", name_trimmed.c_str());
				//	exit(-1);
				//}
			}
		}
		
		// Read the properties and store the confidence values.
		while (std::getline(file, line))
		{
#ifdef RECOMMENDER_SYSTEM_DEBUG
			ROS_INFO("Process the line: %s\n", line.c_str());
#endif
			std::vector<std::string> tokens = split(line, ',');
			std::vector<const Object*> objects;
			
			// Fetch the objects from the file.
			for (unsigned int i = 0; i < max_arity; ++i)
			{
				if (tokens[i].size() == 0)
				{
					continue;
				}
				
				const std::string& name = tokens[i].substr(1, tokens[i].size() - 2);
				std::string name_trimmed = trimCopy(name);
				
				// Find the object with the given name.
				const Object* o = Object::getObject(name_trimmed);
				if (o == NULL)
				{
					ROS_ERROR("Could not find the object: %s. Stopping!", name_trimmed.c_str());
					exit(-1);
				}
#ifdef RECOMMENDER_SYSTEM_DEBUG
				ROS_INFO("Added the object: %s.", name_trimmed.c_str());
#endif
				objects.push_back(o);
			}
#ifdef RECOMMENDER_SYSTEM_DEBUG
			std::cout << "Create the facts: " << predicates.size() << ", " << max_arity << ", " << tokens.size() << std::endl;
#endif

			// Get the actual values of these facts.
			std::vector<int> value;
			for (unsigned int i = max_arity; i < max_arity + predicates.size() + 2; ++i)
			{
#ifdef RECOMMENDER_SYSTEM_DEBUG
				std::cout << "True values: Process value: " << tokens[i] << std::endl;
#endif
				value.push_back(::atoi(tokens[i].c_str()));
			}
#ifdef RECOMMENDER_SYSTEM_DEBUG
			std::cout << "Found " << value.size() << " values!" << std::endl;
			std::cout << "Found " << tokens.size() << " tokens!" << std::endl;
#endif			

			// Create the facts and read the values.
//			for (unsigned int i = max_arity + predicates.size() + 2; i < predicates.size() + predicates.size() + max_arity; ++i)
			unsigned int fact_nr = 0;
			for (unsigned int i = max_arity + predicates.size() + 2; i < predicates.size() + predicates.size() + max_arity + 2; ++i)
			{
				float confidence = ::atof(tokens[i].c_str());
				const Predicate* p = predicates[i - max_arity - predicates.size() - 2];
#ifdef RECOMMENDER_SYSTEM_DEBUG
				std::cout << "Fact values: Process value: " << tokens[i] << " - Predicate: " << p->getName() << std::endl;
#endif
				std::vector<const Object*> o;
				for (unsigned int j = 0; j < p->getArity(); ++j)
				{
					o.push_back(objects[j]);
				}
				
				const Fact& fact = Fact::getFact(*p, o, value[fact_nr] != 2);

				results[&fact] = confidence;
#ifdef RECOMMENDER_SYSTEM_DEBUG
				std::cout << fact << "; p=" << confidence << "; value=" << value[fact_nr] << std::endl;
#endif
				++fact_nr;
			}
		}
	}
	
	float RecommenderSystem::calculateKnowledge(const std::map<const Fact*, float>& results, const std::vector<const Fact*>& interesting_facts) const
	{
#ifdef RECOMMENDER_SYSTEM_DEBUG
		ROS_INFO("KCL: (RecommenderSystem) RecommenderSystem::calculateKnowledge");
#endif
		// Check the current values of the interesting facts. This forms the baseline for determining which observation
		// actions are best to improve these.
		float value = 0;
		
		std::vector<const Fact*> facts_to_sense;
		for (std::map<const Fact*, float>::const_iterator ci = results.begin(); ci != results.end(); ++ci)
		{
			const Fact* fact = (*ci).first;
			if (fact->getPredicate().getName() != "belongs_in") continue;
			if (fact->getObjects()[0]->getType().getName() != "object") continue;
			if (fact->getObjects()[1]->getType().getName() != "box") continue;
			float p = (*ci).second;
			
			for (unsigned int i = 0; i < interesting_facts.size(); ++i)
			{
				if (interesting_facts[i] == fact)
				{
					value += p;
#ifdef RECOMMENDER_SYSTEM_DEBUG
					std::cout << "\t" << *fact << " -- Increase knowledge gain by " << p << " total value: " << value << std::endl;
#endif
				}
			}
		}
		return value;
	}
	
	std::map<const Fact*, float> RecommenderSystem::runRecommender(const std::vector<const Object*>& objects, const std::vector<const Predicate*>& predicates, const std::map<const Fact*, float>& weighted_facts)
	{
#ifdef RECOMMENDER_SYSTEM_DEBUG
		ROS_INFO("KCL: (RecommenderSystem) Run recommender.");
#endif
		writeCSV(objects, predicates, weighted_facts);
		
		squirrel_prediction_msgs::RecommendRelations rr;
		rr.request.data_path = data_path;
		rr.request.input_file = input_file;
		rr.request.output_file = output_file;
		rr.request.number_of_columns = predicates.size() + 2;
		
		
		if (!recommender_client.call(rr))
		{
			ROS_ERROR("Could not call the recommender system. Stopping!\n");
			exit(-1);
		}
		
		if (!rr.response.finished)
		{
			ROS_ERROR("The recommender system did not reported it finished, something went wrong? Stopping!\n");
			exit(-1);
		}
		
		std::map<const Fact*, float> results;
		readCSV(results, predicates);
		
#ifdef RECOMMENDER_SYSTEM_DEBUG
		for (std::map<const Fact*, float>::const_iterator ci = results.begin(); ci != results.end(); ++ci)
		{
			const Fact* fact = ci->first;
			float p = ci->second;
			std::cout << "KCL: (RecommenderSystem) " << *fact << " = " << p << std::endl;
		}
#endif
		return results;
	}
	
	std::pair<float, float> RecommenderSystem::calculateKnowledgeIncrease(const std::vector<const Object*>& objects, const std::vector<const Predicate*>& predicates, std::map<const Fact*, float> weighted_facts, const std::vector<const Fact*>& interesting_facts, const Fact& sensing_action)
	{
		// 0 = unknown
		// 1 = false
		// 2 = true
		
		float knowledge_value_baseline = calculateKnowledge(weighted_facts, interesting_facts);
		
//#ifdef RECOMMENDER_SYSTEM_DEBUG
		ROS_INFO("KCL: (RecommenderSystem) Simulate fact to be true.");
//#endif
		std::map<const Fact*, float> weighted_facts_copy = weighted_facts;
		weighted_facts_copy[&sensing_action] = 2;
		processMutualExclusive(weighted_facts_copy, interesting_facts);
		std::cout << "Pre recog..." << std::endl;
		for (std::map<const Fact*, float>::const_iterator ci = weighted_facts_copy.begin(); ci != weighted_facts_copy.end(); ++ci)
		{
			if (ci->first->getPredicate().getName() == "belongs_in" && ci->first->getObjects()[0]->getType().getName() == "object" && ci->first->getObjects()[1]->getType().getName() == "box")
				std::cout << *ci->first << " = " << ci->second << std::endl;
		}
		std::map<const Fact*, float> results = runRecommender(objects, predicates, weighted_facts_copy);
		std::cout << "Post recog..." << std::endl;
		for (std::map<const Fact*, float>::const_iterator ci = results.begin(); ci != results.end(); ++ci)
		{
			if (ci->first->getPredicate().getName() == "belongs_in" && ci->first->getObjects()[0]->getType().getName() == "object" && ci->first->getObjects()[1]->getType().getName() == "box")
				std::cout << *ci->first << " = " << ci->second << std::endl;
		}
		
		float knowledge_value_if_true = calculateKnowledge(results, interesting_facts) - knowledge_value_baseline;
//#ifdef RECOMMENDER_SYSTEM_DEBUG
		ROS_INFO("KCL: (RecommenderSystem) Simulate fact to be false.");
//#endif
		weighted_facts_copy = weighted_facts;
		weighted_facts_copy[&sensing_action] = 1;
		processMutualExclusive(weighted_facts_copy, interesting_facts);
		std::cout << "Pre recog..." << std::endl;
		for (std::map<const Fact*, float>::const_iterator ci = weighted_facts_copy.begin(); ci != weighted_facts_copy.end(); ++ci)
		{
			if (ci->first->getPredicate().getName() == "belongs_in" && ci->first->getObjects()[0]->getType().getName() == "object" && ci->first->getObjects()[1]->getType().getName() == "box")
				std::cout << *ci->first << " = " << ci->second << std::endl;
		}
		results = runRecommender(objects, predicates, weighted_facts_copy);
		std::cout << "Post recog..." << std::endl;
		for (std::map<const Fact*, float>::const_iterator ci = results.begin(); ci != results.end(); ++ci)
		{
			if (ci->first->getPredicate().getName() == "belongs_in" && ci->first->getObjects()[0]->getType().getName() == "object" && ci->first->getObjects()[1]->getType().getName() == "box")
				std::cout << *ci->first << " = " << ci->second << std::endl;
		}
		
		float knowledge_value_if_false = calculateKnowledge(results, interesting_facts) - knowledge_value_baseline;
		
		return std::make_pair(knowledge_value_if_true, knowledge_value_if_false);
	}
	
	std::vector<const Fact*> RecommenderSystem::getBestSensingActions(const std::vector<const Object*>& objects, const std::vector<const Predicate*>& predicates, const std::map<const Fact*, float>& weighted_facts, const std::vector<const Fact*>& interesting_facts, unsigned int max_depth)
	{
		ROS_INFO("KCL: (RecommenderSystem) Get best sensing actions %zd.", interesting_facts.size());
		std::vector<const Fact*> best_facts_to_sense;
		
		std::map<const Fact*, float> results = runRecommender(objects, predicates, weighted_facts);
		
		float knowledge_value = calculateKnowledge(results, interesting_facts);
		
		std::cout << "[RecommenderSystem::getBestSensingActions] Baseline: " << knowledge_value << std::endl;
		
		// Given the baseline, we now check all facts we can observe and pick the one that increases it the most.
		float max_gain = 0;
		std::priority_queue<const UtilityFact*> queue;
		
		std::set<const Fact*> processed_facts;
		int sample_size = 3;
		for (unsigned int i = 0; i < sample_size; ++i)
		{
			int index;
			do
			{
				index = rand() % interesting_facts.size();
			} while (processed_facts.find(interesting_facts[index]) != processed_facts.end());
			processed_facts.insert(interesting_facts[index]);
			
			const Fact* fact_to_sense = interesting_facts[index];
			
			// Ignore this fact if we know whether it is true or false.
			std::map<const Fact*, float>::const_iterator map_ci = weighted_facts.find(fact_to_sense);
			if (map_ci == weighted_facts.end() || map_ci->second != 0)
			{
				continue;
			}
			std::cout << "[RecommenderSystem::getBestSensingActions] (" << i << "/" << sample_size << ") Check: " << *fact_to_sense << std::endl;
			
			std::pair<float, float> knowledge_gain = calculateKnowledgeIncrease(objects, predicates, weighted_facts, interesting_facts, *fact_to_sense);
			
			float d = (knowledge_gain.first + knowledge_gain.second) / 2.0f;
			if (d > max_gain)
			{
				best_facts_to_sense.clear();
				best_facts_to_sense.push_back(fact_to_sense);
				max_gain = d;
				std::cout << "Sensing " << *fact_to_sense << " gives us <" << knowledge_gain.first << ", " << knowledge_gain.second << ")" << std::endl;
			}
			else
			{
				std::cout << "Sensing " << *fact_to_sense << " gives us <" << knowledge_gain.first << ", " << knowledge_gain.second << ") -- IGNORE" << std::endl;
			}
			queue.push(new UtilityFact(*fact_to_sense, d));
		}
		
		ROS_INFO("KCL: (RecommenderSystem) Baseline results:");
		while (!queue.empty())
		{
			const UtilityFact* uf = queue.top();
			queue.pop();
			std::cout <<  *uf << std::endl;
		}
		
		return best_facts_to_sense;
	}
	
	void RecommenderSystem::processMutualExclusive(std::map<const Fact*, float>& weighted_facts_copy, const std::vector<const Fact*>& interesting_facts)
	{
#ifdef RECOMMENDER_SYSTEM_DEBUG
		std::cout << "callRecogniser:" << std::endl;
		for (std::vector<const Fact*>::const_iterator ci = interesting_facts.begin(); ci != interesting_facts.end(); ++ci)
		{
			std::cout << "\t-" << **ci << " -> " << weighted_facts_copy[*ci] << std::endl;
		}
#endif
		
		std::map<const Object*, std::vector<const Fact*>* > variable_domains;
		
#ifdef RECOMMENDER_SYSTEM_DEBUG
		std::cout << "Build variable domains. Fact: " << last_sensed_fact << std::endl;
#endif
		for (std::vector<const Fact*>::const_iterator ci = interesting_facts.begin(); ci != interesting_facts.end(); ++ci)
		{
#ifdef RECOMMENDER_SYSTEM_DEBUG
			std::cout << "Interesting fact: " << **ci << " = " << weighted_facts_copy[*ci] << std::endl;
#endif
			
			if (weighted_facts_copy[*ci] != 1)
			{
				std::vector<const Fact*>* domain = NULL;
				
				if (variable_domains.find((*ci)->getObjects()[0]) == variable_domains.end())
				{
					domain = new std::vector<const Fact*>();
					variable_domains[(*ci)->getObjects()[0]] = domain;
				}
				else
				{
					domain = variable_domains[(*ci)->getObjects()[0]];
				}
				domain->push_back(*ci);
			}
		}
		
		// Check if the domain of a variable has only a single entry. If so make that fact true.
#ifdef RECOMMENDER_SYSTEM_DEBUG
		std::cout << "Variable domains: " << std::endl;
#endif
		for (std::map<const Object*, std::vector<const Fact*>* >::const_iterator ci = variable_domains.begin(); ci != variable_domains.end(); ++ci)
		{
			const Object* object = ci->first;
			std::vector<const Fact*>* facts = ci->second;
			
#ifdef RECOMMENDER_SYSTEM_DEBUG
			std::cout << "- " << object->getName() << " = {";
			for (std::vector<const Fact*>::const_iterator ci = facts->begin(); ci != facts->end(); ++ci)
			{
				std::cout << **ci << ", ";
			}
			std::cout << "}" << std::endl;
#endif
			
			if (facts->size() == 1)
			{
				weighted_facts_copy[(*facts)[0]] = 2.0f;
			}
		}
		
		// If a single entry is already true, make the rest false.
		for (std::map<const Object*, std::vector<const Fact*>* >::const_iterator ci = variable_domains.begin(); ci != variable_domains.end(); ++ci)
		{
			const Object* object = ci->first;
			std::vector<const Fact*>* facts = ci->second;
			bool value_found = false;
			
			for (std::vector<const Fact*>::const_iterator ci = facts->begin(); ci != facts->end(); ++ci)
			{
				if (weighted_facts_copy[*ci] == 2.0f)
				{
					value_found = true;
					break;
				}
			}
			
			if (value_found)
			{
				for (std::vector<const Fact*>::const_iterator ci = facts->begin(); ci != facts->end(); ++ci)
				{
					if (weighted_facts_copy[*ci] != 2.0f)
					{
						weighted_facts_copy[*ci] = 1.0f;
					}
				}
			}
		}
		
#ifdef RECOMMENDER_SYSTEM_DEBUG
		std::cout << "Final facts: " << std::endl;
		for (std::vector<const Fact*>::const_iterator ci = interesting_facts.begin(); ci != interesting_facts.end(); ++ci)
		{
			std::cout << "\t-" << **ci << " -> " << weighted_facts_copy[*ci] << std::endl;
		}
#endif
	}

	
	void RecommenderSystem::callRecogniser(FactObserveTree& node, const Fact& last_sensed_fact, unsigned int current_depth, unsigned int max_depth, const std::vector<const Object*>& objects, const std::vector<const Predicate*>& predicates, const std::map<const Fact*, float>& weighted_facts, const std::vector<const Fact*>& interesting_facts)
	{
		if (current_depth == max_depth) return;
		
		std::map<const Fact*, float> weighted_facts_copy(weighted_facts);
		
		// Make the best sensing action true and run the system again.
		weighted_facts_copy[&last_sensed_fact] = 2.0;
		processMutualExclusive(weighted_facts_copy, interesting_facts);
		
#ifdef RECOMMENDER_SYSTEM_DEBUG
		std::cout << "Get sensing actions for TRUE branch: " << std::endl;
		for (std::vector<const Fact*>::const_iterator ci = interesting_facts.begin(); ci != interesting_facts.end(); ++ci)
		{
			std::cout << "\t-" << **ci << " -> " << weighted_facts_copy[*ci] << std::endl;
		}
#endif
		
		std::vector<const KCL_rosplan::Fact*> facts_to_sense = getBestSensingActions(objects, predicates, weighted_facts_copy, interesting_facts, 3);
		
		ROS_INFO("KCL: (RecommanderSystem) [%u] TRUE BRANCH Find %zd possible facts to sense.\n", current_depth, facts_to_sense.size());
		//for (std::vector<const KCL_rosplan::Fact*>::const_iterator ci = facts_to_sense.begin(); ci != facts_to_sense.end(); ++ci)
		for (unsigned int i = 0; i < std::min((int)facts_to_sense.size(), 5); ++i)
		{
			std::cout << *facts_to_sense[i] << std::endl;
		}
		
		if (facts_to_sense.size() > 0)
		{
			FactObserveTree* true_branch = new FactObserveTree(*facts_to_sense[0]);
			node.true_branch_ = true_branch;
			callRecogniser(*true_branch, *facts_to_sense[0], current_depth + 1, max_depth, objects, predicates, weighted_facts_copy, interesting_facts);
		}
		
		// Make the best sensing action false and run the system again.
		weighted_facts_copy = weighted_facts;
		weighted_facts_copy[&last_sensed_fact] = 1.0;
		processMutualExclusive(weighted_facts_copy, interesting_facts);
		
#ifdef RECOMMENDER_SYSTEM_DEBUG
		std::cout << "Get sensing actions for FALSE branch: " << std::endl;
		for (std::vector<const Fact*>::const_iterator ci = interesting_facts.begin(); ci != interesting_facts.end(); ++ci)
		{
			std::cout << "\t-" << **ci << " -> " << weighted_facts_copy[*ci] << std::endl;
		}
#endif
		
		facts_to_sense = getBestSensingActions(objects, predicates, weighted_facts_copy, interesting_facts, 3);
		
		ROS_INFO("KCL: (RecommanderSystem) [%u] FALSE BRANCH Find %zd possible facts to sense.\n", current_depth, facts_to_sense.size());
		//for (std::vector<const KCL_rosplan::Fact*>::const_iterator ci = facts_to_sense.begin(); ci != facts_to_sense.end(); ++ci)
		for (unsigned int i = 0; i < std::min((int)facts_to_sense.size(), 5); ++i)
		{
			std::cout << *facts_to_sense[i] << std::endl;
		}
		if (facts_to_sense.size() > 0)
		{
			FactObserveTree* false_branch = new FactObserveTree(*facts_to_sense[0]);
			node.false_branch_ = false_branch;
			callRecogniser(*false_branch, *facts_to_sense[0], current_depth + 1, max_depth, objects, predicates, weighted_facts_copy, interesting_facts);
		}
	}
	
	void RecommenderSystem::visualise(const std::string& path, const std::vector<const Object*>& objects, const std::vector<const Predicate*>& predicates, const std::vector<const KCL_rosplan::Fact*>& all_facts, const std::set<const KCL_rosplan::Fact*>& true_facts)
	{
		// Get all the facts from the knowledge base and run the recommender. 
		// We can then visualise the results.
		
		/* Mark all facts that are valid with 0 (uncertain). */
		std::map<const KCL_rosplan::Fact*, float> weighted_facts;
		std::vector<const KCL_rosplan::Fact*> interesting_facts;
		for (std::vector<const KCL_rosplan::Fact*>::const_iterator ci = all_facts.begin(); ci != all_facts.end(); ++ci)
		{
		       const KCL_rosplan::Fact* fact = *ci;
		       
		       // Check if this fact is known to be true.
		       if (true_facts.find(fact) != true_facts.end())
		       {
			       weighted_facts[*ci] = 2.0;
		       }
		       // Otherwise we set it to false.
		       else
		       {
			       weighted_facts[*ci] = 1.0;
		       }
		       
		       // All facts that are certain are set true.
		       if ((*ci)->getPredicate().getName() == "belongs_in")
		       {
				interesting_facts.push_back(*ci);
				
				rosplan_knowledge_msgs::KnowledgeQueryService knowledge_query;
				rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
				knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
				knowledge_item.attribute_name = "belongs_in";
				knowledge_item.is_negative = false;
				
				diagnostic_msgs::KeyValue kv;
				kv.key = "o";
				kv.value = fact->getObjects()[0]->getName();
				knowledge_item.values.push_back(kv);
				
				kv.key = "b";
				kv.value = fact->getObjects()[1]->getName();
				knowledge_item.values.push_back(kv);
				
				knowledge_query.request.knowledge.push_back(knowledge_item);
				//ROS_INFO("KCL: (RecommenderSystem) Check if (belongs_in %s %s) is true.", fact->getObjects()[0]->getName().c_str(), fact->getObjects()[1]->getName().c_str());
				
				// Check if any of these facts are true.
				if (!query_knowledge_client.call(knowledge_query))
				{
					ROS_ERROR("KCL: (RecommenderSystem) Could not call the query knowledge server.");
					exit(1);
				}
				
				if (knowledge_query.response.results.size() > 0 && knowledge_query.response.results[0] == 1)
				{
					weighted_facts[*ci] = 2.0;
				}
				else
				{
					knowledge_query.response.results.clear();
					
					// Check if the fact is negative.
					knowledge_item.is_negative = false;
					
					//ROS_INFO("KCL: (RecommenderSystem) Check if (not (belongs_in %s %s)) is true.", fact->getObjects()[0]->getName().c_str(), fact->getObjects()[1]->getName().c_str());
				
					// Check if any of these facts are true.
					if (!query_knowledge_client.call(knowledge_query))
					{
						ROS_ERROR("KCL: (RecommenderSystem) Could not call the query knowledge server.");
						exit(1);
					}
					if (knowledge_query.response.results.size() > 0 && knowledge_query.response.results[0] == 1)
					{
						weighted_facts[*ci] = 1.0;
					}
					else
					{
						weighted_facts[*ci] = 0.0;
					}
				}
				
			}
		}
		processMutualExclusive(weighted_facts, interesting_facts);
		
		std::map<const Fact*, float> results = runRecommender(objects, predicates, weighted_facts);
		
		// Visualise the mapping.
		// output file
		std::stringstream dest;

		dest << "digraph recogniser {" << std::endl;

		// nodes
		for (std::vector<const Object*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
		{
			const Object* o = *ci;
			if (o->getType().getName() == "box")
			{
				dest << o->getName() << "[ shape=box label=\"" << o->getName() << "\" style=\"fill: #fff; \"];" << std::endl;
			}
			else if (o->getType().getName() == "object")
			{
				dest << o->getName() << "[ shape=ellipse label=\"" << o->getName() << "\" style=\"fill: #fff; \"];" << std::endl;
			}
		}

		// For every object, check which boxes are most prominent.
		for (std::vector<const Object*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
		{
			const Object* o = *ci;
			if (o->getType().getName() != "object") continue;

			std::vector<const Fact*> all_facts;

			std::vector<const Object*> objects;
			objects.push_back(o);

			objects.push_back(Object::getObject("box1"));
			const Fact& box1_fact = Fact::getFact(*Predicate::getPredicate("belongs_in"), objects);
			all_facts.push_back(&box1_fact);
			objects.pop_back();
			objects.push_back(Object::getObject("box2"));
			const Fact& box2_fact = Fact::getFact(*Predicate::getPredicate("belongs_in"), objects);
			all_facts.push_back(&box2_fact);
			objects.pop_back();
			objects.push_back(Object::getObject("box3"));
			const Fact& box3_fact = Fact::getFact(*Predicate::getPredicate("belongs_in"), objects);
			all_facts.push_back(&box3_fact);

			float highest_value = 0;
			for (std::vector<const Fact*>::const_iterator ci = all_facts.begin(); ci != all_facts.end(); ++ci)
			{
				if (results[*ci] > highest_value) highest_value = results[*ci];
			}

			if (highest_value == 0) continue;
			
			for (std::vector<const Fact*>::const_iterator ci = all_facts.begin(); ci != all_facts.end(); ++ci)
			{
				const Fact* fact = *ci;
				if (results[fact] == 1)
				{
					dest << "\"" << fact->getObjects()[0]->getName() << "\"" << " -> \"" << fact->getObjects()[1]->getName() << "\"[penwidth=3, color=\"#11EE22\"];" << std::endl;
				}
				else if (results[fact] == highest_value)
				{
					dest << "\"" << fact->getObjects()[0]->getName() << "\"" << " -> \"" << fact->getObjects()[1]->getName() << "\"[penwidth=2, color=\"#CDDDDD\"];" << std::endl;
				}
				else if (results[fact] > 0.55f)
				{
					dest << "\"" << fact->getObjects()[0]->getName() << "\"" << " -> \"" << fact->getObjects()[1]->getName() << "\"[weight=0.3, color=\"#444444\"];" << std::endl;
				}
			}
		}

/*
		// edges
		for (std::vector<const KCL_rosplan::Fact*>::const_iterator ci = interesting_facts.begin(); ci != interesting_facts.end(); ++ci)
		{
			const KCL_rosplan::Fact* fact = *ci;
			if (fact->getPredicate().getName() != "belongs_in") continue;
			
			float value = results[fact];
			std::cout << *fact << " = " << value << std::endl;
			if (value > 0.5 && !fact->isNegative())
			{
				dest << "\"" << fact->getObjects()[0]->getName() << "\"" << " -> \"" << fact->getObjects()[1]->getName() << "\";" << std::endl;
			}
		}
*/

		dest << "}" << std::endl;
		
		// write to file
		std::ofstream file;
		std::stringstream ss;
		ss << path << "/d3_viz/recommender.dot";
		file.open(ss.str().c_str());
		file << dest.str();
		file.close();
	}
	
}; // close namespace

void tokenise(const std::string& s, std::vector<std::string>& tokens)
{
	size_t current;
	size_t next = -1;

	do
	{
		current = next + 1;
		next = s.find_first_of(" ", current);
		tokens.push_back(s.substr(current, next - current));
	} 
	while (next != std::string::npos);
}

geometry_msgs::Pose transformToPose(const std::string& s)
{
#ifdef RECOMMENDER_SYSTEM_DEBUG
	std::cout << "Tranform to pose: " << s << std::endl;
#endif
	geometry_msgs::Pose p;
	int first_break = s.find(',');
	int second_break = s.find(',', first_break + 1);

	p.position.x = ::atof(s.substr(1, first_break - 1).c_str());
	p.position.y = ::atof(s.substr(first_break + 1, second_break - (first_break + 1)).c_str());
	p.position.z = ::atof(s.substr(second_break + 1, s.size() - (second_break + 2)).c_str());

	return p;
}

/** 
 * Read in db file
 */
void readDBFile(mongodb_store::MessageStoreProxy& message_store, ros::ServiceClient& update_knowledge_client, const std::string& file_name, std::vector<const KCL_rosplan::Type*>& types, std::vector<const KCL_rosplan::Object*>& objects, std::vector<const KCL_rosplan::Predicate*>& predicates, std::set<const KCL_rosplan::Fact*>& facts)
{
	ROS_INFO("KCL: (RecommenderSystem) Load scenarion from file: %s.\n", file_name.c_str());
	std::ifstream f(file_name.c_str());
	std::string line;

	rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;

	if (f.is_open())
	{
		while (getline(f, line))
		{

			if (line.size() == 0 || line[0] == '#') continue;

			std::vector<std::string> tokens;
			tokenise(line, tokens);

			// Types.
			if (line[0] == 't')
			{
				if (tokens.size() != 2)
				{
					ROS_ERROR("KCL (RecommenderSystem) Malformed line, expected t type. Read %s\n", line.c_str());
					exit(0);
				}
				const std::string& type_name = tokens[1];
				
				const KCL_rosplan::Type& type = KCL_rosplan::Type::createType(type_name, NULL);
				types.push_back(&type);
			}
			else if (line[0] == 'o')
			{
				if (tokens.size() != 3)
				{
					ROS_ERROR("KCL (RecommenderSystem) Malformed line, expected o object type. Read %s\n", line.c_str());
					exit(0);
				}
				const std::string& object_name = tokens[1];
				const std::string& type_name = tokens[2];
				objects.push_back(&KCL_rosplan::Object::createObject(object_name, KCL_rosplan::Type::getType(type_name)));

				// Add to the knowledge base.
				rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
				
				knowledge_item.instance_type = type_name;
				knowledge_item.instance_name = object_name;
				
				knowledge_update_service.request.knowledge = knowledge_item;knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
				if (!update_knowledge_client.call(knowledge_update_service)) {
					ROS_ERROR("KCL: (RobotKnowsGame) Could not add the %s %s to the knowledge base.", type_name.c_str(), object_name.c_str());
					exit(-1);
				}
			}
			else if (line[0] == 'p')
			{
				if (tokens.size() < 2)
				{
					ROS_ERROR("KCL (RecommenderSystem) Malformed line, expected p predicate [type]. Read %s\n", line.c_str());
					exit(0);
				}
				
				const std::string& predicate_name = tokens[1];
				
				std::vector<const KCL_rosplan::Type*> predicate_types;
				for (unsigned int i = 2; i < tokens.size(); ++i)
				{
					predicate_types.push_back(&KCL_rosplan::Type::getType(tokens[i]));
				}
				predicates.push_back(&KCL_rosplan::Predicate::getPredicate(predicate_name, predicate_types));
			}
			else if (line[0] == 'f')
			{
				if (tokens.size() < 2)
				{
					ROS_ERROR("KCL (RecommenderSystem) Malformed line, expected f fact [object]. Read %s\n", line.c_str());
					exit(0);
				}
				
				const std::string& predicate_name = tokens[1];
				
				std::vector<const KCL_rosplan::Object*> fact_objects;
				for (unsigned int i = 2; i < tokens.size(); ++i)
				{
					fact_objects.push_back(KCL_rosplan::Object::getObject(tokens[i]));
				}
				
				facts.insert(&KCL_rosplan::Fact::getFact(*KCL_rosplan::Predicate::getPredicate(predicate_name), fact_objects));
			}
			else if (line[0] == 'r')
			{
				if (tokens.size() < 3)
				{
					ROS_ERROR("KCL (RecommenderSystem) Malformed line, expected r waypoint (f,f,f). Read %s\n", line.c_str());
					exit(0);
				}
				std::string waypoint_predicate = tokens[1];
				geometry_msgs::Pose waypoint_location = transformToPose(tokens[2]);
				
				geometry_msgs::PoseStamped pose;
				pose.header.seq = 0;
				pose.header.stamp = ros::Time::now();
				pose.header.frame_id = "/map";
				pose.pose = waypoint_location;
				pose.pose.orientation.x = 0;
				pose.pose.orientation.y = 0;
				pose.pose.orientation.z = 0;
				pose.pose.orientation.w = 1;
				std::string id (message_store.insertNamed(waypoint_predicate, pose));

				// Calculate it's near component.
				float centre_x = 0.372;
				float centre_y = -0.342;
				float distance = 0.8f;

				float direction_x = centre_x - pose.pose.position.x;
				float direction_y = centre_y - pose.pose.position.y;
				float length = sqrt((direction_x * direction_x) + (direction_y * direction_y));
				direction_x = direction_x / length;
				direction_y = direction_y / length;
				std::stringstream ss;
				ss << "near_" << waypoint_predicate;

				float near_x = pose.pose.position.x + direction_x * distance;
				float near_y = pose.pose.position.y + direction_y * distance;

				float angle = atan2(pose.pose.position.y - near_y, pose.pose.position.x - near_x);
				if(isnan(angle)) angle = 0;

				pose.pose.position.x = near_x;
				pose.pose.position.y = near_y;
				pose.pose.position.z = 0;
				pose.pose.orientation = tf::createQuaternionMsgFromYaw(angle);


				std::string near_id(message_store.insertNamed(ss.str(), pose));
			}
		}
	}
}

/**
 * Dummy PDDL problem generator.
 */
bool generatePDDLProblemFile(rosplan_knowledge_msgs::GenerateProblemService::Request &req, rosplan_knowledge_msgs::GenerateProblemService::Response &res) {
	return true;
}

int main(int argc, char** argv)
{
        srand(time(NULL));
        ROS_INFO("KCL: (RecommenderSystem) Started!\n");

        ros::init(argc, argv, "rosplan_RecommanderSystem");
        ros::NodeHandle nh;

        mongodb_store::MessageStoreProxy ms(nh);
        ros::ServiceClient update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");

        std::string domain_path;
        nh.getParam("/rosplan/domain", domain_path);

        std::string data_path;
        nh.getParam("/rosplan/data_path", data_path);

        std::string config_file;
        nh.getParam("/facts_db", config_file);
	
	std::vector<const KCL_rosplan::Type*> types;
	std::vector<const KCL_rosplan::Object*> objects;
	std::vector<const KCL_rosplan::Predicate*> predicates;
	std::set<const KCL_rosplan::Fact*> true_facts;
	readDBFile(ms, update_knowledge_client, config_file, types, objects, predicates, true_facts);

        std::vector<const KCL_rosplan::Object*> relevant_objects;
        for (std::vector<const KCL_rosplan::Object*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
        {
                if ((*ci)->getType().getName() == "box" ||
                    (*ci)->getType().getName() == "object")
                relevant_objects.push_back(*ci);
        }
        
        // Ground all Facts .
        std::vector<const KCL_rosplan::Fact*> all_facts;
        for (std::vector<const KCL_rosplan::Predicate*>::const_iterator ci = predicates.begin(); ci != predicates.end(); ++ci)
        {
                const KCL_rosplan::Predicate* predicate = *ci;
                predicate->ground(all_facts);
        }


	/* Mark all facts that are valid with 0 (uncertain). */
	std::map<const KCL_rosplan::Fact*, float> weighted_facts;
	std::vector<const KCL_rosplan::Fact*> interesting_facts;
	for (std::vector<const KCL_rosplan::Fact*>::const_iterator ci = all_facts.begin(); ci != all_facts.end(); ++ci)
	{
	       const KCL_rosplan::Fact* fact = *ci;
	       
	       // Check if this fact is known to be true.
	       if (true_facts.find(fact) != true_facts.end())
	       {
		       weighted_facts[*ci] = 2.0;
	       }
	       // Otherwise we set it to false.
	       else
	       {
		       weighted_facts[*ci] = 1.0;
	       }
	       
	       // All facts that are certain are set true.
	       if ((*ci)->getPredicate().getName() == "belongs_in")
	       {
		       interesting_facts.push_back(*ci);
		       weighted_facts[*ci] = 0.0;
	       }
	}
	ROS_INFO("KCL: (RecommenderSystem) Weighted facts created.\n");

	// Start the listen to feedback action.
	KCL_rosplan::ListenToFeedbackPDDLAction listener(nh);

	// Start inspection action.
	KCL_rosplan::InspectObjectPDDLAction inspection_action(nh);

	ros::ServiceServer pddl_generation_service = nh.advertiseService("/kcl_rosplan/generate_planning_problem", &generatePDDLProblemFile);

	// Start the recommender system and initialise all the facts.
	KCL_rosplan::RecommenderSystem rs(nh);
	ROS_INFO("KCL: (RecommenderSystem) Recommender System started.");
	
	std::vector<const KCL_rosplan::Predicate*> relevant_predicates;
	for (std::vector<const KCL_rosplan::Predicate*>::const_iterator ci = predicates.begin(); ci != predicates.end(); ++ci)
	{
		if ((*ci)->getName() == "object_at" || 
		    (*ci)->getName() == "robot_at" || 
		    (*ci)->getName() == "box_at")
		{
			continue;
		}
		relevant_predicates.push_back(*ci);
	}
	
/*
	std::vector<const KCL_rosplan::Object*> relevant_objects;
	
	const KCL_rosplan::Fact* previous_fact = facts_to_sense[0];
	for (unsigned int i = 0; i < std::min((int)facts_to_sense.size(), 5); ++i)
	{
		std::cout << *facts_to_sense[i] << std::endl;
	}
	
	KCL_rosplan::FactObserveTree root(*previous_fact);
	rs.callRecogniser(root, *previous_fact, 0, 1, relevant_objects, relevant_predicates, weighted_facts, interesting_facts);
	
	ROS_INFO("KCL: (RecommenderSystem) Recogniser is finished, create the planning problem.");
	
	// Create waypoints for all the toys and boxes.
	std::vector<const KCL_rosplan::Object*> toys;
	KCL_rosplan::Object::getObjects(KCL_rosplan::Type::getType("object"), toys);
	
	// Only include those objects that we need to sense.
	std::map<std::string, std::string> object_to_location_mapping;
	for (std::vector<const KCL_rosplan::Object*>::const_iterator ci = toys.begin(); ci != toys.end(); ++ci)
	{
		const KCL_rosplan::Object* object = *ci;
		if (!root.contains(*object))
		{
			continue;
		}
		
		std::stringstream ss;
		ss << object->getName() << "_wp";
		
		object_to_location_mapping[object->getName()] = ss.str();
	}
	
	std::vector<const KCL_rosplan::Object*> boxes;
	KCL_rosplan::Object::getObjects(KCL_rosplan::Type::getType("box"), boxes);
	
	std::map<std::string, std::string> box_to_location_mapping;
	for (std::vector<const KCL_rosplan::Object*>::const_iterator ci = boxes.begin(); ci != boxes.end(); ++ci)
	{
		const KCL_rosplan::Object* object = *ci;
		
		std::stringstream ss;
		ss << object->getName() << "_location";
		
		box_to_location_mapping[object->getName()] = ss.str();
	}
	
//	KCL_rosplan::PlanToSensePDDLGenerator pta;
//	pta.createPDDL(root, data_path, "domain.pddl", "problem.pddl", "kenny_wp", object_to_location_mapping, box_to_location_mapping);
*/
	
	// Start the planner using the generated domain / problem files.
	
	std::string planner_path;
	nh.getParam("/planner_path", planner_path);
	
	std::stringstream ss;
	ss << data_path << "domain_ask.pddl";
	std::string domain_path_ask = ss.str();

	ss.str(std::string());
	ss << data_path << "domain_sense.pddl";
	std::string domain_path_sense = ss.str();
	
	ss.str(std::string());
	ss << data_path << "problem.pddl";
	std::string problem_path = ss.str();
	std::cout << problem_path << std::endl;
	
	std::string planner_command_ask;
	nh.getParam("/squirrel_planning_execution/planner_command_ask", planner_command_ask);

	std::string planner_command_sense;
	nh.getParam("/squirrel_planning_execution/planner_command_sense", planner_command_sense);
	
	rosplan_dispatch_msgs::PlanGoal psrv;
	psrv.domain_path = domain_path_ask;
	psrv.problem_path = problem_path;
	psrv.data_path = data_path;
	psrv.planner_command = planner_command_ask;
	psrv.start_action_id = 0;

	ROS_INFO("KCL: (RobotKnowsGame) Start plan action");
	actionlib::SimpleActionClient<rosplan_dispatch_msgs::PlanAction> plan_action_client("/kcl_rosplan/start_planning", true);

	plan_action_client.waitForServer();
	ROS_INFO("KCL: (RobotKnowsGame) Start planning server found");
	
	// send goal
	plan_action_client.sendGoal(psrv);
	ROS_INFO("KCL: (RobotKnowsGame) Goal sent");

	// Update the visualisation every 10 seconds.
	double last_update = ros::Time::now().toSec();
	while (ros::ok())
	{
		double now = ros::Time::now().toSec();
		if (now - last_update > 10)
		{
			rs.visualise(data_path, objects, relevant_predicates, all_facts, true_facts);
		}
		ros::spinOnce();
		
		// Check if the planner has finished yet.
		actionlib::SimpleClientGoalState state = plan_action_client.getState();
		if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
//			KCL_rosplan::PlanToSensePDDLGenerator pts;
//			pts.createPDDL(root, data_path, "domain.pddl", "problem.pddl", "kenny_wp", object_to_location_mapping, box_to_location_mapping);
			
			// Start the planner using the generated domain / problem files.
			psrv.domain_path = domain_path_sense;
			psrv.problem_path = problem_path;
			psrv.data_path = data_path;
			psrv.planner_command = planner_command_sense;
			psrv.start_action_id = 0;

			// send goal
			plan_action_client.sendGoal(psrv);
			ROS_INFO("KCL: (RobotKnowsGame) Goal sent");
		}
	}
	return 0;
}

