/**
 * This file retrieves and stores the objects and attributes that are used to contruct the problem.
 * The objects are fetched using the planning_knowledge_msgs and then used to construct a PDDL
 * instance (see PDDLProblemGenerator).
 */
#ifndef KCL_environment
#define KCL_environment

#include <fstream>
#include <sstream>
#include <string>
#include <cstdio>
#include <iostream>

#include "VALfiles/ptree.h"
#include "FlexLexer.h"

#include "planning_knowledge_msgs/GetInstancesOfType.h"
#include "planning_knowledge_msgs/GetAttributesOfInstance.h"

extern int yyparse();
extern int yydebug;

namespace VAL {
	parse_category* top_thing=NULL;
	analysis an_analysis;
	analysis* current_analysis;
	yyFlexLexer* yfl;
};

// Global var needed for VALfile/parse_error.h :(
char * current_filename;

namespace KCL_rosplan
{
	/* simulation information */
	bool use_plan_visualisation;

	/* knowledge-base filter */
	ros::Publisher filterPublisher;
	ros::Subscriber notificationSub;
	std::vector<planning_knowledge_msgs::KnowledgeItem> knowledge_filter;

	/* PDDL to Ontology naming map */
	std::map<std::string,std::string> name_map;

	/* domain information */
	std::string domainName;
	std::vector<std::string> domainTypes;
	std::map<std::string,std::vector<std::string> > domainPredicates;
	std::map<std::string,std::vector<std::string> > domainOperators;
	// operator conditions (for filter)

	/* problem information */
	std::map<std::string,std::vector<string> > typeObjectMap;
	std::vector<planning_knowledge_msgs::KnowledgeItem> instanceAttributes;
	std::vector<planning_knowledge_msgs::KnowledgeItem> goalAttributes;

	/*----------------*/
	/* parsing domain */
	/*----------------*/

	/**
	 * TODO read types from domain and domain name
	 */
	void parseDomain(const std::string dataPath) {
		
		std::string domainFileName = (dataPath + "domain.pddl");
		ROS_INFO("KCL: Parsing domain: %s.", domainFileName.c_str());

		// save filename for VAL
		std::vector<char> writable(domainFileName.begin(), domainFileName.end());
		writable.push_back('\0');
		current_filename = &writable[0];

		// parse domain
		VAL::current_analysis = &VAL::an_analysis;
		std::ifstream domainFile;
		domainFile.open(domainFileName.c_str());
		yydebug = 0; // Set to 1 to output yacc trace 

		VAL::yfl = new yyFlexLexer;

		if (domainFile.bad()) {
			ROS_ERROR("KCL: Failed to open domain file.");
			line_no = 0;
			VAL::log_error(VAL::E_FATAL,"Failed to open file");
		} else {
			line_no = 1;
			VAL::yfl->switch_streams(&domainFile,&std::cout);
			yyparse();
						
			// domain name
			VAL::domain* domain = VAL::current_analysis->the_domain;
			domainName = domain->name;

			// types
			VAL::pddl_type_list* types = domain->types;
			for (VAL::pddl_type_list::const_iterator ci = types->begin(); ci != types->end(); ci++) {
				const VAL::pddl_type* type = *ci;
				domainTypes.push_back(type->getName());
			}

			// predicates
			VAL::pred_decl_list* predicates = domain->predicates;
			for (VAL::pred_decl_list::const_iterator ci = predicates->begin(); ci != predicates->end(); ci++) {
				const VAL::pred_decl* predicate = *ci;
				// predicate name
				domainPredicates[predicate->getPred()->symbol::getName()];
				// parameters
				for (VAL::var_symbol_list::const_iterator vi = predicate->getArgs()->begin(); vi != predicate->getArgs()->end(); vi++) {
					const VAL::var_symbol* var = *vi;
					domainPredicates[predicate->getPred()->symbol::getName()].push_back(var->pddl_typed_symbol::getName());
				}
			}

			// operators
			VAL::operator_list* operators = domain->ops;
			for (VAL::operator_list::const_iterator ci = operators->begin(); ci != operators->end(); ci++) {			
				const VAL::operator_* op = *ci;
				// operator name
				domainOperators[op->name->symbol::getName()];
				// parameters
				for (VAL::var_symbol_list::const_iterator vi = op->parameters->begin(); vi != op->parameters->end(); vi++) {
					const VAL::var_symbol* var = *vi;
					domainOperators[op->name->symbol::getName()].push_back(var->pddl_typed_symbol::getName());
				}
				// const VAL::goal* precondition = op->precondition;
			}
		}
		domainFile.close();

		// Output the errors from all input files
		// VAL::current_analysis->error_list.report();
		delete VAL::yfl;
	}

	/*---------------------*/
	/* loading environment */
	/*---------------------*/

	/**
	 * requests all the information required to build a problem instance.
	 */
	void updateEnvironment(ros::NodeHandle nh) {

		// setup service calls
		ros::ServiceClient GetInstancesClient = nh.serviceClient<planning_knowledge_msgs::GetInstancesOfType>("/kcl_rosplan/get_type_instances");
		ros::ServiceClient GetInstanceAttrsClient = nh.serviceClient<planning_knowledge_msgs::GetAttributesOfInstance>("/kcl_rosplan/get_instance_attributes");
		ros::ServiceClient GetCurrentGoalsClient = nh.serviceClient<planning_knowledge_msgs::GetAttributesOfInstance>("/kcl_rosplan/get_current_goals");

		// for each type fetch instances
		for(size_t t=0; t<domainTypes.size(); t++) {

			planning_knowledge_msgs::GetInstancesOfType instanceSrv;
			instanceSrv.request.name = domainTypes[t];
			KCL_rosplan::typeObjectMap[domainTypes[t]];

			if (GetInstancesClient.call(instanceSrv)) {
				for(size_t i=0;i<instanceSrv.response.instances.size();i++) {

					// add new instance (popf converts names to lowercase)
					std::string name = instanceSrv.response.instances[i];
					KCL_rosplan::name_map[KCL_rosplan::toLowerCase(name)] = name;
					KCL_rosplan::typeObjectMap[domainTypes[t]].push_back(name);

					// get instance attributes
					planning_knowledge_msgs::GetAttributesOfInstance instanceAttrSrv;
					instanceAttrSrv.request.instance_name = name;
					instanceAttrSrv.request.type_name = domainTypes[t];
					if (GetInstanceAttrsClient.call(instanceAttrSrv)) {
						for(size_t j=0;j<instanceAttrSrv.response.attributes.size();j++) {
							// if knowledge item corresponds to an attribute of this object, then store it.
							planning_knowledge_msgs::KnowledgeItem attr = instanceAttrSrv.response.attributes[j];
							if(attr.knowledge_type == planning_knowledge_msgs::KnowledgeItem::ATTRIBUTE
									&& attr.instance_type.compare(domainTypes[t])==0
									&& attr.instance_name.compare(name)==0)
								instanceAttributes.push_back(attr);
						}
					} else {
						ROS_ERROR("KCL: Failed to call service get_instance_attributes %s %s",
							instanceAttrSrv.request.type_name.c_str(), instanceAttrSrv.request.instance_name.c_str());
					}

					// get current goals as attributes
					if (GetCurrentGoalsClient.call(instanceAttrSrv)) {
						for(size_t j=0;j<instanceAttrSrv.response.attributes.size();j++) {
							// if knowledge item corresponds to an attribute of this object, then store it.
							planning_knowledge_msgs::KnowledgeItem attr = instanceAttrSrv.response.attributes[j];
							if(attr.knowledge_type == planning_knowledge_msgs::KnowledgeItem::ATTRIBUTE
									&& attr.instance_type.compare(domainTypes[t])==0
									&& attr.instance_name.compare(name)==0)
								goalAttributes.push_back(attr);
						}
					} else {
						ROS_ERROR("KCL: Failed to call service get_current_goals %s %s",
							instanceAttrSrv.request.type_name.c_str(), instanceAttrSrv.request.instance_name.c_str());
					}
				}
			} else {
				ROS_ERROR("KCL: Failed to call service get_instances %s", instanceSrv.request.name.c_str());
			}
		}
	}
} // close namespace

#endif
