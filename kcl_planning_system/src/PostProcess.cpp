/**
 * This file parses the output of popf and generates a list of ActionDispatch messages.
 * It can be replaced or extended to add domain-specific post-processing of PDDL actions, or to
 * change parsing to another planner.
 */
#include "planning_dispatch_msgs/ActionDispatch.h"

#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>

namespace KCL_rosplan
{
	/**
	 * processes the parameters of a single PDDL action into an ActionDispatch message.
	 */
	void processParameters(planning_dispatch_msgs::ActionDispatch &msg, std::vector<std::string> &params) {

		std::map<std::string,std::vector<std::string> >::iterator ait;
		ait = domainOperators.find(msg.name);
		if(ait != domainOperators.end()) {

			// Parse the PDDL parameters back into the object attributes stored in the PDDL_knowledge map.
			for(size_t i=0; i<ait->second.size(); i++) {
				diagnostic_msgs::KeyValue pair;
				pair.key = ait->second[i];
				pair.value = params[i];
				msg.parameters.push_back(pair);
			}

			// add non-PDDL knowledge items to action dispatch
			std::map<std::string,std::vector<std::string> >::iterator pit;
			for(size_t i=0; i<params.size(); i++) {
				pit = domainPredicates.find(ait->second[i]);
				for(size_t j=0; j<KCL_rosplan::instanceAttributes.size(); j++) {
					// TODO hava knowledge items be passed nicer in dispatch
					if(0==KCL_rosplan::instanceAttributes[j].instance_name.compare(params[i])
							&& KCL_rosplan::instanceAttributes[j].knowledge_type == planning_knowledge_msgs::KnowledgeItem::ATTRIBUTE
							&& std::find(pit->second.begin(),pit->second.end(),KCL_rosplan::instanceAttributes[j].attribute_name)==pit->second.end()) {
						for(size_t k=0; k<instanceAttributes[j].values.size(); k++) {
							diagnostic_msgs::KeyValue pair;
							pair.key = instanceAttributes[j].instance_name + "_" + instanceAttributes[j].attribute_name + "_" + instanceAttributes[j].values[k].key;
							pair.value = instanceAttributes[j].values[k].value;
							msg.parameters.push_back(pair);
						}
					}
				}
			}
		}
		
		// TODO: store what we care about for the filter
		planning_knowledge_msgs::KnowledgeItem itemMessage;
		itemMessage.knowledge_type = planning_knowledge_msgs::KnowledgeItem::INSTANCE;
		itemMessage.instance_type = "TYPENAME";
		// itemMessage.instance_name = params[0];
		knowledge_filter.push_back(itemMessage);
	}

	/**
	 * parses the output of popf, generating a list of action messages.
	 */
	void preparePlan(std::string &dataPath)
	{
		// popf output
		std::ifstream planfile;
		planfile.open((dataPath + "plan.pddl").c_str());
		
		int curr, next; 
		std::string line;
		std::vector<planning_dispatch_msgs::ActionDispatch> potentialPlan;
		double planDuration;
		double expectedPlanDuration = 0;

		while(!planfile.eof()) {

			getline(planfile, line);

			if (line.substr(0,6).compare("; Plan") == 0) {
				expectedPlanDuration = atof(line.substr(25).c_str());
			} else if (line.substr(0,6).compare("; Time")!=0) {
				//consume useless lines
			} else {

				potentialPlan.clear();
				KCL_rosplan::freeActionID = KCL_rosplan::currentAction;
				planDuration = 0;

				while(!planfile.eof() && line.compare("")!=0) {

					getline(planfile, line);
					if (line.length()<2)
						break;

					planning_dispatch_msgs::ActionDispatch msg;

					// action ID
					msg.action_id = KCL_rosplan::freeActionID;
					KCL_rosplan::freeActionID++;

					// name
					curr=line.find("(")+1;
					next=line.find(" ",curr);
					std::string name = line.substr(curr,next-curr).c_str();
					msg.name = name;

					// parameters
					std::vector<std::string> params;
					curr=next+1;
					next=line.find(")",curr);
					int at = curr;
					while(at < next) {
						int cc = line.find(" ",curr);
						int cc1 = line.find(")",curr);
						curr = cc<cc1?cc:cc1;
						std::string param = name_map[line.substr(at,curr-at)];
						params.push_back(param);
						++curr;
						at = curr;
					}
					processParameters(msg, params);

					// duration
					curr=line.find("[",curr)+1;
					next=line.find("]",curr);
					msg.duration = (double)atof(line.substr(curr,next-curr).c_str());

					potentialPlan.push_back(msg);

					// update plan duration
					curr=line.find(":");
					planDuration = msg.duration + atof(line.substr(0,curr).c_str());
				}

				if(planDuration - expectedPlanDuration < 0.01)  {

					// trim any previously read plan
					while(KCL_rosplan::actionList.size() > KCL_rosplan::currentAction) {
						KCL_rosplan::actionList.pop_back();
					}

					// save better optimised plan
					for(size_t i=0;i<potentialPlan.size();i++) {
						KCL_rosplan::actionList.push_back(potentialPlan[i]);
					}

					KCL_rosplan::totalPlanDuration = planDuration;

				} else {
					ROS_INFO("Duration: %f, expected %f; plan discarded", planDuration, expectedPlanDuration);
				}
			}
		}
		planfile.close();
	}
} // close namespace
