#include "squirrel_planning_execution/KnowledgeBase.h"
#include <map>
#include <string>
#include <sstream>
#include <iostream>
#include <vector>

#include <diagnostic_msgs/KeyValue.h>

#include <rosplan_knowledge_msgs/KnowledgeQueryService.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>
#include <rosplan_knowledge_msgs/GetInstanceService.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_knowledge_msgs/GetDomainAttributeService.h>
#include <rosplan_knowledge_msgs/GetDomainTypeService.h>

namespace KCL_rosplan
{
KnowledgeBase::KnowledgeBase(ros::NodeHandle& nh, mongodb_store::MessageStoreProxy& message_store)
	: nh_(&nh), message_store_(&message_store)
{
	update_knowledge_client_ = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
	query_knowledge_client_ = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeQueryService>("/kcl_rosplan/query_knowledge_base");
	
	get_domain_predicates_client_ = nh.serviceClient<rosplan_knowledge_msgs::GetDomainAttributeService>("/kcl_rosplan/get_domain_predicates");
	get_domain_types_client_ = nh.serviceClient<rosplan_knowledge_msgs::GetDomainTypeService>("/kcl_rosplan/get_domain_types");
	get_instance_client_ = nh.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_current_instances");
	get_attribute_client_ = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_knowledge");
	get_current_goals_client_ = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_goals");
}

bool KnowledgeBase::addInstance(const std::string& type, const std::string& name)
{
	rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
	knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
	
	rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
	knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
	knowledge_item.instance_type = type;
	knowledge_item.instance_name = name;
	
	knowledge_update_service.request.knowledge = knowledge_item;
	if (!update_knowledge_client_.call(knowledge_update_service)) {
		ROS_ERROR("KCL: (KnowledgeBase) Could not add the instance %s of type %s to the knowledge base.", name.c_str(), type.c_str());
		return false;
	}
	ROS_INFO("KCL: (KnowledgeBase) Added the instance %s of type %s to the knowledge base.", name.c_str(), type.c_str());
	return true;
}

bool KnowledgeBase::removeInstance(const std::string& type, const std::string& name)
{
	rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
	knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
	
	rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
	knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
	knowledge_item.instance_type = type;
	knowledge_item.instance_name = name;
	
	knowledge_update_service.request.knowledge = knowledge_item;
	if (!update_knowledge_client_.call(knowledge_update_service)) {
		ROS_ERROR("KCL: (KnowledgeBase) Could not remove the instance %s of type %s from the knowledge base.", name.c_str(), type.c_str());
		return false;
	}
	ROS_INFO("KCL: (KnowledgeBase) Removed the instance %s of type %s from the knowledge base.", name.c_str(), type.c_str());
	return true;
}

bool KnowledgeBase::getAllInstances(std::vector<std::string>& store)
{
	ROS_INFO("KCL: (KnowledgeBase) Get all instances.");
	
	rosplan_knowledge_msgs::GetDomainTypeService get_domain_type_service;
	if (!get_domain_types_client_.call(get_domain_type_service))
	{
		ROS_ERROR("KCL: (KnowledgeBase) error getting all types.");
		return false;
	}
	
	for (std::vector<std::string>::const_iterator ci = get_domain_type_service.response.types.begin();
		  ci != get_domain_type_service.response.types.end(); ++ci)
	{
		if (!getInstances(store, *ci))
		{
			return false;
		}
	}
	return true;
}

bool KnowledgeBase::getInstances(std::vector< std::string >& store, const std::string& type)
{
	rosplan_knowledge_msgs::GetInstanceService getInstances;
	getInstances.request.type_name = type;
	if (!get_instance_client_.call(getInstances))
	{
		ROS_ERROR("KCL: (KnowledgeBase) Failed to get all the instances of type %s.", type.c_str());
		return false;
	}
	ROS_INFO("KCL: (KnowledgeBase) Received all the instances of type %s.", type.c_str());
	store.insert(store.end(), getInstances.response.instances.begin(), getInstances.response.instances.end());
}

bool KnowledgeBase::addFact(const std::string& predicate, const std::map<std::string, std::string>& parameters, bool is_true, AddUpdateTarget target)
{
	rosplan_knowledge_msgs::KnowledgeItem knowledge_item = createFact(predicate, parameters, is_true);
	return addFact(knowledge_item, target);
}

bool KnowledgeBase::addFact(const rosplan_knowledge_msgs::KnowledgeItem& fact, AddUpdateTarget target)
{
	rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
	if (target == KB_ADD_GOAL)
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
	else
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		
	std::string s = toString(fact);
	knowledge_update_service.request.knowledge = fact;
	if (!update_knowledge_client_.call(knowledge_update_service)) {
		ROS_ERROR("KCL: (KnowledgeBase) Could not add %s to the knowledge base.", s.c_str());
		return false;
	}
	ROS_INFO("KCL: (KnowledgeBase) Added %s to the knowledge base.", s.c_str());
	return true;
}

bool KnowledgeBase::removeFact(const std::string& predicate, const std::map<std::string, std::string>& parameters, bool is_true, RemoveUpdateTarget target)
{
	rosplan_knowledge_msgs::KnowledgeItem knowledge_item = createFact(predicate, parameters, is_true);
	return removeFact(knowledge_item, target);
}

bool KnowledgeBase::removeFact(const rosplan_knowledge_msgs::KnowledgeItem& fact, RemoveUpdateTarget target)
{
	rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
	if (target == KB_REMOVE_GOAL)
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_GOAL;
	else
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
	
	std::string s = toString(fact);
	knowledge_update_service.request.knowledge = fact;
	if (!update_knowledge_client_.call(knowledge_update_service)) {
		ROS_ERROR("KCL: (KnowledgeBase) Could not remove %s from the knowledge base.", s.c_str());
		return false;
	}
	ROS_INFO("KCL: (KnowledgeBase) Removed %s from the knowledge base.", s.c_str());
	return true;
}

bool KnowledgeBase::addFunction(const std::string& predicate, const std::map<std::string, std::string>& parameters, float value, AddUpdateTarget target)
{
	rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
	if (target == KB_ADD_GOAL)
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
	else
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
	
	rosplan_knowledge_msgs::KnowledgeItem knowledge_item = createFunction(predicate, parameters, value);
	
	std::string s = toString(knowledge_item);
	knowledge_update_service.request.knowledge = knowledge_item;
	if (!update_knowledge_client_.call(knowledge_update_service)) {
		ROS_ERROR("KCL: (KnowledgeBase) Could not add %s to the knowledge base.", s.c_str());
		return false;
	}
	ROS_INFO("KCL: (KnowledgeBase) Added %s to the knowledge base.", s.c_str());
	return true;
}

bool KnowledgeBase::removeFunction(const std::string& predicate, const std::map<std::string, std::string>& parameters, RemoveUpdateTarget target)
{
	rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
	if (target == KB_REMOVE_GOAL)
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_GOAL;
	else
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
	
	rosplan_knowledge_msgs::KnowledgeItem knowledge_item = createFunction(predicate, parameters, 0.0f);
	
	std::string s = toString(knowledge_item);
	knowledge_update_service.request.knowledge = knowledge_item;
	if (!update_knowledge_client_.call(knowledge_update_service)) {
		ROS_ERROR("KCL: (KnowledgeBase) Could not remove %s from the knowledge base.", s.c_str());
		return false;
	}
	ROS_INFO("KCL: (KnowledgeBase) Removed %s from the knowledge base.", s.c_str());
	return true;
}

bool KnowledgeBase::removeAllGoals()
{
	ROS_INFO("KCL: (KnowledgeBase) Removing all goals...");
	
	// Remove all current goals.
	rosplan_knowledge_msgs::GetAttributeService gat;
	if (!get_current_goals_client_.call(gat))
	{
		ROS_ERROR("KCL: (KnowledgeBase) Failed to get all current goals.");
		return false;
	}
	
	rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
	knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_GOAL;
	for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = gat.response.attributes.begin(); ci != gat.response.attributes.end(); ++ci)
	{
		knowledge_update_service.request.knowledge = *ci;
		std::string s = toString(*ci);
		
		ROS_INFO("KCL: (KnowledgeBase) Removing the goal: %s.", s.c_str());
		
		if (!update_knowledge_client_.call(knowledge_update_service))
		{
			ROS_ERROR("KCL: (KnowledgeBase) error removing all goals predicate.");
			return false;
		}
	}
	return true;
}

bool KnowledgeBase::getAllFacts(std::vector<rosplan_knowledge_msgs::KnowledgeItem>& store)
{
	ROS_INFO("KCL: (KnowledgeBase) Get all facts.");
	
	rosplan_knowledge_msgs::GetDomainAttributeService get_domain_predicates_service;
	if (!get_domain_predicates_client_.call(get_domain_predicates_service))
	{
		ROS_ERROR("KCL: (KnowledgeBase) error getting all facts.");
		return false;
	}
	
	for (std::vector<rosplan_knowledge_msgs::DomainFormula>::const_iterator ci = get_domain_predicates_service.response.items.begin();
		  ci != get_domain_predicates_service.response.items.end(); ++ci)
	{
		ROS_INFO("KCL: (KnowledgeBase) Getting all facts with the predicate %s.", ci->name.c_str());
		if (!getFacts(store, ci->name))
		{
			return false;
		}
	}
	return true;
}

bool KnowledgeBase::getFacts(std::vector<rosplan_knowledge_msgs::KnowledgeItem>& store, const std::string& predicate)
{
	rosplan_knowledge_msgs::GetAttributeService get_attribute;
	get_attribute.request.predicate_name = predicate;
	if (!get_attribute_client_.call(get_attribute)) {
		ROS_ERROR("KCL: (KnowledgeBase) Failed to recieve the attributes of the predicate '%s'", predicate.c_str());
		return false;
	}
	
	for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = get_attribute.response.attributes.begin();
			ci != get_attribute.response.attributes.end(); ++ci)
	{
		std::string s = toString(*ci);
		ROS_DEBUG("KCL: (KnowledgeBase) Found the fact %s.", s.c_str());
	}
	
	store.insert(store.end(), get_attribute.response.attributes.begin(), get_attribute.response.attributes.end());
	return true;
}

bool KnowledgeBase::isFactTrue(const std::string& predicate, const std::map<std::string, std::string>& parameters, bool is_true)
{
	// Check if this object has been examined.
	rosplan_knowledge_msgs::KnowledgeItem knowledge_item = createFact(predicate, parameters, is_true);
	
	// Query the knowledge base.
	rosplan_knowledge_msgs::KnowledgeQueryService knowledge_query;
	knowledge_query.request.knowledge.push_back(knowledge_item);
	
	std::string s = toString(knowledge_item);
	
	// Check if any of these facts are true.
	if (!query_knowledge_client_.call(knowledge_query))
	{
		ROS_ERROR("KCL: (KnowledgeBase) Could not call the query knowledge server to check whether %s is true.", s.c_str());
		return false;
	}
	
	ROS_ERROR("KCL: (KnowledgeBase) %s is %s.", s.c_str(), knowledge_query.response.all_true ? "true" : "false");
	return knowledge_query.response.all_true;
}

rosplan_knowledge_msgs::KnowledgeItem KnowledgeBase::createFact(const std::string& predicate, const std::map<std::string, std::string>& parameters, bool is_true)
{
	rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
	knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
	
	knowledge_item.attribute_name = predicate;
	knowledge_item.is_negative = !is_true;
	
	for (std::map<std::string, std::string>::const_iterator ci = parameters.begin(); ci != parameters.end(); ++ci)
	{
		diagnostic_msgs::KeyValue kv;
		kv.key = ci->first;
		kv.value = ci->second;
		knowledge_item.values.push_back(kv);
	}
	return knowledge_item;
}

rosplan_knowledge_msgs::KnowledgeItem KnowledgeBase::createFunction(const std::string& predicate, const std::map<std::string, std::string>& parameters, float value)
{
	rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
	knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FUNCTION;
	
	knowledge_item.attribute_name = predicate;
	knowledge_item.function_value = value;
	
	for (std::map<std::string, std::string>::const_iterator ci = parameters.begin(); ci != parameters.end(); ++ci)
	{
		diagnostic_msgs::KeyValue kv;
		kv.key = ci->first;
		kv.value = ci->second;
		knowledge_item.values.push_back(kv);
	}
	return knowledge_item;
}

std::string KnowledgeBase::toString(const rosplan_knowledge_msgs::KnowledgeItem& knowledge_item) const
{
	std::stringstream ss;
	if (knowledge_item.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::INSTANCE)
	{
		ss << "[INSTANCE] " << knowledge_item.instance_name << " of type " << knowledge_item.instance_type;
		return ss.str();
	}
	else if (knowledge_item.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::FACT)
	{
		ss << "[FACT] (";
		if (knowledge_item.is_negative)
			ss << "NOT (";
		ss << knowledge_item.attribute_name;
	}
	else if (knowledge_item.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::FUNCTION)
	{
		ss << "[FUNCTION] (" << knowledge_item.attribute_name;
	}
	else
	{
		std::cout << "[UNKNOWN KNOWLEDGE ITEM TYPE!!!]" << std::endl;
		exit(-1);
	}
	
	for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = knowledge_item.values.begin(); ci != knowledge_item.values.end(); ++ci)
	{
		const diagnostic_msgs::KeyValue& kv = *ci;
		ss << " ?" << kv.key << "=" << kv.value;
	}
	ss << ")";
	if (knowledge_item.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::FACT && knowledge_item.is_negative)
		ss << ")";
	return ss.str();
}

};
