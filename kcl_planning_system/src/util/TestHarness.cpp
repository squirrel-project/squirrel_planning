#include <ros/ros.h>
#include <vector>
#include "planning_knowledge_msgs/GetInstancesOfType.h"
#include "planning_knowledge_msgs/GetAttributesOfInstance.h"
#include "planning_knowledge_msgs/KnowledgeItem.h"
#include <iostream>
#include <fstream>

bool getInstances(planning_knowledge_msgs::GetInstancesOfType::Request  &req, planning_knowledge_msgs::GetInstancesOfType::Response &res)
{

	ros::NodeHandle n;
	ROS_INFO("Sending getInstances.");
	
	if(req.name.compare("gripper")==0) { 
		res.instances.push_back("g1");
	}

	if(req.name.compare("block")==0) {
		res.instances.push_back("b1");
		res.instances.push_back("b2");
		res.instances.push_back("b3");
	}

	return true;
}

bool getInstanceAttr(planning_knowledge_msgs::GetAttributesOfInstance::Request  &req, planning_knowledge_msgs::GetAttributesOfInstance::Response &res)
{

	ros::NodeHandle n;

	ROS_INFO("Sending getInstanceAttr response.");

	if(req.type_name.compare("gripper")==0) {

		{ // predicate attributes
			planning_knowledge_msgs::KnowledgeItem attr;
			attr.knowledge_type = planning_knowledge_msgs::KnowledgeItem::ATTRIBUTE;
			attr.instance_type = req.type_name;
			attr.instance_name = req.instance_name;
			attr.attribute_name = "empty";
			diagnostic_msgs::KeyValue pair;
			pair.key = "g";
			pair.value = attr.instance_name;
			attr.values.push_back(pair);
			res.attributes.push_back(attr);
		}

		{ // other attributes
			planning_knowledge_msgs::KnowledgeItem attr;
			attr.knowledge_type = planning_knowledge_msgs::KnowledgeItem::ATTRIBUTE;
			attr.instance_type = req.type_name;
			attr.instance_name = req.instance_name;
			attr.attribute_name = "pose";
			diagnostic_msgs::KeyValue pair;
			pair.key = "pose_hash";
			pair.value = "000000";
			attr.values.push_back(pair);
			res.attributes.push_back(attr);
		}
	}

	if(req.type_name.compare("block")==0) {

		{ // predicate attributes
			planning_knowledge_msgs::KnowledgeItem attr_onfloor;
			attr_onfloor.knowledge_type = planning_knowledge_msgs::KnowledgeItem::ATTRIBUTE;
			attr_onfloor.instance_type = req.type_name;
			attr_onfloor.instance_name = req.instance_name;
			attr_onfloor.attribute_name = "onfloor";
			diagnostic_msgs::KeyValue pair;
			pair.key = "b";
			pair.value = attr_onfloor.instance_name;
			attr_onfloor.values.push_back(pair);
			res.attributes.push_back(attr_onfloor);

			planning_knowledge_msgs::KnowledgeItem attr_clear;
			attr_clear.knowledge_type = planning_knowledge_msgs::KnowledgeItem::ATTRIBUTE;
			attr_clear.instance_type = req.type_name;
			attr_clear.instance_name = req.instance_name;
			attr_clear.attribute_name = "clear";
			attr_clear.values.push_back(pair);
			res.attributes.push_back(attr_clear);
		}
	}

	return true;
}

bool getCurrentGoals(planning_knowledge_msgs::GetAttributesOfInstance::Request  &req, planning_knowledge_msgs::GetAttributesOfInstance::Response &res)
{

	ros::NodeHandle n;

	ROS_INFO("Sending getCurrentGoals response.");

	if(req.instance_name.compare("b1")==0) {

		{ // predicate attributes
			planning_knowledge_msgs::KnowledgeItem attr_on1;
			attr_on1.knowledge_type = planning_knowledge_msgs::KnowledgeItem::ATTRIBUTE;
			attr_on1.instance_type = req.type_name;
			attr_on1.instance_name = req.instance_name;
			attr_on1.attribute_name = "on";
			diagnostic_msgs::KeyValue pair_top;
			pair_top.key = "b1"; pair_top.value = "b1";
			attr_on1.values.push_back(pair_top);
			diagnostic_msgs::KeyValue pair_bottom;
			pair_bottom.key = "b2"; pair_bottom.value = "b2";
			attr_on1.values.push_back(pair_bottom);
			res.attributes.push_back(attr_on1);
		}
	}

	if(req.instance_name.compare("b2")==0) {

		{ // predicate attributes
			planning_knowledge_msgs::KnowledgeItem attr_on1;
			attr_on1.knowledge_type = planning_knowledge_msgs::KnowledgeItem::ATTRIBUTE;
			attr_on1.instance_type = req.type_name;
			attr_on1.instance_name = req.instance_name;
			attr_on1.attribute_name = "on";
			diagnostic_msgs::KeyValue pair_top;
			pair_top.key = "b1"; pair_top.value = "b2";
			attr_on1.values.push_back(pair_top);
			diagnostic_msgs::KeyValue pair_bottom;
			pair_bottom.key = "b2"; pair_bottom.value = "b3";
			attr_on1.values.push_back(pair_bottom);
			res.attributes.push_back(attr_on1);
		}
	}

	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "KCL_rosplan_harness");
	ros::NodeHandle n;
	ros::ServiceServer service3 = n.advertiseService("/kcl_rosplan/get_type_instances", getInstances);
	ros::ServiceServer service4 = n.advertiseService("/kcl_rosplan/get_instance_attributes", getInstanceAttr);
	ros::ServiceServer service5 = n.advertiseService("/kcl_rosplan/get_current_goals", getCurrentGoals);
	ROS_INFO("Ready to receive.");
	ros::spin();

	return 0;
}
