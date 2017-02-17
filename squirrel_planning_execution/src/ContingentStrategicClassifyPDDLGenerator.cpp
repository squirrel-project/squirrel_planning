#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <stdlib.h>
#include <map>
#include <set>
#include <string>
#include <ros/ros.h>

#include "squirrel_planning_execution/ContingentStrategicClassifyPDDLGenerator.h"

namespace KCL_rosplan {

void ContingentStrategicClassifyPDDLGenerator::generateProblemFile(const std::string& file_name, const KnowledgeBase& current_knowledge_base, const std::vector<const KnowledgeBase*>& knowledge_base, const Location& robot_location, const std::vector<Location*>& locations, const std::vector<Object*>& objects, unsigned int max_classification_attemps)
{
	std::vector<const State*> states;
	for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_base.begin(); ci != knowledge_base.end(); ++ci)
	{
		const KnowledgeBase* knowledge_base = *ci;
		for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
		{
			const State* state = *ci;
			states.push_back(state);
		}
	}
	
	std::cout << "Created " << states.size() << " states. About to write to: " << file_name << std::endl;
	
	std::ofstream myfile;
	myfile.open(file_name.c_str());
	myfile << "(define (problem squirrel)" << std::endl;
	myfile << "(:domain classify_objects)" << std::endl;
	myfile << std::endl;
	
	myfile << "(:init" << std::endl;
	myfile << "\t(resolve-axioms)" << std::endl;
	myfile << "\t(lev l0)" << std::endl;
	myfile << "\t(next l0 l1)" << std::endl;
	//myfile << "\t(next l1 l2)" << std::endl;
	
	for (unsigned int i = 0; i < max_classification_attemps; ++i)
	{
		myfile << "\t(plus c" << i << " c" << (i + 1) << ")" << std::endl;
	}
	
	myfile << "\t(current_kb " << current_knowledge_base.name_ << ")" << std::endl;
	
	const Location* clear_location = NULL;
	for (std::vector<Location*>::const_iterator ci = locations.begin(); ci != locations.end(); ++ci)
	{
		const Location* location = *ci;
		if (location->is_clear_)
		{
			clear_location = location;
		}
	}
	
	std::cout << "Create facts that are true for the current knowledge base. " << current_knowledge_base.states_.size() << std::endl;
	for (std::vector<const State*>::const_iterator ci = current_knowledge_base.states_.begin(); ci != current_knowledge_base.states_.end(); ++ci)
	{
		const State* state = *ci;
		//myfile << "\t(part-of " << state->state_name_ << " " << current_knowledge_base.name_ << ")" << std::endl;
		myfile << "\t(m " << state->state_name_ << ")" << std::endl;
		myfile << "\t(robot_at robot " << robot_location.name_ << " " << state->state_name_ << ")" << std::endl;
		myfile << "\t(gripper_empty robot " << " " << state->state_name_ << ")" << std::endl;
		
		// All the objects are clear inially.
		for (std::vector<Object*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
		{
			const Object* object = *ci;
			myfile << "\t(object_at " << object->name_ << " " << object->location_->name_ << " " << state->state_name_ << ")" << std::endl;
			//myfile << "\t(remaining_examination_attempts " << object->name_ << " c" << max_counter << " " << state->state_name_ << ")" << std::endl;
		}
	}
	
	for (std::vector<Location*>::const_iterator ci = locations.begin(); ci != locations.end(); ++ci)
	{
		const Location* location = *ci;
		for (std::vector<const Location*>::const_iterator ci = location->connected_locations_.begin(); ci != location->connected_locations_.end(); ++ci)
		{
			const Location* location2 = *ci;
			if (location == location2) continue;
			myfile << "\t(connected " << location->name_ << " " << location2->name_ << ")" << std::endl;
		}
		
		for (std::vector<const Location*>::const_iterator ci = location->near_locations_.begin(); ci != location->near_locations_.end(); ++ci)
		{
			const Location* location2 = *ci;
			if (location == location2) continue;
			myfile << "\t(near " << location->name_ << " " << location2->name_ << ")" << std::endl;
		}
		
		if (location->is_clear_)
		{
			myfile << "\t(clear_area " << location->name_ << ")" << std::endl;
		}
	}
	
	for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_base.begin(); ci != knowledge_base.end(); ++ci)
	{
		const KnowledgeBase* knowledge_base = *ci;
		
		for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
		{
			const State* state = *ci;
			myfile << "\t(part-of " << state->state_name_ << " " << knowledge_base->name_ << ")" << std::endl;
			
			for (std::map<const Object*, unsigned int>::const_iterator ci = state->classifiable_at_attempt_.begin(); ci != state->classifiable_at_attempt_.end(); ++ci)
			{
				const Object* object = (*ci).first;
				unsigned int classiable_on_attempt = (*ci).second;
				
				myfile << "\t(classifiable_on_attempt " << object->name_ << " c" << classiable_on_attempt << " " << state->state_name_ << ")" << std::endl;
				myfile << "\t(current_counter " << object->name_ << " c0 " << state->state_name_ << ")" << std::endl;
				myfile << "\t(contains " << object->name_ << " " << state->state_name_ << ")" << std::endl;
			}
		}
		
		for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_base->children_.begin(); ci != knowledge_base->children_.end(); ++ci)
		{
			myfile << "\t(parent " << knowledge_base->name_ << " " << (*ci)->name_ << ")" << std::endl;
		}
	}
	myfile << ")" << std::endl;
	myfile << "(:goal (and" << std::endl;
	for (std::vector<Object*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
	{
		const Object* object = *ci;
		for (std::vector<const State*>::const_iterator ci = current_knowledge_base.states_.begin(); ci != current_knowledge_base.states_.end(); ++ci)
		{
			myfile << "\t(classified " << object->name_ << " " << (*ci)->state_name_ << ")" << std::endl;
		}
	}
	myfile << ")" << std::endl;
	myfile << ")" << std::endl;
	myfile << ")" << std::endl;
	myfile.close();
}

void ContingentStrategicClassifyPDDLGenerator::generateDomainFile(const std::string& file_name, const KnowledgeBase& current_knowledge_base, const std::vector<const KnowledgeBase*>& knowledge_bases, const Location& robot_location, const std::vector<Location*>& locations, const std::vector<Object*>& objects, unsigned int max_classification_attemps)
{
	std::vector<const State*> states;
	for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_bases.begin(); ci != knowledge_bases.end(); ++ci)
	{
		const KnowledgeBase* knowledge_base = *ci;
		for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
		{
			const State* state = *ci;
			states.push_back(state);
		}
	}
	
	std::ofstream myfile;
	myfile.open (file_name.c_str());
	myfile << "(define (domain classify_objects)" << std::endl;
	myfile << "(:requirements :typing :conditional-effects :negative-preconditions :disjunctive-preconditions)" << std::endl;
	myfile << std::endl;
	myfile << "(:types" << std::endl;
	myfile << "\twaypoint robot object" << std::endl;
	myfile << "\tlevel" << std::endl;
	myfile << "\tstate" << std::endl;
	myfile << "\tknowledgebase" << std::endl;
	myfile << "\tcounter" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	
	myfile << "(:constants" << std::endl;
	myfile << "\t; Waypoints." << std::endl;
	
	for (std::vector<Location*>::const_iterator ci = locations.begin(); ci != locations.end(); ++ci)
	{
		myfile << "\t" << (*ci)->name_ << " - waypoint" << std::endl;
	}

	myfile << "\t; The objects." << std::endl;
	for (std::vector<Object*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
	{
		myfile << "\t" << (*ci)->name_ << " - object" << std::endl;
	}
	
	myfile << "\t; The counters." << std::endl;
	for (unsigned int i = 0; i <= max_classification_attemps; ++i)
	{
		myfile << "\tc" << i << " - counter" << std::endl;
		myfile << "\tl" << i << " - LEVEL" << std::endl;
	}
	
	myfile << "\t; The robot." << std::endl;
	myfile << "\trobot - robot" << std::endl;
	
	myfile << "\t; All the states." << std::endl;
	for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_bases.begin(); ci != knowledge_bases.end(); ++ci)
	{
		const KnowledgeBase* knowledge_base = *ci;
		for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
		{
			myfile << "\t" << (*ci)->state_name_ << " - state" << std::endl;
		}
	}

	myfile << "\t; The knowledge bases" << std::endl;
	for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_bases.begin(); ci != knowledge_bases.end(); ++ci)
	{
		const KnowledgeBase* knowledge_base = *ci;
		myfile << "\t" << knowledge_base->name_ << " - knowledgebase" << std::endl;
	}

	myfile << ")" << std::endl;
	myfile << std::endl;
	
	myfile << "(:predicates" << std::endl;
	myfile << "\t(robot_at ?v - robot ?wp - waypoint ?s - state)" << std::endl;
	myfile << "\t(Rrobot_at ?v - robot ?wp - waypoint ?s - state)" << std::endl;
	myfile << "\t(object_at ?o - object ?wp - waypoint ?s - state)" << std::endl;
	myfile << "\t(Robject_at ?o - object ?wp - waypoint ?s - state)" << std::endl;
	myfile << "\t(gripper_empty ?v - robot ?s - state)" << std::endl;
	myfile << "\t(Rgripper_empty ?v - robot ?s - state)" << std::endl;
	myfile << "\t(holding ?v - robot ?o - object ?s - state)" << std::endl;
	myfile << "\t(Rholding ?v - robot ?o - object ?s - state)" << std::endl;
	myfile << "\t(classified ?o - object ?s - state)" << std::endl;
	myfile << "\t(Rclassified ?o - object ?s - state)" << std::endl;
	myfile << "\t(cleared ?o - object ?s - state)" << std::endl;
	myfile << "\t(Rcleared ?o - object ?s - state)" << std::endl;
	
	myfile << "\t(connected ?from ?to - waypoint)" << std::endl;
	myfile << "\t(clear_area ?w - waypoint)" << std::endl;
	myfile << "\t(near ?wp1 ?wp2 - waypoint)" << std::endl;
	
	myfile << "\t(classifiable_on_attempt ?o - object ?c - counter ?s - state)" << std::endl;
	myfile << "\t(Rclassifiable_on_attempt ?o - object ?c - counter ?s - state)" << std::endl;
	
	myfile << "\t(plus ?c ?c2 - counter)" << std::endl;
	myfile << "\t(contains ?o - object ?s - state)" << std::endl;
	myfile << "\t(current_counter ?o - object ?c - counter ?s - state)" << std::endl;
	myfile << "\t(Rcurrent_counter ?o - object ?c - counter ?s - state)" << std::endl;
	
	myfile << std::endl;
	myfile << "\t;; Bookkeeping predicates." << std::endl;
	myfile << "\t(part-of ?s - state ?kb - knowledgebase)" << std::endl;
	myfile << "\t(current_kb ?kb - knowledgebase)" << std::endl;
	myfile << "\t(parent ?kb ?kb2 - knowledgebase)" << std::endl;
	myfile << "\t(next ?l ?l2 - level)" << std::endl;
	myfile << "\t(lev ?l - LEVEL)" << std::endl;
	myfile << "\t(m ?s - STATE)" << std::endl;
	myfile << "\t(stack ?s - STATE ?l - LEVEL)" << std::endl;
	
	myfile << "\t(resolve-axioms)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	
	/**
	 * PICK-UP OBJECT.
	 *
	myfile << "(:action pickup_object" << std::endl;
	myfile << "\t:parameters (?v - robot ?wp - waypoint ?o - object)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	myfile << "\t\t(not (resolve-axioms))" << std::endl;
	for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_bases.begin(); ci != knowledge_bases.end(); ++ci)
	{
		const KnowledgeBase* knowledge_base = *ci;
		for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
		{
			myfile << "\t\t(Rrobot_at ?v ?wp " << (*ci)->state_name_ << ")" << std::endl;
			myfile << "\t\t(Robject_at ?o ?wp " << (*ci)->state_name_ << ")" << std::endl;
			myfile << "\t\t(Rgripper_empty ?v " << (*ci)->state_name_ << ")" << std::endl;
		}
	}
	
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;
	myfile << "\t\t;; For every state ?s" << std::endl;
	
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		myfile << "\t\t(when (m " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t(and" << std::endl;
		myfile << "\t\t\t\t(not (gripper_empty ?v " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(not (Rgripper_empty ?v " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(not (object_at ?o ?wp " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(not (Robject_at ?o ?wp " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(not (cleared ?o " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(not (Rcleared ?o " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(holding ?v ?o " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t\t(Rholding ?v ?o " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t)" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	
	**
	 * PUT-DOWN OBJECT.
	 *
	myfile << "(:action putdown_object" << std::endl;
	myfile << "\t:parameters (?v - robot ?wp - waypoint ?o - object)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	myfile << "\t\t(not (resolve-axioms))" << std::endl;
	for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_bases.begin(); ci != knowledge_bases.end(); ++ci)
	{
		const KnowledgeBase* knowledge_base = *ci;
		for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
		{
			myfile << "\t\t(Rrobot_at ?v ?wp " << (*ci)->state_name_ << ")" << std::endl;
			myfile << "\t\t(Rholding ?v ?o " << (*ci)->state_name_ << ")" << std::endl;
		}
	}
	
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;
	myfile << "\t\t;; For every state ?s" << std::endl;
	
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		myfile << "\t\t(when (m " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t(and" << std::endl;
		myfile << "\t\t\t\t(not (holding ?v ?o " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(not (Rholding ?v ?o " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(gripper_empty ?v " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t\t(Rgripper_empty ?v " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t\t(object_at ?o ?wp " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t\t(Robject_at ?o ?wp " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t)" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	*/
	/**
	 * GOTO WAYPOINT.
	 */
	myfile << "(:action goto_waypoint" << std::endl;
	myfile << "\t:parameters (?v - robot ?from ?to - waypoint)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	myfile << "\t\t(connected ?from ?to)" << std::endl;
	
	for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_bases.begin(); ci != knowledge_bases.end(); ++ci)
	{
		const KnowledgeBase* knowledge_base = *ci;
		for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
		{
			myfile << "\t\t(Rrobot_at ?v ?from " << (*ci)->state_name_ << ")" << std::endl;
		}
	}
	
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;
	myfile << "\t\t;; For every state ?s" << std::endl;
	
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		myfile << "\t\t(when (m " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t(and" << std::endl;

		myfile << "\t\t\t\t(not (robot_at ?v ?from " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(not (Rrobot_at ?v ?from " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(robot_at ?v ?to " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t\t(Rrobot_at ?v ?to " << (*ci)->state_name_ << ")" << std::endl;

		myfile << "\t\t\t)" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	
	/**
	 * PUSH OBJECT.
	 */
	myfile << "(:action push_object" << std::endl;
	myfile << "\t:parameters (?v - robot ?ob - object ?from ?to ?near_from - waypoint)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	myfile << "\t\t(connected ?from ?to)" << std::endl;
	myfile << "\t\t(not (resolve-axioms))" << std::endl;
	for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_bases.begin(); ci != knowledge_bases.end(); ++ci)
	{
		const KnowledgeBase* knowledge_base = *ci;
		for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
		{
			myfile << "\t\t(Rrobot_at ?v ?near_from " << (*ci)->state_name_ << ")" << std::endl;
			myfile << "\t\t(Robject_at ?ob ?from " << (*ci)->state_name_ << ")" << std::endl;
		}
		myfile << "\t\t(near ?near_from ?from)" << std::endl;
	}
	
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;
	myfile << "\t\t;; For every state ?s" << std::endl;
	
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		myfile << "\t\t(when (m " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t(and" << std::endl;
		
		myfile << "\t\t\t\t(not (robot_at ?v ?near_from " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(not (Rrobot_at ?v ?near_from " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(not (object_at ?ob ?from " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(not (Robject_at ?ob ?from " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(robot_at ?v ?to " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t\t(Rrobot_at ?v ?to " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t\t(object_at ?ob ?to " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t\t(Robject_at ?ob ?to " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t)" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	
	/**
	 * CLEAR OBJECT
	 */
	myfile << "(:action clear_object" << std::endl;
	myfile << "\t:parameters (?ob - object ?w - waypoint)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	myfile << "\t\t(clear_area ?w)" << std::endl;
	myfile << "\t\t(not (resolve-axioms))" << std::endl;
	for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_bases.begin(); ci != knowledge_bases.end(); ++ci)
	{
		const KnowledgeBase* knowledge_base = *ci;
		for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
		{
			myfile << "\t\t(Robject_at ?ob ?w " << (*ci)->state_name_ << ")" << std::endl;
		}
	}
	
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;
	myfile << "\t\t;; For every state ?s" << std::endl;
	
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		myfile << "\t\t(when (m " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t(and" << std::endl;
		
		myfile << "\t\t\t\t(cleared ?ob " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t\t(Rcleared ?ob " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t)" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	
	/**********************
	 * SENSE actions.     *
	 *********************/
	
	/**
	 * Abstract classify action.
	 */
	myfile << ";; Attempt to classify the object." << std::endl;
	myfile << "(:action observe-classifiable_on_attempt" << std::endl;

	myfile << "\t:parameters (?o - object ?c - counter ?v - robot ?from - waypoint ?c2 - counter ?l ?l2 - level ?kb - knowledgebase)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	myfile << "\t\t(not (resolve-axioms))" << std::endl;
	myfile << "\t\t(next ?l ?l2)" << std::endl;
	myfile << "\t\t(lev ?l)" << std::endl;
	myfile << "\t\t(plus ?c ?c2)" << std::endl;
	
	myfile << "\t\t(current_kb ?kb)" << std::endl;
	myfile << "\t\t(not (current_kb basis_kb))" << std::endl;
	myfile << std::endl;
	//myfile << "\t\t(clear_area ?view)" << std::endl;
	
	for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_bases.begin(); ci != knowledge_bases.end(); ++ci)
	{
		const KnowledgeBase* knowledge_base = *ci;
		for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
		{
			myfile << "\t\t(Rrobot_at ?v ?from " << (*ci)->state_name_ << ")" << std::endl;
			myfile << "\t\t(Robject_at ?o ?from " << (*ci)->state_name_ << ")" << std::endl;
			myfile << "\t\t(Rcleared ?o " << (*ci)->state_name_ << ")" << std::endl;
			myfile << "\t\t(Rcurrent_counter ?o ?c " << (*ci)->state_name_ << ")" << std::endl;
		}
	}

	myfile << "\t\t;; This action is only applicable if there are world states where the outcome can be different." << std::endl;
	
	myfile << "\t\t(exists (?s - state) (and (m ?s) (classifiable_on_attempt ?o ?c ?s) (part-of ?s ?kb)))" << std::endl;
	myfile << "\t\t(exists (?s - state) (and (m ?s) (not (classifiable_on_attempt ?o ?c ?s)) (part-of ?s ?kb)))" << std::endl;;
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;
	myfile << "\t\t(not (lev ?l))" << std::endl;
	myfile << "\t\t(lev ?l2)" << std::endl;
	myfile << std::endl;
	myfile << "\t\t;; For every state ?s" << std::endl;
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		myfile << "\t\t(when (and (m " << (*ci)->state_name_ << ") (not (classifiable_on_attempt ?o ?c " << (*ci)->state_name_ << ")))" << std::endl;
		myfile << "\t\t\t(and (stack " << (*ci)->state_name_ << " ?l) (not (m " << (*ci)->state_name_ << ")))" << std::endl;
		myfile << "\t\t)" << std::endl;
		
		myfile << "\t\t(when (and (m " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t(current_counter ?o ?c2 " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	myfile << "\t\t(resolve-axioms)" << std::endl;
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	
	/**
	 * A fake sense action that is there as a failsafe. Sometimes an object cannot be classified even after ', unsigned int max_classification_attemps' attempts
	 * when this happens we stop our attempts to classify the object. Later on we might decide to learn about this object, for now 
	 * we just stop.
	 */
	myfile << "(:action finalise_classification_success" << std::endl;
	myfile << "\t:parameters (?ob - object ?c - counter ?w - waypoint)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	myfile << "\t\t(not (= ?c c" << max_classification_attemps << " ))" << std::endl;
	myfile << "\t\t(not (resolve-axioms))" << std::endl;
	
	for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_bases.begin(); ci != knowledge_bases.end(); ++ci)
	{
		const KnowledgeBase* knowledge_base = *ci;
		for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
		{
			myfile << "\t\t(Rcurrent_counter ?ob ?c " << (*ci)->state_name_ << ")" << std::endl;
			myfile << "\t\t(Rclassifiable_on_attempt ?ob ?c " << (*ci)->state_name_ << ")" << std::endl;
		}
	}
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;
	myfile << "\t\t;; For every state ?s" << std::endl;
	
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		myfile << "\t\t(when (m " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t(and" << std::endl;
		
		myfile << "\t\t\t\t(classified ?ob " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t\t(Rclassified ?ob " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t)" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	
	/**
	 * A fake sense action that is there as a failsafe. Sometimes an object cannot be classified even after ', unsigned int max_classification_attemps' attempts
	 * when this happens we stop our attempts to classify the object. Later on we might decide to learn about this object, for now 
	 * we just stop.
	 */
	myfile << "(:action finalise_classification_fail" << std::endl;
	myfile << "\t:parameters (?ob - object ?w - waypoint)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	myfile << "\t\t(not (resolve-axioms))" << std::endl;
	
	for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_bases.begin(); ci != knowledge_bases.end(); ++ci)
	{
		const KnowledgeBase* knowledge_base = *ci;
		for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
		{
			myfile << "\t\t(Rcurrent_counter ?ob c" << max_classification_attemps << " " << (*ci)->state_name_ << ")" << std::endl;
		}
	}

	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;
	myfile << "\t\t;; For every state ?s" << std::endl;
	
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		myfile << "\t\t(when (m " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t(and" << std::endl;
		
		myfile << "\t\t\t\t(classified ?ob " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t\t(Rclassified ?ob " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t)" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	
	/**
	 * POP action.
	 */
	myfile << ";; Exit the current branch." << std::endl;
	myfile << "(:action pop" << std::endl;
	myfile << "\t:parameters (?l ?l2 - level ?o - object)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	myfile << "\t\t(lev ?l)" << std::endl;
	myfile << "\t\t(next ?l2 ?l)" << std::endl;
	myfile << "\t\t(not (resolve-axioms))" << std::endl;
	
	//myfile << "\t\t(and" << std::endl;
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		const State* state = *ci;
		myfile << "\t\t\t(or " << std::endl;
		myfile << "\t\t\t\t(not (m " << state->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(not (contains ?o " << state->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(classified ?o " << state->state_name_ << ")" << std::endl;
		myfile << "\t\t\t)" << std::endl;
	}
	//myfile << "\t\t)" << std::endl;
	
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and " << std::endl;
	myfile << "\t\t(not (lev ?l))" << std::endl;
	myfile << "\t\t(lev ?l2)" << std::endl;
	myfile << "\t\t(resolve-axioms)" << std::endl;

	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		myfile << "\t\t(when (m " << (*ci)->state_name_ << ") " << std::endl;
		myfile << "\t\t\t(not (m " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t)" << std::endl;
		myfile << "\t\t(when (stack " << (*ci)->state_name_ << " ?l2)" << std::endl;
		myfile << "\t\t\t(and " << std::endl;
		myfile << "\t\t\t\t(m " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t\t(not (stack " << (*ci)->state_name_ << " ?l2))" << std::endl;
		myfile << "\t\t\t)" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;

	myfile << ";; Resolve the axioms manually." << std::endl;
	myfile << "(:action ramificate" << std::endl;
	myfile << "\t:parameters ()" << std::endl;
	myfile << "\t:precondition (resolve-axioms)" << std::endl;
	myfile << "\t:effect (and " << std::endl;
	myfile << "\t\t(not (resolve-axioms))" << std::endl;
	myfile << "\t\t;; For every state ?s" << std::endl;
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		const State* state = *ci;
		
		myfile << "\t\t(when (or (gripper_empty robot " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
		myfile << "\t\t\t(Rgripper_empty robot " << state->state_name_ << ")" << std::endl;
		myfile << "\t\t)" << std::endl;
		
		myfile << "\t\t(when (and (not (gripper_empty robot " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
		myfile << "\t\t\t(not (Rgripper_empty robot " << state->state_name_ << "))" << std::endl;
		myfile << "\t\t)" << std::endl;
		
		for (std::vector<Location*>::const_iterator ci = locations.begin(); ci != locations.end(); ++ci)
		{
			const Location* location = *ci;
			
			myfile << "\t\t(when (or (robot_at robot " << location->name_ << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
			myfile << "\t\t\t(Rrobot_at robot " << location->name_ << " " << state->state_name_ << ")" << std::endl;
			myfile << "\t\t)" << std::endl;
			
			myfile << "\t\t(when (and (not (robot_at robot " << location->name_ << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t\t(not (Rrobot_at robot " << location->name_ << " " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t)" << std::endl;
		}
		
		for (std::vector<Object*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
		{
			const Object* object = *ci;
			myfile << "\t\t(when (or (holding robot " << object->name_ << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
			myfile << "\t\t\t(Rholding robot " << object->name_ << " " << state->state_name_ << ")" << std::endl;
			myfile << "\t\t)" << std::endl;
			
			myfile << "\t\t(when (and (not (holding robot " << object->name_ << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t\t(not (Rholding robot " << object->name_ << " " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t)" << std::endl;
			
			myfile << "\t\t(when (or (cleared " << object->name_ << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
			myfile << "\t\t\t(Rcleared " << object->name_ << " " << state->state_name_ << ")" << std::endl;
			myfile << "\t\t)" << std::endl;
			
			myfile << "\t\t(when (and (not (cleared " << object->name_ << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t\t(not (Rcleared " << object->name_ << " " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t)" << std::endl;
			
			myfile << "\t\t(when (or (classified " << object->name_ << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
			myfile << "\t\t\t(Rclassified " << object->name_ << " " << state->state_name_ << ")" << std::endl;
			myfile << "\t\t)" << std::endl;
			
			myfile << "\t\t(when (and (not (classified " << object->name_ << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t\t(not (Rclassified " << object->name_ << " " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t)" << std::endl;
			
			for (unsigned int counter = 0; counter <= max_classification_attemps; ++counter)
			{
				myfile << "\t\t(when (or (current_counter " << object->name_ << " c" << counter << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
				myfile << "\t\t\t(Rcurrent_counter " << object->name_ << " c" << counter << " " << state->state_name_ << ")" << std::endl;
				myfile << "\t\t)" << std::endl;
				
				myfile << "\t\t(when (and (not (current_counter " << object->name_ << " c" << counter << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t(not (Rcurrent_counter " << object->name_ << " c" << counter << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t)" << std::endl;
			}
			
			for (std::vector<Location*>::const_iterator ci = locations.begin(); ci != locations.end(); ++ci)
			{
				const Location* location = *ci;
				
				myfile << "\t\t(when (or (object_at " << object->name_ << " " << location->name_ << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
				myfile << "\t\t\t(Robject_at " << object->name_ << " " << location->name_ << " " << state->state_name_ << ")" << std::endl;
				myfile << "\t\t)" << std::endl;
				
				myfile << "\t\t(when (and (not (object_at " << object->name_ << " " << location->name_ << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t(not (Robject_at " << object->name_ << " " << location->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t)" << std::endl;
			}
			
			for (unsigned int i = 0; i < max_classification_attemps; ++i)
			{
				myfile << "\t\t(when (or (classifiable_on_attempt " << object->name_ << " c" << i << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
				myfile << "\t\t\t(Rclassifiable_on_attempt " << object->name_ << " c" << i << " " << state->state_name_ << ")" << std::endl;
				myfile << "\t\t)" << std::endl;
				
				myfile << "\t\t(when (and (not (classifiable_on_attempt " << object->name_ << " c" << i << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t(not (Rclassifiable_on_attempt " << object->name_ << " c" << i<< " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t)" << std::endl;
			}
		}
	}
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	
	myfile << ";; Move 'down' into the knowledge base." << std::endl;
	myfile << "(:action assume_knowledge" << std::endl;
	myfile << "\t:parameters (?old_kb ?new_kb - knowledgebase)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	myfile << "\t\t(not (resolve-axioms))" << std::endl;
	myfile << "\t\t(current_kb ?old_kb)" << std::endl;
	myfile << "\t\t(parent ?old_kb ?new_kb)" << std::endl;
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;
	myfile << "\t\t(not (current_kb ?old_kb))" << std::endl;
	myfile << "\t\t(current_kb ?new_kb)" << std::endl;
	myfile << "\t\t(resolve-axioms)" << std::endl;
	myfile << std::endl;
	
	myfile << "\t\t;; Now we need to delete all knowledge from the old_kb and insert it to" << std::endl;
	myfile << "\t\t;; the new_kb level." << std::endl;

	myfile << "\t\t;; For every state ?s, ?s2" << std::endl;
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		const State* state = *ci;
		
		// Get rid of all states that are not part of this knowledge base.
		myfile << "\t\t(when (and (m " << state->state_name_ << ") (not (part-of " << state->state_name_ << " ?new_kb)))" << std::endl;
		myfile << "\t\t\t(not (m " << state->state_name_ << "))" << std::endl;
		myfile << "\t\t)" << std::endl;
		
		// Enable the states that are encapsulated in this knowledge base.
		myfile << "\t\t(when (part-of " << state->state_name_ << " ?new_kb)" << std::endl;
		myfile << "\t\t\t(and (m " << state->state_name_ << "))" << std::endl;
		myfile << "\t\t)" << std::endl;
		
		// Copy all knowledge that is part of ?old_kb to all the new states.
		for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
		{
			const State* state2 = *ci;
			
			myfile << "\t\t(when (and (part-of " << state->state_name_ << " ?old_kb) (gripper_empty robot " << state->state_name_ << ") (part-of " << state2->state_name_ << " ?new_kb))" << std::endl;
			myfile << "\t\t\t(and " << std::endl;
			myfile << "\t\t\t\t(not (Rgripper_empty robot " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t\t\t(not (gripper_empty robot " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t\t\t(Rgripper_empty robot " << state2->state_name_ << ")" << std::endl;
			myfile << "\t\t\t\t(gripper_empty robot " << state2->state_name_ << ")" << std::endl;
			myfile << "\t\t\t)" << std::endl;
			myfile << "\t\t)" << std::endl;
			
			for (std::vector<Object*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
			{
				const Object* object = *ci;
				myfile << "\t\t(when (and (part-of " << state->state_name_ << " ?old_kb) (holding robot " << object->name_ << " " << state->state_name_ << ") (part-of " << state2->state_name_ << " ?new_kb))" << std::endl;
				myfile << "\t\t\t(and " << std::endl;
				myfile << "\t\t\t\t(not (Rholding robot " << object->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t\t(not (holding robot " << object->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t\t(Rholding robot " << object->name_ << " " << state2->state_name_ << ")" << std::endl;
				myfile << "\t\t\t\t(holding robot " << object->name_ << " " << state2->state_name_ << ")" << std::endl;
				myfile << "\t\t\t)" << std::endl;
				myfile << "\t\t)" << std::endl;
				
				myfile << "\t\t(when (and (part-of " << state->state_name_ << " ?old_kb) (cleared " << object->name_ << " " << state->state_name_ << ") (part-of " << state2->state_name_ << " ?new_kb))" << std::endl;
				myfile << "\t\t\t(and " << std::endl;
				myfile << "\t\t\t\t(not (Rcleared " << object->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t\t(not (cleared " << object->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t\t(Rcleared " << object->name_ << " " << state2->state_name_ << ")" << std::endl;
				myfile << "\t\t\t\t(cleared " << object->name_ << " " << state2->state_name_ << ")" << std::endl;
				myfile << "\t\t\t)" << std::endl;
				myfile << "\t\t)" << std::endl;
				
				myfile << "\t\t(when (and (part-of " << state->state_name_ << " ?old_kb) (classified " << object->name_ << " " << state->state_name_ << ") (part-of " << state2->state_name_ << " ?new_kb))" << std::endl;
				myfile << "\t\t\t(and " << std::endl;
				myfile << "\t\t\t\t(not (Rclassified " << object->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t\t(not (classified " << object->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t\t(Rclassified " << object->name_ << " " << state2->state_name_ << ")" << std::endl;
				myfile << "\t\t\t\t(classified " << object->name_ << " " << state2->state_name_ << ")" << std::endl;
				myfile << "\t\t\t)" << std::endl;
				myfile << "\t\t)" << std::endl;
				
				for (unsigned int counter = 0; counter <= max_classification_attemps; ++counter)
				{
					myfile << "\t\t(when (and (part-of " << state->state_name_ << " ?old_kb) (classifiable_on_attempt " << object->name_ << " c" << counter << " " << state->state_name_ << ") (part-of " << state2->state_name_ << " ?new_kb))" << std::endl;
					myfile << "\t\t\t(and " << std::endl;
					myfile << "\t\t\t\t(not (Rclassifiable_on_attempt " << object->name_ << " c" << counter << " " << state->state_name_ << "))" << std::endl;
					myfile << "\t\t\t\t(not (classifiable_on_attempt " << object->name_ << " c" << counter << " " << state->state_name_ << "))" << std::endl;
					myfile << "\t\t\t\t(Rclassifiable_on_attempt " << object->name_ << " c" << counter << " " << state2->state_name_ << ")" << std::endl;
					myfile << "\t\t\t\t(classifiable_on_attempt " << object->name_ << " c" << counter << " " << state2->state_name_ << ")" << std::endl;
					myfile << "\t\t\t)" << std::endl;
					myfile << "\t\t)" << std::endl;
				}
			}
			
			for (std::vector<Location*>::const_iterator ci = locations.begin(); ci != locations.end(); ++ci)
			{
				const Location* location = *ci;
				
				myfile << "\t\t(when (and (part-of " << state->state_name_ << " ?old_kb) (robot_at robot " << location->name_ << " " << state->state_name_ << ") (part-of " << state2->state_name_ << " ?new_kb))" << std::endl;
				myfile << "\t\t\t(and " << std::endl;
				myfile << "\t\t\t\t(not (Rrobot_at robot " << location->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t\t(not (robot_at robot " << location->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t\t(Rrobot_at robot " << location->name_ << " " << state2->state_name_ << ")" << std::endl;
				myfile << "\t\t\t\t(robot_at robot " << location->name_ << " " << state2->state_name_ << ")" << std::endl;
				myfile << "\t\t\t)" << std::endl;
				myfile << "\t\t)" << std::endl;
				
				for (std::vector<Object*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
				{
					const Object* object = *ci;
					myfile << "\t\t(when (and (part-of " << state->state_name_ << " ?old_kb) (object_at " << object->name_ << " " << location->name_ << " " << state->state_name_ << ") (part-of " << state2->state_name_ << " ?new_kb))" << std::endl;
					myfile << "\t\t\t(and " << std::endl;
					myfile << "\t\t\t\t(not (Robject_at " << object->name_ << " " << location->name_ << " " << state->state_name_ << "))" << std::endl;
					myfile << "\t\t\t\t(not (object_at " << object->name_ << " " << location->name_ << " " << state->state_name_ << "))" << std::endl;
					myfile << "\t\t\t\t(Robject_at " << object->name_ << " " << location->name_ << " " << state2->state_name_ << ")" << std::endl;
					myfile << "\t\t\t\t(object_at " << object->name_ << " " << location->name_ << " " << state2->state_name_ << ")" << std::endl;
					myfile << "\t\t\t)" << std::endl;
					myfile << "\t\t)" << std::endl;
				}
			}
		}
	}
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;

	myfile << ";; Move 'up' into the knowledge base." << std::endl;;
	myfile << "(:action shed_knowledge" << std::endl;
	myfile << "\t:parameters (?old_kb ?new_kb - knowledgebase)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	myfile << "\t\t(not (resolve-axioms))" << std::endl;
	myfile << "\t\t(current_kb ?old_kb)" << std::endl;
	myfile << "\t\t(parent ?new_kb ?old_kb)" << std::endl;
	
	// We can only move back up the knowledge base if there are not states that belong to this knowledge base.
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		myfile << "\t\t(or " << std::endl;
		myfile << "\t\t\t(not (part-of " << (*ci)->state_name_ << " ?old_kb))" << std::endl;
		myfile << "\t\t\t(not (exists (?l - level ) (stack " << (*ci)->state_name_ << " ?l)))" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	
	// Make sure the robot is in the same location.
	myfile << "\t\t(or" << std::endl;
	for (std::vector<Location*>::const_iterator ci = locations.begin(); ci != locations.end(); ++ci)
	{
		const Location* location = *ci;
		myfile << "\t\t\t(and" << std::endl;
		for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
		{
			const State* state = *ci;
			// Make sure the state of the toilets are the same.
			myfile << "\t\t\t\t(or " << std::endl;
			myfile << "\t\t\t\t\t(not (part-of " << (*ci)->state_name_ << " ?old_kb))" << std::endl;
			myfile << "\t\t\t\t\t(robot_at robot " << location->name_ << " " << state->state_name_ << ")" << std::endl;
			myfile << "\t\t\t\t)" << std::endl;
		}
		myfile << "\t\t\t)" << std::endl;
	}
	myfile << "\t\t)" << std::endl;
	
	// Make sure the object is classified.
	//myfile << "\t\t(and" << std::endl;
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		const State* state = *ci;
		for (std::vector<Object*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
		{
			const Object* object = *ci;
			// Make sure the state of the toilets are the same.
			myfile << "\t\t\t(or " << std::endl;
			myfile << "\t\t\t\t(not (part-of " << state->state_name_ << " ?old_kb))" << std::endl;
			myfile << "\t\t\t\t(classified " << object->name_ << " " << state->state_name_ << ")" << std::endl;
			myfile << "\t\t\t\t(not (contains " << object->name_ << " " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t\t)" << std::endl;
		}
	}
	//myfile << "\t\t)" << std::endl;
	
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;
	myfile << "\t\t(not (current_kb ?old_kb))" << std::endl;
	myfile << "\t\t(current_kb ?new_kb)" << std::endl;
	myfile << "\t\t(resolve-axioms)" << std::endl;
	myfile << std::endl;
	
	myfile << "\t\t;; Now we need to push all knowledge that is true for all states part of " << std::endl;
	myfile << "\t\t;; kb_old up to kb_new." << std::endl;
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		const State* state = *ci;
		
		// Make the states that were held in suspection active again.
		myfile << "\t\t(when (part-of " << state->state_name_ << " ?new_kb)" << std::endl;
		myfile << "\t\t\t(and (m " << state->state_name_ << "))" << std::endl;
		myfile << "\t\t)" << std::endl;
		
		// And those that were active, inactive.
		myfile << "\t\t(when (not (part-of " << state->state_name_ << " ?new_kb))" << std::endl;
		myfile << "\t\t\t(and (not (m " << state->state_name_ << ")))" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		const State* state = *ci;
		myfile << "\t\t(when (and " << std::endl;
		myfile << "\t\t\t\t;; For every state ?s, ?s2" << std::endl;
	
		for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
		{
			const State* state2 = *ci;
			myfile << "\t\t\t\t\t(or " << std::endl;
			myfile << "\t\t\t\t\t\t(not (part-of " << state2->state_name_ << " ?old_kb))" << std::endl;
			myfile << "\t\t\t\t\t\t(gripper_empty robot " << state2->state_name_ << ")" << std::endl;
			myfile << "\t\t\t\t\t)" << std::endl;
		}
		myfile << "\t\t\t\t\t(part-of " << state->state_name_ << " ?new_kb)" << std::endl;
		myfile << "\t\t\t)" << std::endl;

		myfile << "\t\t\t;; Conditional effects" << std::endl;
		myfile << "\t\t\t(and " << std::endl;
		for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
		{
			const State* state2 = *ci;
			myfile << "\t\t\t\t(not (gripper_empty robot " << state2->state_name_ << "))" << std::endl;
		}
		myfile << "\t\t\t\t(gripper_empty robot " << state->state_name_ << ")" << std::endl;
		myfile << "\t\t\t)" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	
	for (std::vector<Object*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
	{
		const Object* object = *ci;
		for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
		{
			// Deal with the location of the robot.
			const State* state = *ci;
			myfile << "\t\t(when (and " << std::endl;
			myfile << "\t\t\t\t;; For every state ?s, ?s2" << std::endl;
		
			// Holding.
			for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
			{
				const State* state2 = *ci;
				myfile << "\t\t\t\t\t(or " << std::endl;
				myfile << "\t\t\t\t\t\t(not (part-of " << state2->state_name_ << " ?old_kb))" << std::endl;
				myfile << "\t\t\t\t\t\t(holding robot " << object->name_ << " " << state2->state_name_ << ")" << std::endl;
				myfile << "\t\t\t\t\t)" << std::endl;
				
			}
			myfile << "\t\t\t\t\t(part-of " << state->state_name_ << " ?new_kb)" << std::endl;
			myfile << "\t\t\t)" << std::endl;

			myfile << "\t\t\t;; Conditional effects" << std::endl;
			myfile << "\t\t\t(and " << std::endl;
			for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
			{
				const State* state2 = *ci;
				myfile << "\t\t\t\t(not (holding robot " << object->name_ << " " << state2->state_name_ << "))" << std::endl;
			}
			myfile << "\t\t\t\t(holding robot " << object->name_ << " " << state->state_name_ << ")" << std::endl;
			myfile << "\t\t\t)" << std::endl;
			myfile << "\t\t)" << std::endl;
			
			// Classified.
			myfile << "\t\t(when (and " << std::endl;
			myfile << "\t\t\t\t;; For every state ?s, ?s2" << std::endl;
		
			for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
			{
				const State* state2 = *ci;
				myfile << "\t\t\t\t\t(or " << std::endl;
				myfile << "\t\t\t\t\t\t(not (part-of " << state2->state_name_ << " ?old_kb))" << std::endl;
				myfile << "\t\t\t\t\t\t(classified " << object->name_ << " " << state2->state_name_ << ")" << std::endl;
				myfile << "\t\t\t\t\t)" << std::endl;
				
			}
			myfile << "\t\t\t\t\t(part-of " << state->state_name_ << " ?new_kb)" << std::endl;
			myfile << "\t\t\t)" << std::endl;

			myfile << "\t\t\t;; Conditional effects" << std::endl;
			myfile << "\t\t\t(and " << std::endl;
			for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
			{
				const State* state2 = *ci;
				myfile << "\t\t\t\t(not (classified " << object->name_ << " " << state2->state_name_ << "))" << std::endl;
			}
			myfile << "\t\t\t\t(classified " << object->name_ << " " << state->state_name_ << ")" << std::endl;
			myfile << "\t\t\t)" << std::endl;
			myfile << "\t\t)" << std::endl;
			
			// Cleared.
			myfile << "\t\t(when (and " << std::endl;
			myfile << "\t\t\t\t;; For every state ?s, ?s2" << std::endl;
		
			for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
			{
				const State* state2 = *ci;
				myfile << "\t\t\t\t\t(or " << std::endl;
				myfile << "\t\t\t\t\t\t(not (part-of " << state2->state_name_ << " ?old_kb))" << std::endl;
				myfile << "\t\t\t\t\t\t(cleared " << object->name_ << " " << state2->state_name_ << ")" << std::endl;
				myfile << "\t\t\t\t\t)" << std::endl;
				
			}
			myfile << "\t\t\t\t\t(part-of " << state->state_name_ << " ?new_kb)" << std::endl;
			myfile << "\t\t\t)" << std::endl;

			myfile << "\t\t\t;; Conditional effects" << std::endl;
			myfile << "\t\t\t(and " << std::endl;
			for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
			{
				const State* state2 = *ci;
				myfile << "\t\t\t\t(not (cleared " << object->name_ << " " << state2->state_name_ << "))" << std::endl;
			}
			myfile << "\t\t\t\t(cleared " << object->name_ << " " << state->state_name_ << ")" << std::endl;
			myfile << "\t\t\t)" << std::endl;
			myfile << "\t\t)" << std::endl;
			
			// Deal with the classification attempts on an object.
			for (unsigned int counter = 0; counter < max_classification_attemps; ++counter)
			{
				myfile << "\t\t(when (and " << std::endl;
				myfile << "\t\t\t\t;; For every state ?s, ?s2" << std::endl;
			
				for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
				{
					const State* state2 = *ci;
					myfile << "\t\t\t\t\t(or " << std::endl;
					myfile << "\t\t\t\t\t\t(not (part-of " << state2->state_name_ << " ?old_kb))" << std::endl;
					myfile << "\t\t\t\t\t\t(classifiable_on_attempt " << object->name_ << " c" << counter << " " << state2->state_name_ << ")" << std::endl;
					myfile << "\t\t\t\t\t)" << std::endl;
					
				}
				myfile << "\t\t\t\t\t(part-of " << state->state_name_ << " ?new_kb)" << std::endl;
				myfile << "\t\t\t)" << std::endl;

				myfile << "\t\t\t;; Conditional effects" << std::endl;
				myfile << "\t\t\t(and " << std::endl;
				for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
				{
					const State* state2 = *ci;
					myfile << "\t\t\t\t(not (classifiable_on_attempt " << object->name_ << " c" << counter << " " << state2->state_name_ << "))" << std::endl;
				}
				myfile << "\t\t\t\t(classifiable_on_attempt " << object->name_ << " c" << counter << " " << state->state_name_ << ")" << std::endl;
				myfile << "\t\t\t)" << std::endl;
				myfile << "\t\t)" << std::endl;
			}
		}
	}
	
	// Location of the agent.
	for (std::vector<Location*>::const_iterator ci = locations.begin(); ci != locations.end(); ++ci)
	{
		const Location* location = *ci;
		for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
		{
			// Deal with the location of the robot.
			const State* state = *ci;
			myfile << "\t\t(when (and " << std::endl;
			myfile << "\t\t\t\t;; For every state ?s, ?s2" << std::endl;
		
			for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
			{
				const State* state2 = *ci;
				myfile << "\t\t\t\t(and " << std::endl;
				myfile << "\t\t\t\t\t(or " << std::endl;
				myfile << "\t\t\t\t\t\t(not (part-of " << state2->state_name_ << " ?old_kb))" << std::endl;
				myfile << "\t\t\t\t\t\t(robot_at robot " << location->name_ << " " << state2->state_name_ << ")" << std::endl;
				myfile << "\t\t\t\t\t)" << std::endl;
				myfile << "\t\t\t\t)" << std::endl;
			}
			myfile << "\t\t\t\t\t(part-of " << state->state_name_ << " ?new_kb)" << std::endl;
			myfile << "\t\t\t)" << std::endl;

			myfile << "\t\t\t;; Conditional effects" << std::endl;
			myfile << "\t\t\t(and " << std::endl;
			
			for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
			{
				const State* state2 = *ci;
				myfile << "\t\t\t\t(not (robot_at robot " << location->name_ << " " << state2->state_name_ << "))" << std::endl;
			}
			myfile << "\t\t\t\t(robot_at robot " << location->name_ << " " << state->state_name_ << ")" << std::endl;
			
			myfile << "\t\t\t)" << std::endl;
			myfile << "\t\t)" << std::endl;
			
			// The locations of the objects.
			for (std::vector<Object*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
			{
				const Object* object = *ci;
				
				// Deal with the location of the robot.
				myfile << "\t\t(when (and " << std::endl;
				myfile << "\t\t\t\t;; For every state ?s, ?s2" << std::endl;
			
				for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
				{
					const State* state2 = *ci;
					myfile << "\t\t\t\t(and " << std::endl;
					myfile << "\t\t\t\t\t(or " << std::endl;
					myfile << "\t\t\t\t\t\t(not (part-of " << state2->state_name_ << " ?old_kb))" << std::endl;
					myfile << "\t\t\t\t\t\t(object_at " << object->name_ << " " << location->name_ << " " << state2->state_name_ << ")" << std::endl;
					myfile << "\t\t\t\t\t)" << std::endl;
					myfile << "\t\t\t\t)" << std::endl;
				}
				myfile << "\t\t\t\t\t(part-of " << state->state_name_ << " ?new_kb)" << std::endl;
				myfile << "\t\t\t)" << std::endl;

				myfile << "\t\t\t;; Conditional effects" << std::endl;
				myfile << "\t\t\t(and " << std::endl;
				
				for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
				{
					const State* state2 = *ci;
					myfile << "\t\t\t\t(not (object_at " << object->name_ << " " << location->name_ << " " << state2->state_name_ << "))" << std::endl;
				}
				myfile << "\t\t\t\t(object_at " << object->name_ << " " << location->name_ << " " << state->state_name_ << ")" << std::endl;
				
				myfile << "\t\t\t)" << std::endl;
				myfile << "\t\t)" << std::endl;
			}
		}
	}
	
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	myfile << ")" << std::endl;
	myfile.close();
}

void ContingentStrategicClassifyPDDLGenerator::createPDDL(const std::string& path, const std::string& domain_file, const std::string& problem_file, const std::string& robot_location_predicate, const std::map<std::string, std::string>& object_location_predicates, const std::map<std::string, std::vector<std::string> >& near_waypoint_mapping, unsigned int max_classification_attemps)
{
	std::vector<Location*> locations;
	std::vector<Object*> objects;
	std::map<std::string, Object*> predicate_to_object_mapping;
	
	std::cout << "[ContingentStrategicClassifyPDDLGenerator::createPDDL] " << path << "; domain=" << domain_file << "; problem=" << problem_file << std::endl;
	std::cout << "Robot is at: " << robot_location_predicate << std::endl;
	
	// Create the objects.
	for (std::map<std::string, std::string>::const_iterator ci = object_location_predicates.begin(); ci != object_location_predicates.end(); ++ci)
	{
		const std::string& object_predicate = (*ci).first;
		const std::string& location_predicate = (*ci).second;
		
		std::cout << object_predicate << " is at " << location_predicate << std::endl;
		
		Location* object_location = new Location(location_predicate, true);
		Object* object = new Object(object_predicate, *object_location);
		
		locations.push_back(object_location);
		objects.push_back(object);
		
		predicate_to_object_mapping[object_predicate] = object;
		
		// Check if this location has any waypoints near it.
		std::map<std::string, std::vector<std::string> >::const_iterator mi = near_waypoint_mapping.find(location_predicate);
		if (mi != near_waypoint_mapping.end())
		{
			const std::vector<std::string>& near_waypoints = (*mi).second;
			for (std::vector<std::string>::const_iterator ci = near_waypoints.begin(); ci != near_waypoints.end(); ++ci)
			{
				const std::string& near_location_predicate = *ci;
				Location* near_location = new Location(near_location_predicate, false);
				locations.push_back(near_location);
				near_location->near_locations_.push_back(object_location);
				std::cout << near_location->name_ << " is near " << object_location << std::endl;
			}
		}
	}
	
	Location* robot_location = new Location(robot_location_predicate, false);
	locations.push_back(robot_location);
	
	// Make all locations fully connected.
	for (std::vector<Location*>::const_iterator ci = locations.begin(); ci != locations.end(); ++ci)
	{
		Location* location = *ci;
		for (std::vector<Location*>::const_iterator ci = locations.begin(); ci != locations.end(); ++ci)
		{
			Location* other_location = *ci;
			location->connected_locations_.push_back(other_location);
			other_location->connected_locations_.push_back(location);
			
			std::cout << location->name_ << " is connected to " << other_location->name_ << std::endl;
		}
	}
	std::stringstream ss;

	std::vector<const KnowledgeBase*> knowledge_bases;
	std::map<const Object*, unsigned int> empty_classifiable_when;
	State basic_state("basic", empty_classifiable_when);
	
	KnowledgeBase basis_kb("basis_kb");
	basis_kb.addState(basic_state);
	knowledge_bases.push_back(&basis_kb);
	
	unsigned int state_id = 0;
	
	std::cout << "Create knowledge bases for " << objects.size() << " objects." << std::endl;
	
	// Create a new knowledge base for each object.
	for (std::vector<Object*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
	{
		const Object* object = *ci;
		ss.str(std::string());
		ss << "kb_" << object->name_;
		
		KnowledgeBase* kb_location = new KnowledgeBase(ss.str());
		basis_kb.addChild(*kb_location);
		knowledge_bases.push_back(kb_location);
		
		std::map<const Object*, unsigned int> classifiable_counter;
		for (unsigned int i = 0; i <= max_classification_attemps; ++i)
		{
			classifiable_counter.clear();
			classifiable_counter[object] = i;
			
			ss.str(std::string());
			ss << "s" << state_id;
			State* state_pickupable = new State(ss.str(), classifiable_counter);
			kb_location->addState(*state_pickupable);
			++state_id;
		}
	}
	
	ss.str(std::string());
	ss << path << domain_file;
	ROS_INFO("KCL: (ContingentStrategicClassifyPDDLGenerator) Generate domain... %s", ss.str().c_str());
	generateDomainFile(ss.str(), basis_kb, knowledge_bases, *robot_location, locations, objects, max_classification_attemps);
	ss.str(std::string());
	ss << path  << problem_file;
	ROS_INFO("KCL: (ContingentStrategicClassifyPDDLGenerator) Generate problem... %s", ss.str().c_str());
	generateProblemFile(ss.str(), basis_kb, knowledge_bases, *robot_location, locations, objects, max_classification_attemps);
}

};
