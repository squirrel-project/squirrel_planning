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

#include "squirrel_planning_execution/PlanToAskPDDLGenerator.h"

namespace KCL_rosplan {

void PlanToAskPDDLGenerator::generateProblemFile(const std::string& file_name, const KnowledgeBase& current_knowledge_base, const std::vector<const KnowledgeBase*>& knowledge_bases, const Location& robot_location, const std::vector<Location*>& locations, const std::vector<Toy*>& objects, const std::vector<const Box*>& boxes, const TreeNode& root)
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
	myfile.open(file_name.c_str());
	myfile << "(define (problem squirrel)" << std::endl;
	myfile << "(:domain classify_objects)" << std::endl;
	myfile << "(:objects" << std::endl;
	
	myfile << "\tl0 - LEVEL" << std::endl;
	myfile << "\tl1 - LEVEL" << std::endl;
	myfile << "\tl2 - LEVEL" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	
	myfile << "(:init" << std::endl;
	myfile << "\t(resolve-axioms)" << std::endl;
	myfile << "\t(lev l0)" << std::endl;
	myfile << "\t(next l0 l1)" << std::endl;
	myfile << "\t(next l1 l2)" << std::endl;
	
	for (std::vector<const Box*>::const_iterator ci = boxes.begin(); ci != boxes.end(); ++ci)
	{
		const Box* box = *ci;
		myfile << "\t(box_at " << box->name_ << " " << box->location_->name_ << ")" << std::endl;
	}
	
	myfile << "\t(current_kb " << current_knowledge_base.name_ << ")" << std::endl;
	
	for (std::vector<const State*>::const_iterator ci = current_knowledge_base.states_.begin(); ci != current_knowledge_base.states_.end(); ++ci)
	{
		const State* state = *ci;
		myfile << "\t(part-of " << state->state_name_ << " " << current_knowledge_base.name_ << ")" << std::endl;
		myfile << "\t(m " << state->state_name_ << ")" << std::endl;
		myfile << "\t(robot_at robot " << robot_location.name_ << " " << state->state_name_ << ")" << std::endl;
		//myfile << "\t(child_at child child_wp " << state->state_name_ << ")" << std::endl;
		myfile << "\t(gripper_empty robot " << " " << state->state_name_ << ")" << std::endl;
		
		// All the objects are clear inially.
		for (std::vector<Toy*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
		{
			const Toy* object = *ci;
			//myfile << "\t(cleared " << object->name_ << " " << state->state_name_ << ")" << std::endl;
			myfile << "\t(object_at " << object->name_ << " " << object->location_->name_ << " " << state->state_name_ << ")" << std::endl;
			/*
			for (std::vector<const Location*>::const_iterator ci = object->observable_locations_.begin(); ci != object->observable_locations_.end() - 1; ++ci)
			{
				const Location* location = *ci;
				myfile << "\t(can_see_waypoint " << location->name_ << " " << clear_location->name_ << " " << object->name_ << " " << state->state_name_ << ")" << std::endl;
			}
			*/
		}
		
		// Set the top of the tree as 'active'.
		myfile << "\t(to_observe " << root.object_->name_ << " " << root.box_->name_ << " " << state->state_name_ << ")" << std::endl;
	}
	
	for (std::vector<Location*>::const_iterator ci = locations.begin(); ci != locations.end(); ++ci)
	{
		const Location* location = *ci;
		for (std::vector<const Location*>::const_iterator ci = location->near_locations_.begin(); ci != location->near_locations_.end(); ++ci)
		{
			const Location* location2 = *ci;
			if (location == location2) continue;
			myfile << "\t(near " << location->name_ << " " << location2->name_ << ")" << std::endl;
		}
		/*
		if (location->is_clear_)
		{
			myfile << "\t(clear_area " << location->name_ << ")" << std::endl;
		}
		*/
	}
	
	for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_bases.begin(); ci != knowledge_bases.end(); ++ci)
	{
		const KnowledgeBase* knowledge_base = *ci;
		
		for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
		{
			const State* state = *ci;
			myfile << "\t(part-of " << state->state_name_ << " " << knowledge_base->name_ << ")" << std::endl;
			
			for (unsigned int i = 0; i < state->sense_sequence_.size() - 1; ++i)
			{
				const TreeNode* node = state->sense_sequence_[i];
				const TreeNode* next_node = state->sense_sequence_[i + 1];
				myfile << "\t(order " << node->object_->name_ << " " << node->box_->name_ << " " << next_node->object_->name_ << " " << next_node->box_->name_ << " " << state->state_name_ << ")" << std::endl;
			}
			
			for (std::map<const Toy*, const Box*>::const_iterator ci = state->believe_state_.begin(); ci != state->believe_state_.end(); ++ci)
			{
				myfile << "\t(belongs_in " << ci->first->name_ << " " << ci->second->name_ << " " << state->state_name_ << ")" << std::endl;
			}
			
			const TreeNode* last_node = state->sense_sequence_[state->sense_sequence_.size() - 1];
			myfile << "\t(last_observation " << last_node->object_->name_ << " " << last_node->box_->name_ << " " << state->state_name_ << ")" << std::endl;
		}
		
		for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_base->children_.begin(); ci != knowledge_base->children_.end(); ++ci)
		{
			myfile << "\t(parent " << knowledge_base->name_ << " " << (*ci)->name_ << ")" << std::endl;
		}
	}
	myfile << ")" << std::endl;
	myfile << "(:goal (and" << std::endl;
	for (std::vector<const State*>::const_iterator ci = current_knowledge_base.states_.begin(); ci != current_knowledge_base.states_.end(); ++ci)
	{
		myfile << "\t(finished " << " " << (*ci)->state_name_ << ")" << std::endl;
	}
	myfile << ")" << std::endl;
	myfile << ")" << std::endl;
	myfile << ")" << std::endl;
	myfile.close();
}

void PlanToAskPDDLGenerator::generateDomainFile(const std::string& file_name, const KnowledgeBase& current_knowledge_base, const std::vector<const KnowledgeBase*>& knowledge_bases, const Location& robot_location, const std::vector<Location*>& locations, const std::vector<Toy*>& objects, const std::vector<const Box*>& boxes, const TreeNode& root)
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
	myfile << ")" << std::endl;
	myfile << std::endl;
	myfile << "(:predicates" << std::endl;
	myfile << "\t(robot_at ?v - robot ?wp - waypoint ?s - state)" << std::endl;
	myfile << "\t(Rrobot_at ?v - robot ?wp - waypoint ?s - state)" << std::endl;
	myfile << "\t(object_at ?o - object ?wp - waypoint ?s - state)" << std::endl;
	myfile << "\t(Robject_at ?o - object ?wp - waypoint ?s - state)" << std::endl;
	myfile << "\t(child_at ?o - object ?wp - waypoint ?s - state)" << std::endl;
	myfile << "\t(Rchild_at ?o - object ?wp - waypoint ?s - state)" << std::endl;
	myfile << "\t(finished ?s - state)" << std::endl;
	myfile << "\t(Rfinished ?s - state)" << std::endl;
	
	myfile << "\t(order ?o1 - object ?b1 - box ?o2 - object ?b2 - box ?s - state)" << std::endl;
	myfile << "\t(Rorder ?o1 - object ?b1 - box ?o2 - object ?b2 - box ?s - state)" << std::endl;
	
	myfile << "\t(to_observe ?o - object ?b - box ?s - state)" << std::endl;
	myfile << "\t(Rto_observe ?o - object ?b - box ?s - state)" << std::endl;
	
	myfile << "\t(observed ?o - object ?b - box ?s - state)" << std::endl;
	myfile << "\t(Robserved ?o - object ?b - box ?s - state)" << std::endl;
	
	myfile << "\t(belongs_in ?o - object ?b - box ?s - state)" << std::endl;
	myfile << "\t(Rbelongs_in ?o - object ?b - box ?s - state)" << std::endl;
	
	myfile << "\t(holding ?v - robot ?o - object ?s - state)" << std::endl;
	myfile << "\t(Rholding ?v - robot ?o - object ?s - state)" << std::endl;
	
	myfile << "\t(child_holding ?c - child ?o - object ?s - state)" << std::endl;
	myfile << "\t(Rchild_holding ?c - child ?o - object ?s - state)" << std::endl;
	
	myfile << "\t(gripper_empty ?v - robot ?s - state)" << std::endl;
	myfile << "\t(Rgripper_empty ?v - robot ?s - state)" << std::endl;
	
	myfile << "\t(received_feedback ?o - object ?b - box ?s - state)" << std::endl;
	myfile << "\t(Rreceived_feedback ?o - object ?b - box ?s - state)" << std::endl;
	
	myfile << "\t(can_inspect ?o - object ?s - state)" << std::endl;
	myfile << "\t(Rcan_inspect ?o - object ?s - state)" << std::endl;

	// new
	myfile << "\t(looked_at ?o - object ?s - state)" << std::endl;
	myfile << "\t(Rlooked_at ?o - object ?s - state)" << std::endl;
	
	// Encode closness of a waypoint to a box / child.
	myfile << "\t(near ?from ?to - waypoint)" << std::endl;
	myfile << "\t(box_at ?b - box ?wp - waypoint)" << std::endl;

	
	
	myfile << "\t(last_observation ?o - object ?b - box ?s - state)" << std::endl;
	myfile << "\t(Rlast_observation ?o - object ?b - box ?s - state)" << std::endl;
	myfile << "\t(part-of ?s - state ?kb - knowledgebase)" << std::endl;
	myfile << "\t(current_kb ?kb - knowledgebase)" << std::endl;
	myfile << "\t(parent ?kb ?kb2 - knowledgebase)" << std::endl;
	
	myfile << std::endl;
	myfile << "\t;; Bookkeeping predicates." << std::endl;
	myfile << "\t(next ?l ?l2 - level)" << std::endl;
	myfile << "\t(lev ?l - LEVEL)" << std::endl;
	myfile << "\t(m ?s - STATE)" << std::endl;
	myfile << "\t(stack ?s - STATE ?l - LEVEL)" << std::endl;
	
	myfile << "\t(resolve-axioms)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	/*
	myfile << "(:functions" << std::endl;
	myfile << "\t(remaining_examination_attempts ?o - object ?s - state)" << std::endl;
	myfile << ")" << std::endl;
	*/
	myfile << "(:constants" << std::endl;
	myfile << "\t; All the waypoints." << std::endl;
	
	for (std::vector<Location*>::const_iterator ci = locations.begin(); ci != locations.end(); ++ci)
	{
		myfile << "\t" << (*ci)->name_ << " - waypoint" << std::endl;
	}
	
	myfile << "\t; The objects." << std::endl;
	for (std::vector<Toy*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
	{
		myfile << "\t" << (*ci)->name_ << " - object" << std::endl;
	}
	
	myfile << "\t; The boxes." << std::endl;
	for (std::vector<const Box*>::const_iterator ci = boxes.begin(); ci != boxes.end(); ++ci)
	{
		myfile << "\t" << (*ci)->name_ << " - box" << std::endl;
	}
	myfile << "\t; The robot." << std::endl;
	myfile << "\trobot - robot" << std::endl;
	
	myfile << "\t; The child." << std::endl;
	myfile << "\tchild - child" << std::endl;
	
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
	
	/**
	 * GOTO WAYPOINT.
	 */
	myfile << "(:action goto_waypoint" << std::endl;
	myfile << "\t:parameters (?v - robot ?from ?to - waypoint)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	//myfile << "\t\t(connected ?from ?to)" << std::endl;
	myfile << "\t\t(not (resolve-axioms))" << std::endl;
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
	 * LOOK AT OBJECT
	 */
	myfile << "(:action look_at_object" << std::endl;
	myfile << "\t:parameters (?v - robot ?view ?from - waypoint ?o - object)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	myfile << "\t\t(near ?from ?view)" << std::endl;
	myfile << "\t\t(not (resolve-axioms))" << std::endl;
	for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_bases.begin(); ci != knowledge_bases.end(); ++ci)
	{
		const KnowledgeBase* knowledge_base = *ci;
		for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
		{
			myfile << "\t\t(Rrobot_at ?v ?from " << (*ci)->state_name_ << ")" << std::endl;
			myfile << "\t\t(Robject_at ?o ?view " << (*ci)->state_name_ << ")" << std::endl;
		}
	}
	
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;
	myfile << "\t\t;; For every state ?s" << std::endl;
	
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		myfile << "\t\t(when (m " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t(and" << std::endl;

		myfile << "\t\t\t\t(looked_at ?o " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t\t(Rlooked_at ?o " << (*ci)->state_name_ << ")" << std::endl;

		myfile << "\t\t\t)" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;

	/**
	 * Pickup an object.
	 */
	myfile << "(:action pickup_object" << std::endl;
	myfile << "\t:parameters (?v - robot ?robot_wp ?object_wp - waypoint ?o - object)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	myfile << "\t\t(not (resolve-axioms))" << std::endl;
	myfile << "\t\t(near ?robot_wp ?object_wp)" << std::endl;
	
	for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_bases.begin(); ci != knowledge_bases.end(); ++ci)
	{
		const KnowledgeBase* knowledge_base = *ci;
		for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
		{
			myfile << "\t\t(Rrobot_at ?v ?robot_wp " << (*ci)->state_name_ << ")" << std::endl;
			myfile << "\t\t(Rlooked_at ?o " << (*ci)->state_name_ << ")" << std::endl;
		}
		
		for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
		{
			myfile << "\t\t(Robject_at ?o ?object_wp " << (*ci)->state_name_ << ")" << std::endl;
		}
		
		for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
		{
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
		myfile << "\t\t\t\t(not (object_at ?o ?object_wp " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(not (Robject_at ?o ?object_wp " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(holding ?v ?o " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t\t(Rholding ?v ?o " << (*ci)->state_name_ << ")" << std::endl;

		myfile << "\t\t\t)" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	
	/**
	 * Drop an object.
	 */
	myfile << "(:action give_object" << std::endl;
	myfile << "\t:parameters (?v - robot ?wp - waypoint ?o - object ?c - child)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	myfile << "\t\t(not (resolve-axioms))" << std::endl;
	
	for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_bases.begin(); ci != knowledge_bases.end(); ++ci)
	{
		const KnowledgeBase* knowledge_base = *ci;
		for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
		{
			myfile << "\t\t(Rrobot_at ?v kenny_wp " << (*ci)->state_name_ << ")" << std::endl;
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
		myfile << "\t\t\t\t(child_holding ?c ?o " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t\t(Rchild_holding ?c ?o " << (*ci)->state_name_ << ")" << std::endl;

		myfile << "\t\t\t)" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	
	/**
	 * Follow child.
	 */
	myfile << "(:action follow_child" << std::endl;
	myfile << "\t:parameters (?v - robot ?c - child ?child_wp - waypoint ?o - object)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	myfile << "\t\t(not (resolve-axioms))" << std::endl;
	
	for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_bases.begin(); ci != knowledge_bases.end(); ++ci)
	{
		const KnowledgeBase* knowledge_base = *ci;
		for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
		{
			myfile << "\t\t(Rrobot_at ?v ?child_wp " << (*ci)->state_name_ << ")" << std::endl;
			//myfile << "\t\t(Rchild_at ?v ?child_wp " << (*ci)->state_name_ << ")" << std::endl;
			myfile << "\t\t(Rchild_holding ?c ?o " << (*ci)->state_name_ << ")" << std::endl;
		}
	}
	
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;
	myfile << "\t\t;; For every state ?s" << std::endl;
	
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		myfile << "\t\t(when (m " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t(and" << std::endl;

		myfile << "\t\t\t\t(not (child_holding ?c ?o " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(not (Rchild_holding ?c ?o " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(not (robot_at ?v ?child_wp " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(not (Rrobot_at ?v ?child_wp " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(robot_at ?v end_follow_wp " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t\t(Rrobot_at ?v end_follow_wp " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t\t(can_inspect ?o " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t\t(Rcan_inspect ?o " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t)" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	
	/**
	 * Finish.
	 */
	myfile << "(:action finish" << std::endl;
	myfile << "\t:parameters (?o - object ?b - box)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	myfile << "\t\t(not (resolve-axioms))" << std::endl;
	for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_bases.begin(); ci != knowledge_bases.end(); ++ci)
	{
		const KnowledgeBase* knowledge_base = *ci;
		for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
		{
			const State* state = *ci;
			myfile << "\t\t\t(Robserved ?o ?b " << state->state_name_ << ")" << std::endl;
			myfile << "\t\t\t(Rlast_observation ?o ?b " << state->state_name_ << ")" << std::endl;
		}
	}
	
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;
	myfile << "\t\t;; For every state ?s" << std::endl;
	
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		const State* state = *ci;
		myfile << "\t\t(when (m " << state->state_name_ << ")" << std::endl;
		myfile << "\t\t\t(and" << std::endl;
		myfile << "\t\t\t\t(finished " << state->state_name_ << ")" << std::endl;
		myfile << "\t\t\t)" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	
	/**
	 * Check if the object is at the right place.
	 */
	myfile << "(:action inspect_object" << std::endl;
	myfile << "\t:parameters (?v - robot ?o - object)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	myfile << "\t\t(not (resolve-axioms))" << std::endl;
	for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_bases.begin(); ci != knowledge_bases.end(); ++ci)
	{
		const KnowledgeBase* knowledge_base = *ci;
		for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
		{
			myfile << "\t\t(Rcan_inspect ?o " << (*ci)->state_name_ << ")" << std::endl;
			myfile << "\t\t(Rrobot_at ?v end_follow_wp " << (*ci)->state_name_ << ")" << std::endl;
		}
	}
	
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;
	myfile << "\t\t;; For every state ?s" << std::endl;
	
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		const State* state = *ci;
		myfile << "\t\t(when (m " << state->state_name_ << ")" << std::endl;
		myfile << "\t\t\t(and" << std::endl;
		myfile << "\t\t\t\t(not (Rcan_inspect ?o " << state->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(can_inspect ?o " << state->state_name_ << ")" << std::endl;
		for (std::vector<const Box*>::const_iterator ci = boxes.begin(); ci != boxes.end(); ++ci)
		{
			myfile << "\t\t\t\t(received_feedback ?o " << (*ci)->name_ << " " << state->state_name_ << ")" << std::endl;
			myfile << "\t\t\t\t(Rreceived_feedback ?o " << (*ci)->name_ << " " << state->state_name_ << ")" << std::endl;
		}
		myfile << "\t\t\t)" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	
	/**
	 * Check which observation to make next.
	 */
	myfile << "(:action next_observation" << std::endl;
	myfile << "\t:parameters (?o ?o2 - object ?b ?b2 - box)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	//myfile << "\t\t(connected ?from ?to)" << std::endl;
	myfile << "\t\t(not (resolve-axioms))" << std::endl;
	for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_bases.begin(); ci != knowledge_bases.end(); ++ci)
	{
		const KnowledgeBase* knowledge_base = *ci;
		for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
		{
			myfile << "\t\t(Robserved ?o ?b " << (*ci)->state_name_ << ")" << std::endl;
			myfile << "\t\t(Rto_observe ?o ?b " << (*ci)->state_name_ << ")" << std::endl;
			myfile << "\t\t(Rorder ?o ?b ?o2 ?b2 " << (*ci)->state_name_ << ")" << std::endl;
		}
	}
	
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;
	myfile << "\t\t;; For every state ?s" << std::endl;
	
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		myfile << "\t\t(when (m " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t(and" << std::endl;

		myfile << "\t\t\t\t(not (to_observe ?o ?b " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(not (Rto_observe ?o ?b " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t\t\t(to_observe ?o2 ?b2 " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t\t(Rto_observe ?o2 ?b2 " << (*ci)->state_name_ << ")" << std::endl;

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
	 * Sensing action that applies to all states.
	 */
	myfile << ";; Attempt to classify the object." << std::endl;
	myfile << "(:action check_belongs_in" << std::endl;
	
	myfile << "\t:parameters (?b - box ?o - object ?v - robot ?kb - knowledgebase)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	myfile << "\t\t(not (resolve-axioms))" << std::endl;
	
	myfile << "\t\t(current_kb ?kb)" << std::endl;
	myfile << std::endl;
	
	for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_bases.begin(); ci != knowledge_bases.end(); ++ci)
	{
		const KnowledgeBase* knowledge_base = *ci;
		for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
		{
			myfile << "\t\t(Rbelongs_in ?o ?b " << (*ci)->state_name_ << ")" << std::endl;
		}
	}
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;
	myfile << std::endl;
	myfile << "\t\t;; For every state ?s" << std::endl;
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		myfile << "\t\t(when (m " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t(and" << std::endl;
		myfile << "\t\t\t\t(observed ?o ?b " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t\t(Robserved ?o ?b " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t)" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	
	/**
	 * Classify sension action.
	 */
	myfile << ";; Attempt to classify the object." << std::endl;
	myfile << "(:action observe-belongs_in" << std::endl;
	
	myfile << "\t:parameters (?o - object ?b - box ?v - robot ?l ?l2 - level ?kb - knowledgebase)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	myfile << "\t\t(not (resolve-axioms))" << std::endl;
	myfile << "\t\t(next ?l ?l2)" << std::endl;
	myfile << "\t\t(lev ?l)" << std::endl;
	myfile << "\t\t(current_kb ?kb)" << std::endl;
	myfile << std::endl;
	
	for (std::vector<const KnowledgeBase*>::const_iterator ci = knowledge_bases.begin(); ci != knowledge_bases.end(); ++ci)
	{
		const KnowledgeBase* knowledge_base = *ci;
		for (std::vector<const State*>::const_iterator ci = knowledge_base->states_.begin(); ci != knowledge_base->states_.end(); ++ci)
		{
			myfile << "\t\t(Rrobot_at robot end_follow_wp " << (*ci)->state_name_ << ")" << std::endl;
			myfile << "\t\t(Rto_observe ?o ?b " << (*ci)->state_name_ << ")" << std::endl;
			myfile << "\t\t(Rreceived_feedback ?o ?b " << (*ci)->state_name_ << ")" << std::endl;
		}
	}
	
	myfile << "\t\t;; This action is only applicable if there are world states where the outcome can be different." << std::endl;
	
	myfile << "\t\t(exists (?s - state) (and (m ?s) (belongs_in ?o ?b ?s) (part-of ?s ?kb)))" << std::endl;
	myfile << "\t\t(exists (?s - state) (and (m ?s) (not (belongs_in ?o ?b ?s)) (part-of ?s ?kb)))" << std::endl;;
	myfile << "\t)" << std::endl;
	myfile << "\t:effect (and" << std::endl;
	myfile << "\t\t(not (lev ?l))" << std::endl;
	myfile << "\t\t(lev ?l2)" << std::endl;
	myfile << std::endl;
	myfile << "\t\t;; For every state ?s" << std::endl;
	for (std::vector<const State*>::const_iterator ci = states.begin(); ci != states.end(); ++ci)
	{
		myfile << "\t\t(when (and (m " << (*ci)->state_name_ << ") (not (belongs_in ?o ?b " << (*ci)->state_name_ << ")))" << std::endl;
		myfile << "\t\t\t(and (stack " << (*ci)->state_name_ << " ?l) (not (m " << (*ci)->state_name_ << ")))" << std::endl;
		myfile << "\t\t)" << std::endl;
		
		myfile << "\t\t(when (m " << (*ci)->state_name_ << ")" << std::endl;
		myfile << "\t\t\t(and (observed ?o ?b " << (*ci)->state_name_ << "))" << std::endl;
		myfile << "\t\t)" << std::endl;
	}
	myfile << "\t\t(resolve-axioms)" << std::endl;
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	myfile << std::endl;
	
	/**
	 * POP action.
	 */
	myfile << ";; Exit the current branch." << std::endl;
	myfile << "(:action pop" << std::endl;
	myfile << "\t:parameters (?l ?l2 - level)" << std::endl;
	myfile << "\t:precondition (and" << std::endl;
	myfile << "\t\t(lev ?l)" << std::endl;
	myfile << "\t\t(next ?l2 ?l)" << std::endl;
	myfile << "\t\t(not (resolve-axioms))" << std::endl;
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

	/**
	 * Ramificate.
	 */
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
		
		// finished.
		myfile << "\t\t(when (or (finished " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
		myfile << "\t\t\t(Rfinished " << state->state_name_ << ")" << std::endl;
		myfile << "\t\t)" << std::endl;
		
		myfile << "\t\t(when (and (not (finished " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
		myfile << "\t\t\t(not (Rfinished " << state->state_name_ << "))" << std::endl;
		myfile << "\t\t)" << std::endl;
		
		// gripper_empty.
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
			
			myfile << "\t\t(when (or (child_at child " << location->name_ << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
			myfile << "\t\t\t(Rchild_at child " << location->name_ << " " << state->state_name_ << ")" << std::endl;
			myfile << "\t\t)" << std::endl;
			
			myfile << "\t\t(when (and (not (child_at child " << location->name_ << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t\t(not (Rchild_at child " << location->name_ << " " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t)" << std::endl;
		}
		
		for (std::vector<Toy*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
		{
			const Toy* object = *ci;
			
			// holding.
			myfile << "\t\t(when (or (holding robot " << object->name_ << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
			myfile << "\t\t\t(Rholding robot " << object->name_ << " " << state->state_name_ << ")" << std::endl;
			myfile << "\t\t)" << std::endl;
			
			myfile << "\t\t(when (and (not (holding robot " << object->name_ << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t\t(not (Rholding robot " << object->name_ << " " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t)" << std::endl;
			
			myfile << "\t\t(when (or (child_holding child " << object->name_ << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
			myfile << "\t\t\t(Rchild_holding child " << object->name_ << " " << state->state_name_ << ")" << std::endl;
			myfile << "\t\t)" << std::endl;
			
			myfile << "\t\t(when (and (not (child_holding child " << object->name_ << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t\t(not (Rchild_holding child " << object->name_ << " " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t)" << std::endl;
			
			myfile << "\t\t(when (or (can_inspect " << object->name_ << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
			myfile << "\t\t\t(Rcan_inspect " << object->name_ << " " << state->state_name_ << ")" << std::endl;
			myfile << "\t\t)" << std::endl;
			
			myfile << "\t\t(when (and (not (can_inspect " << object->name_ << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t\t(not (Rcan_inspect " << object->name_ << " " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t)" << std::endl;
			
			myfile << "\t\t(when (or (looked_at " << object->name_ << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
			myfile << "\t\t\t(Rlooked_at " << object->name_ << " " << state->state_name_ << ")" << std::endl;
			myfile << "\t\t)" << std::endl;
			
			myfile << "\t\t(when (and (not (looked_at " << object->name_ << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t\t(not (Rlooked_at " << object->name_ << " " << state->state_name_ << "))" << std::endl;
			myfile << "\t\t)" << std::endl;

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
			
			for (std::vector<const Box*>::const_iterator ci = boxes.begin(); ci != boxes.end(); ++ci)
			{
				const Box* box = *ci;
				
				myfile << "\t\t(when (or (belongs_in " << object->name_ << " " << box->name_ << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
				myfile << "\t\t\t(Rbelongs_in " << object->name_ << " " << box->name_ << " " << state->state_name_ << ")" << std::endl;
				myfile << "\t\t)" << std::endl;
				
				myfile << "\t\t(when (and (not (belongs_in " << object->name_ << " " << box->name_ << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t(not (Rbelongs_in " << object->name_ << " " << box->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t)" << std::endl;
				
				myfile << "\t\t(when (or (to_observe " << object->name_ << " " << box->name_ << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
				myfile << "\t\t\t(Rto_observe " << object->name_ << " " << box->name_ << " " << state->state_name_ << ")" << std::endl;
				myfile << "\t\t)" << std::endl;
				
				myfile << "\t\t(when (and (not (to_observe " << object->name_ << " " << box->name_ << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t(not (Rto_observe " << object->name_ << " " << box->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t)" << std::endl;
				
				myfile << "\t\t(when (or (observed " << object->name_ << " " << box->name_ << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
				myfile << "\t\t\t(Robserved " << object->name_ << " " << box->name_ << " " << state->state_name_ << ")" << std::endl;
				myfile << "\t\t)" << std::endl;
				
				myfile << "\t\t(when (and (not (observed " << object->name_ << " " << box->name_ << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t(not (Robserved " << object->name_ << " " << box->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t)" << std::endl;
				
				myfile << "\t\t(when (or (last_observation " << object->name_ << " " << box->name_ << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
				myfile << "\t\t\t(Rlast_observation " << object->name_ << " " << box->name_ << " " << state->state_name_ << ")" << std::endl;
				myfile << "\t\t)" << std::endl;
				
				myfile << "\t\t(when (and (not (last_observation " << object->name_ << " " << box->name_ << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t(not (Rlast_observation " << object->name_ << " " << box->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t)" << std::endl;
				
				myfile << "\t\t(when (or (received_feedback " << object->name_ << " " << box->name_ << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
				myfile << "\t\t\t(Rreceived_feedback " << object->name_ << " " << box->name_ << " " << state->state_name_ << ")" << std::endl;
				myfile << "\t\t)" << std::endl;
				
				myfile << "\t\t(when (and (not (received_feedback " << object->name_ << " " << box->name_ << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t\t(not (Rreceived_feedback " << object->name_ << " " << box->name_ << " " << state->state_name_ << "))" << std::endl;
				myfile << "\t\t)" << std::endl;
				
				for (std::vector<Toy*>::const_iterator ci = objects.begin(); ci != objects.end(); ++ci)
				{
					const Toy* object2 = *ci;
					for (std::vector<const Box*>::const_iterator ci = boxes.begin(); ci != boxes.end(); ++ci)
					{
						const Box* box2 = *ci;
						myfile << "\t\t(when (or (order " << object->name_ << " " << box->name_ << " " << object2->name_ << " " << box2->name_ << " " << state->state_name_ << ") (not (m " << state->state_name_ << ")))" << std::endl;
						myfile << "\t\t\t(Rorder " << object->name_ << " " << box->name_ << " " << object2->name_ << " " << box2->name_ << " " << state->state_name_ << ")" << std::endl;
						myfile << "\t\t)" << std::endl;
						
						myfile << "\t\t(when (and (not (order " << object->name_ << " " << box->name_ << " " << object2->name_ << " " << box2->name_ << " " << state->state_name_ << ")) (m " << state->state_name_ << "))" << std::endl;
						myfile << "\t\t\t(not (Rorder " << object->name_ << " " << box->name_ << " " << object2->name_ << " " << box2->name_ << " " << state->state_name_ << "))" << std::endl;
						myfile << "\t\t)" << std::endl;
					}
				}
			}
		}
	}
	myfile << "\t)" << std::endl;
	myfile << ")" << std::endl;
	myfile << ")" << std::endl;
	myfile.close();
}

void PlanToAskPDDLGenerator::createPDDL(FactObserveTree& root, const std::string& path, const std::string& domain_file, const std::string& problem_file, const std::string& robot_location_predicate, const std::map<std::string, std::string>& object_to_location_mapping, const std::map<std::string, std::string>& box_to_location_mapping)
{
	ROS_INFO("KCL: (PlanToAskPDDLGenerator) Create the PDDL domain.");
	// Now generate the waypoints / boxes / toys / etc.
	std::vector<Location*> locations;
	std::vector<Toy*> objects;
	std::vector<const Box*> boxes;
	
	// Predicate to object mapping for ease of use :).
	std::map<std::string, const Toy*> toy_mapping;
	std::map<std::string, const Box*> box_mapping;
	
	// Initialise the robot's location.
	Location* robot_location = new Location(robot_location_predicate, false);
	locations.push_back(robot_location);
	
	// Initialise the object's locations.
	for (std::map<std::string, std::string>::const_iterator ci = object_to_location_mapping.begin(); ci != object_to_location_mapping.end(); ++ci)
	{
		Location* location = new Location(ci->second, false);
		locations.push_back(location);
		Toy* object = new Toy(ci->first, *location);
		objects.push_back(object);
		
		toy_mapping[object->name_] = object;
		
		// Create a near location for every toy.
		std::stringstream ss;
		ss << "near_" << ci->second;
		Location* near_location = new Location(ss.str().c_str(), false);
		locations.push_back(near_location);
		location->near_locations_.push_back(near_location);
		near_location->near_locations_.push_back(location);
	}
	
	// Initialise the box's locations.
	for (std::map<std::string, std::string>::const_iterator ci = box_to_location_mapping.begin(); ci != box_to_location_mapping.end(); ++ci)
	{
		Location* location = new Location(ci->second, false);
		locations.push_back(location);
		std::vector<const Toy*> objects_inside;
		Box* box = new Box(ci->first, *location, objects_inside);
		boxes.push_back(box);
		
		box_mapping[box->name_] = box;
		
		// Create a near location for every box.
		std::stringstream ss;
		ss << "near_" << ci->second;
		Location* near_location = new Location(ss.str().c_str(), false);
		locations.push_back(near_location);
		location->near_locations_.push_back(near_location);
		near_location->near_locations_.push_back(location);
	}
	
	// Special waypoint.
	Location* end_follow_wp = new Location("end_follow_wp", false);
	locations.push_back(end_follow_wp);
	
	// Make all locations fully connected.
	for (std::vector<Location*>::const_iterator ci = locations.begin(); ci != locations.end(); ++ci)
	{
		Location* location = *ci;
		for (std::vector<Location*>::const_iterator ci = locations.begin(); ci != locations.end(); ++ci)
		{
			Location* other_location = *ci;
			location->connected_locations_.push_back(other_location);
			other_location->connected_locations_.push_back(location);
		}
	}
	
	ROS_INFO("KCL: (PlanToAskPDDLGenerator) Create the believe states.");
	
	std::vector<const KnowledgeBase*> knowledge_bases;
	//std::vector<const TreeNode*> empty_stacked_objects_mapping;
	//State basic_state("basic", empty_stacked_objects_mapping);
	
	KnowledgeBase basis_kb("basis_kb");
	knowledge_bases.push_back(&basis_kb);
	
	unsigned int state_id = 0;
	
	// Create the tree from the data stored in the knowledge base.
	std::vector<State*> generated_states;
	TreeNode tree_root(NULL, true, root, toy_mapping, box_mapping, generated_states);
	
	// Generate all sequences through that tree that form the states.
	for (std::vector<State*>::const_iterator ci = generated_states.begin(); ci != generated_states.end(); ++ci)
	{
		basis_kb.addState(**ci);
	}
	
	/*
	// Test case to test the model and planner.
	// Root of the tree.
	TreeNode root(*boxes[0], *objects[0], NULL);
	TreeNode tn11(*boxes[0], *objects[1], &root);
	TreeNode tn12(*boxes[1], *objects[1], &root);
	
	root.true_branch_ = &tn11;
	root.false_branch_ = &tn12;
	
	// Create 2 dummy states.
	std::vector<const TreeNode*> sense_sequence1;
	sense_sequence1.push_back(&root);
	sense_sequence1.push_back(&tn11);
	std::vector<const TreeNode*> sense_sequence2;
	sense_sequence2.push_back(&root);
	sense_sequence2.push_back(&tn12);
	
	std::map<const Toy*, const Box*> believe_state1;
	believe_state1[objects[0]] = boxes[0];
	believe_state1[objects[1]] = boxes[0];
	std::map<const Toy*, const Box*> believe_state2;
	believe_state2[objects[0]] = boxes[0];
	believe_state2[objects[1]] = boxes[1];
	std::map<const Toy*, const Box*> believe_state3;
	believe_state3[objects[0]] = boxes[1];
	believe_state3[objects[1]] = boxes[0];
	std::map<const Toy*, const Box*> believe_state4;
	believe_state4[objects[0]] = boxes[1];
	believe_state4[objects[1]] = boxes[1];
	
	State* state1 = new State("state1", sense_sequence1, believe_state1);
	State* state2 = new State("state2", sense_sequence1, believe_state2);
	State* state3 = new State("state3", sense_sequence2, believe_state3);
	State* state4 = new State("state4", sense_sequence2, believe_state4);
	basis_kb.addState(*state1);
	basis_kb.addState(*state2);
	basis_kb.addState(*state3);
	basis_kb.addState(*state4);
	*/
	
	std::stringstream ss;
	ss << path << domain_file;
	ROS_INFO("KCL: (PlanToSensePDDLGenerator) Generate domain... %s", ss.str().c_str());
	generateDomainFile(ss.str(), basis_kb, knowledge_bases, *robot_location, locations, objects, boxes, tree_root);
	ss.str(std::string());
	ss << path  << problem_file;
	ROS_INFO("KCL: (PlanToSensePDDLGenerator) Generate problem... %s", ss.str().c_str());
	generateProblemFile(ss.str(), basis_kb, knowledge_bases, *robot_location, locations, objects, boxes, tree_root);
}

};
