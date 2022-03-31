/*
 * Author: Hebba Quraishi
 * Email: quraishi@tf.uni-freiburg.de
 * The objective of this file is to implement a Node of the Constraint Forest
*/

#include "Node.h"

#include <utility>

void Node::set_as_root() {
    this->root = true;
}


void Node::assign_parent(Node *parent) {
    this->parent = parent;
}


void Node::set_agent_constraints(const std::string& agent_name, constraint c) {
    this->agent_constraints[agent_name].emplace_back(c);
}

void Node::set_agent_configuration(const std::string& agent_name, int ptr_index){
    this->configuration[agent_name] = ptr_index;
}


void Node::set_agent_goal_traversal_order(const std::string &agent_name, std::pair<int, std::vector<int>> assignment) {
    this->assignment[agent_name] = std::move(assignment);
}

void Node::compute_solution(const Graph& graph, const std::map<std::pair<int, int>,int>& h_values) {
    LowLevelSearch lowLevelSearch = LowLevelSearch(graph, h_values, assignment, agent_constraints);
    this->path = lowLevelSearch.get_agent_wise_solutions();
    this->cost = lowLevelSearch.get_total_solution_cost();
    this->largest_solution_cost = lowLevelSearch.get_largest_solution_cost();
}

int Node::get_agent_configuration(const std::string& agent_name){
    return this->configuration[agent_name];
}

std::unordered_map<std::string, int> Node::get_node_configuration(){
    return this->configuration;
}

std::unordered_map<std::string, std::vector<int>> Node::get_node_solution(){
    return this->path;
}

int Node::get_node_cost(){
    return this->cost;
}

int Node::get_largest_solution_cost(){
    return this->largest_solution_cost;
}

std::pair<int, std::vector<int>> Node::get_agent_goal_traversal_order(std::string agent_name){
    return this->assignment[agent_name];
}

bool Node::is_root(){
    return this->root;
}

std::vector<int> Node::get_agent_path(std::string agent_name){
    return this->path[agent_name];
}