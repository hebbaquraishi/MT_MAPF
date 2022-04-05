/*
 * Author: Hebba Quraishi
 * Email: quraishi@tf.uni-freiburg.de
 * The objective of this file is to implement a Constraint Forest
*/

#include "ConstraintForest.h"

#include <utility>
#include <iostream>

ConstraintForest::ConstraintForest(const Graph& graph, std::vector<Agent> agents, const HValues& h_values) {
    this->graph = graph;
    this->h_values = h_values;
    this->agents = agents;
    auto* gto = new GoalTraversalOrders(graph, std::move(agents), this->h_values);
    this->goal_traversal_order_ids = gto->get_agent_goal_traversal_order_ids();
    this->goal_traversal_order = gto->get_goal_traversal_orders();
    delete gto;
    this->root = new Node();
    initialise_root_node();
}

void ConstraintForest::initialise_root_node(){
    this->root->set_as_root();
    first_assignment();
    this->root->compute_solution(this->graph, this->h_values);
    this->root->assign_parent(nullptr);
}

void ConstraintForest::first_assignment(){
    for(auto& agent: this->agents){
        this->root->set_agent_configuration(agent.name, 0);
        this->root->set_agent_constraints(agent.name, {});
        int goal_id = goal_traversal_order_ids[agent.name][this->root->get_agent_configuration(agent.name)];
        this->root->set_agent_goal_traversal_order(agent.name, make_pair(goal_id, goal_traversal_order[goal_id].first));
    }
}

void ConstraintForest::next_assignment(Node* node){
    int min = INT_MAX;
    int my_ptr = -1;
    int min_goal_id = -1;
    string agent_name;
    for(auto& agent: node->get_node_configuration()){
        if(agent.second < (int)goal_traversal_order_ids[agent.first].size()-1){
            int next_ptr = agent.second + 1;
            int next_goal_id = goal_traversal_order_ids[agent.first][next_ptr];
            if(goal_traversal_order[next_goal_id].second < min){
                min = goal_traversal_order[next_goal_id].second;
                min_goal_id = next_goal_id;
                agent_name = agent.first;
                my_ptr = next_ptr;
            }
        }
    }
    node->set_agent_configuration(agent_name, my_ptr);
    node->set_agent_goal_traversal_order(agent_name, make_pair(min_goal_id, goal_traversal_order[min_goal_id].first));
}


Conflict ConstraintForest::validate_paths(Node *node){
    vector<pair<string, vector<int>>> node_solution = {}; //key:= agent name, value:= path from low level search

    //padding individual agent solutions so that the solutions are all of the same length
    for(auto& agent: node->get_node_solution()){
        int padding = (int)(node->get_largest_solution_cost() - agent.second.size());
        vector<int> path = agent.second;
        for(int i = 0; i <= padding; i++){
            path.push_back(-1);
        }
        node_solution.emplace_back(make_pair(agent.first, path));
    }

    //performing validation of solutions
    for(int i = 0; i < (int)node_solution.size()-1; i++){
        for(int j = i + 1; j < (int)node_solution.size(); j++){
            for(int k = 0; k < (int)node_solution[i].second.size(); k++){
                if((node_solution[i].second)[k] == (node_solution[j].second)[k] && (node_solution[i].second)[k] !=-1){
                    Conflict c;
                    c.agent1 = node_solution[i].first;
                    c.agent2 = node_solution[j].first;
                    c.timestamp = k;
                    c.vertex = (node_solution[i].second)[k];
                    return c;
                }
            }
        }
    }
    return Conflict{};
}

Node* ConstraintForest::create_new_root_node(Node* node){
    Node* new_root = new Node();
    for(auto& agent: this->agents){
        new_root->set_agent_constraints(agent.name, {});
        new_root->set_agent_configuration(agent.name, node->get_agent_configuration(agent.name));
        new_root->set_agent_goal_traversal_order(agent.name, node->get_agent_goal_traversal_order(agent.name));
    }
    new_root->assign_parent(node);
    next_assignment(new_root);
    new_root->set_as_root();
    new_root->compute_solution(this->graph, this->h_values);
    return new_root;
}


std::vector<Node*> ConstraintForest::create_new_children_nodes(Node* node, const Conflict& conflict){
    vector<string> conflicting_agents{};
    conflicting_agents.emplace_back(conflict.agent1);
    conflicting_agents.emplace_back(conflict.agent2);
    constraint c = make_pair(conflict.vertex, conflict.timestamp);
    std::vector<Node*> children_nodes;

    for(auto& conflicting_agent: conflicting_agents){
        Node* child_node = new Node();
        child_node->set_agent_constraints(conflicting_agent, c);
        for(auto& agent: this->agents){
            child_node->set_agent_configuration(agent.name, node->get_agent_configuration(agent.name));
            child_node->set_agent_goal_traversal_order(agent.name, node->get_agent_goal_traversal_order(agent.name));
        }
        child_node->compute_solution(this->graph, this->h_values);
        children_nodes.emplace_back(child_node);
    }
    return children_nodes;
}


void ConstraintForest::run() {
    priority_queue_sorted_by_node_cost open_list;
    open_list.push(this->root);

    while(!open_list.empty()){
        Node* current_node = open_list.top();
        Conflict conflict = validate_paths(current_node);
        if(conflict.timestamp == -1){
            //we have found a solution
            for(auto& agent: this->agents){
                cout<<"\nAgent: "<<agent.name<<"\tInit: "<<agent.get_init_loc().name<<"\tGoals: ";
                for(auto& goal: agent.get_goals()){
                    cout<<goal.name<<" ";
                }
                cout<<"\nGoal Traversal Order: ";
                for(auto& vertex : current_node->get_agent_goal_traversal_order(agent.name).second){
                    cout<<this->graph.get_vertex_from_id(vertex).name<<" ";
                }
                cout<<"\nPath: ";
                for(auto& vertex : current_node->get_agent_path(agent.name)){
                    cout<<this->graph.get_vertex_from_id(vertex).name<<" ";
                }
                cout<<"\nPath Cost = "<<(int)current_node->get_agent_path(agent.name).size()-1;
                cout<<endl;
            }
            cout<<"\nTotal Solution Cost = "<<(int)current_node->get_node_cost();
            break;
        }

        if(current_node->is_root()){
            //add new root to the forest
            Node* new_root = this->create_new_root_node(current_node);
            open_list.push(new_root);
        }

        std::vector<Node*> children_nodes = create_new_children_nodes(current_node, conflict);
        open_list.push(children_nodes[0]);
        open_list.push(children_nodes[1]);
    }
}

