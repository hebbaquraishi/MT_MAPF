/*
 * Author: Hebba Quraishi
 * Email: quraishi@tf.uni-freiburg.de
 * The objective of this file is to implement a Constraint Forest
*/

#include "ConstraintForest.h"

#include <utility>
#include <iostream>
#include <fstream>

ConstraintForest::ConstraintForest(const Graph& graph, std::vector<Agent> agents, const HValues& h_values) {
    this->start = std::chrono::high_resolution_clock::now();
    this->graph = graph;
    this->h_values = h_values;
    this->agents = agents;
    auto* gto = new GoalTraversalOrders(graph, std::move(agents), this->h_values);
    this->goal_traversal_order_ids = gto->get_agent_goal_traversal_order_ids();
    this->goal_traversal_order = gto->get_goal_traversal_orders();
    this->preprocessing = (duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start)).count();
    delete gto;
    this->start = std::chrono::high_resolution_clock::now();
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

void ConstraintForest::next_assignment(Node* node, const string& agent_name){
    int config = node->get_agent_configuration(agent_name);
    if(config < (int)goal_traversal_order_ids[agent_name].size()-1){
        int next_ptr = config + 1;
        int next_goal_id = goal_traversal_order_ids[agent_name][next_ptr];
        node->set_agent_configuration(agent_name, next_ptr);
        node->set_agent_goal_traversal_order(agent_name, make_pair(next_goal_id, goal_traversal_order[next_goal_id].first));
    }
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

vector<Node*> ConstraintForest::create_new_root_node(Node* node){
    vector<Node*> new_roots;
    int num_of_agents = (int)this->agents.size();
    for(int i = 0; i < num_of_agents; i++){
        Node* new_root = new Node();
        for(auto& agent: this->agents){
            new_root->set_agent_constraints(agent.name, {});    //root has empty set of constraints
            new_root->set_agent_configuration(agent.name, node->get_agent_configuration(agent.name));
            new_root->set_agent_goal_traversal_order(agent.name, node->get_agent_goal_traversal_order(agent.name));
            new_root->set_as_root();
        }
        new_roots.push_back(new_root);
    }
    int i = 0;
    for(auto& agent: this->agents){
        next_assignment(new_roots[i], agent.name);
        new_roots[i]->compute_solution(this->graph, this->h_values);
        i += 1;
    }
    return new_roots;
}


std::vector<Node*> ConstraintForest::create_new_children_nodes(Node* node, const Conflict& conflict){
    vector<string> conflicting_agents{};
    conflicting_agents.emplace_back(conflict.agent1);
    conflicting_agents.emplace_back(conflict.agent2);
    constraint c0 = make_pair(conflict.vertex, conflict.timestamp);
    std::vector<Node*> children_nodes;

    for(auto& conflicting_agent: conflicting_agents){
        Node* child_node = new Node();
        std::vector<constraint> c{};
        c.emplace_back(c0);
        child_node->set_agent_constraints(conflicting_agent, c);
        for(auto& agent: this->agents){
            child_node->set_agent_configuration(agent.name, node->get_agent_configuration(agent.name));
            child_node->set_agent_goal_traversal_order(agent.name, node->get_agent_goal_traversal_order(agent.name));
            child_node->set_agent_constraints(agent.name, node->get_agent_constraints(agent.name));
        }
        child_node->assign_parent(node);
        child_node->compute_solution(this->graph, this->h_values);
        children_nodes.emplace_back(child_node);
    }
    return children_nodes;
}


void ConstraintForest::run(string results_location) {
    vector<Node*> open_list;
    open_list.emplace_back(this->root);
    int num_nodes_expanded = 0, num_conflicts = 0;


    while(!open_list.empty()){
        auto time_check = duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
        if(time_check.count() > 300000){
            json result_file;
            result_file["preprocessing_time"] = (int)preprocessing;
            result_file["agent_count"] = (int)agents.size();
            result_file["goals_per_agent"] = (int)agents[0].get_goals().size();
            result_file["execution_time"] = std::to_string(time_check.count());
            result_file["solution_cost"] = 0;
            result_file["nodes_expanded"] = num_nodes_expanded;
            result_file["num_conflicts"] = num_conflicts;
            result_file["timeout"] = "true";
            ofstream ostream(results_location, std::ios_base::app);
            ostream<<result_file<<endl;
            cout<<"Timeout!!"<<endl;
            break;
        }
        Node* current_node = open_list.front();
        open_list.erase(open_list.begin());
        Conflict conflict = validate_paths(current_node);
        if(conflict.timestamp == -1){
            //we have found a solution
            json result_file;
            result_file["agent_count"] = (int)agents.size();
            result_file["goals_per_agent"] = (int)agents[0].get_goals().size();
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = duration_cast<std::chrono::milliseconds>(stop - start);
            cout<<"\nTime taken = " << duration.count() <<"ms" <<endl;
            result_file["execution_time"] = std::to_string(duration.count());
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
            result_file["solution_cost"] = (int)current_node->get_node_cost();
            cout<<"\nTotal Solution Cost = "<<(int)current_node->get_node_cost();

            result_file["nodes_expanded"] = num_nodes_expanded;
            cout<<"\nNumber of nodes expanded = "<<num_nodes_expanded;
            result_file["timeout"] = "false";

            result_file["num_conflicts"] = num_conflicts;
            cout<<"\nNumber of conflicts = "<<num_conflicts;
            ofstream ostream(results_location, std::ios_base::app);
            ostream<<result_file<<endl;
            break;
        }
        num_conflicts++;

        if(current_node->is_root()){
            //add new root to the forest
            cout<<"Adding new root"<<endl;
            vector<Node*> new_roots = this->create_new_root_node(current_node);
            for(auto& new_root : new_roots){
                open_list.emplace_back(new_root);
            }
            cout<<"New root added"<<endl;
        }

        cout<<"Creating new children"<<endl;
        std::vector<Node*> children_nodes = create_new_children_nodes(current_node, conflict);
        open_list.emplace_back(children_nodes[0]);
        open_list.emplace_back(children_nodes[1]);
        num_nodes_expanded++;
        std::sort(open_list.begin(), open_list.end(), sort_node_by_cost());
        cout<<"Children created"<<endl;
        cout<<"Loop value: "<<num_nodes_expanded<<endl;
    }
}

