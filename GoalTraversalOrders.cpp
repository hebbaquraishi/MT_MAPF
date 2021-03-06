/*
 * Author: Hebba Quraishi
 * Email: quraishi@tf.uni-freiburg.de
 * The objective of this file is to find all possible Goal Traversal Orders for an Agent
*/

#include "GoalTraversalOrders.h"
#include <iostream>

GoalTraversalOrders::GoalTraversalOrders(const std::unordered_map<std::string, int>& inverse_vertex_ids, std::vector<Agent> agents, const std::map<std::pair<int, int>,int>& h_values){
    this->inverse_vertex_ids = inverse_vertex_ids;
    std::vector<int> goal_ids;
    int start_id = -1;
    for(auto& agent: agents){
        goal_ids.clear();
        for(const auto& goal: agent.get_goals()){
            goal_ids.push_back(this->inverse_vertex_ids[goal.name]);
        }
        std::sort(goal_ids.begin(), goal_ids.end());
        start_id = brute_force_approach(agent, h_values, goal_ids, start_id);
    }
}

int GoalTraversalOrders::get_cost(const std::map<std::pair<int, int>,int>& h_values, std::vector<int> traversal_order){
    int cost = 0;
    for(int i = 0; i <= (int)traversal_order.size()-2; i++){
        if (traversal_order[i] != traversal_order[i+1]){
            cost += h_values.at(make_pair(traversal_order[i], traversal_order[i+1]));
        }
    }
    return cost;
}

int GoalTraversalOrders::brute_force_approach(Agent agent, const std::map<std::pair<int, int>,int>& h_values, std::vector<int> goal_ids, int start_id){
    do{
        start_id += 1;
        cout<<"Found GTO: "<<start_id<<endl;
        this->agent_goal_traversal_order_ids[agent.name].push_back(start_id);
        std::vector<int> temp = goal_ids;
        temp.insert(temp.begin(), this->inverse_vertex_ids[agent.get_init_loc().name]);
        this->goal_traversal_orders[start_id] = make_pair(temp, get_cost(h_values, temp));
    } while(next_permutation(goal_ids.begin(), goal_ids.end()));
    this->sort_goal_traversal_orders(agent.name);
    return start_id;
}



void GoalTraversalOrders::sort_goal_traversal_orders(const string& agent_name){
    vector<pair<int, int>> temp_goal_traversal_orders; //key:= goal_id, value:= cost
    for(auto& goal_id: this->agent_goal_traversal_order_ids[agent_name]){
        temp_goal_traversal_orders.emplace_back(make_pair(goal_id, this->goal_traversal_orders[goal_id].second));
    }
    sort(temp_goal_traversal_orders.begin(), temp_goal_traversal_orders.end(), sort_by_path_cost());
    this->agent_goal_traversal_order_ids[agent_name] = {};
    for(auto& goal: temp_goal_traversal_orders){
        this->agent_goal_traversal_order_ids[agent_name].push_back(goal.first);
    }
}


std::unordered_map<std::string, std::vector<int>> GoalTraversalOrders::get_agent_goal_traversal_order_ids(){
    return this->agent_goal_traversal_order_ids;
}

std::unordered_map<int, std::pair<std::vector<int>, int>> GoalTraversalOrders::get_goal_traversal_orders(){
    return this->goal_traversal_orders;
}