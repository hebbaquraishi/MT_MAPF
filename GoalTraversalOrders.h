//
// Created by Hebba Quraishi on 31.03.22.
//

#ifndef MT_MAPF_GOALTRAVERSALORDERS_H
#define MT_MAPF_GOALTRAVERSALORDERS_H
#include "Definitions.h"
#include "Graph.h"
#include "Agent.h"

class GoalTraversalOrders {
    void sort_goal_traversal_orders(const std::string&);
    std::unordered_map<std::string, std::vector<int>> agent_goal_traversal_order_ids; //key:= agent name, value:= all goal traversal order ids
    std::unordered_map<int, std::pair<std::vector<int>, int>> goal_traversal_orders; //key:= goal traversal order id, value:= <goal traversal order, cost>

public:
     GoalTraversalOrders(Graph graph, std::vector<Agent> agents, const std::map<std::pair<int, int>,int>& h_values);
    ~GoalTraversalOrders() = default;
    int get_cost(std::map<std::pair<int, int>,int> h_values, std::vector<int> goal_traversal_order);
    int brute_force_approach(Graph graph, Agent agent, const std::map<std::pair<int, int>,int>& h_values, std::vector<int> goal_ids, int start_id);
    std::unordered_map<std::string, std::vector<int>> get_agent_goal_traversal_order_ids();
    std::unordered_map<int, std::pair<std::vector<int>, int>> get_goal_traversal_orders();
};


#endif //MT_MAPF_GOALTRAVERSALORDERS_H
