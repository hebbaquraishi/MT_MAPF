/*
 * Author: Hebba Quraishi
 * Email: quraishi@tf.uni-freiburg.de
 * The objective of this file is to find all possible Goal Traversal Orders for an Agent
*/

#ifndef MT_MAPF_GOALTRAVERSALORDERS_H
#define MT_MAPF_GOALTRAVERSALORDERS_H
#include "Definitions.h"
#include "Graph.h"
#include "Agent.h"
#include "HValues.h"

class GoalTraversalOrders {
    std::unordered_map<std::string, std::vector<int>> agent_goal_traversal_order_ids; //key:= agent name, value:= all goal traversal order ids
    std::unordered_map<int, std::pair<std::vector<int>, int>> goal_traversal_orders; //key:= goal traversal order id, value:= <goal traversal order, cost>
    void sort_goal_traversal_orders(const std::string&);
public:
    GoalTraversalOrders(Graph graph, std::vector<Agent> agents, const HValues& h_values);
    ~GoalTraversalOrders() = default;
    int get_cost(HValues h_values, std::vector<int> goal_traversal_order);
    int brute_force_approach(Graph graph, Agent agent, const HValues& h_values, std::vector<int> goal_ids, int start_id);
    std::unordered_map<std::string, std::vector<int>> get_agent_goal_traversal_order_ids();
    std::unordered_map<int, std::pair<std::vector<int>, int>> get_goal_traversal_orders();
};


#endif //MT_MAPF_GOALTRAVERSALORDERS_H
