/*
 * Author: Hebba Quraishi
 * Email: quraishi@tf.uni-freiburg.de
 * The objective of this file is to implement a Constraint Forest
*/

#ifndef MT_MAPF_CONSTRAINTFOREST_H
#define MT_MAPF_CONSTRAINTFOREST_H
#include "Definitions.h"
#include "Graph.h"
#include "Node.h"
#include "GoalTraversalOrders.h"
#include "HValues.h"
#include <chrono>



class ConstraintForest {
    std::chrono::steady_clock::time_point start{};
    long preprocessing;
    Node *root;
    Graph graph;
    std::vector<Agent> agents;
    HValues h_values;
    std::unordered_map<std::string, std::vector<int>> goal_traversal_order_ids = {}; //key:= agent name, value:= all goal traversal order ids
    std::unordered_map<int, std::pair<std::vector<int>, int>> goal_traversal_order = {}; //key:= goal traversal order id, value:= <goal traversal order, cost>
public:
    ConstraintForest(const Graph& graph, std::vector<Agent> agents, const HValues& h_values);
    void initialise_root_node();
    void first_assignment();
    void next_assignment(Node* parent, const string& agent_name);
    Conflict validate_paths(Node *node);
    vector<Node*> create_new_root_node(Node* node);
    std::vector<Node*> create_new_children_nodes(Node* node, const Conflict& conflict);
    void run(string results_location);


};


#endif //MT_MAPF_CONSTRAINTFOREST_H
