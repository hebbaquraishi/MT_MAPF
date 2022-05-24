/*
 * Author: Hebba Quraishi
 * Email: quraishi@tf.uni-freiburg.de
 * The objective of this file is to implement a Node of the Constraint Forest
*/

#ifndef MT_MAPF_NODE_H
#define MT_MAPF_NODE_H
#include "Definitions.h"
#include "Graph.h"
#include "LowLevelSearch.h"
#include "HValues.h"


class Node {
    bool root = false;
    int cost = 0;
    int largest_solution_cost = 0;
    [[maybe_unused]] Node* parent = nullptr;
    std::unordered_map<std::string, std::vector<constraint>> agent_constraints{}; //key:= agent name, value:= agent constraints
    std::unordered_map<std::string, int> configuration{};        //key:= agent name, value := ptr to current goal traversal order id
    std::unordered_map<std::string, std::pair<int, std::vector<int>>> assignment{}; //key:= agent name, value:= <goal traversal order id, goal traversal order>
    std::unordered_map<std::string, std::vector<int>> path{}; //key:= agent name, value:= path from low level search
public:
    Node() = default;
    void set_as_root();
    void assign_parent(Node* parent);
    void set_agent_constraints(const std::string& agent_name, const std::vector<constraint>& constraints);
    std::vector<constraint> get_agent_constraints(const std::string& agent_name);

    void set_agent_configuration(const std::string& agent_name, int ptr_index);
    int get_agent_configuration(const std::string& agent_name);

    void set_agent_goal_traversal_order(const std::string& agent_name, std::pair<int, std::vector<int>> assignment);
    std::pair<int, std::vector<int>> get_agent_goal_traversal_order(const std::string& agent_name);

    void compute_solution(const Graph& graph, const HValues& h_values);

    [[maybe_unused]] std::unordered_map<std::string, int> get_node_configuration();
    std::unordered_map<std::string, std::vector<int>> get_node_solution();
    std::vector<int> get_agent_path(const std::string& agent_name);
    [[nodiscard]] int get_node_cost() const;
    [[nodiscard]] int get_largest_solution_cost() const;

    bool is_root();
};

struct sort_node_by_cost {
    bool operator()(Node *x, Node *y){
        return x->get_node_cost() < y->get_node_cost();
    }
};



#endif //MT_MAPF_NODE_H
