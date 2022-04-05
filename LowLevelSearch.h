/*
 * Author: Hebba Quraishi
 * Email: quraishi@tf.uni-freiburg.de
 * The objective of this file is to implement the low level search of the Constraint Forest
*/

#ifndef MT_MAPF_LOWLEVELSEARCH_H
#define MT_MAPF_LOWLEVELSEARCH_H
#include "Definitions.h"
#include "Graph.h"
#include "AStarSearch.h"
#include "HValues.h"



class LowLevelSearch {
    std::unordered_map<std::string, std::vector<int>> solutions; //key:= agent name, value:= solution path
    int total_solution_cost = 0;
    int largest_solution_cost = 0;
public:
    LowLevelSearch(const Graph& graph, HValues h_values, std::unordered_map<std::string, std::pair<int, std::vector<int>>> assignment, std::unordered_map<std::string, std::vector<constraint>> agent_constraints);
    std::unordered_map<std::string, std::vector<int>> get_agent_wise_solutions();
    [[nodiscard]] int get_total_solution_cost() const;
    [[nodiscard]] int get_largest_solution_cost() const;

};


#endif //MT_MAPF_LOWLEVELSEARCH_H
