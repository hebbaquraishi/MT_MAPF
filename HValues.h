/*
 * Author: Hebba Quraishi
 * Email: quraishi@tf.uni-freiburg.de
 * The objective of this file is to find the h_values using Breadth First Search
*/

#ifndef MT_MAPF_HVALUES_H
#define MT_MAPF_HVALUES_H
#include "Graph.h"
#include "Agent.h"
#include <map>
#include <queue>
#include <utility>


class HValues {
    std::map<std::pair<int, int>,int> h_values; //key:= <source_vertex_id, destination_vertex_id>, value:= distance
    std::unordered_map<int, std::vector<int>> neighbours; //key:= vertex_id, value:= ids of all neighbours
public:
    HValues() = default;
    HValues(const std::unordered_map<std::string, int>& inverse_vertex_ids, const std::unordered_map<int, std::vector<int>>& neighbours, std::vector<Agent> agents);
    ~HValues();
    void run_bfs(const std::pair<int, int>& root, std::queue<std::pair<int, int>> q, std::map<int, bool> discovered);
    int get_h_values(std::pair<int, int> h);
    std::map<std::pair<int, int>,int> get_all_h_values();
};


#endif //MT_MAPF_HVALUES_H
