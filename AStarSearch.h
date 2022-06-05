/*
 * Author: Hebba Quraishi
 * Email: quraishi@tf.uni-freiburg.de
 * The objective of this file is to implement A* Search
*/

#ifndef MT_MAPF_ASTARSEARCH_H
#define MT_MAPF_ASTARSEARCH_H
#include "Graph.h"
#include "HValues.h"


class AStarSearch {
    std::vector<int> best_path; //stores the path computed by A* search
    std::unordered_map<int, std::vector<int>> neighbours;
    std::unordered_map<int, Vertex> vertex_ids; //key:= vertex_id, value:= Vertex
public:
    AStarSearch(const std::unordered_map<int, Vertex>& vertex_ids, const std::unordered_map<int, std::vector<int>>& neighbours, const std::map<std::pair<int, int>,int>& h_values, const std::vector<constraint>& constraints, int start, int goal, int shift=0);
    map<pair<int,int>, int> initialise_map_with_infinity(int shift);
    vector<pair<int, int>> get_keys(const map<pair<int,int>, pair<int,int>>& came_from);
    void reconstruct_path(map<pair<int,int>, pair<int,int>> came_from, pair<int, int> current);
    bool in_frontier(int id, priority_queue_sorted_by_f_value frontier);
    bool in_constraints(std::vector<constraint> constraints, int vertex_id, int time_step);
    void run(const std::map<std::pair<int, int>,int>& h_values, const std::vector<constraint>& constraints, int start, int goal, int shift);
    std::vector<int> get_best_path();

};


#endif //MT_MAPF_ASTARSEARCH_H
