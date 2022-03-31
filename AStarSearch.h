/*
 * Author: Hebba Quraishi
 * Email: quraishi@tf.uni-freiburg.de
 * The objective of this file is to implement A* Search
*/

#ifndef MT_MAPF_ASTARSEARCH_H
#define MT_MAPF_ASTARSEARCH_H
#include "Graph.h"


class AStarSearch {
    std::vector<int> best_path; //stores the path computed by A* search
public:
    AStarSearch(Graph graph, const std::map<std::pair<int, int>,int>& h_values, const std::vector<constraint>& constraints, int start, int goal, int shift=0);
    std::map<int, int> initialise_map_with_infinity(Graph graph);
    vector<int> get_keys(const map<int, int>& came_from);
    vector<int> reconstruct_path(map<int, int> came_from, pair<int, int> current);
    bool in_frontier(int id, priority_queue_sorted_by_f_value frontier);
    bool in_constraints(std::vector<constraint> constraints, int vertex_id, int time_step);
    std::vector<int> run(Graph graph, std::map<std::pair<int, int>,int> h_values, const std::vector<constraint>& constraints, int start, int goal, int shift);
    std::vector<int> get_best_path();

};


#endif //MT_MAPF_ASTARSEARCH_H
