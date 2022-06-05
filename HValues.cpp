/*
 * Author: Hebba Quraishi
 * Email: quraishi@tf.uni-freiburg.de
 * The objective of this file is to find the h_values using Breadth First Search
*/

#include "HValues.h"
#include <utility>


HValues::HValues(const std::unordered_map<std::string, int>& inverse_vertex_ids, const std::unordered_map<int, std::vector<int>>& neighbours, std::vector<Agent> agents) {
    this->neighbours = neighbours;
    std::vector<int> all_goals;
    for (auto &agent : agents) {
        std::vector<Vertex> goals = agent.get_goals();
        for (auto &goal : goals) {
            if (find(all_goals.begin(), all_goals.end(), inverse_vertex_ids.at(goal.name)) == all_goals.end()) {
                all_goals.emplace_back(inverse_vertex_ids.at(goal.name));

                std::map<int, bool> discovered;
                std::queue<std::pair<int, int>> q; //key:= vertex_id; value:= distance from root

                std::pair<int, int> root; //key:= vertex_id; value:= distance from root
                root.first = inverse_vertex_ids.at(goal.name);
                root.second = 0;

                q.push(root);
                discovered[inverse_vertex_ids.at(goal.name)] = true;
                run_bfs(root, q, discovered);
            }
        }
    }
}


void HValues::run_bfs(const std::pair<int, int>& root, std::queue<std::pair<int, int>> q, std::map<int, bool> discovered){
    if(q.empty()){
        return;
    }
    std::pair<int, int> n = q.front(); //key:= vertex_id; value:= distance from root
    q.pop();
    for(auto& node : neighbours[n.first]){
        if(discovered.find(node) == discovered.end()){
            discovered[node] = true;

            std::pair<int, int> child;
            child.first = node;
            child.second = n.second+1;

            q.push(child);
            h_values[{root.first, child.first}] = child.second;
            h_values[{child.first, root.first}] = child.second;
        }
    }
    run_bfs(root,q, std::move(discovered));
}

int HValues::get_h_values(std::pair<int, int> h) {
    return h_values[h];
}

HValues::~HValues()= default;

std::map<std::pair<int, int>,int> HValues::get_all_h_values(){
    return this->h_values;
}
