//
// Created by Hebba Quraishi on 31.03.22.
//

#include "HValues.h"

#include <utility>


HValues::HValues(Graph graph, std::vector<Agent> agents) {
    std::vector<int> all_goals;
    for (auto &agent : agents) {
        std::vector<Vertex> goals = agent.get_goals();
        for (auto &goal : goals) {
            if (find(all_goals.begin(), all_goals.end(), graph.get_vertex_id_from_name(goal.name)) == all_goals.end()) {
                all_goals.emplace_back(graph.get_vertex_id_from_name(goal.name));

                std::map<int, bool> discovered;
                std::queue<std::pair<int, int>> q; //key:= vertex_id; value:= distance from root

                std::pair<int, int> root; //key:= vertex_id; value:= distance from root
                root.first = graph.get_vertex_id_from_name(goal.name);
                root.second = 0;

                q.push(root);
                discovered[graph.get_vertex_id_from_name(goal.name)] = true;
                run_bfs(graph, root, q, discovered);
            }
        }
    }
}


void HValues::run_bfs(Graph graph, const std::pair<int, int>& root, std::queue<std::pair<int, int>> q, std::map<int, bool> discovered){
    if(q.empty()){
        return;
    }
    std::pair<int, int> n = q.front(); //key:= vertex_id; value:= distance from root
    q.pop();
    for(auto& node : graph.get_neighbours(n.first)){
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
    run_bfs(graph, root,q, std::move(discovered));
}

std::map<std::pair<int, int>,int> HValues::get_h_values() {
    return h_values;
}

HValues::~HValues()= default;
