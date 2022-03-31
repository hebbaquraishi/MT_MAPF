//
// Created by Hebba Quraishi on 31.03.22.
//

#include "LowLevelSearch.h"



LowLevelSearch::LowLevelSearch(const Graph& graph, std::map<std::pair<int, int>,int> h_values, std::unordered_map<std::string, std::pair<int, std::vector<int>>> assignment, std::unordered_map<std::string, std::vector<constraint>> agent_constraints) {
    for(auto& value: assignment){
        int shift = 0;
        for(int i = 0; i < (int)value.second.second.size()-1; i++){
            AStarSearch a_star = AStarSearch(graph, h_values, agent_constraints[value.first], value.second.second[i], value.second.second[i+1], shift);
            vector<int> path = a_star.get_best_path();
            if(i == 0){
                shift = shift + (int)(path.size()-1);
            }
            else{
                path.erase(path.begin());
                shift = shift + (int)(path.size());
            }
            this->solutions[value.first].insert(this->solutions[value.first].end(), path.begin(), path.end());
        }
        this->total_solution_cost += (int)this->solutions[value.first].size() -1;
        if((int)this->solutions[value.first].size() -1 > (int)this->largest_solution_cost){
            this->largest_solution_cost = (int)this->solutions[value.first].size() -1;
        }
    }
}

std::unordered_map<std::string, std::vector<int>> LowLevelSearch::get_agent_wise_solutions(){
    return this->solutions;
}

int LowLevelSearch::get_total_solution_cost() const{
    return this->total_solution_cost;
}

int LowLevelSearch::get_largest_solution_cost() const{
    return this->largest_solution_cost;
}
