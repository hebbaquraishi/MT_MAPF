/*
 * Author: Hebba Quraishi
 * Email: quraishi@tf.uni-freiburg.de
 * The objective of this file is to implement A* Search
*/

#include "AStarSearch.h"
#include <utility>
#include <iostream>
#include <limits>

AStarSearch::AStarSearch(const std::unordered_map<int, Vertex>& vertex_ids, const std::unordered_map<int, std::vector<int>>& neighbours, const std::map<std::pair<int, int>,int>& h_values, const std::vector<constraint>& constraints, int start, int goal, int shift){
    this->neighbours = neighbours;
    this->vertex_ids = vertex_ids;
    run(h_values, constraints, start, goal, shift);
}

map<pair<int,int>, int> AStarSearch::initialise_map_with_infinity(int shift){
    map<pair<int,int>, int> m;
    for(auto& v : this->vertex_ids){
        m[make_pair(v.first, shift)] = numeric_limits<int>::max();
    }
    return m;
}


vector<pair<int, int>> AStarSearch::get_keys(const map<pair<int,int>, pair<int,int>>& came_from){
    vector<pair<int, int>> keys{};
    keys.reserve(came_from.size());
    for(auto &key : came_from){
        keys.emplace_back(key.first);
    }
    return keys;
}


void AStarSearch::reconstruct_path(map<pair<int,int>, pair<int,int>> came_from, pair<int, int> current){
    best_path.emplace_back(current.first);
    pair<int, int> id = current;
    vector<pair<int, int>> keys = get_keys(came_from);
    while (find(keys.begin(), keys.end(), id) != keys.end()){
        id = came_from[id];
        best_path.emplace_back(id.first);
    }
    reverse(best_path.begin(),best_path.end());
}


bool AStarSearch::in_frontier(int id, priority_queue_sorted_by_f_value frontier){
    priority_queue_sorted_by_f_value f = std::move(frontier);
    while (!f.empty()){
        if(f.top().first.first == id){
            return true;
        }
        f.pop();
    }
    return false;
}


bool AStarSearch::in_constraints(std::vector<constraint> constraints, int vertex_id, int time_step){
    //if the next vertex is in a constraint, pop it from the frontier and choose the next best vertex
    constraint c = make_pair(vertex_id, time_step);
    //cout<<"\t\tConstraint checked: "<<v.name<<" at time_step "<<time_step<<endl;
    if(find(constraints.begin(), constraints.end(), c) != constraints.end()){
        return true;
    }
    else{
        return false;
    }
}

void AStarSearch::run(const std::map<std::pair<int, int>,int>& h_values, const std::vector<constraint>& constraints, int start, int goal, int shift){
    priority_queue_sorted_by_f_value frontier;  //priority queue ordered by f-values. key := vertex id, value := f-value
    map<pair<int,int>, pair<int,int>> came_from; //key := id of current vertex, value:= id of current vertex's parent
    //map<int, int> visited_at_time;  //key := vertex id, value:= time step at which this vertex was explored
    map<pair<int,int>, int> g_value = initialise_map_with_infinity(shift);// key:= vertex id, value := g-value
    map<pair<int,int>, int> f_value = initialise_map_with_infinity(shift);// key:= vertex id, value := f-value
    g_value[make_pair(start, shift)] = 0;
    if(start == goal){
        f_value[make_pair(start, shift)] = 0;
    }
    else{
        f_value[make_pair(start, shift)] = h_values.at(make_pair(start, goal));
    }



    frontier.push(make_pair(make_pair(start, shift), f_value[make_pair(start, shift)]));
    //visited_at_time[start] = shift;

    while (!frontier.empty()){
        std::pair<std::pair<int, int>, int> current = frontier.top(); //key := <vertex id, time step>, value := f-value
        frontier.pop();

        if(current.first.first == goal){
            reconstruct_path(came_from, current.first);
            break;
        }
        for(auto& nhbr : this->neighbours[current.first.first]){
            int temp = g_value[current.first] + 1;
            if(temp < g_value[make_pair(nhbr, shift)]){
                int next_time_step = current.first.second + 1;
                //visited_at_time[nhbr] = time_step + 1;
                if(in_constraints(constraints, nhbr, next_time_step)){
                    came_from[make_pair(current.first.first, next_time_step)] = current.first;
                    g_value[make_pair(current.first.first, next_time_step)] = temp;
                    if(current.first.first == goal){
                        f_value[make_pair(current.first.first, next_time_step)] = g_value[make_pair(current.first.first, next_time_step)];
                    }
                    else{
                        f_value[make_pair(current.first.first, next_time_step)] = g_value[make_pair(current.first.first, next_time_step)] + h_values.at(make_pair(current.first.first, goal));
                    }

                    frontier.push(make_pair(make_pair(current.first.first, next_time_step), f_value[make_pair(current.first.first, next_time_step)]));
                    continue;
                }
                else{

                    came_from[make_pair(nhbr, next_time_step)] = current.first;
                    g_value[make_pair(nhbr, next_time_step)] = temp;
                    //cout<<"h-value("<<nhbr<<","<<goal<<")"<<endl;
                    if (nhbr == goal){
                        f_value[make_pair(nhbr, next_time_step)] = g_value[make_pair(nhbr, next_time_step)];
                    }
                    else{
                        f_value[make_pair(nhbr, next_time_step)] = g_value[make_pair(nhbr, next_time_step)] + h_values.at(make_pair(nhbr, goal));
                    }
                    if(!in_frontier(nhbr, frontier)){
                        frontier.push(make_pair(make_pair(nhbr, next_time_step), f_value[make_pair(nhbr, next_time_step)]));
                    }
                }
            }
        }
    }
}


std::vector<int> AStarSearch::get_best_path(){
    return this->best_path;
}
