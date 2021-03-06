/*
 * Author: Hebba Quraishi
 * Email: quraishi@tf.uni-freiburg.de
 * The objective of this file is to centralise all definitions
*/


#ifndef MT_MAPF_DEFINITIONS_H
#define MT_MAPF_DEFINITIONS_H
#include "Vertex.h"
#include "nlohmann/json.hpp"
#include <queue>
using namespace std;


typedef std::vector<Vertex> vertices_vector;
typedef std::pair<int, int> constraint;  //key:= vertex_id, value:= time stamp

struct sort_by_f_value {
    bool operator()(std::pair<std::pair<int, int>, int> x, std::pair<std::pair<int, int>, int> y){
        return x.second > y.second;
    }
};
typedef std::priority_queue<std::pair<std::pair<int, int>, int>, std::vector<std::pair<std::pair<int, int>, int>>, sort_by_f_value> priority_queue_sorted_by_f_value; //priority queue ordered by f-values. key := vertex id, value := f-value

struct sort_by_path_cost{
    bool operator() (pair<int, int> a, pair<int, int> b){
        return ( a.second < b.second);
    }
};

struct Conflict{
    std::string agent1;
    std::string agent2;
    int vertex = -1;
    int timestamp = -1;
    Conflict() = default;
};

#endif //MT_MAPF_DEFINITIONS_H


