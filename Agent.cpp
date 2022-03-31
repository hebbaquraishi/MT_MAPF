/*
 * Author: Hebba Quraishi
 * Email: quraishi@tf.uni-freiburg.de
 * The objective of this file is to implement an Agent
*/

#include "Agent.h"
#include <nlohmann/json.hpp>
#include <utility>

using json = nlohmann::json;
using namespace std;

Agent::Agent(std::string name, Vertex start, vertices_vector goals){
    this->name = std::move(name);
    this->start = std::move(start);
    this->goals=std::move(goals);
}

Vertex Agent::get_init_loc(){
    return this->start;
}

std::vector<Vertex> Agent::get_goals(){
    return this->goals;
}
