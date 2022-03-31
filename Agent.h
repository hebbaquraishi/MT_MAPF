/*
 * Author: Hebba Quraishi
 * Email: quraishi@tf.uni-freiburg.de
 * The objective of this file is to implement an Agent
*/

#ifndef MT_MAPF_AGENT_H
#define MT_MAPF_AGENT_H
#include "Definitions.h"
#include "Vertex.h"


class Agent {
    Vertex start;
    vertices_vector goals;
public:
    std::string name;
    Agent() = default;
    Agent(std::string name, Vertex start, vertices_vector goals);
    Vertex get_init_loc();
    vertices_vector get_goals();
};


#endif //MT_MAPF_AGENT_H
