/*
 * Author: Hebba Quraishi
 * Email: quraishi@tf.uni-freiburg.de
 * The objective of this file is to implement a Graph data structure
*/

#ifndef MT_MAPF_GRAPH_H
#define MT_MAPF_GRAPH_H
#include "Definitions.h"

#include <unordered_map>
using json = nlohmann::json;


class Graph {
    int height{}, width{};
    vertices_vector vertices;
    std::unordered_map<int, Vertex> vertex_ids; //key:= vertex_id, value:= Vertex
    std::unordered_map<std::string, int> inverse_vertex_ids; //key:= Vertex name, value:= vertex_id
    std::unordered_map<int, std::vector<int>> neighbours; //key:= vertex_id, value:= ids of all neighbours
public:
    explicit Graph(const std::string& map_location);
    Graph() = default;
    void initialise_vertices(json input_map_json);     //used to initialise the vertices of the edges
    void initialise_neighbours();
    Vertex get_vertex_from_id(int id);
    int get_vertex_id_from_name(const std::string& name);
    std::vector<int> get_neighbours(int id);
    std::unordered_map<int, Vertex> get_vertex_ids();
};


#endif //MT_MAPF_GRAPH_H
