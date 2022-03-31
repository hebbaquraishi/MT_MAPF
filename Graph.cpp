/*
 * Author: Hebba Quraishi
 * Email: quraishi@tf.uni-freiburg.de
 * The objective of this file is to implement a Graph data structure
*/

#include "Graph.h"
#include <fstream>
#include <utility>
using namespace std;
using json = nlohmann::json;


Graph::Graph(const std::string& map_location){
    json input_map_json;
    ifstream input_map(map_location);
    input_map >> input_map_json;
    this->height = input_map_json["height"];
    this->width = input_map_json["width"];
    initialise_vertices(input_map_json);
    initialise_neighbours();
}

void Graph::initialise_vertices(json input_map_json) {
    int id = 0;
    for(auto & i : input_map_json["vertices"]){
        Vertex v = Vertex(i[0], i[1]);
        this->vertex_ids[id] = v;
        this->inverse_vertex_ids[v.name] = id;
        this->vertices.emplace_back(v);
        id++;
    }
}

void Graph::initialise_neighbours(){
    std::vector<std::pair<int, int>> directions = {{0,1}, {0, -1}, {-1, 0}, {1,0}};

    for(const auto& v: this->vertices){
        for(auto d: directions){
            int x = v.get_coordinates().first + d.first;
            int y = v.get_coordinates().second + d.second;
            Vertex u = Vertex(x, y);
            if((this->inverse_vertex_ids.find(u.name) != this->inverse_vertex_ids.end()) && x >= 0 && x < height && y >= 0 && y < width){
                this->neighbours[inverse_vertex_ids[v.name]].push_back(inverse_vertex_ids[u.name]);
            }
            else{
                continue;
            }
        }
    }
}


Vertex Graph::get_vertex_from_id(int id) {
    return this->vertex_ids[id];
}

int Graph::get_vertex_id_from_name(const std::string& name){
    return this->inverse_vertex_ids[name];
}

vector<int> Graph::get_neighbours(int id) {
    return this->neighbours[id];
}

std::unordered_map<int, Vertex> Graph::get_vertex_ids(){
    return this->vertex_ids;
}