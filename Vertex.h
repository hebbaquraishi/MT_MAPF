/*
 * Author: Hebba Quraishi
 * Email: quraishi@tf.uni-freiburg.de
 * The objective of this file is to implement a Vertex
*/

#ifndef MT_MAPF_VERTEX_H
#define MT_MAPF_VERTEX_H
#include <string>
#include <set>
#include <vector>


class Vertex {
    int x_coordinate;
    int y_coordinate;
public:
    int id;
    std::string name;
    Vertex()=default;
    Vertex(int x, int y);
    [[nodiscard]] std::pair<int, int> get_coordinates() const;
};



#endif //MT_MAPF_VERTEX_H