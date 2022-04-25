#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include "Definitions.h"
#include "Agent.h"
#include "Graph.h"
#include "HValues.h"
#include "ConstraintForest.h"

using namespace std;
using json = nlohmann::json;



int main() {
    //Provide locations of the map and agents json files
    string map_location = "/Users/hebbaquraishi/Desktop/MT_MAPF/results/maps/random-32-32-20.json";
    string agent_location = "/Users/hebbaquraishi/Desktop/MT_MAPF/results/current/my_agents.json";

    cout<<"*********************************** MT_MAPF ***********************************"<<endl;


    //Initialise the graph
    Graph graph = Graph(map_location);
    cout<<"\nGraph initialised"<<endl;

    //Initialise agents
    json input_agents_json;
    ifstream input_agents(agent_location);
    input_agents >> input_agents_json;
    std::vector<Vertex> goals;
    std::vector<Agent> agents;
    for (int i = 0; i< int(input_agents_json["names"].size()); i++){
        for (auto & j : input_agents_json["goal"][i]){
            Vertex v = Vertex(j[0], j[1]);
            v.id = graph.get_vertex_id_from_name(v.name);
            goals.emplace_back(v);
        }
        Vertex start = Vertex(input_agents_json["initial"][i][0], input_agents_json["initial"][i][1]);
        start.id = graph.get_vertex_id_from_name(start.name);
        Agent a = Agent(input_agents_json["names"][i], start, goals);
        agents.emplace_back(a);
        goals = {};
    }

    //Calculate h-values
    HValues h_values = HValues(graph, agents);
    cout<<"h-values calculated"<<endl;

    //Run the code
    ConstraintForest forest = ConstraintForest(graph, agents, h_values);
    forest.run();
    return 0;
}
