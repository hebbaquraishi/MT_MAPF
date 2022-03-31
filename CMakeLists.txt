cmake_minimum_required(VERSION 3.20)
project(MT_MAPF)

set(CMAKE_CXX_STANDARD 23)

add_executable(MT_MAPF main.cpp Vertex.cpp Vertex.h Agent.cpp Agent.h Definitions.h Graph.cpp Graph.h HValues.cpp HValues.h Node.cpp Node.h LowLevelSearch.cpp LowLevelSearch.h AStarSearch.cpp AStarSearch.h ConstraintForest.cpp ConstraintForest.h GoalTraversalOrders.cpp GoalTraversalOrders.h)
target_include_directories(MT_MAPF SYSTEM PRIVATE /opt/homebrew/Cellar/nlohmann-json/3.9.1_1/include)