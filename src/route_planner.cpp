#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Find notes that belong to the inputs
  	this->start_node = &(m_Model.FindClosestNode(start_x, start_y));
  	this->end_node = &(m_Model.FindClosestNode(end_x, end_y));
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  	/*
    	Calculates heuristic value for a given node
    */
	return node->distance(*(this->end_node));
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  	/*
    	Adds neighbors of the current node to the open list.
    */
	current_node->FindNeighbors();
  	for (auto neighbor : current_node->neighbors) {
      	neighbor->parent = current_node;
   		neighbor->h_value = CalculateHValue(neighbor); 
      	neighbor->g_value = current_node->g_value + neighbor->distance(*current_node);
      	neighbor->visited = true;
      	this->open_list.emplace_back(neighbor);
    }
}

bool compareNodes(RouteModel::Node* node1, RouteModel::Node* node2) {
 	/*
    	Compares two nodes by their F value (g+h)
    */
  	float f1 = node1->g_value + node1->h_value;
  	float f2 = node2->g_value + node2->h_value;
  	return f1 > f2; 
}

void sortNodes(std::vector<RouteModel::Node*> *list) {
  /*
  	  Sorts a list of nodes by their f values
  */
  	std::sort(list->begin(), list->end(), compareNodes);
}

RouteModel::Node *RoutePlanner::NextNode() {
  	/*
    	Finds node, that has the lowest f value
    */
	sortNodes(&(this->open_list));
  	RouteModel::Node *next = open_list.back();
  	open_list.pop_back();
  	return next;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
  	/*
    	Constructs the final path from start to end node.
    */
  
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // Add current node (goal) to the path
  	RouteModel::Node *follower = current_node;
  	path_found.push_back(*current_node);
  
  	//Iterate by the parent back to the start node
  	while (follower != this->start_node) {
      	distance += follower->distance(*(follower->parent));
      	follower = follower->parent;
      	path_found.push_back(*follower);
    }
  	// Reverse the path, so it starts by the starting point and ends at the goal
  	std::reverse(path_found.begin(), path_found.end());
  
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {
  	/*
    	Performs an A* search to find the optimal path between start and end node.
    */
    RouteModel::Node *current_node = nullptr;

    // Add start to open list
  	this->open_list.push_back(this->start_node);
  	start_node->visited = true;
  
  	while(open_list.size() != 0) {
      	current_node = NextNode();
      	// Check if goal is reached
      	if (current_node == this->end_node) {
          	m_Model.path = ConstructFinalPath(current_node);
          	return;
        }
      	// if not continue
      	AddNeighbors(current_node);
    }
}