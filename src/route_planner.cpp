#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Store the nodes found in the RoutePlanner's start_node and end_node attributes.
  
    // FindClosestNode returns a reference to a Node, and start_node and end_node are both
    // pointers  to Nodes, so update the pointers with references to the new returned odes
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);

}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    float h_val = node->distance(*end_node);
    return (h_val); 
}


// AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // populate current_node.neighbors vector with all the neighbors
    current_node->FindNeighbors();
  
    for (int i = 0; i<current_node->neighbors.size(); i++){
        current_node->neighbors[i]->parent = current_node;
        current_node->neighbors[i]->h_value = CalculateHValue(current_node->neighbors[i]);
        current_node->neighbors[i]->g_value = current_node->g_value + current_node->distance(*(current_node->neighbors[i]));
    
        open_list.push_back(current_node->neighbors[i]);
        current_node->neighbors[i]->visited = true;
    }
  
}


/**
 * Compare the F values of two cells.
 */
bool Compare(const RouteModel::Node *a, const RouteModel::Node *b) {
    float f1 = a->g_value + a->h_value; // f1 = g1 + h1
    float f2 = b->g_value + b->h_value; // f2 = g2 + h2
    return f1 > f2; 
}


// NextNode method to sort the open list and return the next node.
// - Sorts the open_list according to the sum of the h value and g value.
// - Creates a pointer to the node in the list with the lowest sum.
// - Removes that node from the open_list.
// - Returns the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
    sort(open_list.begin(), open_list.end(), Compare);
    RouteModel::Node *lowest_sum  = open_list.back();
    open_list.pop_back();
    return lowest_sum;
}


// ConstructFinalPath method to return the final path found from A* search.
// - Takes the current (final) node as an argument and iteratively follows the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, adds the distance from the node to its parent to the distance variable.
// - The returned vector must in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // make a copy of the current_node ptr to manipulate 
    RouteModel::Node *curr = current_node;
    while (curr->parent != nullptr){
        distance += curr->distance(*(curr->parent));
        //add node to final path vector
        path_found.push_back(*curr);
        //update curr to point to the parent node
        curr = curr->parent;
    }
  
    // add the starting node
    path_found.push_back(*curr);
    
    // reverse order of nodes so first node in vector
    // is first node in path
    reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// A* Search algorithm
// - Uses AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Uses NextNode() method to sort the open_list and return the next node.
// - When search has reached the end_node, uses the ConstructFinalPath method to return the final path that was found.
// - Stores the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
  
    start_node->visited = true;
    open_list.push_back(start_node);
    current_node = start_node;
  
    while (current_node != end_node){
        AddNeighbors(current_node);
        current_node = NextNode();
    }
  
    m_Model.path = ConstructFinalPath(current_node);
    
}