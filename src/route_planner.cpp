#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // setup start and end nodes with inputs
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// Use distance to end node for h value
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    float h = node->distance(*end_node); //alternatively, (*node).distance(*end_node)
    return h;
}


// Expand current bode by adding all unvisited neighbors to the open list 
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();

    for (auto node : current_node->neighbors)
    {
        if (!node->visited)
        {
            // update neighbor's parameters and set to visited
            node->parent = current_node;
            node->h_value = CalculateHValue(node);
            node->g_value = current_node->g_value + current_node->distance(*node);
            node->visited = true;

            //add node to open list 
            open_list.push_back(node);
        }
    }
}

// compare the f-value of two nodes here
bool Compare(RouteModel::Node* node1 , RouteModel::Node* node2)
{
    // find lowest h+g
    float f1 = node1->g_value + node1->h_value;
    float f2 = node2->g_value + node2->h_value;

    return f1 > f2;
}

// sort the open list and return the next node
RouteModel::Node *RoutePlanner::NextNode() {
    //sort open list based on h+g (gets lowest cost?)
    sort(open_list.begin(), open_list.end(), Compare);

    // get closest node
    RouteModel::Node *closest = open_list.back();

    // remove first element from open list 
    open_list.pop_back();

    return closest;
}


// Return the final path found from A* search
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // Generate the path found, follow chain of parents until starting node is found
    while (current_node->parent != nullptr)
    {
        // add curr node to path_found vector
        path_found.emplace_back(*current_node);

        //add distance from current to parent to total distance
        distance += current_node->distance(*current_node->parent);

        //advance current node to parent
        current_node = current_node->parent;
    }

    // add final node to list since it wont have a parent  
    path_found.emplace_back(*current_node);

    // reverse order of path found, start node should be first node of vector
    reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}

// Implementation of A* to find the shortest path the end node
void RoutePlanner::AStarSearch() {
    // initialize starting node
    RouteModel::Node *current_node = nullptr;
    current_node = start_node;
    current_node->h_value = CalculateHValue(current_node);
    current_node->g_value = 0;
    current_node->visited = true;

    // A* search, run the search until the end node is hit 
    while(current_node != end_node)
    {
        // add all of the neighbors of the current node to the open_list.
        AddNeighbors(current_node);

        // sort the open_list and return the next node
        current_node = NextNode();
    }

    // set m_Model.path to final path for the display 
    m_Model.path = ConstructFinalPath(end_node);
}
