#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &model.FindClosestNode(start_x, start_y);
    end_node = &model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return end_node->distance(*node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    const auto g = current_node->g_value;
    current_node->FindNeighbors();
    for(auto neighbor : current_node->neighbors) {
        if (neighbor->visited) {
            continue;
        }
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = g + current_node->distance(*neighbor);
        neighbor->parent = current_node;
        neighbor->visited = true;
        open_list.push_back(neighbor);
    }
}

bool Compare(const  RouteModel::Node *a, const RouteModel::Node *b) {
    const float f_a = a->h_value + a->g_value;
    const float f_b = b->h_value + b->g_value;
    return f_a > f_b;
}

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), Compare);
    const auto next_node = open_list.back();
    open_list.pop_back();
    return next_node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found = {*current_node};
    while (start_node != current_node) {
        auto parent = current_node->parent;
        distance += current_node->distance(*parent);
        path_found.push_back(*parent); 
        current_node = parent;
    } 
    std::reverse(path_found.begin(), path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}

void RoutePlanner::AStarSearch() {
     RouteModel::Node *current_node = nullptr;
    
    open_list.push_back(start_node);
    start_node->visited = true;
    current_node = start_node;
    while (end_node != current_node)  {
        AddNeighbors(current_node);
        current_node = NextNode();
    }
    m_Model.path =  ConstructFinalPath(current_node);
}