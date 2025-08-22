#pragma once

#include <vector>

// A simple structure for a generic element.
struct Element {
    int id;
    std::vector<int> connectivity; // List of node IDs that form the element
};