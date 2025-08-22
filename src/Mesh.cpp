#include "Mesh.h"
#include <fstream>
#include <sstream>
#include <iostream>

// Constructor implementation
Mesh::Mesh() {
    // The node and element vectors are created empty by default.
}

// getNumNodes implementation
size_t Mesh::getNumNodes() const {
    return nodes_.size();
}

// getNumElements implementation
size_t Mesh::getNumElements() const {
    return elements_.size();
}

bool Mesh::loadFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open mesh file " << filename << std::endl;
        return false;
    }

    // Clear any existing data
    nodes_.clear();
    elements_.clear();

    std::string line;
    std::string keyword;
    int count;

    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') {
            continue; // Skip empty lines and comments
        }

        std::stringstream ss(line);
        ss >> keyword;

        if (keyword == "NODES") {
            ss >> count;
            nodes_.reserve(count);
            for (int i = 0; i < count; ++i) {
                Node n;
                file >> n.id >> n.x >> n.y >> n.z;
                nodes_.push_back(n);
            }
        } else if (keyword == "ELEMENTS") {
            ss >> count;
            elements_.reserve(count);
            for (int i = 0; i < count; ++i) {
                Element e;
                int type; // We read the type but don't use it yet
                file >> e.id >> type;
                e.connectivity.resize(type);
                for (int j = 0; j < type; ++j) {
                    file >> e.connectivity[j];
                }
                elements_.push_back(e);
            }
        }
    }

    return true;
}

const std::vector<Node>& Mesh::getNodes() const {
    return nodes_;
}

const std::vector<Element>& Mesh::getElements() const {
    return elements_;
}

void Mesh::addNode(int id, double x, double y, double z) {
    nodes_.push_back({id, x, y, z});
}

void Mesh::addElement(const std::vector<int>& connectivity) {
    // Automatically assign the next available element ID
    int new_id = static_cast<int>(elements_.size()) + 1;
    elements_.push_back({new_id, connectivity});
}