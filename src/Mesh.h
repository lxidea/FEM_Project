#pragma once

#include <vector>
#include <string>
#include "Node.h"     // <-- Include the new header
#include "Element.h"  // <-- Include the new header

class Mesh {
public:
    Mesh();
    size_t getNumNodes() const;
    size_t getNumElements() const;
    bool loadFromFile(const std::string& filename);
    const std::vector<Node>& getNodes() const;
    const std::vector<Element>& getElements() const;
    void addNode(int id, double x, double y, double z);
    void addElement(const std::vector<int>& connectivity);

private:
    std::vector<Node> nodes_;
    std::vector<Element> elements_;
};