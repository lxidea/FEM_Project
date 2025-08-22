#include <gtest/gtest.h>
#include "Mesh.h" // Include the class we want to test
#include <fstream> // Needed to write our test file

// Test #1: Checks that a newly created Mesh object is empty.
TEST(MeshTest, IsEmptyInitially) {
    Mesh myMesh;
    ASSERT_EQ(myMesh.getNumNodes(), 0);
    ASSERT_EQ(myMesh.getNumElements(), 0);
}

// Test #2: Checks if we can load a mesh from our simple file format.
TEST(MeshTest, LoadFromFile) {
    // Create a temporary mesh file for our test
    std::ofstream testFile("sample.mesh");
    testFile << "NODES 4\n";
    testFile << "1 0.0 0.0 0.0\n";
    testFile << "2 1.0 0.0 0.0\n";
    testFile << "3 1.0 1.0 0.0\n";
    testFile << "4 0.0 1.0 0.0\n";
    testFile << "ELEMENTS 2\n";
    testFile << "1 4 1 2 3 4\n";
    testFile << "2 4 1 2 4 3\n";
    testFile.close();

    Mesh myMesh;
    bool loadSuccess = myMesh.loadFromFile("sample.mesh");

    // Assert that the file was loaded successfully and has the correct data
    ASSERT_TRUE(loadSuccess);
    ASSERT_EQ(myMesh.getNumNodes(), 4);
    ASSERT_EQ(myMesh.getNumElements(), 2);
}