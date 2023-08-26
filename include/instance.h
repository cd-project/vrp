//
// Created by cuong on 12/08/2023.
//

#ifndef VRP_INSTANCE_H
#define VRP_INSTANCE_H

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include <sstream>
#include <unordered_set>

using namespace std;

class Node {
public:
    int Index;
    int X;
    int Y;
    int Demand;
    Node(int idx, int x, int y);
};

class NodeSubset {
public:
    vector<Node> Nodes;
    vector<Node> ComplementSet;
    int Q;
    int R;
    NodeSubset(vector<Node> ns, vector<Node> cs, int q, int r);
};

class Instance {
public:
    string Name; // Instance's name
    string Type; // Instance's type: CVRP
    string Comment; // Additional comment
    int Dimension;
    int NumberOfVehicles;
    vector<vector<int>> DistanceMatrix;
    vector<Node> Nodes; // List of nodes presented in this instance
    vector<Node> NodesWithoutDepots; // List of nodes presented in this instance, without depots
    vector<NodeSubset> NodeSubsets; // Contains subset of nodes, its demand and r(S): minimum number of vehicles needed for this subset.
    int Capacity; // Truck's capacity.
    string EdgeWeightType;
    string EdgeWeightFormat;
    string DisplayDataType;
    string EdgeDataFormat;
    string NodeCoordType;
    Instance();
    explicit Instance(string filePath);
    void PrintInstanceInfo();

    vector<string> SplitStringWithDelimiter(string s, string delimiter);
};
#endif //VRP_INSTANCE_H
