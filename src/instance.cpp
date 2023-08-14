//
// Created by cuong on 12/08/2023.
//

#include "../include/instance.h"

void GenerateSubsets(const std::vector<Node>& nodes, std::vector<Node>& current, int index, std::vector<std::vector<Node>>& subsets) {
    if (current.size() >= 2) {  // Ensure at least two elements in the subset
        subsets.push_back(current);
    }

    if (index == nodes.size()) {
        return;
    }

    if (nodes[index].Demand > 0) {  // Exclude nodes with zero demand (depots)
        GenerateSubsets(nodes, current, index + 1, subsets);

        bool alreadyInSubset = false;
        for (const Node& node : current) {
            if (node.Index == nodes[index].Index) {
                alreadyInSubset = true;
                break;
            }
        }
        if (!alreadyInSubset) {
            current.push_back(nodes[index]);
            GenerateSubsets(nodes, current, index + 1, subsets);
            current.pop_back();
        }
    } else {
        GenerateSubsets(nodes, current, index + 1, subsets);
    }
}

std::vector<std::vector<Node>> GetAllSubsets(const std::vector<Node>& nodes) {
    std::vector<std::vector<Node>> subsets;
    std::vector<Node> current;
    GenerateSubsets(nodes, current, 1, subsets);  // Start from index 1 to exclude depots
    return subsets;
}

vector<Node> GetComplementSet(vector<Node> originalSet, vector<Node> subset) {
    std::unordered_set<int> subsetIds;

    for (const Node& node : subset) {
        subsetIds.insert(node.Index);
    }

    std::vector<Node> complementSet;

    for (const Node& node : originalSet) {
        if (subsetIds.find(node.Index) == subsetIds.end()) {
            complementSet.push_back(node);
        }
    }

    return complementSet;
}
vector<string> SplitStringWithDelimiter(string s, string delimiter) {
    vector<string> returnValue;
    string::size_type start = 0;
    string::size_type end = s.find(delimiter);

    while(end != string::npos) {
        returnValue.push_back(s.substr(start, end-start));
        start = end + 1;
        end = s.find(delimiter, start);
    }

    returnValue.push_back(s.substr(start));
    return returnValue;
}

string TrimSpace(const std::string& str) {
    size_t first = str.find_first_not_of(' ');
    if (first == string::npos) {
        return ""; // String contains only spaces
    }

    size_t last = str.find_last_not_of(' ');
    return str.substr(first, (last - first + 1));
}

Node::Node(int idx, int x, int y) {
    this->Index = idx;
    this->X = x;
    this->Y = y;
}
NodeSubset::NodeSubset(vector<Node> ns, vector<Node> cs, int q, int r) {
    this->Nodes = ns;
    this->ComplementSet = cs;
    this->Q = q;
    this->R = r;
}

Instance::Instance(string filePath) {
    vector<Node> nodes;
    vector<NodeSubset> nodeSubsets;
    string line;
    ifstream data(filePath);
    bool startReadCoordDist, startReadDemand, haveNumVehicles = false;

    while (getline(data, line)) {
        if (!startReadCoordDist && !startReadDemand) {
            // Instance's specification part
            if (line.find("NAME") != string::npos) {
                auto lc = SplitStringWithDelimiter(line, ":");
                Name = TrimSpace(lc[1]);
                continue;
            }
            if (line.find("TYPE") != string::npos && line.find("EDGE") == string::npos &&
                line.find("DATA") == string::npos) {
                auto lc = SplitStringWithDelimiter(line, ":");
                Type = TrimSpace(lc[1]);
                continue;
            }
            if (line.find("COMMENT") != string::npos) {
                auto lc = SplitStringWithDelimiter(line, ":");
                Comment = TrimSpace(lc[1]);
                continue;
            }
            if (line.find("DIMENSION") != string::npos) {
                auto lc = SplitStringWithDelimiter(line, ":");
                Dimension = stoi(TrimSpace(lc[1]));
                continue;
            }

            if (line.find("CAPACITY") != string::npos) {
                auto lc = SplitStringWithDelimiter(line, ":");
                Capacity = stoi(TrimSpace(lc[1]));
                continue;
            }
            if (line.find("EDGE_WEIGHT_TYPE") != string::npos) {
                auto lc = SplitStringWithDelimiter(line, ":");
                EdgeWeightType = TrimSpace(lc[1]);
                continue;
            }
            if (line.find("EDGE_WEIGHT_FORMAT") != string::npos) {
                auto lc = SplitStringWithDelimiter(line, ":");
                EdgeWeightFormat = TrimSpace(lc[1]);
                continue;
            }
            if (line.find("EDGE_DATA_FORMAT") != string::npos) {
                auto lc = SplitStringWithDelimiter(line, ":");
                EdgeDataFormat = TrimSpace(lc[1]);
                continue;
            }
            if (line.find("NODE_COORD_TYPE") != string::npos) {
                auto lc = SplitStringWithDelimiter(line, ":");
                NodeCoordType = TrimSpace(lc[1]);
                continue;
            }
            if (line.find("DISPLAY_DATA_TYPE") != string::npos) {
                auto lc = SplitStringWithDelimiter(line, ":");
                DisplayDataType = TrimSpace(lc[1]);
                continue;
            }

            // Instance's data part
            if (line.find("NODE_COORD_SECTION") != string::npos) {
                startReadCoordDist = true;
                continue;
            }
            if (line.find("EDGE_WEIGHT_SECTION") != string::npos) {
                startReadCoordDist = true;
                continue;
            }
            if (line.find("DEMAND_SECTION") != string::npos) {
                startReadDemand = true;
                continue;
            }
            if (line.find("EDGE_DATA_SECTION") != string::npos) {
                startReadCoordDist = true;
                continue;
            }
            if (line.find("FIXED_EDGES_SECTION") != string::npos) {
                startReadCoordDist = true;
                continue;
            }
            if (line.find("DISPLAY_DATA_SECTION") != string::npos) {
                startReadCoordDist = true;
                continue;
            }
            if (line.find("TOUR_SECTION") != string::npos) {
                startReadCoordDist = true;
                continue;
            }
            if (line.find("NUMBER_VEHICLES") != string::npos) {
                auto lc = SplitStringWithDelimiter(line, ":");
                NumberOfVehicles = stoi(TrimSpace(lc[1]));
                haveNumVehicles = true;
                continue;
            }

        }
        if (startReadCoordDist && !startReadDemand) {
            if (EdgeWeightType == "EUC_2D" || EdgeWeightType == "ATT") {
                istringstream iss(line);
                int num1, num2, num3;
                iss >> num1 >> num2 >> num3;
                auto n0 = Node(num1-1, num2, num3);
                nodes.push_back(n0);
                for (int i = 1; i < Dimension; ++i) {
                    int nid, x, y;
                    data >> nid >> x >> y;
                    auto n = Node(nid - 1, x, y);
                    nodes.push_back(n);
                }
                startReadCoordDist = false;
                startReadDemand = true;
                cout << "Done reading nodes" << endl;
            }
//            else if (EdgeWeightType == "GEO") {
//                vector<double> xVec(Dimension);
//                vector<double> yVec(Dimension);
//                istringstream iss(line);
//                double num1, num2, num3;
//                iss >> num1 >> num2 >> num3;
//                xVec[0] = num2;
//                yVec[0] = num3;
//                for (int i = 1; i < Dimension; ++i) {
//                    int id;
//                    double x, y;
//                    data >> id >> x >> y;
//                    xVec[i] = x;
//                    yVec[i] = y;
//                }
//
//                vector<vector<int>> dist(Dimension, vector<int>(Dimension));
//                for (int i = 0; i < Dimension; i++) {
//                    for (int j = 0; j < Dimension; j++) {
//                        if (i == j) {
//                            dist[i][j] = 0;
//                        } else {
//
//                            double PI = 3.141592;
//                            auto degIX = int(xVec[i] + 0.5);
//                            auto minIX = xVec[i] - degIX;
//                            auto latI = PI * (degIX + 5.0 * minIX / 3.0) / 180.0;
//
//                            auto degIY = int(yVec[i] + 0.5);
//                            auto minIY = yVec[i] - degIY;
//                            auto longI = PI * (degIY + 5.0 * minIY / 3.0) / 180.0;
//
//                            auto degJX = int(xVec[j] + 0.5);
//                            auto minJX = xVec[j] - degJX;
//                            auto latJ = PI * (degJX + 5.0 * minJX / 3.0) / 180.0;
//
//                            auto degJY = int(yVec[j] + 0.5);
//                            auto minJY = yVec[j] - degJY;
//                            auto longJ = PI * (degJY + 5.0 * minJY / 3.0) / 180.0;
//
//                            auto RRR = 6378.388;
//                            auto q1 = cos(longI - longJ);
//                            auto q2 = cos(latI - latJ);
//                            auto q3 = cos(latI + latJ);
//                            dist[i][j] = int(RRR * acos(0.5 * ((1.0 + q1) * q1 - (1.0 - q1) * q3)) + 1.0);
//                        }
//                    }
//                }
//
//                DistanceMatrix = dist;
//                break;
//            } else if (EdgeWeightType == "EXPLICIT") {
//
//                // FULL_MATRIX - TWOD_DISPLAY
//                // LOWER_DIAG_ROW - TWOD_DISPLAY
//                if (EdgeWeightFormat == "FULL_MATRIX") {
//                    vector<vector<int>> dist;
//                    while (dist.size() < Dimension) {
//                        istringstream iss(line);
//                        vector<int> row;
//                        int num;
//                        while (iss >> num) {
//                            row.push_back(num);
//                            // Handle the extra space (if any) between the numbers
//                            if (iss.peek() == ' ')
//                                iss.ignore();
//                        }
//                        dist.push_back(row);
//                        getline(data, line);
//                    }
//                    DistanceMatrix = dist;
//                    break;
//
//                } else if (EdgeWeightFormat == "LOWER_DIAG_ROW") {
//                    vector<vector<int>> dist(Dimension, vector<int>(Dimension));
//                    vector<int> nums;
//                    int num;
//                    istringstream iss(line);
//                    while (iss >> num) {
//                        nums.push_back(num);
//                    }
//                    while (getline(data, line)) {
//                        if (TrimSpace(line) != "DISPLAY_DATA_SECTION") {
//                            istringstream iss2(line);
//                            while (iss2 >> num) {
//                                nums.push_back(num);
//                            }
//                        } else {
//                            break;
//                        }
//                    }
//                    int numIdx = 0;
//                    for (int i = 0; i < Dimension; i++) {
//                        for (int j = 0; j <= i; j++) {
//                            dist[i][j] = dist[j][i] = nums[numIdx];
//                            numIdx++;
//                        }
//                    }
//                    DistanceMatrix = dist;
//                    break;
//                }
//            }
        }
        if (startReadDemand && !startReadCoordDist) {
            // "data" stream was not read?
            string str_dm_st;
            data >> str_dm_st;
            for (int i = 0; i < Dimension; i++) {
                int idx, dm;
                data >> idx >> dm;
                nodes[idx - 1].Demand = dm;

            }
            startReadDemand = false;
            cout << "Done read demand" << endl;
        }


        // got node and demand.
        // time to gen subset.

    }
    Nodes = nodes;
    for (int i = 0; i < Nodes.size(); i++) {
        if (Nodes[i].Demand != 0) {
            NodesWithoutDepots.push_back(Nodes[i]);
        }
    }
    auto allNS = GetAllSubsets(NodesWithoutDepots);
    for (int i = 0; i < allNS.size(); i++) {
        auto ns = allNS[i];
        int q = 0;
        int r;
        // calculat
        for (int nid = 0; nid < ns.size(); nid++) {
            q += ns[nid].Demand;
        }
        r = ceil(q / Capacity);
        if (r == 0) {
            r = 1;
        }
        auto cs = GetComplementSet(Nodes, ns);
        auto nss = NodeSubset(ns, cs, q, r);
        NodeSubsets.push_back(nss);
    }

    // time to calculate actual data?
    // if EUC_2D || ATT => cal.
    // if not then it is done as the dist is calculated in the instance's text file///
    if (EdgeWeightType == "EUC_2D" || EdgeWeightType == "ATT") {
        vector<vector<int>> dist(Dimension, vector<int>(Dimension));
        for (int i = 0; i < Dimension; i++) {
            for (int j = 0; j < Dimension; j++) {
                if (i == j) {
                    dist[i][j] = 0;
                } else {
                    auto xDif = Nodes[i].X - Nodes[j].X;
                    auto yDif = Nodes[i].Y - Nodes[j].Y;
                    dist[i][j] = ceil((sqrt(xDif * xDif + yDif * yDif)));
                }
            }
        }
        DistanceMatrix = dist;
    }
    if (!haveNumVehicles) {
        NumberOfVehicles = 1;
    }
}
void Instance::PrintInstanceInfo() {
    cout << "Instance name: " << Name << endl;
    cout << "Type: " << Type << endl;
    if (!Comment.empty()) {
        cout << "Comment: " << Comment << endl;
    }

    cout << "Number of vertices: " << Dimension << endl;
    // Nodes info
    for (int i = 0; i < Dimension; i++) {
        cout << "Node " << i << ", X = " << Nodes[i].X << ", Y = " << Nodes[i].Y << ", Demand = " << Nodes[i].Demand << endl;
    }
    for (int i = 0; i < NodeSubsets.size(); i++) {
        cout << "This subset consists of " << NodeSubsets[i].Nodes.size() << " nodes" << endl;
        cout << "Demand of this subset is: " << NodeSubsets[i].Q << endl;
        cout << "Minimum number of vehicles needed for this subset is: " << NodeSubsets[i].R << endl;
    }

    for (int i = 0; i < Dimension; i++) {
        for (int j = 0; j < Dimension; j++) {
            cout << DistanceMatrix[i][j] <<  " ";
        }
        cout << endl;
    }
}

