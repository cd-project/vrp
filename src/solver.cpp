//
// Created by cuong on 12/08/2023.
//

#include <map>
#include <unordered_map>
#include <set>
#include "../include/solver.h"

std::vector<std::tuple<int, int>> getUniqueTuples(const std::vector<std::tuple<int, int>>& tupleVector) {
    std::set<std::tuple<int, int>> uniqueTuples;
    std::vector<std::tuple<int, int>> uniqueTupleOrder;

    for (const auto& tuple : tupleVector) {
        if (uniqueTuples.find(tuple) == uniqueTuples.end()) {
            uniqueTuples.insert(tuple);
            uniqueTupleOrder.push_back(tuple);
        }
    }

    return uniqueTupleOrder;
}

tuple<int, int, double, bool, double> Solver::TwoIndexFlow(double tiLim) {
    tuple<int, int, double, bool, double> result;
    try {
        GRBEnv env;
        GRBModel model(env);
        auto nodes = instance.Nodes;
        auto c = instance.DistanceMatrix;
        auto n = instance.Dimension;
        auto M = instance.NumberOfVehicles; // number of vehicles.
        auto nss = instance.NodeSubsets; // subsets of nodes; contains at least 2 nodes; do not contain depots.
        auto nwd = instance.NodesWithoutDepots;
        std::map<std::string, bool> stringToBoolMap;
        // Variable x (phi) and constraints 5 and 6.
        auto **x = new GRBVar *[n];
        for (int nodeIndex = 0; nodeIndex < nodes.size(); nodeIndex++) {
            x[nodes[nodeIndex].Index] = reinterpret_cast<GRBVar *>(new GRBVar *[n]);
            for (int nwdIndex = 0; nwdIndex < nodes.size(); nwdIndex++) {
                auto i = nodes[nodeIndex].Index;
                auto j = nodes[nwdIndex].Index;
                if (i != j) {
                    if (nodes[nodeIndex].Demand == 0) {
                        x[i][j] = model.addVar(0, 1, 0.0, GRB_INTEGER, "x_" + to_string(i) + "_" + to_string(j));
                        cout << "added " << "x_" + to_string(i) + "_" + to_string(j) << " as = (0, 1, 2)" << endl;
                        stringToBoolMap["x_" + to_string(i) + "_" + to_string(j)] = true;
                    } else {
                        x[i][j] = model.addVar(0, 1, 0.0, GRB_INTEGER, "x_" + to_string(i) + "_" + to_string(j));
                        stringToBoolMap["x_" + to_string(i) + "_" + to_string(j)] = true;
                        cout << "added " << "x_" + to_string(i) + "_" + to_string(j) << " as = (0, 1)" << endl;
                    }
                }
            }
        }

        // Cóntraint 2
        for (int iIdx = 0; iIdx < nwd.size(); iIdx++) {
            GRBLinExpr sum1;
            GRBLinExpr sum2;
            auto i = nwd[iIdx].Index;
            for (int jIdx = 0; jIdx < nodes.size(); jIdx++) {
                auto j = nodes[jIdx].Index;
                if (i < j) {
                    auto key = "x_" + to_string(i) + "_" + to_string(j);
                    if (stringToBoolMap.find(key) == stringToBoolMap.end()) {
                        std::cout << key << " 2.1 not exist in the map." << std::endl;
                    }
                    sum1 += x[i][j];
                }
                if (i > j) {
                    auto key = "x_" + to_string(j) + "_" + to_string(i);
                    if (stringToBoolMap.find(key) == stringToBoolMap.end()) {
                        std::cout << key << " 2.2 not exist in the map." << std::endl;
                    }
                    sum2 += x[j][i];
                }
            }
            model.addConstr(sum1+sum2, GRB_EQUAL, 2);
        }

        // Constraint 3
        for (int ssId = 0; ssId < nss.size(); ssId++) {
            auto cs = nss[ssId].ComplementSet;
            auto s = nss[ssId].Nodes;
            auto r = nss[ssId].R;
            GRBLinExpr sum1;
            GRBLinExpr sum2;
            for (int iIdx = 0; iIdx < s.size(); iIdx++) {
                for (int jIdx = 0; jIdx < cs.size(); jIdx++) {
                    auto i = s[iIdx].Index;
                    auto j = cs[jIdx].Index;
                    if (i < j) {
                        auto key = "x_" + to_string(i) + "_" + to_string(j);
                        if (stringToBoolMap.find(key) == stringToBoolMap.end()) {
                            std::cout << key << "3.1 not exist in the map." << std::endl;
                        }
                        sum1 += x[i][j];
                    }
                }
            }
            for (int iIdx = 0; iIdx < cs.size(); iIdx++) {
                for (int jIdx = 0; jIdx < s.size(); jIdx++) {
                    auto i = cs[iIdx].Index;
                    auto j = s[jIdx].Index;
                    if (i < j) {

                        auto key = "x_" + to_string(i) + "_" + to_string(j);
                        if (stringToBoolMap.find(key) == stringToBoolMap.end()) {
                            cout << i << ", " << j << endl;
                            std::cout << key << " 3.2 not exist in the map." << std::endl;
                        }
                        sum2 += x[i][j];
                    }
                }
            }
            model.addConstr(sum1+sum2, GRB_GREATER_EQUAL, 2*r);

        }

//        // Conỏnaint 7777777
//        for (int subIndex = 0; subIndex < nss.size(); subIndex++) {
//            auto subsetNodes = nss[subIndex];
//            GRBLinExpr sum;
//            for (int iIdx = 0; iIdx < subsetNodes.Nodes.size(); iIdx++) {
//                auto i = subsetNodes.Nodes[iIdx].Index;
//                for (int jIdx = 0; jIdx < subsetNodes.Nodes.size(); jIdx++) {
//                    auto j = subsetNodes.Nodes[jIdx].Index;
//                    if (i < j) {
//                        sum += x[i][j];
//                    }
//                }
//            }
//            model.addConstr(sum, GRB_LESS_EQUAL, subsetNodes.Nodes.size() - subsetNodes.R);
//        }

        // Constraint 4
        GRBLinExpr constr4;
        string cstr4 = "cstr4";
        for (int idx = 0; idx < nwd.size(); idx++) {
            constr4 += x[0][nwd[idx].Index];
            cstr4 += " + x[0][" + to_string(nwd[idx].Index) + "]";
        }
        model.addConstr(constr4, GRB_EQUAL, 2 * M);

        constr4 = 0;
        cout << cstr4 << endl;
        GRBLinExpr objective;
        for (int nodeIndex = 0; nodeIndex < nodes.size(); nodeIndex++) {
            auto i = nodes[nodeIndex].Index;
            for (int node2Idx = 0; node2Idx < nodes.size(); node2Idx++) {
                auto j = nodes[node2Idx].Index;
                if (i != j) {
                    objective += c[i][j] * x[i][j];
                    auto key = "x_" + to_string(i) + "_" + to_string(j);
                    if (stringToBoolMap.find(key) == stringToBoolMap.end()) {
                        std::cout << key << "not exist in the map." << std::endl;
                    }
                }
            }
        }
        model.setObjective(objective, GRB_MINIMIZE);
        model.set(GRB_DoubleParam_TimeLimit, tiLim);
        model.update();
        model.write("model_two_index.lp");
        auto startTime = std::chrono::high_resolution_clock::now();
        model.optimize();
        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        bool optimal = false;
        if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
            optimal = true;
        }
        result = make_tuple(model.get(GRB_DoubleAttr_ObjBound), model.get(GRB_DoubleAttr_ObjVal), duration.count(),
                             optimal, model.get(GRB_DoubleAttr_MIPGap));
        for (int nodeIndex = 0; nodeIndex < nodes.size(); nodeIndex++) {
            auto i = nodes[nodeIndex].Index;
            for (int nwdIndex = 0; nwdIndex < nwd.size(); nwdIndex++) {
                auto j = nwd[nwdIndex].Index;
                auto key = "x_" + to_string(i) + "_" + to_string(j);
                if (stringToBoolMap.find(key) == stringToBoolMap.end()) {
                    std::cout << key << "not exist in the map." << std::endl;
                }
                if (i < j) {
                    cout << "x[" << i << "][" << j << "] = " << x[i][j].get(GRB_DoubleAttr_X) << endl;
                }
            }
        }

    } catch (GRBException e) {
        std::cout << "Error code: " << e.getErrorCode() << std::endl;
        std::cout << e.getMessage() << std::endl;
    }

    return result;

}

tuple<int, int, double, bool, double> Solver::MulticommodityFlow(double tiLim) {
    tuple<int, int, double, bool, double> result;
    try {
        GRBEnv env;
        GRBModel model(env);
        auto nodes = instance.Nodes;
        auto c = instance.DistanceMatrix;
        auto n = instance.Dimension;
        auto M = instance.NumberOfVehicles; // number of vehicles.
        auto nss = instance.NodeSubsets; // subsets of nodes; contains at least 2 nodes; do not contain depots.
        auto nwd = instance.NodesWithoutDepots;
        std::map<std::string, bool> mapX;
        std::map<std::string, bool> mapY;
        auto** x = new GRBVar * [n];
        for (int iIdx = 0; iIdx < n; iIdx++) {
            auto i = nodes[iIdx].Index;
            x[i] = reinterpret_cast<GRBVar *>(new GRBVar *[n]);
            for (int jIdx = 0; jIdx < n; jIdx++) {
                auto j = nodes[jIdx].Index;
                    if (i != j) {
                        x[i][j] = model.addVar(0, 1, 0.0, GRB_BINARY, "x_" + to_string(i) + "_" + to_string(j));
                        mapX["x_" + to_string(i) + "_" + to_string(j)] = true;
                    }
            }
        }

        auto*** y = new GRBVar ** [n];
        for (int iIdx = 0; iIdx < n; iIdx++) {
            auto i = nodes[iIdx].Index;
            y[i] = reinterpret_cast<GRBVar **>(new GRBVar ** [n]);
            for (int jIdx = 0; jIdx < n; jIdx++) {
                auto j = nodes[jIdx].Index;
                y[i][j] = reinterpret_cast<GRBVar *>(new GRBVar * [nwd.size()]);
                if (i != j) {
                    for (int lIdx = 0; lIdx < nwd.size(); lIdx++) {
                        auto l = nwd[lIdx].Index;
                        y[i][j][l] = model.addVar(0, GRB_INFINITY, 0.0, GRB_CONTINUOUS,
                                                      "y_" + to_string(i) + "_" + to_string(j) + "_" +to_string(l));
                        mapY["y_" + to_string(i) + "_" + to_string(j) + "_" +to_string(l)] = true;
                    }
                }
            }
        }

        // Constraint 9
        for (int jIdx = 0; jIdx < nwd.size(); jIdx++) {
            GRBLinExpr expr1;
            auto j = nwd[jIdx].Index;
            for (int iIdx = 0; iIdx < nodes.size(); iIdx++) {
                auto i = nodes[iIdx].Index;
                    if (i != j) {
                        expr1 += x[i][j];
                        if (mapX.find("x_" + to_string(i) + "_" + to_string(j)) == mapX.end()) {
                            cout << "constr 9: not found: " << "x_" + to_string(i) + "_" + to_string(j) << endl;
                        }
                    }
            }
            model.addConstr(expr1, GRB_EQUAL, 1);
            expr1 = 0;
        }

        // Constraint 10
        for (int iIdx = 0; iIdx < nwd.size(); iIdx++) {
            GRBLinExpr expr2;
            auto i = nwd[iIdx].Index;
            for (int jIdx = 0; jIdx < nodes.size(); jIdx++) {
                auto j = nodes[jIdx].Index;
                    if (i != j) {
                        expr2 += x[i][j];
                        if (mapX.find("x_" + to_string(i) + "_" + to_string(j)) == mapX.end()) {
                            cout << "constr 10: not found: " << "x_" + to_string(i) + "_" + to_string(j) << endl;
                        }
                    }
            }
            model.addConstr(expr2, GRB_EQUAL, 1);
            expr2 = 0;
        }

        // Constraint 11
        GRBLinExpr constr11;
        for (int jIdx = 0; jIdx < nwd.size(); jIdx++) {
            auto j = nwd[jIdx].Index;
            constr11 += x[0][j];
            if (mapX.find("x_0_" + to_string(j)) == mapX.end()) {
                            cout << "constr 11: not found: " << "x_0_" + to_string(j) << endl;
            }
        }
        model.addConstr(constr11, GRB_EQUAL, M);
        constr11 = 0;

        // Constraint 12
        GRBLinExpr constr12;
        for (int jIdx = 0; jIdx < nwd.size(); jIdx++) {
            auto j = nwd[jIdx].Index;
            constr12 += x[j][0];
            if (mapX.find("x_" + to_string(j) + "_0") == mapX.end()) {
                            cout << "constr 12: not found: " << "x_" + to_string(j) + "_0" << endl;
            }
        }
        model.addConstr(constr12, GRB_EQUAL, M);
        constr12 = 0;

        // Constraint 13
//        for (int lIdx = 0; lIdx < nwd.size(); lIdx++) {
//            auto l = nwd[lIdx].Index;
//            auto lDemand = nwd[lIdx].Demand;
//            for (int jIdx = 0; jIdx < nodes.size(); jIdx++) {
//                auto j = nodes[jIdx].Index;
//                auto jDemand = nodes[jIdx].Demand;
//                if (j == l) {
//                    GRBLinExpr constr13_sum1;
//                    GRBLinExpr constr13_sum2;
//                    for (int iIdx = 0; iIdx < nodes.size(); iIdx++) {
//                        auto i = nodes[iIdx].Index;
//                        cout <<  "In j = l: " <<i << ", " <<  j << ", " << l << endl;
//                        constr13_sum1 += y[i][j][l];
//                        constr13_sum2 += y[j][i][l];
//                    }
//                    model.addConstr(constr13_sum1 - constr13_sum2, GRB_EQUAL, lDemand);
//                }
//                if (j == 0) {
//                    GRBLinExpr constr13_sum1;
//                    GRBLinExpr constr13_sum2;
//                    for (int iIdx = 0; iIdx < nodes.size(); iIdx++) {
//                        auto i = nodes[iIdx].Index;
//                        cout <<  "In j = 0: " <<i << ", " <<  j << ", " << l << endl;
//                        constr13_sum1 += y[i][j][l];
//                        constr13_sum2 += y[j][i][l];
//                    }
//                    model.addConstr(constr13_sum1 - constr13_sum2, GRB_EQUAL, -lDemand);
//                }
//                if (j != l && jDemand != 0) {
//                    GRBLinExpr constr13_sum1;
//                    GRBLinExpr constr13_sum2;
//                    for (int iIdx = 0; iIdx < nodes.size(); iIdx++) {
//                        auto i = nodes[iIdx].Index;
//                        cout <<  "In j != l and j != 0: " <<i << ", " <<  j << ", " << l << endl;
//                        constr13_sum1 += y[i][j][l];
//                        constr13_sum2 += y[j][i][l];
//                    }
//                    model.addConstr(constr13_sum1 - constr13_sum2, GRB_EQUAL, 0);

//                }
// //            constr13_sum1 = 0; constr13_sum2 = 0;
//            }
//        }

        // Constraint 13 rewrite
        // 13.1: j == l
        for (int lIdx = 0; lIdx < nwd.size(); lIdx++) {
            auto l = nwd[lIdx].Index;
//            cout << "start of l = " << l << endl;
            auto lDemand = nwd[lIdx].Demand;
            // 13.1: j == l
            GRBLinExpr s1;
            auto j = l;
            for (int iIdx = 0; iIdx < nodes.size(); iIdx++) {
                auto i = nodes[iIdx].Index;
                if (i != j) {
                    s1 += y[i][j][l] - y[j][i][l];
                    if (mapY.find("y_" + to_string(i) + "_" + to_string(j) + "_" + to_string(l)) == mapY.end()) {
                            cout << "constr 13.1: not found: " << "y_" + to_string(i) + "_" + to_string(j) + "_" + to_string(l) << endl;
                    }
                    if (mapY.find("y_" + to_string(j) + "_" + to_string(i) + "_" + to_string(l)) == mapY.end()) {
                            cout << "constr 13.1: not found: " << "y_" + to_string(j) + "_" + to_string(i) + "_" + to_string(l) << endl;
                    }
                    
                }
            }
            model.addConstr(s1, GRB_EQUAL, lDemand);
        }

    // 13.2: j == 0
    for (int lIdx = 0; lIdx < nwd.size(); lIdx++) {
        auto l = nwd[lIdx].Index;
        auto lDemand = nwd[lIdx].Demand;
        // 13.2: j == 0
        GRBLinExpr s1;
        auto j = 0;
        for (int iIdx = 0; iIdx < nodes.size(); iIdx++) {
            auto i = nodes[iIdx].Index;
            if (i != j) {
                s1 += y[i][j][l] - y[j][i][l];
                if (mapY.find("y_" + to_string(i) + "_" + to_string(j) + "_" + to_string(l)) == mapY.end()) {
                            cout << "constr 13.2 not found: " << "y_" + to_string(i) + "_" + to_string(j) + "_" + to_string(l) << endl;
                    }
                    if (mapY.find("y_" + to_string(j) + "_" + to_string(i) + "_" + to_string(l)) == mapY.end()) {
                            cout << "constr 13.2: not found: " << "y_" + to_string(j) + "_" + to_string(i) + "_" + to_string(l) << endl;
                    }
            }
        }
        model.addConstr(s1, GRB_EQUAL, -lDemand);
    }

    // 13.3: j != l
    for (int lIdx = 0; lIdx < nwd.size(); lIdx++) {
        auto l = nwd[lIdx].Index;
        auto lDemand = nwd[lIdx].Demand;
        // 13.2: j != l
        for (int jIdx = 0; jIdx < nwd.size(); jIdx++) {
            auto j = nwd[jIdx].Index;
            if (j != l) {
                GRBLinExpr s1;
                for (int iIdx = 0; iIdx < nodes.size(); iIdx++) {
                    auto i = nodes[iIdx].Index;
                    if (i != j) {
                        s1 += y[i][j][l] - y[j][i][l];
                        if (mapY.find("y_" + to_string(i) + "_" + to_string(j) + "_" + to_string(l)) == mapY.end()) {
                            cout << "constr 13.3: not found: " << "y_" + to_string(i) + "_" + to_string(j) + "_" + to_string(l) << endl;
                    }
                    if (mapY.find("y_" + to_string(j) + "_" + to_string(i) + "_" + to_string(l)) == mapY.end()) {
                            cout << "constr 13.3: not found: " << "y_" + to_string(j) + "_" + to_string(i) + "_" + to_string(l) << endl;
                    }
                    }
                }
                model.addConstr(s1, GRB_EQUAL, 0);
            }
        }
    }
    // Constraint 14
        for (int iIdx = 0; iIdx < nodes.size(); iIdx++) {
            for (int jIdx = 0; jIdx < nodes.size(); jIdx++) {
                auto i = nodes[iIdx].Index;
                auto j = nodes[jIdx].Index;
                if (i != j) {
                    for (int lIdx = 0; lIdx < nwd.size(); lIdx++) {
                        auto l = nwd[lIdx].Index;
                        model.addConstr(y[i][j][l], GRB_LESS_EQUAL, nwd[lIdx].Demand * x[i][j]);
                        if (mapY.find("y_" + to_string(i) + "_" + to_string(j) + "_" + to_string(l)) == mapY.end()) {
                            cout << "constr 14: not found: " << "y_" + to_string(i) + "_" + to_string(j) + "_" + to_string(l) << endl;
                        }
                        if (mapX.find("x_" + to_string(i) + "_" + to_string(j)) == mapX.end()) {
                            cout << "constr 14: not found: " << "x_" + to_string(i) + "_" + to_string(j) << endl;
                        }
                    }
                }
            }
        }

        // Constraint 15
        for (int iIdx = 0; iIdx < nwd.size(); iIdx++) {
            GRBLinExpr constr15;
            auto i = nwd[iIdx].Index;
            auto iDemand = nwd[iIdx].Demand;
            for (int jIdx = 0; jIdx < nwd.size(); jIdx++) {
                auto j = nwd[jIdx].Index;
                if (i != j) {
                    for (int lIdx = 0; lIdx < nwd.size(); lIdx++) {
                        auto l = nwd[lIdx].Index;
                            constr15 += y[i][j][l];
                            if (mapY.find("y_" + to_string(i) + "_" + to_string(j) + "_" + to_string(l)) == mapY.end()) {
                            cout << "constr 15: not found: " << "y_" + to_string(i) + "_" + to_string(j) + "_" + to_string(l) << endl;
                        }
                    }
                }        
            }
            model.addConstr(constr15, GRB_LESS_EQUAL, instance.Capacity - iDemand);
        }

        GRBLinExpr objective;
        for (int iIdx = 0; iIdx < nodes.size(); iIdx++) {
            auto i = nodes[iIdx].Index;
            for (int jIdx = 0; jIdx < nodes.size(); jIdx++) {
                auto j = nodes[jIdx].Index;
                if (i != j) {
                     if (mapX.find("x_" + to_string(i) + "_" + to_string(j)) == mapX.end()) {
                            cout << "constr obj: not found: " << "x_" + to_string(i) + "_" + to_string(j) << endl;
                        }
                    objective += c[i][j] * x[i][j];
                }
            }
        }

        model.setObjective(objective, GRB_MINIMIZE);
        model.set(GRB_DoubleParam_TimeLimit, tiLim);
        model.update();
        model.write("model_multi.lp");
        auto startTime = std::chrono::high_resolution_clock::now();
        model.optimize();
        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        bool optimal = false;
        if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
            optimal = true;
        }
        result = make_tuple(model.get(GRB_DoubleAttr_ObjBound), model.get(GRB_DoubleAttr_ObjVal), duration.count(),
                            optimal, model.get(GRB_DoubleAttr_MIPGap));
    } catch (GRBException e) {
        std::cout << "Error code: " << e.getErrorCode() << std::endl;
        std::cout << e.getMessage() << std::endl;
    }

    return result;
}

tuple<int, int, double, bool, double> Solver::MulticommodityFlowSingleDepotAtO(double tiLim) {
    tuple<int, int, double, bool, double> result;
    try {
        GRBEnv env;
    GRBModel model(env);

    auto nodes = instance.Nodes;
    auto c = instance.DistanceMatrix;
    auto n = instance.Dimension;
    auto M = instance.NumberOfVehicles; // number of vehicles.
    auto nss = instance.NodeSubsets; // subsets of nodes; contains at least 2 nodes; do not contain depots.
    auto nwd = instance.NodesWithoutDepots;

    map<string, bool> mapX, mapY;
    auto** x = new GRBVar *[n];
    for (int i = 0; i < n; i++) {
        x[i] = reinterpret_cast<GRBVar *>(new GRBVar * [n]);
        for (int j = 0; j < n; j++) {
            if (i != j) {
                x[i][j] = model.addVar(0, 1, 0.0, GRB_BINARY, "x_" + to_string(i) + "_" + to_string(j));
                mapX["x_" + to_string(i) + "_" + to_string(j)] = true;
            }
        }
    }

    auto ***y = new GRBVar **[n];
    for (int i = 0; i < n;i ++) {
        y[i] = reinterpret_cast<GRBVar **>(new GRBVar ** [n]);
        for (int j = 0; j < n; j++) {
            if (i != j) {
                y[i][j] = reinterpret_cast<GRBVar *>(new GRBVar * [n]);
                for (int l = 1; l < n; l++) {
                    y[i][j][l] = model.addVar(0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "y_" + to_string(i) + "_" + to_string(j) + "_" + to_string(l));
                    mapY["y_" + to_string(i) + "_" + to_string(j) + "_" + to_string(l)] = true;
                }
            }
        }
    }

    // Constraint 9:
    for (int j = 1; j < n; j++) {
        GRBLinExpr constr9;
        for (int i = 0; i < n; i++) {
            if (i != j) {
                auto key = "x_" + to_string(i) + "_" + to_string(j);
                if (mapX.find(key) == mapX.end()) {
                    std::cout << key << " c9 not exist in the map." << std::endl;
                }
                constr9 += x[i][j];
            }
        }
        model.addConstr(constr9, GRB_EQUAL, 1);
    }

    // Constraint 10:
    for (int i = 1; i < n; i++) {
        GRBLinExpr constr10;
        for (int j = 0; j < n; j++) {
            if (i != j) {
                auto key = "x_" + to_string(i) + "_" + to_string(j);
                if (mapX.find(key) == mapX.end()) {
                    std::cout << key << " c10 not exist in the map." << std::endl;
                }
                constr10 += x[i][j];
               
            }
        }
        model.addConstr(constr10, GRB_EQUAL, 1);
    }

    // Constraint 11+12
    GRBLinExpr constr11;
    GRBLinExpr constr12;
    for (int j = 1; j < n; j++) {
        auto key1 = "x_" + to_string(0) + "_" + to_string(j);
        if (mapX.find(key1) == mapX.end()) {
            std::cout << key1 << " c11 not exist in the map." << std::endl;
        }
        auto key2 = "x_" + to_string(j) + "_" + to_string(0);
        if (mapX.find(key2) == mapX.end()) {
            std::cout << key2 << " c12 not exist in the map." << std::endl;
        }
        constr11 += x[0][j];
        constr12 += x[j][0];
    }
    model.addConstr(constr11, GRB_EQUAL, constr12);
    
    // Constraint 13
    for (int l = 1; l < n; l++) {
        auto ql = nodes[l].Demand;
        for (int j = 0; j < n; j++) {
            GRBLinExpr constr13;
            for (int i = 0; i < n; i++) {
                if (i != j) {
                    auto key1 = "y_" + to_string(i) + "_" + to_string(j) + "_" + to_string(l);
                    if (mapY.find(key1) == mapY.end()) {
                        std::cout << key1 << " c131 not exist in the map." << std::endl;
                    }
                    auto key2 = "y_" + to_string(j) + "_" + to_string(i) + "_" + to_string(l);
                    if (mapY.find(key2) == mapY.end()) {
                        std::cout << key2 << " c132 not exist in the map." << std::endl;
                    }
                    constr13 += y[i][j][l] - y[j][i][l];
                }
            }
            if (j == l) {
                model.addConstr(constr13, GRB_EQUAL, ql);
            } else if (j == 0) {
                model.addConstr(constr13, GRB_EQUAL, -ql);
            } else if (j != l) {
                model.addConstr(constr13, GRB_EQUAL, 0);
            }
        }
    }
    
    // Constraint 14
    for (int l = 1; l < n; l++) {
        auto ql = nodes[l].Demand;
        for (int i = 0; i < n; i++) {
            for (int j= 0; j < n; j++) {
                if (i != j) {
                    auto key1 = "y_" + to_string(i) + "_" + to_string(j) + "_" + to_string(l);
                    if (mapY.find(key1) == mapY.end()) {
                        std::cout << key1 << " c141 not exist in the map." << std::endl;
                    }
                    auto key2 = "x_" + to_string(i) + "_" +  to_string(j);
                    if (mapX.find(key2) == mapX.end()) {
                        std::cout << key2 << " c142 not exist in the map." << std::endl;
                    }
                    model.addConstr(y[i][j][l], GRB_LESS_EQUAL, ql * x[i][j]);
                }
            }
        }
    }

    // Constraint 15
    for (int i = 1; i < n; i++) {
        GRBLinExpr constr15;
        auto qi = nodes[i].Demand;
        for (int j = 1; j < n; j++) {
            for (int l = 1; l < n; l++) {
                if (i != j) {
                    auto key1 = "y_" + to_string(i) + "_" + to_string(j) + "_" + to_string(l);
                    if (mapY.find(key1) == mapY.end()) {
                        std::cout << key1 << " c15 not exist in the map." << std::endl;
                    }
                    constr15 += y[i][j][l];
                }
            }
        }
        model.addConstr(constr15, GRB_LESS_EQUAL, instance.Capacity - qi);
    }


    GRBLinExpr objective;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            if (i != j) {
                auto key2 = "x_" + to_string(i) + "_" +  to_string(j);
                if (mapX.find(key2) == mapX.end()) {
                    std::cout << key2 << " cobj not exist in the map." << std::endl;
                }
                objective += c[i][j] * x[i][j];
            }
        }
    }

    model.setObjective(objective, GRB_MINIMIZE);
    model.set(GRB_DoubleParam_TimeLimit, tiLim);
        model.update();
        model.write("model_two_index.lp");
        auto startTime = std::chrono::high_resolution_clock::now();
        model.optimize();
        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        bool optimal = false;
        if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
            optimal = true;
        }
        result = make_tuple(model.get(GRB_DoubleAttr_ObjBound), model.get(GRB_DoubleAttr_ObjVal), duration.count(),
                             optimal, model.get(GRB_DoubleAttr_MIPGap));

    } catch (GRBException e) {
        std::cout << "Error code: " << e.getErrorCode() << std::endl;
        std::cout << e.getMessage() << std::endl;
    }

    return result;

}

tuple<int, int, double, bool, double> Solver::TwoCommodityFlow(double tiLim) {
    tuple<int, int, double, bool, double> result;
//    try {
        GRBEnv env;
        GRBModel model(env);

        auto nodes = instance.Nodes;
        auto Q = instance.Capacity;
//        auto nss = instance.NodeSubsets; // subsets of nodes; contains at least 2 nodes; do not contain depots.
//        auto nwd = instance.NodesWithoutDepots;
        // add to nodes: node n+1

        nodes.push_back(nodes[0]);
        auto n = nodes.size();
        cout << "check node: " << nodes[n-1].X << ", "  << nodes[n-1].Y << ", "  << nodes[n-1].Demand << endl;
        vector<vector<int>> c(n, vector<int>(n));
        for (int i = 0; i < n-1; i++) {
            for (int j = 0; j < n-1; j++) {
                c[i][j] = instance.DistanceMatrix[i][j];
            }
        }

        for (int j = 0; j < n; j++) {
            c[n-1][j] = c[0][j];
        }
        for (int i = 0; i < n; i++) {
            c[i][n-1] = c[i][0];
        }

        map<string, bool> mapX;
        map<string, bool> mapXI;

        auto** xi = new GRBVar *[n];
        for (int i = 0; i < n; i++) {
            xi[i] = reinterpret_cast<GRBVar *>(new GRBVar * [n]);
            for (int j = 0; j < n; j++) {
                if (i != j) {
                    xi[i][j] = model.addVar(0, 1, 0.0, GRB_BINARY, "xi_" + to_string(i) + "_" + to_string(j));
                    mapXI["xi_" + to_string(i) + "_" + to_string(j)] = true;
                }
            }
        }

        // x: flow variables, i, j belong to V~ (0,n); (AS n now = n + node (n+1);
        auto** x = new GRBVar* [n];
        for (int i = 0; i < n; i++) {
            x[i] = reinterpret_cast<GRBVar *>(new GRBVar * [n]);
            for (int j = 0; j < n; j++) {
                if (i != j) {
                    x[i][j] = model.addVar(0,   GRB_INFINITY, 0.0, GRB_CONTINUOUS, "x_" + to_string(i) + "_" + to_string(j));
                    mapX["x_" + to_string(i) + "_" + to_string(j)] = true;
                }
            }
        }


        // in V': < n-1 since the n-1 = depot copy nodes
        // Constraint 28+
        for (int i = 1; i < n-1; i++) {
            GRBLinExpr expr28;
            auto qi = nodes[i].Demand;
            cout << "Node " << i << ", " << qi << ", node idx: " << nodes[i].Index << endl;
            for (int j = 0; j < n; j++) {
                if (i != j) {
                    expr28 += x[j][i] - x[i][j];
                }
            }
            model.addConstr(expr28, GRB_EQUAL, 2 * qi);
        }

        // Constraint 29
        GRBLinExpr expr29;
        int qvAp = 0;
        for (int j = 1; j < n-1; j++) {
            expr29 += x[0][j];
            qvAp += nodes[j].Demand;
        }
        model.addConstr(expr29, GRB_EQUAL, qvAp);

        // Constraint 30 + 31
        GRBVar M = model.addVar(1, instance.NodesWithoutDepots.size(), 0.0, GRB_INTEGER);
//        model.addConstr(M <= n);
        GRBLinExpr expr30;
        GRBLinExpr expr31;
        for (int j = 1; j < n-1; j++) {
            expr30 += x[j][0];
            expr31 += x[n-1][j];
        }
        model.addConstr(expr30, GRB_EQUAL, M * Q - qvAp);
        model.addConstr(expr31, GRB_EQUAL, M * Q);
//        model.addConstr(expr31 - expr30, GRB_EQUAL, qvAp);
        // Constraint 32
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                if (i != j)  {
                    model.addConstr(x[i][j] + x[j][i], GRB_EQUAL, Q * xi[i][j]);
                }
            }
        }

        // Constraint 33
        for (int i = 1; i < n-1; i++) {
            GRBLinExpr sum1;
            GRBLinExpr sum2;
            for (int j = 0; j < n; j++) {
                if (i < j) {
                    sum1 += xi[i][j];
                } else if (i > j) {
                    sum2 += xi[j][i];
                }
            }
            model.addConstr(sum1+ sum2, GRB_EQUAL, 2);
        }

        // Cons add
//        for (int i = 0; i < n; i++) {
//            for (int j = 0; j < n; j++) {
//                if (i != j) {
//                    model.addConstr(xi[i][j] + xi[j][i], GRB_LESS_EQUAL, 1);
//                }
//            }
//        }

        GRBLinExpr objective;
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                if (i < j)  {
                    objective += c[i][j] * xi[i][j];
                }
            }
        }
        model.setObjective(objective, GRB_MINIMIZE);
        model.set(GRB_DoubleParam_TimeLimit, tiLim);
        model.update();
        model.write("model_two_comm.lp");
        auto startTime = std::chrono::high_resolution_clock::now();
        model.optimize();
        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        bool optimal = false;
        if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
            optimal = true;
        }
        result = make_tuple(model.get(GRB_DoubleAttr_ObjBound), model.get(GRB_DoubleAttr_ObjVal), duration.count(),
                            optimal, model.get(GRB_DoubleAttr_MIPGap));

      for (int i = 0; i < n; i++) {
          for (int j = 0; j <  n; j++) {
              if (i < j) {
                  if (xi[i][j].get(GRB_DoubleAttr_X) == 1) {
                      cout << i << " " << j << endl;
                  }
              }

          }
      }
//    } catch (GRBException e) {
//        std::cout << "Error code: " << e.getErrorCode() << std::endl;
//        std::cout << e.getMessage() << std::endl;
//    }
    return result;
}
