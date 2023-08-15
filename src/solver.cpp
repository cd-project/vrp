//
// Created by cuong on 12/08/2023.
//

#include "../include/solver.h"


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

        // Variable x (phi) and constraints 5 and 6.
        auto **x = new GRBVar *[n];
//        for (int i = 0; i < n; i++) {
//            x[i] = reinterpret_cast<GRBVar *>(new GRBVar *[n]);
//            for (int j = 0; j < n; j++) {
//                // i, j: node's index in array; not node's true index.
//                if (nodes[i].Demand == 0 && nodes[j].Demand != 0) {
//                    auto iIdx = nodes[i].Index;
//                    auto jIdx = nodes[j].Index;
//                    if (iIdx != jIdx) {
//                        x[iIdx][jIdx] = model.addVar(0, 2, 0.0, GRB_INTEGER);
//                        cout << "added " << "x_" + to_string(iIdx) + "_" + to_string(jIdx) << " as = (0, 1, 2)" << endl;
//                    }
//
//                }
//                if (nodes[i].Demand != 0 && nodes[j].Demand != 0) {
//                    auto iIdx = nodes[i].Index;
//                    auto jIdx = nodes[j].Index;
//                    if (iIdx != jIdx) {
//                        x[iIdx][jIdx] = model.addVar(0, 1, 0.0, GRB_INTEGER);
//                        cout << "added " << "x_" + to_string(iIdx) + "_" + to_string(jIdx) << " as = (0, 1)"<< endl;
//                    }
//                }
//            }
//        }
    for (int i = 0; i < n; i++) {
        x[i] = reinterpret_cast<GRBVar *>(new GRBVar *[n]);
        for (int j = 0; j < n; j++) {
            if (i != j) {
                x[i][j] = model.addVar(0,2, 0.0, GRB_INTEGER);
            }
        }
    }
    for (int nodeIndex = 0; nodeIndex < nodes.size(); nodeIndex++) {
        for (int nwdIndex = 0; nwdIndex < nwd.size(); nwdIndex++) {
            auto i = nodes[nodeIndex].Index;
            auto j = nwd[nwdIndex].Index;
            if (i != j) {
                if (nodes[nodeIndex].Demand == 0) {
                    model.addConstr(0, GRB_LESS_EQUAL, x[i][j]);
                    model.addConstr(2, GRB_GREATER_EQUAL, x[i][j]);

                } else {
                    model.addConstr(0, GRB_LESS_EQUAL, x[i][j]);
                    model.addConstr(1, GRB_GREATER_EQUAL, x[i][j]);
                }
            }

        }
    }

    cout << "here222?" << endl;
    for (int subIndex = 0; subIndex < nss.size(); subIndex++) {
        auto subsetNodes = nss[subIndex];
        GRBLinExpr sum;
        for (int iIdx = 0; iIdx < subsetNodes.Nodes.size(); iIdx++) {
            auto i = subsetNodes.Nodes[iIdx].Index;
            for (int jIdx = 0; jIdx < subsetNodes.Nodes.size(); jIdx++) {
                auto j = subsetNodes.Nodes[jIdx].Index;
                if (i < j) {
                    sum += x[i][j];
                }
            }
        }
        model.addConstr(sum, GRB_LESS_EQUAL, subsetNodes.Nodes.size() - subsetNodes.R);
    }

    // Constraint 2
    for (int idx = 0; idx < nwd.size(); idx++) {
        // for all i in V': node without depots
        int i = nwd[idx].Index;
        GRBLinExpr sumExpr1;
        GRBLinExpr sumExpr2;
        for (int j = 0; j < n; j++) {
            if (i < j) {
                sumExpr1 += x[i][j];
            }
            if (i > j) {
                sumExpr2 += x[j][i];
            }
        }
        model.addConstr(sumExpr1 + sumExpr2, GRB_EQUAL, 2);
        sumExpr1 = 0;
        sumExpr2 = 0;
    }

    cout << "here333?" << endl;

//    // Constraint 3
//    for (int subIndex = 0; subIndex < nss.size(); subIndex++) {
//        GRBLinExpr sumExpr1;
//        GRBLinExpr sumExpr2;
//        for (int idx = 0; idx < nss[subIndex].Nodes.size(); idx++) {
//            auto i = nss[subIndex].Nodes[idx].Index;
//            for (int csIdx = 0; csIdx < nss[subIndex].ComplementSet.size(); csIdx++) {
//                auto j = nss[subIndex].ComplementSet[csIdx].Index;
//                if (i < j) {
//                    sumExpr1 += x[i][j];
//                }
//            }
//        }
//
//        for (int csIdx = 0; csIdx < nss[subIndex].ComplementSet.size(); csIdx++) {
//            auto i = nss[subIndex].ComplementSet[csIdx].Index;
//            for (int idx = 0; idx < nss[subIndex].Nodes.size(); idx++) {
//                auto j = nss[subIndex].Nodes[idx].Index;
//                if (i < j) {
//                    sumExpr2 += x[i][j];
//                }
//            }
//        }
//
//        model.addConstr(sumExpr1 + sumExpr2, GRB_GREATER_EQUAL, 2 * nss[subIndex].R);
//        sumExpr1 = 0;
//        sumExpr2 = 0;
//    }

        cout << "444?" << endl;

//      Constraint 4
        GRBLinExpr constr4;
        for (int idx = 0; idx < nwd.size(); idx++) {
            constr4 += x[0][nwd[idx].Index];
        }
        model.addConstr(constr4, GRB_EQUAL, 2 * M);
        constr4 = 0;

        GRBLinExpr objective;
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                if (i > j) {
                    objective += c[i][j] * x[i][j];
                    cout << "z += c" << "["  << i << "][" << j << "] * x[" << i << "][" << j << "]" << endl;
                }
            }
        }

        model.addConstr(objective, GRB_EQUAL, 0);
        model.update();

        model.setObjective(objective, GRB_MINIMIZE);
        model.set(GRB_DoubleParam_TimeLimit, tiLim);
        model.update();
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

//tuple<int, int, double, bool, double> Solver::MulticommodityFlow(double tiLim) {
//    tuple<int, int, double, bool, double> result;
//    GRBEnv env;
//    GRBModel model(env);
//    auto nodes = instance.Nodes;
//    auto c = instance.DistanceMatrix;
//    auto n = instance.Dimension;
//    auto M = instance.NumberOfVehicles; // number of vehicles.
//    auto nss = instance.NodeSubsets; // subsets of nodes; contains at least 2 nodes; do not contain depots.
//    auto nwd = instance.NodesWithoutDepots;
//
//    auto** x = new GRBVar * [n];
//    for (int i = 0; i < n; i++) {
//        x[i] = reinterpret_cast<GRBVar *>(new GRBVar *[n]);
//        for (int j = 0; j < n; j++) {
//            x[i][j] = model.addVar(0, 1, 0.0, GRB_BINARY);
//        }
//    }
//
//    auto*** y = new GRBVar ** [n];
//    for (int i = 0; i < n; i++) {
//        y[i] = reinterpret_cast<GRBVar **>(new GRBVar ** [nwd.size()]);
//        for (int j = 0; j < nwd.size(); j++) {
//            y[i][j] = reinterpret_cast<GRBVar *>(new GRBVar * [nwd.size()]);
//            for (int l = 0; l < nwd.size(); l++) {
//
//            }
//        }
//    }
//    // Constraint 9
//    for (int jIdx = 0; jIdx < nwd.size(); jIdx++) {
//        GRBLinExpr expr;
//        auto j = nwd[jIdx].Index;
//        for (int iIdx = 0; iIdx < n; iIdx++) {
//            auto i = nodes[iIdx].Index;
//            expr += x[i][j];
//        }
//        model.addConstr(expr, GRB_EQUAL, 1);
//        expr = 0;
//    }
//
//    // Constraint 10
//    for (int iIdx = 0; iIdx < nwd.size(); iIdx++) {
//        GRBLinExpr expr;
//        auto i = nwd[iIdx].Index;
//        for (int jIdx = 0; jIdx < n; jIdx++) {
//            auto j = nodes[jIdx].Index;
//            expr += x[i][j];
//        }
//        model.addConstr(expr, GRB_EQUAL, 1);
//        expr = 0;
//    }
//
//    // Constraint 11
//    GRBLinExpr constr11;
//    for (int jIdx = 0; jIdx < nwd.size(); jIdx++) {
//        auto j = nwd[jIdx].Index;
//        constr11 += x[0][j];
//    }
//    model.addConstr(constr11, GRB_EQUAL, M);
//    constr11 = 0;
//
//    // Constraint 12
//    GRBLinExpr constr12;
//    for (int jIdx = 0; jIdx < nwd.size(); jIdx++) {
//        auto j = nwd[jIdx].Index;
//        constr11 += x[j][0];
//    }
//    model.addConstr(constr11, GRB_EQUAL, M);
//
//    // Constraint 13
//    for (int lIdx = 0; lIdx < nwd.size(); lIdx++) {
//        auto l = nwd[lIdx].Index;
//
//    }
//}
