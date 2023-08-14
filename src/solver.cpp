//
// Created by cuong on 12/08/2023.
//
#include "../include/solver.h"


tuple<int, int, double, bool, double> Solver::TwoIndexFlow(double tiLim) {
    GRBEnv env;
    GRBModel model(env);

    auto c = instance.DistanceMatrix;
    auto n = instance.Dimension;
    auto M = instance.NumberOfVehicles; // number of vehicles.
    auto nss = instance.NodeSubsets; // subsets of nodes; contains at least 2 nodes; do not contain depots.
    auto nwd = instance.NodesWithoutDepots;

    // Variable x (phi) and constraints 5 and 6.
    auto** x = new GRBVar * [n];
    for (int i = 0; i < n; i++) {
        x[i] = reinterpret_cast<GRBVar *>(new GRBVar *[n]);
        for (int j = 0; j < n; j++) {
            if (instance.Nodes[i].Demand != 0) {
                x[i][j] = model.addVar(0, 1, 0.0, GRB_BINARY);
            } else {
                if (instance.Nodes[j].Demand != 0) {
                    x[i][j] = model.addVar(0, 2, 0.0, GRB_INTEGER);
                }
            }
        }
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
                sumExpr2 += x[i][j];
            }
        }
        model.addConstr(sumExpr1 + sumExpr2, GRB_EQUAL, 2);
    }

    // Constraint 3
    for (int subIndex = 0; subIndex < nss.size(); subIndex++) {
        GRBLinExpr sumExpr1;
        GRBLinExpr sumExpr2;
        for (int idx = 0; idx < nss[subIndex].Nodes.size(); idx++) {
            auto i = nss[subIndex].Nodes[idx].Index;
            for (int csIdx = 0; csIdx < nss[subIndex].ComplementSet.size(); csIdx++) {
                auto j = nss[subIndex].ComplementSet[csIdx].Index;
                if (i < j) {
                    sumExpr1 += x[i][j];
                }
            }
        }

        for (int csIdx = 0; csIdx < nss[subIndex].ComplementSet.size(); csIdx++) {
            auto i = nss[subIndex].ComplementSet[csIdx].Index;
            for (int idx = 0; idx < nss[subIndex].Nodes.size(); idx++) {
                auto j = nss[subIndex].Nodes[idx].Index;
                if (i < j) {
                    sumExpr2 += x[i][j];
                }
            }
        }

        model.addConstr(sumExpr1 + sumExpr2, GRB_GREATER_EQUAL, 2 * nss[subIndex].R);
    }

    // Constraint 4
    GRBLinExpr constr4;
    for (int idx = 0; idx < nwd.size(); idx++) {
        constr4 += x[0][nwd[idx].Index];
    }
    model.addConstr(constr4, GRB_EQUAL, 2 * M);

    GRBLinExpr objective;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            objective += c[i][j] * x[i][j];
        }
    }

    model.setObjective(objective, GRB_MINIMIZE);
    model.set(GRB_DoubleParam_TimeLimit, tiLim);
    model.update();
    model.optimize();

}
