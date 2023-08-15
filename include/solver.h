//
// Created by cuong on 12/08/2023.
//

#ifndef VRP_SOLVER_H
#define VRP_SOLVER_H

#include <tuple>
#include <chrono>

#include "instance.h"
#include "gurobi_c++.h"

class Solver {
public:
    Instance instance;

    explicit Solver(Instance ins): instance{std::move(ins)} {}
    tuple<int, int, double, bool, double> TwoIndexFlow(double tiLim);
    tuple<int, int, double, bool, double> MulticommodityFlow(double tiLim);
};
#endif //VRP_SOLVER_H
