//
// Created by cuong on 12/08/2023.
//

#include "../include/instance.h"
#include "../include/solver.h"

int main() {
    auto instance = Instance("/home/cuong/CLionProjects/VRP/vrplib/eil22.vrp");
    instance.PrintInstanceInfo();
    auto solver = Solver(instance);
    double tiLim = 3600.0;
    solver.TwoIndexFlow(tiLim);
}