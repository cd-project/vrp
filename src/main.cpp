//
// Created by cuong on 12/08/2023.
//

#include "../include/instance.h"
#include "../include/solver.h"
#include "../include/batch_solver.h"

int main() {
    auto instance = Instance();
    auto batchSolver = BatchSolver();
    string folderPath = "/home/cuong/CLionProjects/VRP/test_1_only";
    auto fPSplit = instance.SplitStringWithDelimiter(folderPath, "/");
    string outputPath = fPSplit[fPSplit.size()-1] + ".csv";
    int formula = 2;
    double timeLimit = 3600.0;
    batchSolver.GurobiBatch(folderPath, outputPath, formula, 1000, timeLimit);
//    auto solver = Solver(instance);
//    double tiLim = 3600.0;
//    solver.TwoCommodityFlow(tiLim);
}