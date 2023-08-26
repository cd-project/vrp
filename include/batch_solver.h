//
// Created by cuong on 12/08/2023.
//

#ifndef VRP_BATCH_SOLVER_H
#define VRP_BATCH_SOLVER_H

#include <fstream>
#include <string>
#include <vector>
#include <filesystem>
#include <iostream>

using namespace std;

struct InstanceResult {
    string InstanceName;
    int LB;
    int UB;
    double RunningTime;
    bool IsOptimal;
    double Gap;
};

class BatchSolver{
public:
    BatchSolver();
    void GurobiBatch(string folderPath, string outputPath, int formula, int dimensionLim, double tiLim);
};

#endif //VRP_BATCH_SOLVER_H
