//
// Created by cuong on 12/08/2023.
//


#include "../include/batch_solver.h"
#include "../include/instance.h"
#include "../include/solver.h"

BatchSolver::BatchSolver() = default;

void BatchSolver::GurobiBatch(string folderPath, string outputPath, int formula, int dimensionLim, double tiLim) {
    vector<string> fileList;
    for (const auto& entry : filesystem::directory_iterator(folderPath)) {
        if (entry.is_regular_file()) {
            fileList.push_back(entry.path().filename().string());
        }
    }
    string opF;
    switch (formula) {
        case 1:
            opF = "grb_mtz_" + outputPath;
            break;
        case 2:
            opF = "grb_two_cmm_" + outputPath;
            break;
        case 3:
            opF = "grb_multi_cmm_" + outputPath;
            break;
        case 4:
            opF = "grb_single_cmm_" + outputPath;
    }


    vector<InstanceResult> data;

        for (int i = 0; i < fileList.size(); i++) {
            ofstream file(opF, ios::out | ios::app);
            auto fName = folderPath + "/" + fileList[i];
            cout << "Current instance name: " << fName << endl;
            auto inst = Instance(fName);
            inst.PrintInstanceInfo();
            if (inst.Dimension > dimensionLim) {
                continue;
            }
            auto solver = Solver(inst);
            tuple<int, int, double, bool, double> res;
            InstanceResult iR;
            switch (formula) {
                case 1:
                    cout << "Using MTZ--------------------------------------------" << endl;
                   res = solver.MTZ(tiLim);
                   iR = (InstanceResult{fileList[i], get<0>(res), get<1>(res), get<2>(res), get<3>(res), get<4>(res)});
                   break;
                case 2:
                    cout << "Using 2-comm flow-----------------------------------------------" << endl;
                    res = solver.TwoCommodityFlow(tiLim);
                    iR = (InstanceResult{fileList[i], get<0>(res), get<1>(res), get<2>(res), get<3>(res), get<4>(res)});
                    break;
                case 3:
                    cout << "Using multi-comm flow-----------------------------------------------" << endl;
                    res = solver.MulticommodityFlowSingleDepotAtO(tiLim);
                    iR = (InstanceResult{fileList[i], get<0>(res), get<1>(res), get<2>(res), get<3>(res), get<4>(res)});
                    break;
                case 4:
                    cout << "Using Single-comm flow--------------------------------------------" << endl;
                    res = solver.SingleCommodity(tiLim);
                    iR = (InstanceResult{fileList[i], get<0>(res), get<1>(res), get<2>(res), get<3>(res), get<4>(res)});
                    break;
            }
            file << iR.InstanceName << ","
                 << iR.LB << ","
                 << iR.UB << ","
                 << iR.RunningTime/1000 << ","
                 << iR.IsOptimal << ","
                 << iR.Gap << "\n";
            file.close();
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        std::cout << "CSV file created successfully." << std::endl;
}
