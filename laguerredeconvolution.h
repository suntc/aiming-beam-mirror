#ifndef LAGUERREDECONVOLUTION_H
#define LAGUERREDECONVOLUTION_H
#include "deconvolveprocess.h"

using namespace std;

class LaguerreDeconvolution
{
public:
    LaguerreDeconvolution(vector<double> iIRF_CH1, vector<double> iIRF_CH2, vector<double> iIRF_CH3, vector<double> iIRF_CH4, double resTime);
    double getLifetime(vector<double> fIRF, int channel);
    //DeconvolveProcess precomputeMatrices(vector<double> iIRF_CH1, vector<double> iIRF_CH2, vector<double> iIRF_CH3, vector<double> iIRF_CH4);
private:
    deconMats deconMatrices_CH1;
    deconMats deconMatrices_CH2;
    deconMats deconMatrices_CH3;
    deconMats deconMatrices_CH4;
    DeconvolveProcess deconProc; // Object for the deconvolution
    int Ccols;
    int LaguerreOrder;
    double resTime;
};

#endif // LAGUERREDECONVOLUTION_H
