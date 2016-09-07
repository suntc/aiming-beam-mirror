#ifndef LAGUERREDECONVOLUTION_H
#define LAGUERREDECONVOLUTION_H
#include "deconvolveprocess.h"

using namespace std;

class LaguerreDeconvolution
{
public:
    LaguerreDeconvolution(vector<double> iIRF_CH1, vector<double> iIRF_CH2, vector<double> iIRF_CH3, vector<double> iIRF_CH4, double resTime);
    void getLifetimes(vector<double> fIRF_CH1, vector<double> fIRF_CH2, vector<double> fIRF_CH3, vector<double> fIRF_CH4, double idx);
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
