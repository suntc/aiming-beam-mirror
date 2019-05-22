#ifndef LAGUERREDECONVOLUTION_H
#define LAGUERREDECONVOLUTION_H
#include "deconvolveprocess.h"
//#include "acousticfeedback.h"

using namespace std;

class LaguerreDeconvolution
{
public:
    // constructor
    LaguerreDeconvolution(vector<double> iIRF_CH1, vector<double> iIRF_CH2, vector<double> iIRF_CH3, vector<double> iIRF_CH4, double resTime);
    // deconvolusion function
    double getLifetime(vector<double> fIRF, int channel);
    //DeconvolveProcess precomputeMatrices(vector<double> iIRF_CH1, vector<double> iIRF_CH2, vector<double> iIRF_CH3, vector<double> iIRF_CH4);
    // properties deconMats class
    deconMats deconMatrices_CH1;
    deconMats deconMatrices_CH2;
    deconMats deconMatrices_CH3;
    deconMats deconMatrices_CH4;
    
    cv::Mat fIRF2_copy;
    // function for acoustic indicator
    double getAcousticIndicator();
    // private properties
    private:
    DeconvolveProcess deconProc; // Object for the deconvolution
    int Ccols;
    int LaguerreOrder;
    double resTime;
};

#endif // LAGUERREDECONVOLUTION_H
