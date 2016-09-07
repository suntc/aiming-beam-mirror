#include "laguerredeconvolution.h"
#include "deconvolveprocess.h"
#include <QMessageBox>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

LaguerreDeconvolution::LaguerreDeconvolution(vector<double> iIRF_CH1, vector<double> iIRF_CH2, vector<double> iIRF_CH3, vector<double> iIRF_CH4, double resTime1)
{
    resTime = resTime1;

    // --------------------------------------------------------------------------------------
    // --------------------------------------------------------------------------------------
    // Precompute matrices
    // --------------------------------------------------------------------------------------
    // --------------------------------------------------------------------------------------
    // Params
    double alpha = 0.0; // Alpha value initialization
    //int LaguerreOrder = 0; // Laguerre Order initialization
    int DataLength = (int) iIRF_CH1.size(); // Length of data

    // CH1
    deconMats deconMatrices_CH1; // All matrices required for the deconvolution

    alpha = deconProc.Laguerre_alphaval(DataLength, LaguerreOrder); // Alpha value and laguerre order from lookup table

    if (alpha == 0)
    {
        QMessageBox messageBox;
        messageBox.critical(0, "Error", "Laguerre lookup table not existent or requested parameters outside the table limits");
        messageBox.setFixedSize(500, 200);
        exit(-1);
    }

    deconProc.preMatChannels(iIRF_CH1, deconMatrices_CH1, DataLength, LaguerreOrder, alpha);

    // CH2
    deconMats deconMatrices_CH2; // All matrices required for the deconvolution
    deconProc.preMatChannels(iIRF_CH2, deconMatrices_CH2, DataLength, LaguerreOrder, alpha);

    // CH3
    deconMats deconMatrices_CH3; // All matrices required for the deconvolution
    deconProc.preMatChannels(iIRF_CH3, deconMatrices_CH3, DataLength, LaguerreOrder, alpha);

    // CH4
    deconMats deconMatrices_CH4; // All matrices required for the deconvolution
    deconProc.preMatChannels(iIRF_CH4, deconMatrices_CH4, DataLength, LaguerreOrder, alpha);

    Ccols = deconMatrices_CH1.D.rows;
}

//DeconvolveProcess LaguerreDeconvolution::precomputeMatrices(vector<double> iIRF_CH1, vector<double> iIRF_CH2, vector<double> iIRF_CH3, vector<double> iIRF_CH4)
//{
//}

// --------------------------------------------------------------------------
// --------------------------------------------------------------------------
// Laguerre Deconvolution
// --------------------------------------------------------------------------
// --------------------------------------------------------------------------
// Conditions for CH1
void LaguerreDeconvolution::getLifetimes(vector<double> fIRF_CH1, vector<double> fIRF_CH2, vector<double> fIRF_CH3, vector<double> fIRF_CH4, double idx)
{
    vector<double> channels_L1(5); // LT values @ 1 iteration ago
    vector<double> channels_L2(5); // LT values @ 2 iterations ago
    vector<double> channels_L3(5); // LT values @ current iteration

    if (isnan(fIRF_CH1.at(0)))
    {
        channels_L1.at(0) = fIRF_CH1.at(0);
    }
    else if (isinf(fIRF_CH1.at(0)))
    {
        channels_L1.at(0) = fIRF_CH1.at(0);
    }
    else
    {
        Mat fIRF1(fIRF_CH1);
        double lifet1 = deconProc.lifetCalc(deconMatrices_CH1, fIRF1, LaguerreOrder, Ccols, resTime);
        channels_L1.at(0) = lifet1;
    }

    // Conditions for CH2
    if (isnan(fIRF_CH2.at(0)))
    {
        channels_L1.at(1) = fIRF_CH2.at(0);
    }
    else if (isinf(fIRF_CH2.at(0)))
    {
        channels_L1.at(1) = fIRF_CH2.at(0);
    }
    else
    {
        Mat fIRF2(fIRF_CH2);
        double lifet2 = deconProc.lifetCalc(deconMatrices_CH2, fIRF2, LaguerreOrder, Ccols, resTime);
        channels_L1.at(1) = lifet2;
    }

    // Conditions for CH3
    if (isnan(fIRF_CH3.at(0)))
    {
        channels_L1.at(2) = fIRF_CH3.at(0);
    }
    else if (isinf(fIRF_CH3.at(0)))
    {
        channels_L1.at(2) = fIRF_CH3.at(0);
    }
    else
    {
        Mat fIRF3(fIRF_CH3);
        double lifet3 = deconProc.lifetCalc(deconMatrices_CH3, fIRF3, LaguerreOrder, Ccols, resTime);
        channels_L1.at(2) = lifet3;
    }

    // Conditions for CH4
    if (isnan(fIRF_CH4.at(0)))
    {
        channels_L1.at(3) = fIRF_CH4.at(0);
    }
    else if (isinf(fIRF_CH4.at(0)))
    {
        channels_L1.at(3) = fIRF_CH4.at(0);
    }
    else
    {
        Mat fIRF4(fIRF_CH4);
        double lifet4 = deconProc.lifetCalc(deconMatrices_CH4, fIRF4, LaguerreOrder, Ccols, resTime);
        channels_L1.at(3) = lifet4;
    }

    channels_L1.at(4) = idx;
}
