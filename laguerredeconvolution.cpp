#include "laguerredeconvolution.h"
#include "deconvolveprocess.h"
//#include <QMessageBox>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <QDebug>

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
    //int LaguerreOrder = 12; // Laguerre Order initialization
    int DataLength = (int) iIRF_CH1.size(); // Length of data

    // CH1
    //deconMats deconMatrices_CH1; // All matrices required for the deconvolution

    alpha = deconProc.Laguerre_alphaval(DataLength, LaguerreOrder); // Alpha value and laguerre order from lookup table

    if (alpha == 0)
    {
        //QMessageBox messageBox;
        //messageBox.critical(0, "Error", "Laguerre lookup table not existent or requested parameters outside the table limits");
        //messageBox.setFixedSize(500, 200);
        //exit(-1);
    }
    deconProc.preMatChannels(iIRF_CH1, deconMatrices_CH1, DataLength, LaguerreOrder, alpha);
    // CH2
    //deconMats deconMatrices_CH2; // All matrices required for the deconvolution
    deconProc.preMatChannels(iIRF_CH2, deconMatrices_CH2, DataLength, LaguerreOrder, alpha);

    // CH3
    //deconMats deconMatrices_CH3; // All matrices required for the deconvolution
    deconProc.preMatChannels(iIRF_CH3, deconMatrices_CH3, DataLength, LaguerreOrder, alpha);

    // CH4
    //deconMats deconMatrices_CH4; // All matrices required for the deconvolution
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

double LaguerreDeconvolution::getLifetime(vector<double> fIRF, int channel)
{

    //qDebug() << "deconv 1";
    double lifetime;
    if (isnan(fIRF.at(0)))
    {
        lifetime = fIRF.at(0);

    }
    else if (isinf(fIRF.at(0)))
    {
        lifetime = fIRF.at(0);
    }
    else
    {
        Mat fIRF1(fIRF);

        switch(channel)
        {
        case 1:
            lifetime = deconProc.lifetCalc(deconMatrices_CH1, fIRF1, LaguerreOrder, Ccols, resTime);
            break;
        case 2:
            lifetime = deconProc.lifetCalc(deconMatrices_CH2, fIRF1, LaguerreOrder, Ccols, resTime);
            break;
        case 3:
            lifetime = deconProc.lifetCalc(deconMatrices_CH3, fIRF1, LaguerreOrder, Ccols, resTime);
            break;
        case 4:
            lifetime = deconProc.lifetCalc(deconMatrices_CH4, fIRF1, LaguerreOrder, Ccols, resTime);
            break;
        }
    }
    //qDebug() << "deconv 2";
    //qDebug() << lifetime;
    return lifetime;
}

