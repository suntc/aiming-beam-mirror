#ifndef DECONVOLVEPROCESS_H
#define DECONVOLVEPROCESS_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#define nnls_max(a,b) ((a) >= (b) ? (a) : (b))
#define nnls_abs(x) ((x) >= 0 ? (x) : -(x))


struct deconMats
{
    cv::Mat L;
    cv::Mat H;
    cv::Mat V;
    cv::Mat Lambda;
    cv::Mat D;
    std::vector<double> B;
};

class DeconvolveProcess
{
public:
    DeconvolveProcess();
    // find alpha value for the corresponding Laguerre order
    double Laguerre_alphaval(int, int &);
    // generate Laguerre base functions iteratively
    cv::Mat Laguerre(int, int, double);
    // convolve base functions with iIRF, used in following steps
    cv::Mat LaguerreFilt(std::vector<double>, cv::Mat &);
    // derive 3rd dirivitive matrixes used for calculation.
    cv::Mat deriv3rd(int, cv::Mat &);
    // cholesky factorisation
    cv::Mat CholFact(cv::Mat &);
    // calculate matrixes for each channel
    void preMatChannels(std::vector<double>, deconMats &, int, int, double);
    //calculate lifetimes
    double lifetCalc(deconMats, cv::Mat, int, int, double);
    // solve non-negative non-linear least square problem
    static cv::Mat solveNNLS(cv::Mat, cv::Mat, int, int, double*);
    // solve non-negative non-linear least square problem
    static int nnls(double*,  int,  int,  int, double*, double*, double*, double*, double*, int*, int*);
    // solve non-negative non-linear least square problem
    static int g1(double*, double*, double*, double*, double*);
    // solve non-negative non-linear least square problem
    static int h12(int, int*, int*, int, double*, int*, double*, double*, int*, int*, int*);
};

#endif // DECONVOLVEPROCESS_H
