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

    double Laguerre_alphaval(int, int &);
    cv::Mat Laguerre(int, int, double);
    cv::Mat LaguerreFilt(std::vector<double>, cv::Mat &);
    cv::Mat deriv3rd(int, cv::Mat &);
    cv::Mat CholFact(cv::Mat &);
    void preMatChannels(std::vector<double>, deconMats &, int, int, double);
    double lifetCalc(deconMats, cv::Mat, int, int, double);
    static cv::Mat solveNNLS(cv::Mat, cv::Mat, int, int, double*);
    static int nnls(double*,  int,  int,  int, double*, double*, double*, double*, double*, int*, int*);
    static int g1(double*, double*, double*, double*, double*);
    static int h12(int, int*, int*, int, double*, int*, double*, double*, int*, int*, int*);
};

#endif // DECONVOLVEPROCESS_H
