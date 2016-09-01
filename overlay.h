#ifndef OVERLAY_H
#define OVERLAY_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


using namespace std;
//using namespace cv;

class Overlay
{
public:
    Overlay(int res_x, int res_y, double scale_mn, double scale_mx);
    void drawCircle(int x, int y, int radius, double val);
    cv::Mat getOverlay();
    cv::Mat getAccumulator();
    cv::Mat getColorBar();
    double getLowerBound();
    double getUpperBound();
    void setNewInterval(double mn, double mx);
    void Overlay::drawColorBar();
    cv::Mat values;
private:

    cv::Mat accumulator;
    cv::Mat color_bar;
    cv::Mat readColorMap(string dataPath);
    //Mat colormap;
    vector<float> *r;
    vector<float> *g;
    vector<float> *b;
    double scale_min;
    double scale_max;
    cv::Mat RGBimage;
    char str_mx1[4];
    char str_mn1[4];
};

#endif // OVERLAY_H
