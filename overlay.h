#ifndef OVERLAY_H
#define OVERLAY_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "stereocalibration.h"
#include "abstract_overlay.h"


using namespace std;

class Overlay : public AbstractOverlay
{
public:
    Overlay(int res_x, int res_y, double scale_mn, double scale_mx, int ansi);
    Overlay(int res_x, int res_y, double scale_mn, double scale_mx, StereoCalibration * calib);
    ~Overlay();

    void drawCircle(int x, int y, int radius, double val);
    cv::Mat getOverlay();
    cv::Mat getAccumulator();
    cv::Mat getColorBar();
    double getLowerBound();
    double getUpperBound();
    void setNewInterval(double mn, double mx);
    void setAnsi(int ansi);
    void drawColorBar();
    void drawCurrentVal(double val, int channel);
    cv::Mat values;
    cv::Mat mergeOverlay(cv::Mat frame);
    double getValue(int x, int y);
    bool stereo_mode;

private:
    void init(int size_x, int size_y, double scale_mn, double scale_mx);
    cv::Mat accumulator;
    cv::Mat color_bar;
    cv::Mat readColorMap(string dataPath);
    //Mat colormap;
    vector<float> *r;
    vector<float> *g;
    vector<float> *b;
    double scale_min = 1;
    double scale_max = 6;
    cv::Mat RGBimage;
    char str_mx1[4];
    char str_mn1[4];
    bool autoscale;
    int ansi = 99999;

    double abs_max = 10;
    double abs_min = 0;

    StereoCalibration * calib;
};
#endif // OVERLAY_H
