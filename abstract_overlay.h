#ifndef ABSTRACT_OVERLAY_H
#define ABSTRACT_OVERLAY_H

class AbstractOverlay {
public:
    AbstractOverlay(void) {};
    ~AbstractOverlay(void) {};

    virtual void drawCircle(int x, int y, int radius, double val) = 0;
    virtual cv::Mat getOverlay() = 0;
    virtual cv::Mat getAccumulator() = 0;
    virtual cv::Mat getColorBar() = 0;
    virtual double getLowerBound() = 0;
    virtual double getUpperBound() = 0;
    virtual void setNewInterval(double mn, double mx) = 0;
    virtual void setAnsi(int ansi) = 0;
    virtual void drawColorBar() = 0;
    virtual void drawCurrentVal(double val, int channel) = 0;
    virtual cv::Mat mergeOverlay(cv::Mat frame) = 0;
    virtual double getValue(int x, int y) = 0;
};

#endif // ABSTRACT_OVERLAY_H
