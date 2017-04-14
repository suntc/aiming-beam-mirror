#define ARRAY_SIZE(array) (sizeof((array))/sizeof((array[0])))

#include "segmentation.h"
#include <QDebug>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <sstream>
#include <algorithm>
#include <iostream>
#include <QMessageBox>
#include "stereocalibration.h"


using namespace cv;
using namespace std;

Segmentation::Segmentation(Mat frame, cv::Point point1, cv::Point point2, bool interp, int ch_number, bool interp_succ, StereoCalibration* calib, bool autoscale, bool pentero_mode)
{
    // Initially setup frame size
    Size s = frame.size();
    res_x = s.width;
    res_y = s.height;

    this->scale_auto=autoscale;
    stereo_setup = true;
    this->calib = calib;
    this->pentero_mode = pentero_mode;

    // Setup segmentation (shared with 3D measurement constructor)
    init(frame, point1, point2, interp, ch_number, interp_succ);
}

Segmentation::Segmentation(Mat frame, cv::Point point1, cv::Point point2, bool interp, int ch_number, bool interp_succ, bool autoscale, int ansi, bool pentero_mode)
{
    // Initially setup frame size
    Size s = frame.size();
    res_x = s.width;
    res_y = s.height;

    this->pentero_mode = pentero_mode;
    this->scale_auto=autoscale;
    double lower_bound_;
    double upper_bound_;
    if (autoscale==true)
    {
        lower_bound_ = 2;
        upper_bound_ = 3;
    }
    else
    {
        lower_bound_ = 1;
        upper_bound_ = 6;
    }

    // Initialize Overlays
    ch1_overlay = new Overlay(res_x,res_y,lower_bound_,upper_bound_, ansi);
    ch2_overlay = new Overlay(res_x,res_y,lower_bound_,upper_bound_, ansi);
    ch3_overlay = new Overlay(res_x,res_y,lower_bound_,upper_bound_, ansi);
    ch4_overlay = new Overlay(res_x,res_y,lower_bound_,upper_bound_, ansi);
    stereo_setup = false;

    // the pointer overlay points to the overlay that is currently displayed
    switchChannel(ch_number);

    // Setup segmentation (shared with 3D measurement constructor)
    init(frame, point1, point2, interp, ch_number, interp_succ);
}

void Segmentation::init(Mat frame, cv::Point point1, cv::Point point2, bool interp, int ch_number, bool interp_succ)
{
    // current channel for the overlay
    current_channel = ch_number;

    // last segmentation position, initialized in the center
    last_x = res_x/2;
    last_y = res_y/2;

    // Set up segmentation ROI
    if (point1.x!=point2.x && point1.y!=point2.y)
    {
        // ROI defined by user
        ROI_left_upper = point1;
        ROI_right_lower = point2;
    }
    else
    {
        // ROI covers entire region
        ROI_left_upper.x = 1;
        ROI_left_upper.y = 1;
        ROI_right_lower.x = res_x;
        ROI_right_lower.y = res_y;
    }

    // Set up interpolation settings

    // radius of the area around the last aiming beam position where the segmentation of the next frame is performed
    area_dim = 0.14 * res_y; //50 at standard resolution

    // size of the structural element for morphological erosion that removes specular highliths
    size_struct_elem = 8;

    // adapt morphological filters to current resolution
    int factor = round( res_y / 372 ); //720
    struct_size1 = factor * 3; // 3 at standard resolution
    struct_size2 = factor * 2; // 1 at standard resolution

    // initialize the overlays of all four channels
    x0 = ROI_left_upper.x;
    y0 = ROI_left_upper.y;
    x1 = ROI_right_lower.x-ROI_left_upper.x;
    y1 = ROI_right_lower.y-ROI_left_upper.y;

    thres = 0.5;
}

void Segmentation::switchChannel(int channel)
{
    current_channel = channel;

    switch(channel) {
        case 1:
        overlay=ch1_overlay;
        break;

        case 2:
        overlay=ch2_overlay;
        break;

        case 3:
        overlay=ch3_overlay;
        break;

        case 4:
        overlay=ch4_overlay;
        break;
    }
}

void Segmentation::setAutoScale(bool autoscale)
{
    scale_auto = autoscale;
}

void Segmentation::setAnsi(int ansi)
{
    ch1_overlay->setAnsi(ansi);
    ch2_overlay->setAnsi(ansi);
    ch3_overlay->setAnsi(ansi);
    ch4_overlay->setAnsi(ansi);
}

void Segmentation::setIdx(int idx)
{
    this->idx = idx;
}

void Segmentation::setColorScale(double mn, double mx)
{
    // set interval for all channels
    ch1_overlay->setNewInterval(mn, mx);
    ch2_overlay->setNewInterval(mn, mx);
    ch3_overlay->setNewInterval(mn, mx);
    ch4_overlay->setNewInterval(mn, mx);
}


void Segmentation::startSegmentation(Mat frame, Mat frame_on, Mat frame_off, double lt_ch1, double lt_ch2, double lt_ch3, double lt_ch4, int idx)
{
    if (!firstFrameSet)
    {
        //save the first frame
        firstFrame = frame.clone();
        firstFrameSet = true;
    }

    double lifetime = 0;
    switch(current_channel) {
        case 1:
        lifetime=lt_ch1;
        break;
        case 2:
        lifetime=lt_ch2;
        break;
        case 3:
        lifetime=lt_ch3;
        break;
        case 4:
        lifetime=lt_ch4;
        break;
    }

    // display current lifetime value on the bottom of the screen
    if (!stereo_setup)
        overlay->drawCurrentVal(lifetime, current_channel);

    // log measurements for output
    log_lt_ch1.push_back(lt_ch1);
    log_lt_ch2.push_back(lt_ch2);
    log_lt_ch3.push_back(lt_ch3);
    log_lt_ch4.push_back(lt_ch4);

    log_frame_no.push_back(idx);
    log_timer.push_back(timer);

    // cut current segmentation region from frame
    int x, y;
    int radius = 1;
    float correlation;

    int xfrom = (last_x-area_dim < ROI_left_upper.x) ? ROI_left_upper.x  : last_x-area_dim;
    int yfrom = (last_y-area_dim < ROI_left_upper.y) ? ROI_left_upper.y  : last_y-area_dim;
    int xto   = (last_x+area_dim > ROI_right_lower.x) ? ROI_right_lower.x : last_x+area_dim;
    int yto   = (last_y+area_dim > ROI_right_lower.y) ? ROI_right_lower.y : last_y+area_dim;

    Rect corrArea(xfrom, yfrom, xto-xfrom, yto-yfrom);

    x0 = xfrom;
    y0 = yfrom;
    x1 = xto-xfrom;
    y1 = yto-yfrom;

    //Mat frame_cut = frame(corrArea);

    float xalt, yalt;
    // invoke segmentation
    correlation = pulsedSegmentation(frame_on, frame_off, corrArea, xalt, yalt, radius);
    x = (int) xalt; y = (int) yalt;

    // update beam position
    if (correlation > thres)
    {
        // compute global coordinates
        x = x+xfrom-1;
        y = y+yfrom-1;
    }
    else
    {
        // if segmentation is not valid, use the last valid coordinates
        x = last_x;
        y = last_y;
    }

    // beam segmentation valid?
    if (correlation > thres)
    {
        // update scale bar limits
        if(!stereo_setup && scale_auto)
        {
            if (lt_ch1>0)
            {
                if (lt_ch1>ch1_overlay->getUpperBound())
                    ch1_overlay->setNewInterval(ch1_overlay->getLowerBound(),ceil(lt_ch1));

                if (lt_ch1<ch1_overlay->getLowerBound() && lt_ch1>0)
                    ch1_overlay->setNewInterval(floor(lt_ch1),ch1_overlay->getUpperBound());
            }
            if (lt_ch2>0)
            {
                if (lt_ch2>ch2_overlay->getUpperBound())
                    ch2_overlay->setNewInterval(ch2_overlay->getLowerBound(),ceil(lt_ch2));

                if (lt_ch2<ch2_overlay->getLowerBound() && lt_ch2>0)
                    ch2_overlay->setNewInterval(floor(lt_ch2),ch2_overlay->getUpperBound());
            }
            if (lt_ch3>0)
            {
                if (lt_ch3>ch3_overlay->getUpperBound())
                    ch3_overlay->setNewInterval(ch3_overlay->getLowerBound(),ceil(lt_ch3));

                if (lt_ch3<ch3_overlay->getLowerBound() && lt_ch3>0)
                    ch3_overlay->setNewInterval(floor(lt_ch3),ch3_overlay->getUpperBound());
            }
            if (lt_ch4>0)
            {
                if (lt_ch4>ch4_overlay->getUpperBound())
                    ch4_overlay->setNewInterval(ch4_overlay->getLowerBound(),ceil(lt_ch4));

                if (lt_ch4<ch4_overlay->getLowerBound() && lt_ch4>0)
                    ch4_overlay->setNewInterval(floor(lt_ch4),ch4_overlay->getUpperBound());
            }
            //qDebug() << "startset finished";
        }

        if (!stereo_setup)
        {
            ch1_overlay->drawCircle(x,y,radius*0.5,lt_ch1);
            ch2_overlay->drawCircle(x,y,radius*0.5,lt_ch2);
            ch3_overlay->drawCircle(x,y,radius*0.5,lt_ch3);
            ch4_overlay->drawCircle(x,y,radius*0.5,lt_ch4);
        }

        // update log files
        log_coords_x.push_back(x);
        log_coords_y.push_back(y);
        log_radius.push_back(radius);

        // update position based on button click
        if (new_pos)
        {
            new_pos = false;
        }
        else
        {
            last_x = x;
            last_y = y;
        }

        last_radius=radius;
        last_lt_ch1=lt_ch1;
        last_lt_ch2=lt_ch2;
        last_lt_ch3=lt_ch3;
        last_lt_ch4=lt_ch4;

        last_active = true;
    }
    else
    {
        log_coords_x.push_back(0);
        log_coords_y.push_back(0);
        log_radius.push_back(0);

        last_active=false;
    }

    // merge overlay and frame

    // transparency settings
    float alpha = 0.5;
    float beta = 0.5;

    if (!stereo_setup & !pentero_mode)
    {
        addWeighted( frame, alpha, overlay->mergeOverlay(frame), beta, 0.0, frame);

        // show marker, depending on the background either in black or white
        if (frame.at<Vec3b>(y,x)[0]+frame.at<Vec3b>(y,x)[1]+frame.at<Vec3b>(y,x)[2]>383)
        {
            ellipse(frame, Point(x,y), Size(5,5), 0, 0, 360, Scalar( 0, 0, 0 ), 3, 8, 0);
        }
        else
        {
            ellipse(frame, Point(x,y), Size(5,5), 0, 0, 360, Scalar( 255, 255, 255 ), 3, 8, 0);
        }
    }

    //frame_cut.release();

    return;
}

void Segmentation::setThreshold(double thres)
{
    this->thres = thres;
}

void Segmentation::setRadius(double radius)
{
    radius_factor = radius;
}

float Segmentation::pulsedSegmentation(cv::Mat frame_on, cv::Mat frame_off, Rect corrArea, float &x, float &y, int &radius)
{

    Mat lab_on, lab_off;

    // convert to lab space
    cvtColor(frame_on, lab_on, CV_BGR2Lab);
    extractChannel (lab_on, lab_on, 2 );//2

    cvtColor(frame_off, lab_off, CV_BGR2Lab);
    extractChannel (lab_off, lab_off, 2 );//2

    // compute difference between frames
    Mat img_diff;
    img_diff = abs(lab_on(corrArea) - lab_off(corrArea));

    // threshold for segmentation, experimentally determined
    int thres = 25;
    threshold(img_diff, img_diff, thres, 255, THRESH_BINARY);

    // Floodfill from point (0, 0)
    Mat im_floodfill = img_diff.clone();
    floodFill(im_floodfill, cv::Point(0,0), Scalar(255));

    // Invert floodfilled image
    Mat im_floodfill_inv;
    bitwise_not(im_floodfill, im_floodfill_inv);

    // Combine the two images to get the foreground.
    Mat im_out = (img_diff | im_floodfill_inv);
    img_diff = im_out.clone();

    // Morphologic operations
    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(2*struct_size1 + 1, 2*struct_size1 + 1),Point(-1, -1)); // Creat structured element of size 3
    morphologyEx(img_diff, img_diff, MORPH_ERODE, element);
    vector<vector<Point> > contours;
    findContours(img_diff, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    int size_max=0;
    int ind=0;

    for(int i = 0; i < (int)contours.size(); i++) // For all the contours
    {
        int contourSize = contourArea(contours[i], false);
        // find the contour with maximum size
        if (contourSize>size_max)
        {
            size_max = contourSize;
            ind = i;
        }
    }

    // if vector is empty, segmentation is not valid
    if(contours.size()==0 || size_max<5)
        return 0;

    // Fit ellipse to binary segmentation
    RotatedRect fittedEllipse = fitEllipse(Mat(contours[ind]));
    radius = ( fittedEllipse.size.height+fittedEllipse.size.width ) / 2 * radius_factor; //2
    //radius = radius * radius_factor;
    x = fittedEllipse.center.x;
    y = fittedEllipse.center.y;

    lab_on.release();
    lab_off.release();
    img_diff.release();
    contours.clear();
    element.release();
    im_floodfill.release();
    im_floodfill_inv.release();
    im_out.release();

    // If segmentation is out of the frame area, it is not valid
    if( (x<0) | (y<0) | (x>corrArea.width) | (y>corrArea.height) | (radius>res_y/5) ) // /10
    {
        return 0;
    }

    // Define min. radius
    if (radius< 15)
        radius = 15;

    //defnie max. radius
    if (radius > 50)
        radius = 50;


    // Segmentation is valid
    return 1;
}

void Segmentation::adjustArea(int x, int y)
{
    new_pos = true;
    last_x = x;
    last_y = y;
}

void Segmentation::setTimer(double timer)
{
    this->timer = timer;
}

Segmentation::~Segmentation()
{
    delete ch1_overlay;
    delete ch2_overlay;
    delete ch3_overlay;
    delete ch4_overlay;
    delete overlay;
    //delete calib;
}
