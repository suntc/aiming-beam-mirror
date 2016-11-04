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

Segmentation::Segmentation(Mat frame, cv::Point point1, cv::Point point2, bool interp, int ch_number, bool interp_succ, StereoCalibration* calib, bool autoscale)
{
    Size s = frame.size();
    res_x = s.width;
    res_y = s.height;
    this->scale_auto=autoscale;
    //ch1_overlay = new Overlay(res_x,res_y,2,3);
    //ch2_overlay = new Overlay(res_x,res_y,2,3);
    //ch3_overlay = new Overlay(res_x,res_y,2,3);
    //ch4_overlay = new Overlay(res_x,res_y,2,3);
    stereo_setup = true;
    this->calib = calib;
    init(frame, point1, point2, interp, ch_number, interp_succ);
}

Segmentation::Segmentation(Mat frame, cv::Point point1, cv::Point point2, bool interp, int ch_number, bool interp_succ, bool autoscale, int ansi)
{
    Size s = frame.size();
    res_x = s.width;
    res_y = s.height;
    this->scale_auto=autoscale;
    ch1_overlay = new Overlay(res_x,res_y,1,6, ansi);
    ch2_overlay = new Overlay(res_x,res_y,1,6, ansi);
    ch3_overlay = new Overlay(res_x,res_y,1,6, ansi);
    ch4_overlay = new Overlay(res_x,res_y,1,6, ansi);
    stereo_setup = false;

    // the pointer overlay points to the overlay that is currently displayed
    switchChannel(ch_number);

    init(frame, point1, point2, interp, ch_number, interp_succ);
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

void Segmentation::setColorScale(double mn, double mx)
{
    // set interval for all channels
    ch1_overlay->setNewInterval(mn, mx);
    ch2_overlay->setNewInterval(mn, mx);
    ch3_overlay->setNewInterval(mn, mx);
    ch4_overlay->setNewInterval(mn, mx);
}


void Segmentation::init(Mat frame, cv::Point point1, cv::Point point2, bool interp, int ch_number, bool interp_succ)
{
    // Set up Gaussian functions for invivo segmentation
    radius_values = new int[14]; //14
    int c=0;

    for (int i=6;i<=32;i=i+2) //6-32
    {
        radius_values[c]=i;
        c++;
    }
    // Precompute functions
    for (int i = 0; i < no_gaussians; i++)
      {
         Mat* tmp = createFilter(radius_values[i]);
         gaussians[i] = tmp;
      }

    // Presets. Do not change!
    //last_vanish = 1;

    // current channel for the overlay
    current_channel = ch_number;

    // Set up resolution
    //Size s = frame.size();
    //res_x = s.width;
    //res_y = s.height;
    last_x = res_x/2;
    last_y = res_y/2;


    // Set up ROI
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

//    if (interp==1)
//        segmentation_selection=1;
//    else
//        segmentation_selection=0;

    // Set up interpolation settings

    // 1: pick up marker, 2: automatically pick up aiming beam
    segmentation_mode = 1;
    // 1: Gaussian correlation, 2: Simple threshold
    segmentation_method = 2;

    // interpolate missing frames?
    segmentation_selection = interp;
    // interpolate between successive frames?
    interpolation_successive_frames = interp_succ;
    // if yes, how many interpolation steps?
    num_of_int_steps = 5;

    // Segmentation settings for invivo measurements

    // for postprocessing only: number of frames where beam isnt seen before searching again on the entire ROI
    last_found = 5;
    // Threshold for activation. 0.958 was manually defined and found to be a good choice
    thres = (float) 0.958;

    // radius of the area around the last aiming beam position where the segmentation of the next frame is performed
    area_dim = 0.14 * res_y; //50 at standard resolution

    // if intensity is below this threshold the segmentation candidate is removed
    thres_intensity = 1; //20;
    // size of the structural element for morphological erosion that removes specular highliths
    size_struct_elem = 8; //8;

    // show marker on overlay
    show_marker = true;

    // for exvivo samples: extract first frame to segment only changes
    subtract_first_frame = false; //false;

    // adapt morphological filters to current resolution
    int factor = round( res_y / 372 );
    struct_size1 = factor * 3; // 3 at standard resolution
    struct_size2 = factor * 2; // 1 at standard resolution

    // initialize the overlays of all four channels

    x0 = ROI_left_upper.x;
    y0 = ROI_left_upper.y;
    x1 = ROI_right_lower.x-ROI_left_upper.x;
    y1 = ROI_right_lower.y-ROI_left_upper.y;
}

void Segmentation::startSegmentation(Mat frame, Mat frame_on, Mat frame_off, double lt_ch1, double lt_ch2, double lt_ch3, double lt_ch4)
{

    if (!firstFrameSet)
    {
        firstFrame = frame.clone();
        //cvtColor(firstFrame, firstFrame, CV_BGR2Lab);
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

    if (!stereo_setup)
        overlay->drawCurrentVal(lifetime, current_channel);

    frame_no++;

    Mat frame_diff = frame.clone();
    if (subtract_first_frame)
    {
        frame_diff=frame-firstFrame;
    }
    //if ( overlay == NULL && lifetime>0)
    //{
    //    double lower_lim = floor(lifetime-0.5);
    //    if (lower_lim<0)
    //            lower_lim=0;

    //    overlay = new Overlay(res_x,res_y,lower_lim,ceil(lifetime+0.5)); //1-8
    //}

    //Size s = frame.size();
    //res_x = s.width;
    //res_y = s.height;

    log_lt_ch1.push_back(lt_ch1);
    log_lt_ch2.push_back(lt_ch2);
    log_lt_ch3.push_back(lt_ch3);
    log_lt_ch4.push_back(lt_ch4);

    log_frame_no.push_back(frame_no);
    int x, y, radius;
    float correlation;

    if (segmentation_method==1)
    {
        if (segmentation_mode==2 && last_found > 4)
        {
            Rect corrArea(ROI_left_upper.x, ROI_left_upper.y, ROI_right_lower.x-ROI_left_upper.x, ROI_right_lower.y-ROI_left_upper.y);
            Mat frame_cut = frame_diff(corrArea);

            correlation = correlateGaussian(frame_cut, x, y, radius);

            if (correlation > thres)
            {
                x = x+ROI_left_upper.x-1;
                y = y+ROI_left_upper.y-1;
            }

        }
        else
        {
            int xfrom = (last_x-area_dim < ROI_left_upper.x ) ? ROI_left_upper.x  : last_x-area_dim;
            int yfrom = (last_y-area_dim < ROI_left_upper.y ) ? ROI_left_upper.y  : last_y-area_dim;
            int xto   = (last_x+area_dim > ROI_right_lower.x) ? ROI_right_lower.x : last_x+area_dim;
            int yto   = (last_y+area_dim > ROI_right_lower.y) ? ROI_right_lower.y : last_y+area_dim;

            Rect corrArea(xfrom, yfrom, xto-xfrom, yto-yfrom);
            Mat frame_cut = frame_diff(corrArea);
            correlation = correlateGaussian(frame_cut, x, y, radius);

            int x_n=last_x; int y_n=last_y;
            if (correlation > thres)
            {
                x = x+xfrom-1;
                y = y+yfrom-1;
                x_n = x; y_n = y;
            }
        }
    }
    if (segmentation_method==2)
    {
        if (segmentation_mode==2)
        {
            Rect corrArea(ROI_left_upper.x, ROI_left_upper.y, ROI_right_lower.x-ROI_left_upper.x, ROI_right_lower.y-ROI_left_upper.y);
            Mat frame_cut = frame_diff(corrArea);

            correlation = simpleThreshold(frame_cut, x, y, radius);

            if (correlation > thres)
            {
                x = x+ROI_left_upper.x-1;
                y = y+ROI_left_upper.y-1;
            }
            else
            {
                x = last_x;
                y = last_y;
            }
        }
        else
        {

            int xfrom = (last_x-area_dim < ROI_left_upper.x ) ? ROI_left_upper.x  : last_x-area_dim;
            int yfrom = (last_y-area_dim < ROI_left_upper.y ) ? ROI_left_upper.y  : last_y-area_dim;
            int xto   = (last_x+area_dim > ROI_right_lower.x) ? ROI_right_lower.x : last_x+area_dim;
            int yto   = (last_y+area_dim > ROI_right_lower.y) ? ROI_right_lower.y : last_y+area_dim;

            Rect corrArea(xfrom, yfrom, xto-xfrom, yto-yfrom);

            x0 = xfrom;
            y0 = yfrom;
            x1 = xto-xfrom;
            y1 = yto-yfrom;

            Mat frame_cut = frame_diff(corrArea);

            //correlation = doubleRingSegmentation(frame_cut, x, y, radius);
            correlation = pulsedSegmentation(frame_on, frame_off, corrArea, x, y, radius);

            int x_n=last_x; int y_n=last_y;
            if (correlation > thres)
            {
                x = x+xfrom-1;
                y = y+yfrom-1;
                x_n = x; y_n = y;
            }
            else
            {
                x = last_x;
                y = last_y;
            }
        }
    }
    //char str[10];
    //sprintf(str,"%f",correlation); // %f correlation
    if (correlation > thres)
    {
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
        }


        if (log_lifetime_failed.size()>2 && segmentation_initialized==1 && segmentation_selection)
        {


            int dx = x-log_coords_x_failed;
            int dy = y-log_coords_y_failed;
            int dr = radius-log_radius_failed;
            int no_int_points = log_lifetime_failed.size();
            int loop_counter = 1;
            for (std::vector<double>::iterator it = log_lifetime_failed.begin() ; it != log_lifetime_failed.end(); ++it)
            {
                int x_new   = (int) (log_coords_x_failed+( (float) (loop_counter*dx)/(float) no_int_points ));
                int y_new   = (int) (log_coords_y_failed+( (float) (loop_counter*dy)/(float) no_int_points ));
                int rad_new = (int) (log_radius_failed  +( (float) (loop_counter*dr)/(float) no_int_points ));
                loop_counter++;

                if (!stereo_setup)
                    overlay->drawCircle(x_new,y_new,rad_new*0.5,*it);

            }
            log_lifetime_failed.clear();
        }

        segmentation_initialized=1;
        log_lifetime_failed.clear();

        last_vanish++;

        Point pt;
        pt.x = x;
        pt.y = y;
        //circle(overlay->RGBimage,pt, radius*0.4, (0,0,255), -1);

        int loop_counter = 1;

        if (interpolation_successive_frames==1 && last_found==1 && (last_x-x)*(last_x-x)+(last_y-y)*(last_y-y)>5 )
        {
            int dx = x-last_x;
            int dy = y-last_y;
            int dr = radius-last_radius;
            double dlt1 = lt_ch1-last_lt_ch1;
            double dlt2 = lt_ch2-last_lt_ch2;
            double dlt3 = lt_ch3-last_lt_ch3;
            double dlt4 = lt_ch4-last_lt_ch4;
            loop_counter++;
            for (int i1=1; i1<=num_of_int_steps-1; i1++)
            {
                int x_new = (int)   (last_x     +(  (float) (loop_counter*dx  )/(float) num_of_int_steps ));
                int y_new = (int)   (last_y     +(  (float) (loop_counter*dy  )/(float) num_of_int_steps ));
                int rad_new = (int) (last_radius+(  (float) (loop_counter*dr  )/(float) num_of_int_steps ));
                double lt_new1 =    (last_lt_ch1+(  (float) (loop_counter*dlt1)/(float) num_of_int_steps ));
                double lt_new2 =    (last_lt_ch2+(  (float) (loop_counter*dlt2)/(float) num_of_int_steps ));
                double lt_new3 =    (last_lt_ch3+(  (float) (loop_counter*dlt3)/(float) num_of_int_steps ));
                double lt_new4 =    (last_lt_ch4+(  (float) (loop_counter*dlt4)/(float) num_of_int_steps ));
                loop_counter++;
                if (!stereo_setup)
                {
                    ch1_overlay->drawCircle(x_new,y_new,rad_new*0.5,lt_new1);
                    ch2_overlay->drawCircle(x_new,y_new,rad_new*0.5,lt_new2);
                    ch3_overlay->drawCircle(x_new,y_new,rad_new*0.5,lt_new3);
                    ch4_overlay->drawCircle(x_new,y_new,rad_new*0.5,lt_new4);
                }
            }
        }

        if (!stereo_setup)
        {
            ch1_overlay->drawCircle(x,y,radius*0.5,lt_ch1);
            ch2_overlay->drawCircle(x,y,radius*0.5,lt_ch2);
            ch3_overlay->drawCircle(x,y,radius*0.5,lt_ch3);
            ch4_overlay->drawCircle(x,y,radius*0.5,lt_ch4);
        }


        //putText(frame, str, Point2f(100,100), FONT_HERSHEY_PLAIN, 2,  Scalar(255,0,0,255));

        //if(stereo_setup==true)
        //{
        //    Point beam_pos(x,y);
        //    beam_pos = calib->getRectifiedPoint(beam_pos,0);
        //    log_coords_x.push_back(beam_pos.x);
       //     log_coords_y.push_back(beam_pos.y);
        //}
        //else
        //{
            log_coords_x.push_back(x);
            log_coords_y.push_back(y);
        //}

        log_radius.push_back(radius);
        last_found = 1;
        last_x = x;
        last_y = y;
        last_radius=radius;
        last_lt_ch1=lt_ch1;
        last_lt_ch2=lt_ch2;
        last_lt_ch3=lt_ch3;
        last_lt_ch4=lt_ch4;

        last_active = true;
    }
    else
    {
        last_vanish = 1;
        //last_found == 2;
        last_found++;
        if (last_found == 2)
        {
            log_coords_x_failed = last_x;
            log_coords_y_failed = last_y;
            log_radius_failed = last_radius;
        }
        log_lifetime_failed.push_back(lifetime);
        log_coords_x.push_back(0);
        log_coords_y.push_back(0);
        log_radius.push_back(0);


        //putText(frame, str, Point2f(100,100), FONT_HERSHEY_PLAIN, 2,  Scalar(0,0,255,255));
        last_active=false;
    }

    float alpha = 0.5;
    float beta = 0.5;

    if (!stereo_setup)
        addWeighted( frame, alpha, overlay->mergeOverlay(frame), beta, 0.0, frame);

    if (show_marker==true)
    {
        //qDebug() << x;
        //qDebug() << y;
        // enhance contrast between marker and background
        if (frame.at<Vec3b>(y,x)[0]+frame.at<Vec3b>(y,x)[1]+frame.at<Vec3b>(y,x)[2]>383)
        {
            ellipse(frame, Point(x,y), Size(5,5), 0, 0, 360, Scalar( 0, 0, 0 ), 3, 8, 0);
        }
        else
        {
            ellipse(frame, Point(x,y), Size(5,5), 0, 0, 360, Scalar( 255, 255, 255 ), 3, 8, 0);
        }
    }
    //char str[10];
    //sprintf(str,"%f",lifetime);
    //putText(frame, str, Point2f(200,200), FONT_HERSHEY_PLAIN, 2,  Scalar(0,255,0,255));
    //imshow("activeDisplay", dst );
    //qDebug() << "after merge";
    return;
}

float Segmentation::simpleThreshold(cv::Mat frame, int &x, int &y, int &radius)
{
    Mat frame_hsv;
    cvtColor(frame, frame_hsv, CV_BGR2HSV);
    //Mat ch_L; Mat ch_a; Mat ch_b;
    //extractChannel (frame_lab, ch_L, 0 );
    //extractChannel (frame_lab, ch_a, 1 );
    //extractChannel (frame_lab, ch_b, 2 );

    int hue_min = 85;  // Hue min
    int hue_max = 120; // Hue max
    int sat_min = 50;  // Saturation min
    int sat_max = 255; // Saturation max
    int val_min = 50;  // Value min
    int val_max = 255; // Value max

    Mat frameBlueOCL;
    inRange(frame_hsv, Scalar(hue_min, sat_min, val_min), Scalar(hue_max, sat_max, val_max), frameBlueOCL); // HSV thresholding

    //x = 0;
    //y = 0;
    radius = 10;

    //const char * filename1 = "test1.jpg";
    //cvSaveImage(filename1, &(IplImage(frameBlueOCL)));

    int struct_type = MORPH_ELLIPSE;    // Structured element type is ellipse

    Mat element0 = getStructuringElement(struct_type, Size(2*struct_size2 + 1, 2*struct_size2 + 1),Point(-1, -1)); // Create structured element of size 1
    Mat element = getStructuringElement(struct_type, Size(2*struct_size1 + 1, 2*struct_size1 + 1),Point(-1, -1)); // Creat structured element of size 3
    morphologyEx(frameBlueOCL, frameBlueOCL, MORPH_DILATE, element0); // Dilation with structured element of size 1

    Mat im_floodfill = frameBlueOCL.clone();
    floodFill(im_floodfill,Point(0,0),Scalar(255));
    bitwise_not(im_floodfill,im_floodfill);
    frameBlueOCL = (frameBlueOCL | im_floodfill);


    morphologyEx(frameBlueOCL, frameBlueOCL, MORPH_ERODE, element); // Erosion with structured element of size 3
    morphologyEx(frameBlueOCL, frameBlueOCL, MORPH_DILATE, element); // Dilation with structured element of size 3
    morphologyEx(frameBlueOCL, frameBlueOCL, MORPH_ERODE, element0); // Erosion with structured element of size 1

    // Initialize segmentation vectors
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;



    //const char * filename2 = "test2.jpg";
    //cvSaveImage(filename2, &(IplImage(frameBlueOCL)));



    int maxPos=0; // Minimum Euclidean distance

    //const char * filename1 = "test.jpg";
    //cvSaveImage(filename1, &(IplImage(frameBlueOCL)));

    // Find Contours
    findContours(frameBlueOCL, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    vector<double> meanIntVals(contours.size()); // Create Euclidean distances vector

    //qDebug() << contours.size();



    vector<RotatedRect> minEllipse(contours.size()); // Create ellipses vector
    for(int i = 0; i < (int)contours.size(); i++) // For all the contours
    {

        int contourSize = contourArea(contours[i], false); // Find the size of the current contour



        if (contourSize > 5 && contourSize < 50000 && contours[i].size() > 4) // If contour size is larger than 5 pixels, smaller than 10000 pixels and there are more than 4 contour points
        {
            // Find distance from previous ellipse
            minEllipse[i] = fitEllipse(Mat(contours[i])); // Fit ellipse to the contour
            double x1, y1, x_old, y_old; // Coordinates of the current and previous ellipses centers
            x1 = minEllipse[i].center.x; // Current ellipse center x coordinate
            y1 = minEllipse[i].center.y; // Current ellipse center y coordinate
            if (last_x!=res_x/2)
            {
                x_old = last_x; // Previous ellipse center x coordinate
                y_old = last_y; // Previous ellipse center y coordinate
            }
            else
            {
                x_old = x1;
                y_old = y1;
            }
            double dx;
            dx = minEllipse[i].size.height + minEllipse[i].size.width;
             //       sqrt((( (x1+offset_x) - x_old) * ((x1+offset_x) - x_old)) + (((y1+offset_y) - y_old) * ((y1+offset_y) - y_old))); // Euclidean distance between the centers of the two ellipses
            //dx = sqrt((( (x1) - x_old) * ((x1) - x_old)) + (((y1) - y_old) * ((y1) - y_old))); // Euclidean distance between the centers of the two ellipses
            meanIntVals[i] = dx; // Store Euclidean distance in vectro
        }
        else // else
        {
            meanIntVals[i] = 1000;
        }
    }
    // Draw Ellipse
    if (contours.size() > 0) // If there exist contours
    {

        //maxPos = distance(meanIntVals.begin(), min_element(meanIntVals.begin(), meanIntVals.end()));
        maxPos = distance(meanIntVals.begin(), max_element(meanIntVals.begin(), meanIntVals.end()));

        if ((meanIntVals[maxPos] < 1000)) //100 //|| (dataold.at<double>(0, 0) == round(frameOnline.cols / 2))) // If current Euclidean distance is smaller than 100 pixels or the previous ellipse's center x coordinate is at the center of the image
        {
        //maxPos = distance(meanIntVals.begin(), min_element(meanIntVals.begin(), meanIntVals.end())); // Find the minimum Euclidean distance
            // Fit Gaussian
            double PI = 3.14159265;
            double lineangle = tan((90 + minEllipse[maxPos].angle) * PI / 180.0); // Find the angle of the major axis

            // Create the line that is on the major axis and equal twice of its length (see paper for more details)
            double x1 = minEllipse[maxPos].center.x + ((minEllipse[maxPos].size.height) / sqrt((lineangle * lineangle) + 1));
            double x2 = minEllipse[maxPos].center.x - ((minEllipse[maxPos].size.height) / sqrt((lineangle * lineangle) + 1));
            double y1 = minEllipse[maxPos].center.y + ((lineangle * (minEllipse[maxPos].size.height)) / sqrt((lineangle * lineangle) + 1));
            double y2 = minEllipse[maxPos].center.y - ((lineangle * (minEllipse[maxPos].size.height)) / sqrt((lineangle * lineangle) + 1));

            // Get the blue channel values of the RGB image that fall on the line and create the linear model (see paper for more)
            LineIterator it(frame, Point (x1, y1), Point (x2, y2), 8);
            Mat I_val(Mat_<double>(it.count, 1));
            Mat x_val(Mat_<double>(it.count, 3));

            if (x1==0 || x2==0 || y1==0 || y2==0)
            {
                return 0;
            }
            for(int i = 0; i < it.count; i++, ++it)
            {
                Vec3b val = frame.at<Vec3b>(it.pos());
                x_val.at<double>(i, 0) = (double)i * (double)i;
                x_val.at<double>(i, 1) = (double)i;
                x_val.at<double>(i, 2) = 1.0;
                I_val.at<double>(i, 0) = (double)val[0];
            }
            double minVal;
            double maxVal;
            minMaxIdx(I_val, &minVal, &maxVal);
            subtract(I_val, (minVal - 1), I_val);
            if (it.count<=1)
                return 0;
            I_val.at<double>(0, 0) = 1;
            I_val.at<double>((it.count - 1), 0) = 1;

            log(I_val, I_val);
            Mat I_out;

            // Solve the linear model to fit the Gaussian (see paper for more)
            solve(x_val, I_val, I_out, DECOMP_SVD);

            // Scale Ellipse
            double sigma = abs(sqrt(- 1.0 / (2.0 * I_out.at<double>(0, 0)))); // Estimate the Gaussian sigma value
            double FWHM = 2.0 * sigma * sqrt(- 2.0 * log(0.5)); // Estimate FWHM from sigma value (see paper for equations and more)
            double FW075M = 2.0 * sigma * sqrt(- 2.0 * log(0.75)); // Estimate FW075M from sigma value
            double FW09M = 2.0 * sigma * sqrt(- 2.0 * log(0.9)); // Estimate FW09M from sigma value

            // Scale accordingly the minor axis of the ellipse
            double scaleVal_FWHM = minEllipse[maxPos].size.width / (minEllipse[maxPos].size.height / FWHM);
            double scaleVal_FW09M = minEllipse[maxPos].size.width / (minEllipse[maxPos].size.height / FW09M);
            double scaleVal_FW075M = minEllipse[maxPos].size.width / (minEllipse[maxPos].size.height / FW075M);

            // Initialize the mats that have the current scaled ellipses
            //allImages.singleEllipse_FWHM = Mat::zeros(frameSegmIn.rows, frameSegmIn.cols, CV_8UC1);
            //allImages.singleEllipse_FW09M = Mat::zeros(frameSegmIn.rows, frameSegmIn.cols, CV_8UC1);
            //allImages.singleEllipse_FW075M = Mat::zeros(frameSegmIn.rows, frameSegmIn.cols, CV_8UC1);
            // Make Single Ellipses
            if (!isnan(FWHM)) // If the FWHM is a finite value (Gaussian fit was successful)
            {
       //         minEllipse[maxPos].size.height = FWHM; // Update ellipse's height
       //         minEllipse[maxPos].size.width = scaleVal_FWHM; // Update ellipse's width

                //ellipse(allImages.singleEllipse_FWHM, minEllipse[maxPos], Scalar(255), -1, 8); // Create ellipse

                minEllipse[maxPos].size.height = FW075M; // Update ellipse's height
                minEllipse[maxPos].size.width = scaleVal_FW075M; // Update ellipse's width

                //ellipse(allImages.singleEllipse_FW075M, minEllipse[maxPos], Scalar(255), -1, 8); // Create ellipse

       //         minEllipse[maxPos].size.height = FW09M; // Update ellipse's height
       //         minEllipse[maxPos].size.width = scaleVal_FW09M; // Update ellipse's width
                //ellipse(allImages.singleEllipse_FW09M, minEllipse[maxPos], Scalar(255), -1, 8); // Create ellipse
            }
            else // If the FWHM does not have a finite value (Gaussian fit failed)
            {
                minEllipse[maxPos].size.height = minEllipse[maxPos].size.height / 3; // Scale ellipse's height to 1/3
                minEllipse[maxPos].size.width = minEllipse[maxPos].size.width / 3; // Scale ellipse's width to 1/3

                // Create corresponding ellipses
                //ellipse(allImages.singleEllipse_FWHM, minEllipse[maxPos], Scalar(255), -1, 8);
                //ellipse(allImages.singleEllipse_FW075M, minEllipse[maxPos], Scalar(255), -1, 8);
                //ellipse(allImages.singleEllipse_FW09M, minEllipse[maxPos], Scalar(255), -1, 8);

                // Update widths
                FWHM = minEllipse[maxPos].size.height / 3;
                FW075M = minEllipse[maxPos].size.height / 3;
                FW09M = minEllipse[maxPos].size.height / 3;
            }

            // Make Masks and label cases for painting
       //     if (meanIntVals[maxPos] <= FW09M) // If the Euclidean distance between current and previous ellipses is smaller than or equal to FW09M
       //     {
                //allImages.bwEllipsesMask.setTo(Scalar(255), allImages.singleEllipse_FW075M); // The external ellipse is FW075M (see paper for more)
       //         case_FW = 0; // Case for painting is 0
       //     }
       //     else if ((meanIntVals[maxPos] > FW09M) && (meanIntVals[maxPos] <= FW075M)) // If the Euclidean distance between current and previous ellipses is larger than FW09M and smaller than or equal to FW075M
       //     {
                //allImages.bwEllipsesMask.setTo(Scalar(255), allImages.singleEllipse_FWHM); // The external ellipse is FWHM (see paper for more)
       //         case_FW = 1; // Case for painting is 1
       //     }
       //     else // in all other cases
       //     {
                //allImages.bwEllipsesMask.setTo(Scalar(255), allImages.singleEllipse_FWHM); // The external ellipse is FWHM (see paper for more)
       //         case_FW = 2; // Case for painting is 2
       //     }

            //ellipse(frameOnline, minEllipse[maxPos], Scalar(0, 0, 0), 1, 8); // Draw smallest ellipse in running frame
            //line(frameOnline, Point (x1, y1), Point (x2, y2), Scalar(0, 0, 0), 1); // Draw line used for Gaussian fit in running frame
        }
        else
        {
            return 0;
        }

        radius = ( minEllipse[maxPos].size.height+minEllipse[maxPos].size.width ) / 2;
        x = minEllipse[maxPos].center.x;
        y = minEllipse[maxPos].center.y;

        if (x==0 || y==0)
        {
            return 0;
        }
    }
    else
    {
        x = last_x;
        y = last_y;

        return 0;
    }
    return 1;
}

float Segmentation::correlateGaussian(cv::Mat frame, int &x, int &y, int &radius)
{

    Mat frame_lab;
    Mat intensity;
    cvtColor(frame, frame_lab, CV_BGR2Lab);

    extractChannel (frame_lab, intensity, 0 );
    extractChannel (frame_lab, frame_lab, 2 ); //2

    int x_list[no_gaussians];
    int y_list[no_gaussians];
    double max_list[no_gaussians];

    cv::Point min_loc, max_loc;
    double min, max;
    Mat corr;

    Mat frame_gray;
    cvtColor( frame, frame_gray, CV_BGR2GRAY );
    threshold( frame_gray, frame_gray, 220,  255, 0 );
    Mat structElem = getStructuringElement( MORPH_ELLIPSE, Size( size_struct_elem, size_struct_elem ), Point( -1, -1 ) );
    dilate(frame_gray, frame_gray, structElem);

    for (int i=0; i<no_gaussians; i++)
    {

        int offset = ceil(1.5*1.5*radius_values[i]);
        if (2*offset>=frame_lab.rows || 2*offset>=frame_lab.cols)
        {
            max_list[i] = 0;
            x_list[i] = 0;
            y_list[i] = 0;
            continue;
        }
        matchTemplate(frame_lab, *gaussians[i], corr, CV_TM_CCORR_NORMED);
        Size s = corr.size();
        int corr_dim_x = s.height;
        int corr_dim_y = s.width;

        for (int x0=offset; x0<corr_dim_x; x0++)
            for (int y0=offset; y0<corr_dim_y; y0++)
                if( frame_gray.at<unsigned char>(x0,y0)>0 || intensity.at<unsigned char>(x0,y0)<thres_intensity)
                    corr.at<float>(x0-offset,y0-offset) = 0;

        minMaxLoc(corr, &min, &max, &min_loc, &max_loc);
        x_list[i] = max_loc.x + ceil(1.5*1.5*radius_values[i]);
        y_list[i] = max_loc.y + ceil(1.5*1.5*radius_values[i]);
        max_list[i] = max;
    }

    radius = getRadius(max_list, x_list, y_list, ARRAY_SIZE(max_list), x, y);

    const int N = sizeof(max_list) / sizeof(double);
    int ind = distance(max_list, max_element(max_list, max_list + N));


        //qDebug() << y;
        //qDebug() << frame_lab.at<unsigned char>(y,x);
        //char str[10];
        //sprintf(str,"%u",frame_lab.at<unsigned char>(y,x)); // %f correlation
        //putText(frame, str, Point2f(300,100), FONT_HERSHEY_PLAIN, 2,  Scalar(255,0,0,255));


    return max_list[ind];
}



Mat* Segmentation::createFilter(int sigma)
{
    int radius = ceil(1.5*sigma);
    //Mat gKernel(2*radius+1,2*radius+1, DataType<double>::type);
    cv::Mat * gKernel = new cv::Mat( cv::Mat::zeros(3*radius+1, 3*radius+1, CV_8U) );

    double amplitude = 1; //1 / (2 * sqrt(2*M_PI));

    for (int x = -1.5*radius; x <= 1.5*radius; x++)
    {
        for(int y = -1.5*radius; y <= 1.5*radius; y++)
        {
            double exponent = (double) (-(x*x + y*y)) / (double) (2*sigma*sigma);
            gKernel->at<unsigned char>(x+1.5*radius,y+1.5*radius) =  255 - round( amplitude*exp(exponent)*255 );
        }
    }
    return gKernel;
}

double Segmentation::getRadius(double cr[], int x[], int y[], int arrLength, int &x_out, int &y_out)
{
    //int arrLength = ARRAY_SIZE(cr);

    std::vector<int> x0;
    std::vector<int> y0;
    std::vector<double> weights;
    std::vector<double> rads;
    std::vector<double> nums;
    std::vector<int> skip_inds;
    std::vector<int> valid_inds;
   // std::vector<int> multips;
    for (int i = 0; i < arrLength - 1; i++)
    {
        if (cr[i]>0.2 && cr[i]<=1)
        {
        }
        else
            cr[i] = 0;
    }

    for (int i = 0; i < arrLength - 1; i++)
    {
        double tmp = (cr[i]-0.8)*5;
        weights.push_back( tmp ); //todo if smaller.
       // multips.push_back( (cr[i]-0.8)*5 ); //todo if smaller.
        rads.push_back(radius_values[i]*tmp);
        x0.push_back(x[i]*tmp);
        y0.push_back(y[i]*tmp);
        nums.push_back( tmp );
        if ( find(skip_inds.begin(), skip_inds.end(), i) != skip_inds.end() )
            continue;


        for (int j = i + 1;j < arrLength; j++)
        {

            //if (x[i] == x[j] && y[i] == y[j])
            if (abs(x[i]-x[j])<31 && abs(y[i]-y[j])<31)
            {
//                x0[i] = x[j];
//                y0[i] = y[j];
                double tmp = (cr[j]-0.8)*5; //todo if smaller.
                weights[i] = weights[i]+tmp;
                x0[i] = x0[i] + round(x[i]*tmp);
                y0[i] = y0[i] + round(y[i]*tmp);
                rads[i] = rads[i]+radius_values[j]*tmp;
                nums[i] = nums[i]+tmp;
                skip_inds.push_back(j);
                valid_inds.push_back(i);
            }
        }
    }
    auto mx = max_element( weights.begin(), weights.end() );
    int idx = distance( weights.begin(), mx);
    double mmx = rads[idx] / (double) nums[idx];
    x_out = (int) (x0[idx] / (double) nums[idx]);
    y_out = (int) (y0[idx] / (double) nums[idx]);
    return mmx; //8.0;
}

void Segmentation::setThreshold(double thres)
{
    this->thres = thres;
}

float Segmentation::doubleRingSegmentation(cv::Mat frame, int &x, int &y, int &radius)
{
    // convert to HSV color space
    Mat frame_hsv;
    cvtColor(frame, frame_hsv, CV_BGR2HSV);

    // thresholds for inner ring
    int hue_min = 0; //0; //0;  // Hue min
    int hue_max = 255; //255; //255; // Hue max
    int sat_min = 0;//0;  // Saturation min
    int sat_max = 255;//10; // Saturation max
    int val_min = 200; //0;  // Value min
    int val_max = 255; //255; // Value max

    // thresholds for outer ring
    int hue_min_border = 60; //100;  // Hue min
    int hue_max_border = 160; //120; // Hue max
    int sat_min_border = 0;  // Saturation min
    int sat_max_border = 255; // Saturation max
    int val_min_border = 0;  // Value min
    int val_max_border = 255; // Value max

    // segment candidates for inner ring
    Mat frameBlueOCL;
    inRange(frame_hsv, Scalar(hue_min, sat_min, val_min), Scalar(hue_max, sat_max, val_max), frameBlueOCL); // HSV thresholding
    radius = 10;

    // morphologic erosion and dilation to get a closed contour
    int struct_type = MORPH_ELLIPSE;    // Structured element type is ellipse

    Mat element0 = getStructuringElement(struct_type, Size(2*struct_size2 + 1, 2*struct_size2 + 1),Point(-1, -1)); // Create structured element of size 1
    Mat element = getStructuringElement(struct_type, Size(2*struct_size1 + 1, 2*struct_size1 + 1),Point(-1, -1)); // Creat structured element of size 3
    morphologyEx(frameBlueOCL, frameBlueOCL, MORPH_DILATE, element0); // Dilation with structured element of size 1
    morphologyEx(frameBlueOCL, frameBlueOCL, MORPH_ERODE, element); // Erosion with structured element of size 3
    morphologyEx(frameBlueOCL, frameBlueOCL, MORPH_DILATE, element); // Dilation with structured element of size 3
    morphologyEx(frameBlueOCL, frameBlueOCL, MORPH_ERODE, element0); // Erosion with structured element of size 1

    // extract candidates
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(frameBlueOCL, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    if (contours.size()==0)
        return 0;

    Mat out = Mat::zeros(frame.rows, frame.cols, CV_8UC1);
    Mat out2 = Mat::zeros(frame.rows, frame.cols, CV_8UC1);

    // go through all candidates
    vector<int> beam_sizes;
    for(int i = 0; i < (int)contours.size(); i++) // For all the contours
    {
        int contourSize = contourArea(contours[i], false);
        drawContours(out,contours,i,Scalar( 255),CV_FILLED);
        Mat border;

        // extract outer ring
        morphologyEx(out, out2, MORPH_DILATE, element0);
        bitwise_xor(out,out2,out2);

        Mat beam_borders;
        inRange(frame_hsv, Scalar(hue_min_border, sat_min_border, val_min_border), Scalar(hue_max_border, sat_max_border, val_max_border), beam_borders);

        Mat border_criteria;
        bitwise_and(out2,beam_borders,beam_borders);

        vector<Point> area1;
        vector<Point> area2;
        findNonZero(out2,area1);
        findNonZero(beam_borders,area2);

        double a = area1.size();
        double b = area2.size();
        double beam_criteria = b/a;

        // the outer ring has a dominant blue shimmer
        if (beam_criteria<0.4) //0.58
        {
            // if not delete candidate
            contourSize=0;
        }
        beam_sizes.push_back(contourSize);
    }
    // from the remaining candidates take the biggest one
    double mx_size = *max_element(beam_sizes.begin(), beam_sizes.end());
    int maxPos = distance(beam_sizes.begin(), max_element(beam_sizes.begin(), beam_sizes.end()));

    if (mx_size<5)
    {
            return 0;
    }
    // fit ellipse
    RotatedRect fittedEllipse = fitEllipse(Mat(contours[maxPos]));

    if ((beam_sizes[maxPos] < 10000)) //100 //|| (dataold.at<double>(0, 0) == round(frameOnline.cols / 2))) // If current Euclidean distance is smaller than 100 pixels or the previous ellipse's center x coordinate is at the center of the image
    {
        // Fit Gaussian
        double PI = 3.14159265;
        double lineangle = tan((90 + fittedEllipse.angle) * PI / 180.0); // Find the angle of the major axis

        // Create the line that is on the major axis and equal twice of its length (see paper for more details)
        double x1 = fittedEllipse.center.x + ((fittedEllipse.size.height) / sqrt((lineangle * lineangle) + 1));
        double x2 = fittedEllipse.center.x - ((fittedEllipse.size.height) / sqrt((lineangle * lineangle) + 1));
        double y1 = fittedEllipse.center.y + ((lineangle * (fittedEllipse.size.height)) / sqrt((lineangle * lineangle) + 1));
        double y2 = fittedEllipse.center.y - ((lineangle * (fittedEllipse.size.height)) / sqrt((lineangle * lineangle) + 1));

        // Get the blue channel values of the RGB image that fall on the line and create the linear model (see paper for more)
        LineIterator it(frame, Point (x1, y1), Point (x2, y2), 8);
        Mat I_val(Mat_<double>(it.count, 1));
        Mat x_val(Mat_<double>(it.count, 3));

        if (x1==0 || x2==0 || y1==0 || y2==0)
        {
            return 0;
        }
        for(int i = 0; i < it.count; i++, ++it)
        {
            Vec3b val = frame.at<Vec3b>(it.pos());
            x_val.at<double>(i, 0) = (double)i * (double)i;
            x_val.at<double>(i, 1) = (double)i;
            x_val.at<double>(i, 2) = 1.0;
            I_val.at<double>(i, 0) = (double)val[0];
        }
        double minVal;
        double maxVal;
        minMaxIdx(I_val, &minVal, &maxVal);
        subtract(I_val, (minVal - 1), I_val);
        if (it.count<=1)
        {
            return 0;
        }
        I_val.at<double>(0, 0) = 1;
        I_val.at<double>((it.count - 1), 0) = 1;

        log(I_val, I_val);
        Mat I_out;

        // Solve the linear model to fit the Gaussian (see paper for more)
        solve(x_val, I_val, I_out, DECOMP_SVD);

        // Scale Ellipse
        double sigma = abs(sqrt(- 1.0 / (2.0 * I_out.at<double>(0, 0)))); // Estimate the Gaussian sigma value
        double FWHM = 2.0 * sigma * sqrt(- 2.0 * log(0.5)); // Estimate FWHM from sigma value (see paper for equations and more)
        double FW075M = 2.0 * sigma * sqrt(- 2.0 * log(0.75)); // Estimate FW075M from sigma value
        double FW09M = 2.0 * sigma * sqrt(- 2.0 * log(0.9)); // Estimate FW09M from sigma value

        // Scale accordingly the minor axis of the ellipse
        double scaleVal_FWHM = fittedEllipse.size.width / (fittedEllipse.size.height / FWHM);
        double scaleVal_FW09M = fittedEllipse.size.width / (fittedEllipse.size.height / FW09M);
        double scaleVal_FW075M = fittedEllipse.size.width / (fittedEllipse.size.height / FW075M);

        if (!isnan(FWHM)) // If the FWHM is a finite value (Gaussian fit was successful)
        {
            fittedEllipse.size.height = FW075M; // Update ellipse's height
            fittedEllipse.size.width = scaleVal_FW075M; // Update ellipse's width
        }
        else // If the FWHM does not have a finite value (Gaussian fit failed)
        {
            fittedEllipse.size.height = fittedEllipse.size.height / 3; // Scale ellipse's height to 1/3
            fittedEllipse.size.width = fittedEllipse.size.width / 3; // Scale ellipse's width to 1/3

            // Update widths
            FWHM = fittedEllipse.size.height / 3;
            FW075M = fittedEllipse.size.height / 3;
            FW09M = fittedEllipse.size.height / 3;
        }
    }
    else
    {
        return 0;
    }
    radius = ( fittedEllipse.size.height+fittedEllipse.size.width ) / 2; //2
    x = fittedEllipse.center.x;
    y = fittedEllipse.center.y;

    if (x==0 || y==0)
    {
        return 0;
    }
    return 1;
}

float Segmentation::pulsedSegmentation(cv::Mat frame_on, cv::Mat frame_off, Rect corrArea, int &x, int &y, int &radius)
{

    Mat lab_on, lab_off;

    // convert to lab space
    //const char * filename1 = "frameon.jpg";
    //cvSaveImage(filename1, &(IplImage(frame_on)));

    cvtColor(frame_on, lab_on, CV_BGR2Lab);
    extractChannel (lab_on, lab_on, 2 );

    //const char * filename2 = "frameoff.jpg";
    //cvSaveImage(filename2, &(IplImage(frame_off)));
    cvtColor(frame_off, lab_off, CV_BGR2Lab);
    extractChannel (lab_off, lab_off, 2 );

    //const char * filename2 = "frameon.jpg";
    //cvSaveImage(filename2, &(IplImage(lab_on(corrArea))));

    //const char * filename1 = "frameoff.jpg";
    //cvSaveImage(filename1, &(IplImage(lab_off(corrArea))));

    // compute difference between frames
    Mat img_diff;
    img_diff = abs(lab_on(corrArea) - lab_off(corrArea));
    //const char * filename3 = "framediff.jpg";
    //cvSaveImage(filename3, &(IplImage(img_diff)));


    int thres = 25; //40; //255; //255; // Hue max
    threshold(img_diff, img_diff, thres, 255, THRESH_BINARY);

    // Floodfill from point (0, 0)
    Mat im_floodfill = img_diff.clone();
    floodFill(im_floodfill, cv::Point(0,0), Scalar(255));

    // Invert floodfilled image
    Mat im_floodfill_inv;
    bitwise_not(im_floodfill, im_floodfill_inv);

    // Combine the two images to get the foreground.
    Mat im_out = (img_diff | im_floodfill_inv);
    img_diff = im_out.clone(); //TODO: remove clone

    //const char * filename3 = "framediff.jpg";
    //cvSaveImage(filename3, &(IplImage(img_diff)));

    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(2*struct_size1 + 1, 2*struct_size1 + 1),Point(-1, -1)); // Creat structured element of size 3
    morphologyEx(img_diff, img_diff, MORPH_ERODE, element);

    vector<vector<Point> > contours;
    findContours(img_diff, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    int size_max=0;
    int ind=0;
    for(int i = 0; i < (int)contours.size(); i++) // For all the contours
    {
        int contourSize = contourArea(contours[i], false);
        if (contourSize>size_max)
        {
            size_max = contourSize;
            ind = i;
        }
    }

    if(contours.size()==0 || size_max<5)
        return 0;

    RotatedRect fittedEllipse = fitEllipse(Mat(contours[ind]));
    radius = ( fittedEllipse.size.height+fittedEllipse.size.width ) / 2; //2
    x = fittedEllipse.center.x;
    y = fittedEllipse.center.y;

    if( x<0 | y<0 | x>corrArea.width | y>corrArea.height | radius>res_y/10)
    {
        /*
        const char * filename3 = "wrongfitting.jpg";
        cvSaveImage(filename3, &(IplImage(img_diff)));

        const char * filename4 = "wrongfitting2.jpg";
        cvSaveImage(filename4, &(IplImage(im_out)));

        const char * filename2 = "frameon.jpg";
        cvSaveImage(filename2, &(IplImage(lab_on(corrArea))));

        const char * filename1 = "frameoff.jpg";
        cvSaveImage(filename1, &(IplImage(lab_off(corrArea))));
        */
        return 0;
    }
    if (radius< 15)
        radius = 15;

    return 1;



}

/*
float Segmentation::pulsedSegmentation(cv::Mat frame, Rect corrArea, int &x, int &y, int &radius)
{
    frame2 = frame1;
    Mat frame_hsv;
    Mat frame_lab;
    //cvtColor(frame, frame_hsv, CV_BGR2HSV);
    cvtColor(frame, frame_lab, CV_BGR2Lab);
    extractChannel (frame_lab, frame_lab, 2 );
    frame1 = frame_lab;

    if( frame_no < 2)
        return 0;

    Mat img_diff;
    img_diff = abs(frame1(corrArea) - frame2(corrArea));
//const char * filenamef1 = "frame1.jpg";
//cvSaveImage(filenamef1, &(IplImage(frame1(corrArea))));
//const char * filenamef2 = "frame2.jpg";
//cvSaveImage(filenamef2, &(IplImage(frame2(corrArea))));

    int thres = 15; //40; //255; //255; // Hue max
    threshold(img_diff, img_diff, thres, 255, THRESH_BINARY);
//qDebug() << "1";

//const char * filename1 = "im1.jpg";
//cvSaveImage(filename1, &(IplImage(img_diff)));

    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(2*struct_size1 + 1, 2*struct_size1 + 1),Point(-1, -1)); // Creat structured element of size 3
    morphologyEx(img_diff, img_diff, MORPH_ERODE, element);

//const char * filename2 = "im2.jpg";
//cvSaveImage(filename2, &(IplImage(img_diff)));

    vector<vector<Point> > contours;
    //vector<Vec4i> hierarchy;
    findContours(img_diff, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
//qDebug() << "2";
    int size_max=0;
    int ind=0;
    for(int i = 0; i < (int)contours.size(); i++) // For all the contours
    {
        int contourSize = contourArea(contours[i], false);
        if (contourSize>size_max)
        {
            size_max = contourSize;
            ind = i;
        }
    }
    //qDebug() << contours.size();
    if(contours.size()==0 || size_max<5)
        return 0;

    //const char * filename3 = "im3.jpg";
    //cvSaveImage(filename3, &(IplImage(Mat(contours[ind]))));

//qDebug() << "3";
    RotatedRect fittedEllipse = fitEllipse(Mat(contours[ind]));
//qDebug() << "3.5";
    radius = ( fittedEllipse.size.height+fittedEllipse.size.width ) / 2; //2
    x = fittedEllipse.center.x;
    y = fittedEllipse.center.y;
//qDebug() << "4";
    return 1;


    //Mat frameBlueOCL1;
    //inRange(frame1, Scalar(hue_min, sat_min, val_min), Scalar(hue_max, sat_max, val_max), frameBlueOCL1); // HSV thresholding

    //Mat frameBlueOCL2;
    //inRange(frame2, Scalar(hue_min, sat_min, val_min), Scalar(hue_max, sat_max, val_max), frameBlueOCL2); // HSV thresholding

    //vector<Point> area1;
    //vector<Point> area2;
    //findNonZero(frameBlueOCL1,area1);
    //findNonZero(frameBlueOCL2,area2);

    //double a = area1.size();
    //double b = area2.size();

    //qDebug() << a;
    //qDebug() << b;

    return 0;
}
*/
