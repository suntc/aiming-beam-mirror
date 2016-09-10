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


using namespace cv;
using namespace std;


Segmentation::Segmentation(Mat frame, cv::Point point1, cv::Point point2, bool interp, int ch_number, bool interp_succ)
{

    // set parameters here
    radius_values = new int[14]; //14
    int c=0;

    for (int i=6;i<=32;i=i+2) //6-32
    {
        radius_values[c]=i;
        c++;
    }

    for (int i = 0; i < no_gaussians; i++) //no_gaussians
      {
         Mat* tmp = createFilter(radius_values[i]);
         gaussians[i] = tmp;
      }

    // Presets. Do not change!
    last_vanish = 1;

    current_channel = ch_number;

    Size s = frame.size();

    res_x = s.width;
    res_y = s.height;
    last_x = res_x/2;
    last_y = res_y/2;

    if (point1.x!=point2.x && point1.y!=point2.y)
    {
        ROI_left_upper = point1;
        ROI_right_lower = point2;
    }
    else
    {
        ROI_left_upper.x = 1;
        ROI_left_upper.y = 1;
        ROI_right_lower.x = res_x;
        ROI_right_lower.y = res_y;
    }

//    if (interp==1)
//        segmentation_selection=1;
//    else
//        segmentation_selection=0;

    segmentation_selection = interp;
    interpolation_successive_frames = interp_succ;


    // Segmentation Settings
    last_found = 5;
    //thres = (float) 0.95;//0.958; // manually defined

    area_dim = 40;
    area_dim_l = 150;

    thres_intensity = 1; //20;
    size_struct_elem = 8; //8;

    num_of_int_steps=5;

    ch1_overlay = new Overlay(res_x,res_y,2,3);
    ch2_overlay = new Overlay(res_x,res_y,2,3);
    ch3_overlay = new Overlay(res_x,res_y,2,3);
    ch4_overlay = new Overlay(res_x,res_y,2,3);

    switch(ch_number) {
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

void Segmentation::startSegmentation(Mat frame, double lt_ch1, double lt_ch2, double lt_ch3, double lt_ch4)
{
    if (!firstFrameSet)
    {
        firstFrame = frame.clone();
        firstFrameSet = true;
    }

    double lifetime;
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

    log_lt_ch1.push_back(lt_ch1);
    log_lt_ch2.push_back(lt_ch2);
    log_lt_ch3.push_back(lt_ch3);
    log_lt_ch4.push_back(lt_ch4);

    //if ( overlay == NULL && lifetime>0)
    //{
    //    double lower_lim = floor(lifetime-0.5);
    //    if (lower_lim<0)
    //            lower_lim=0;

    //    overlay = new Overlay(res_x,res_y,lower_lim,ceil(lifetime+0.5)); //1-8
    //}
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
    //Size s = frame.size();
    //res_x = s.width;
    //res_y = s.height;

    int x, y, radius;
    float correlation;

    //if (last_found > 4)
    if (false)
    {
       // int xfrom = res_x/2-area_dim_l;
       // int yfrom = res_y/2-area_dim_l;
       //  int xto = res_x/2+area_dim_l;
       //  int yto = res_y/2+area_dim_l;

        Rect corrArea(ROI_left_upper.x, ROI_left_upper.y, ROI_right_lower.x-ROI_left_upper.x, ROI_right_lower.y-ROI_left_upper.y);
        Mat frame_cut = frame(corrArea);

        correlation = correlateGaussian(frame_cut, x, y, radius);

        //() << "Corr";
        //qDebug() << correlation;
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
        //if (corrArea.width<2*area_dim || corrArea.height<2*area_dim)
        //{
        //    last_found = 5;
        //    float alpha = 0.5;
        //    float beta = 0.5; //0.5; //( 1.0 - alpha );
        //    addWeighted( frame, alpha, overlay->getOverlay(), beta, 0.0, frame);
        //    return;
        //}
        Mat frame_cut = frame(corrArea);
        correlation = correlateGaussian(frame_cut, x, y, radius);

        if (correlation > thres)
        {
            x = x+xfrom-1;
            y = y+yfrom-1;
        }
    }
    char str[10];
    sprintf(str,"%f",correlation); // %f correlation
    if (correlation > thres)
    {

        if (log_lifetime_failed.size()>2 && segmentation_initialized==1 && segmentation_selection)
        {
            int dx = x-log_coords_x_failed;
            int dy = y-log_coords_y_failed;
            int dr = radius-log_radius_failed;
            int no_int_points = log_lifetime_failed.size();
            int loop_counter = 1;
            for (std::vector<double>::iterator it = log_lifetime_failed.begin() ; it != log_lifetime_failed.end(); ++it)
            {
                int x_new = (int) (log_coords_x_failed+( (float) (loop_counter*dx)/(float) no_int_points ));
                int y_new = (int) (log_coords_y_failed+( (float) (loop_counter*dy)/(float) no_int_points ));
                int rad_new = (int) (log_radius_failed+( (float) (loop_counter*dr)/(float) no_int_points ));
                loop_counter++;
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

        if (interpolation_successive_frames==1 && last_found==1 && (last_x-x)*(last_x-x)+(last_y-y)*(last_y-y)>25 )
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
                ch1_overlay->drawCircle(x_new,y_new,rad_new*0.5,lt_new1);
                ch2_overlay->drawCircle(x_new,y_new,rad_new*0.5,lt_new2);
                ch3_overlay->drawCircle(x_new,y_new,rad_new*0.5,lt_new3);
                ch4_overlay->drawCircle(x_new,y_new,rad_new*0.5,lt_new4);
            }
        }

        ch1_overlay->drawCircle(x,y,radius*0.5,lt_ch1);
        ch2_overlay->drawCircle(x,y,radius*0.5,lt_ch2);
        ch3_overlay->drawCircle(x,y,radius*0.5,lt_ch3);
        ch4_overlay->drawCircle(x,y,radius*0.5,lt_ch4);


        putText(frame, str, Point2f(100,100), FONT_HERSHEY_PLAIN, 2,  Scalar(255,0,0,255));

        log_coords_x.push_back(x);
        log_coords_y.push_back(y);
        log_radius.push_back(radius);
        last_found = 1;
        last_x = x;
        last_y = y;
        last_radius=radius;
        last_lt_ch1=lt_ch1;
        last_lt_ch2=lt_ch2;
        last_lt_ch3=lt_ch3;
        last_lt_ch4=lt_ch4;
    }
    else
    {
        last_vanish = 1;
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

        putText(frame, str, Point2f(100,100), FONT_HERSHEY_PLAIN, 2,  Scalar(0,0,255,255));
    }


    //imshow("activeDisplay", frame);
    //frame = overlay->RGBimage;
    //frame.at<uchar>(x,y)[1]=128;

    float alpha = 0.5;
    float beta = 0.5; //0.5; //( 1.0 - alpha );
    //if (overlay != NULL)
        addWeighted( frame, alpha, overlay->getOverlay(), beta, 0.0, frame);
    //addWeighted( frame, alpha, overlay->getColorBar(), beta, 0.0, frame);


    //imshow("activeDisplay", dst );
    return;
}

float Segmentation::correlateGaussian(cv::Mat frame, int &x, int &y, int &radius)
{

    Mat frame_lab;
    Mat intensity;
    cvtColor(frame, frame_lab, CV_BGR2Lab);

    extractChannel (frame_lab, intensity, 0 );
    extractChannel (frame_lab, frame_lab, 2 );

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
        matchTemplate(frame_lab, *gaussians[i], corr, CV_TM_CCORR_NORMED);//CV_TM_CCORR_NORMED
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


    //qDebug() << x;
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
