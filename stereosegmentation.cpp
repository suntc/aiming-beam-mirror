#include "stereosegmentation.h"
#include "stereocalibration.h"
#include "segmentation.h"
#include <QDebug>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <QMessageBox>

using namespace cv;

StereoSegmentation::StereoSegmentation(StereoCalibration * sc, cv::Mat frame, cv::Point ROI_point1, cv::Point ROI_point2, bool interp, int ch_number, bool interp_succ, bool autoscale, int ansi)
{
    // StereoSegmentation acts as a wrapper
    seg = new Segmentation(frame, ROI_point1, ROI_point2, interp, ch_number, interp_succ, sc, autoscale, false);

    // save calibration pointer
    this->calib = sc;

    // set size
    Size s = frame.size();
    res_x = s.width;
    res_y = s.height;

    this->scale_auto=autoscale;
    double lower_bound_;
    double upper_bound_;

    // if autoscale, start with a small bound
    if (autoscale==true)
    {
        lower_bound_ = 2;
        upper_bound_ = 3;
    }
    else
    {
        lower_bound_ = 3;
        upper_bound_ = 6;
    }

    // Initialize Overlays
    ch1_overlay = new Overlay(res_x,res_y,lower_bound_,upper_bound_, ansi);
    ch2_overlay = new Overlay(res_x,res_y,lower_bound_,upper_bound_, ansi);
    ch3_overlay = new Overlay(res_x,res_y,lower_bound_,upper_bound_, ansi);
    ch4_overlay = new Overlay(res_x,res_y,lower_bound_,upper_bound_, ansi);

    // initialize the overlays of all four channels
    x0 = seg->ROI_left_upper.x;
    y0 = seg->ROI_left_upper.y;
    x1 = seg->ROI_right_lower.x-seg->ROI_left_upper.x;
    y1 = seg->ROI_right_lower.y-seg->ROI_left_upper.y;

    // set the overlay pointer to the current channel
    switchChannel(ch_number);
}

void StereoSegmentation::startSegmentation(Mat frame_vis, Mat frame_on, Mat frame_off, Mat frame_on2, Mat frame_off2, double lt_ch1, double lt_ch2, double lt_ch3, double lt_ch4, int idx)
{
    // make a copy of the first frame used for the image export
    if (!firstFrameSet)
    {
        firstFrame = frame_vis.clone();
        firstFrameSet = true;
    }

    // start segmentation
    seg->startSegmentation(frame_vis, frame_on, frame_off, lt_ch1, lt_ch2, lt_ch3, lt_ch4, idx);

    // show marker to be picked up
    // getRectifiedPoint maps from the rectified left camera image (where segmentation is done) back to the original camera image
    ellipse(frame_vis, calib->getRectifiedPoint(Point(seg->last_x,seg->last_y),0), Size(5,5), 0, 0, 360, Scalar( 0, 0, 255 ), 3, 8, 0);

    // get lifetime of current channel
    double lifetime = 0;
    if (current_channel!=0)
    {
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
        // overlay lifetime
        overlay->drawCurrentVal(lifetime,current_channel);
    }

    // Segment beam in rectified left camera image
    if (seg->last_active)
    {

        // Adapt scale bar if necessary
        if (scale_auto)
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

        // if beam is found in the left frame, search for it in the right frame
        // define a narrow ROI where we are looking for the beam in the rectified right camera image
        int height = seg->last_radius*2+20; //last_radius*2+20;

        // beam position in the rectified left camera image
        Point2d beam_pos(seg->last_x, seg->last_y);

        // narrow ROI
        int xfrom = (beam_pos.x - disparity_range < 0 ) ? 0  : beam_pos.x-disparity_range;
        int yfrom = (beam_pos.y - ceil(height/2)  < 0 ) ? 0  : beam_pos.y-ceil(height/2);
        int xto   = (beam_pos.x + disparity_range > frame_vis.cols-1 ) ? frame_vis.cols-1 : beam_pos.x+disparity_range;
        int yto   = (beam_pos.y + ceil(height/2)  > frame_vis.rows-1 ) ? frame_vis.rows-1 : beam_pos.y+ceil(height/2);

        Rect corrArea(xfrom, yfrom, xto-xfrom, yto-yfrom);

        log_synchronized.push_back(is_synchronized ? 1 : 0);

        // segment beam in rectified right camera image
        double correlation = seg->pulsedSegmentation(frame_on2, frame_off2, corrArea, x, y, radius);

        // define ROI for side camera
        int x_0, x_1, y_0, y_1;
        x_0 = ((int) x)+xfrom-seg->area_dim;
        y_0 = ((int) y)+yfrom-seg->area_dim;
        if (x_0<0)
            x_0=0;
        if (y_0<0)
            y_0=0;
        x_1 = ((int) x) +xfrom+seg->area_dim;
        y_1 = ((int) y) +yfrom+seg->area_dim;
        if (x_1> frame_vis.cols-1)
            x_1= frame_vis.cols-1;
        if (y_0> frame_vis.rows-1)
            y_0= frame_vis.rows-1;
        x_1 = x_1-x_0;
        y_1 = y_1-y_0;

        x0 = (x_0 < 1) ? seg->ROI_left_upper.x : x_0;
        x1 = (x_0 + x_1 > seg->ROI_right_lower.x ) ? seg->ROI_right_lower.x-x_0 : x_1;
        y0 = (y_0 < 1) ? seg->ROI_left_upper.y : y_0;
        y1 = (y_0 + y_1 > seg->ROI_right_lower.y) ? seg->ROI_right_lower.y-y_0 : y_1;

        // get beam in unrectified camera image
        Point2d beam_pos_vis(calib->getRectifiedPoint(beam_pos,0));

        // update overlays
        ch1_overlay->drawCircle(beam_pos_vis.x,beam_pos_vis.y,seg->last_radius*0.5,lt_ch1);
        ch2_overlay->drawCircle(beam_pos_vis.x,beam_pos_vis.y,seg->last_radius*0.5,lt_ch2);
        ch3_overlay->drawCircle(beam_pos_vis.x,beam_pos_vis.y,seg->last_radius*0.5,lt_ch3);
        ch4_overlay->drawCircle(beam_pos_vis.x,beam_pos_vis.y,seg->last_radius*0.5,lt_ch4);

        // log coordinates (even if the right beam segmentation fails)
        log_coords_x.push_back(beam_pos_vis.x);
        log_coords_y.push_back(beam_pos_vis.y);
        log_radius.push_back(seg->last_radius);

        if (correlation>(seg->thres))
        {
            // if beam found in right camera image do triangulation
            Mat cam0pnts(1,1,CV_64FC2, Scalar(beam_pos.x,beam_pos.y) );
            Mat cam1pnts(1,1,CV_64FC2, Scalar(x+xfrom,y+yfrom) );
            Mat res = calib->reconstructPoint3D(cam0pnts,cam1pnts);

            // convert from homogenous to Eucledian coordinats
            double height = res.at<Vec4d>(0,0)[2] / res.at<Vec4d>(0,0)[3];

            // update profile overlay (currently not used)
            if (height_profile==NULL)
            {
                height_profile = new Overlay(seg->res_x,seg->res_y,height-0.1,height+0.1, 99999);
                if (current_channel==0)
                    switchChannel(0);
            }
            if (current_channel==0)
                overlay->drawCurrentVal(height, current_channel);

            height_profile->drawCircle(beam_pos_vis.x,beam_pos_vis.y,seg->last_radius*0.5,height);

            // x-, y-coordinates in 3D coordinate system (in inches)
            log_real_x.push_back(res.at<Vec4d>(0,0)[0] / res.at<Vec4d>(0,0)[3]);
            log_real_y.push_back(res.at<Vec4d>(0,0)[1] / res.at<Vec4d>(0,0)[3]);

            // log height and vertical disparity (should be small, depends on calibration)
            log_height.push_back(height);
            log_disparity_y.push_back(beam_pos.y - y - yfrom);
        }
        else
        {
            // if right beam was not found, log dummy variables
            log_height.push_back(0);
            log_real_x.push_back(0);
            log_real_y.push_back(0);
            log_disparity_y.push_back(100);
        }
    }
    else
    {
        // if left beam was not segmented, log dummy variables
        log_synchronized.push_back(0);
        log_height.push_back(0);
        log_real_x.push_back(0);
        log_real_y.push_back(0);
        log_coords_x.push_back(0);
        log_coords_y.push_back(0);
        log_radius.push_back(0);
    }

    // update overlay
    if (height_profile!=NULL)
    {
        float alpha = 0.5;
        float beta = 0.5;
        addWeighted( frame_vis, alpha, overlay->mergeOverlay(frame_vis), beta, 0.0, frame_vis);
    }
}

void StereoSegmentation::setThreshold(double thres)
{
    seg->setThreshold(thres);
}

void StereoSegmentation::switchChannel(int channel)
{
    // update overlay pointer
    current_channel = channel;
    if(channel==1)
        overlay = ch1_overlay;
    if(channel==2)
        overlay = ch2_overlay;
    if(channel==3)
        overlay = ch3_overlay;
    if(channel==4)
        overlay = ch4_overlay;
    if(channel==0)
    {
        overlay = height_profile;
    }

}

void StereoSegmentation::setAnsi(int ansi)
{
    ch1_overlay->setAnsi(ansi);
    ch2_overlay->setAnsi(ansi);
    ch3_overlay->setAnsi(ansi);
    ch4_overlay->setAnsi(ansi);
    if (height_profile) height_profile->setAnsi(ansi);
}

StereoSegmentation::~StereoSegmentation()
{
    delete ch1_overlay;
    delete ch2_overlay;
    delete ch3_overlay;
    delete ch4_overlay;
    delete height_profile;
    delete overlay;
}

void StereoSegmentation::setColorScale(double mn, double mx)
{
    // set interval for all channels
    ch1_overlay->setNewInterval(mn, mx);
    ch2_overlay->setNewInterval(mn, mx);
    ch3_overlay->setNewInterval(mn, mx);
    ch4_overlay->setNewInterval(mn, mx);
}

void StereoSegmentation::setAutoScale(bool autoscale)
{
    scale_auto = autoscale;
}

void StereoSegmentation::set_synchronized( bool is_synchronized)
{
    // is_synchronized indicates if the last images (of left and right cam) were in the same state (on or off)
    // for testing purposes only: being in a different state could lower the reconstruction precision
    this->is_synchronized = is_synchronized;
}
