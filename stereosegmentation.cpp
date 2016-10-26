#include "stereosegmentation.h"
#include "stereocalibration.h"
#include "segmentation.h"
#include <QDebug>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <QMessageBox>

using namespace cv;

StereoSegmentation::StereoSegmentation(StereoCalibration * sc, cv::Mat frame, cv::Point ROI_point1, cv::Point ROI_point2, bool interp, int ch_number, bool interp_succ)
{
    seg = new Segmentation(frame, ROI_point1, ROI_point2, interp, ch_number, interp_succ, sc);
    this->calib = sc;

    Size s = frame.size();
    res_x = s.width;
    res_y = s.height;
    ch1_overlay = new Overlay(res_x,res_y,2,3);
    ch2_overlay = new Overlay(res_x,res_y,2,3);
    ch3_overlay = new Overlay(res_x,res_y,2,3);
    ch4_overlay = new Overlay(res_x,res_y,2,3);

    switchChannel(ch_number);
}

void StereoSegmentation::startSegmentation(Mat frame_l, Mat frame_r, Mat frame_vis, double lt_ch1, double lt_ch2, double lt_ch3, double lt_ch4)
{

    if (!firstFrameSet)
    {
        firstFrame = frame_vis.clone();
        firstFrameSet = true;
    }
    // do segmentation in the left videoframe
    //seg->startSegmentation(frame_l, lt_ch1, lt_ch2, lt_ch3, lt_ch4);

    ellipse(frame_vis, calib->getRectifiedPoint(Point(seg->last_x,seg->last_y),0), Size(5,5), 0, 0, 360, Scalar( 0, 0, 255 ), 3, 8, 0);

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
    overlay->drawCurrentVal(lifetime,current_channel);
    // if beam is found in the left frame, search for it in the right frame
    // we assume that both frames are rectified (calibration required, see stereocalibration.cpp)
    if (seg->last_active)
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


        int height = seg->last_radius*2+20; //*1.5;
        Point2d beam_pos(seg->last_x, seg->last_y);

        //Point2d beam_proj_l(beam_pos);
        //beam_proj_l = calib->getRectifiedPoint(beam_proj_l,0);

        //ellipse(frame_vis, Point(beam_proj_l.x,beam_proj_l.y), Size(5,5), 0, 0, 360, Scalar( 0, 255, 0 ), 3, 8, 0);

        //beam_pos = calib->getRectifiedPoint(beam_pos,0);
//+ disparity_range
//

        // define ROI for finding the beam in the right videoframe
        int xfrom = (beam_pos.x - disparity_range < 0 ) ? 0  : beam_pos.x-disparity_range;
        int yfrom = (beam_pos.y - ceil(height/2)  < 0 ) ? 0  : beam_pos.y-ceil(height/2);
        int xto   = (beam_pos.x  > frame_l.cols-1 ) ? frame_l.cols-1 : beam_pos.x;
        int yto   = (beam_pos.y + ceil(height/2)  > frame_l.rows-1 ) ? frame_l.rows-1 : beam_pos.y+ceil(height/2);
        Rect corrArea(xfrom, yfrom, xto-xfrom, yto-yfrom);

        // cut correlation area from right frame
        Mat frame_r_cut = frame_r(corrArea);
        int x, y, radius;

        //const char * filename1 = "segment.jpg";
        //cvSaveImage(filename1, &(IplImage(frame_r_cut)));


        double correlation = seg->doubleRingSegmentation(frame_r_cut, x, y, radius);

        qDebug() << lifetime;
        qDebug() << correlation;
        if (correlation>(seg->thres))
        {

            // both beam positions are available

            //Point2d beam_proj_l(beam_pos);
            //beam_proj_l = calib->getRectifiedPoint(beam_proj_l,0);

            Mat cam0pnts(1,1,CV_64FC2, Scalar(beam_pos.x,beam_pos.y) );
            Mat cam1pnts(1,1,CV_64FC2, Scalar(x+xfrom,y+yfrom) );
            Mat res = calib->reconstructPoint3D(cam0pnts,cam1pnts); // 1 0

            // do stereo triangulation!

            //ellipse(frame_vis, beam_pos, Size(5,5), 0, 0, 360, Scalar( 0, 0, 255 ), 3, 8, 0);
            //ellipse(frame_r, Point(x+xfrom,y+yfrom), Size(5,5), 0, 0, 360, Scalar( 255, 255, 255 ), 3, 8, 0);

            //beam_proj = calib->getRectifiedPoint(beam_proj,1);
            //ellipse(frame_vis, Point(x+xfrom,y+yfrom), Size(5,5), 0, 0, 360, Scalar( 255, 0, 0 ), 3, 8, 0);

            // convert from homogenous to Eucledian coordinats
            double height = res.at<Vec4d>(0,0)[2] / res.at<Vec4d>(0,0)[3];

            if (current_channel==0)
                overlay->drawCurrentVal(height,current_channel);

            Point2d beam_pos_vis(calib->getRectifiedPoint(beam_pos,0));
            //ellipse(frame_vis, beam_pos_vis, Size(5,5), 0, 0, 360, Scalar( 0, 255, 0 ), 3, 8, 0);

            // visualize height
            if (height_profile==NULL)
                height_profile = new Overlay(seg->res_x,seg->res_y,height-0.5,height+0.5);

            if (height>height_profile->getUpperBound())
                height_profile->setNewInterval(height_profile->getLowerBound(),height+0.05);

            if (height<height_profile->getLowerBound())
                height_profile->setNewInterval(height-0.05,height_profile->getUpperBound());

            qDebug() << "Profile Overlay";
            height_profile->drawCircle(beam_pos_vis.x,beam_pos_vis.y,seg->last_radius,height);
            ch1_overlay->drawCircle(beam_pos_vis.x,beam_pos_vis.y,seg->last_radius,lt_ch1);
            ch2_overlay->drawCircle(beam_pos_vis.x,beam_pos_vis.y,seg->last_radius,lt_ch2);
            ch3_overlay->drawCircle(beam_pos_vis.x,beam_pos_vis.y,seg->last_radius,lt_ch3);
            ch4_overlay->drawCircle(beam_pos_vis.x,beam_pos_vis.y,seg->last_radius,lt_ch4);

            log_height.push_back(height);
            log_real_x.push_back(res.at<Vec4d>(0,0)[0] / res.at<Vec4d>(0,0)[3]);
            log_real_y.push_back(res.at<Vec4d>(0,0)[1] / res.at<Vec4d>(0,0)[3]);
            log_coords_x.push_back(beam_pos_vis.x);
            log_coords_y.push_back(beam_pos_vis.y);
            log_radius.push_back(seg->last_radius);
        }
        else
        {
            log_height.push_back(0);
            log_real_x.push_back(0);
            log_real_y.push_back(0);
            log_coords_x.push_back(0);
            log_coords_y.push_back(0);
            log_radius.push_back(0);
        }
    }
    else
    {
        log_height.push_back(0);
        log_real_x.push_back(0);
        log_real_y.push_back(0);
        log_coords_x.push_back(0);
        log_coords_y.push_back(0);
        log_radius.push_back(0);
    }

    //Point2d beam_proj(seg->last_x,seg->last_y);
    //beam_proj = calib->getRectifiedPoint(beam_proj,1);

    //ellipse(frame_vis, beam_proj, Size(5,5), 0, 0, 360, Scalar( 0, 0, 255 ), 3, 8, 0);
    //frame_vis = frame_r.clone();

    if (height_profile!=NULL)
    {
        float alpha = 0.5;
        float beta = 0.5;
        addWeighted( frame_vis, alpha, overlay->mergeOverlay(frame_vis), beta, 0.0, frame_vis);
        //addWeighted( frame_vis, alpha, ch1_overlay->mergeOverlay(frame_vis), beta, 0.0, frame_vis); //frame_vis

    }
}

void StereoSegmentation::setThreshold(double thres)
{
    seg->setThreshold(thres);
}

void StereoSegmentation::switchChannel(int channel)
{
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
        overlay = height_profile;
}
