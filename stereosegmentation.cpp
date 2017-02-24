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
    seg = new Segmentation(frame, ROI_point1, ROI_point2, interp, ch_number, interp_succ, sc, autoscale); //TODO: pass autoscale here
    this->calib = sc;
    //current_channel = 0;

    Size s = frame.size();
    res_x = s.width;
    res_y = s.height;
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
        lower_bound_ = 3;
        upper_bound_ = 6;
    }
    // Initialize Overlays
    ch1_overlay = new Overlay(res_x,res_y,lower_bound_,upper_bound_, ansi); //TODO: Pass ansi limit here
    ch2_overlay = new Overlay(res_x,res_y,lower_bound_,upper_bound_, ansi);
    ch3_overlay = new Overlay(res_x,res_y,lower_bound_,upper_bound_, ansi);
    ch4_overlay = new Overlay(res_x,res_y,lower_bound_,upper_bound_, ansi);

    // initialize the overlays of all four channels
    x0 = seg->ROI_left_upper.x;
    y0 = seg->ROI_left_upper.y;
    x1 = seg->ROI_right_lower.x-seg->ROI_left_upper.x;
    y1 = seg->ROI_right_lower.y-seg->ROI_left_upper.y;
//qDebug() << "channel";
//qDebug() << ch_number;
    switchChannel(ch_number);
    //switchChannel(0);
}

void StereoSegmentation::startSegmentation(Mat frame_vis, Mat frame_on, Mat frame_off, Mat frame_on2, Mat frame_off2, double lt_ch1, double lt_ch2, double lt_ch3, double lt_ch4, int idx)
{
 //   qDebug() << "test";

    //qDebug() << "stereosegm1";

 //  const char * filename1 = "im_on1.jpg";
 //  cvSaveImage(filename1, &(IplImage(frame_on)));
 //  const char * filename2 = "im_on2.jpg";
 //  cvSaveImage(filename2, &(IplImage(frame_on2)));

    if (!firstFrameSet)
    {
        firstFrame = frame_vis.clone();
        firstFrameSet = true;
    }
    // do segmentation in the left videoframe
    //qDebug() << "stereosegm before segm";
    seg->startSegmentation(frame_vis, frame_on, frame_off, lt_ch1, lt_ch2, lt_ch3, lt_ch4, idx);
    //imshow("frameon", frame_on);
    //imshow("frameoff", frame_off);
//qDebug() << "stereosegm after segm";


    //const char * filename2 = "im_off.jpg";
    //cvSaveImage(filename2, &(IplImage(frame_off)));

    //seg left
    //ellipse(frame_vis, Point(seg->last_x,seg->last_y), Size(5,5), 0, 0, 360, Scalar( 0, 255, 0 ), 3, 8, 0);


    ellipse(frame_vis, calib->getRectifiedPoint(Point(seg->last_x,seg->last_y),0), Size(5,5), 0, 0, 360, Scalar( 0, 0, 255 ), 3, 8, 0);
//qDebug() << current_channel;
    double lifetime = 0;
    //current_channel = 0;
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
        overlay->drawCurrentVal(lifetime,current_channel);
    }

    // if beam is found in the left frame, search for it in the right frame
    // we assume that both frames are rectified (calibration required, see stereocalibration.cpp)
    //qDebug() << "before last active";
    //qDebug() << seg->last_active;
    if (seg->last_active)
    {
       // qDebug() << "stereosegm3";
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
//qDebug() << "lifetime limits adapted";


        int height = seg->last_radius*2+20; //last_radius*2+20;
        Point2d beam_pos(seg->last_x, seg->last_y);



        //Point2d beam_proj_l(beam_pos);
        //Point2d beam_proj_r(beam_pos);
        //beam_proj_l = calib->getRectifiedPoint(beam_proj_l,0);
        //beam_proj_l = calib->getRectifiedPoint(beam_proj_r,1);

       // ellipse(frame_vis, Point(beam_proj_l.x,beam_proj_l.y), Size(5,5), 0, 0, 360, Scalar( 0, 255, 0 ), 3, 8, 0);
        //ellipse(frame_vis, Point(beam_proj_r.x,beam_proj_r.y), Size(5,5), 0, 0, 360, Scalar( 0, 0, 255 ), 3, 8, 0);

        //beam_pos = calib->getRectifiedPoint(beam_pos,0);
//+ disparity_range
//
//qDebug() << "stereosegm4";
        // define ROI for finding the beam in the right videoframe
        int xfrom = (beam_pos.x - disparity_range < 0 ) ? 0  : beam_pos.x-disparity_range;
        int yfrom = (beam_pos.y - ceil(height/2)  < 0 ) ? 0  : beam_pos.y-ceil(height/2);
        int xto   = (beam_pos.x +disparity_range > frame_vis.cols-1 ) ? frame_vis.cols-1 : beam_pos.x+disparity_range;
        int yto   = (beam_pos.y + ceil(height/2)  > frame_vis.rows-1 ) ? frame_vis.rows-1 : beam_pos.y+ceil(height/2);

        Rect corrArea(xfrom, yfrom, xto-xfrom, yto-yfrom);

        log_synchronized.push_back(is_synchronized ? 1 : 0);
        double correlation = seg->pulsedSegmentation(frame_on2, frame_off2, corrArea, x, y, radius);






        // define ROI for side camera
        x0 = ((int) x)+xfrom-seg->area_dim;
        y0 = ((int) y)+yfrom-seg->area_dim;
        if (x0<0)
            x0=0;
        if (y0<0)
            y0=0;
        x1 = ((int) x) +xfrom+seg->area_dim;
        y1 = ((int) y) +yfrom+seg->area_dim;
        if (x1> frame_vis.cols-1)
            x1= frame_vis.cols-1;
        if (y0> frame_vis.rows-1)
            y0= frame_vis.rows-1;
        x1 = x1-x0;
        y1 = y1-y0;

        x0 = (x0 < 1) ? seg->ROI_left_upper.x : x0;
        x1 = (x0 + x1 > seg->ROI_right_lower.x ) ? seg->ROI_right_lower.x-x0 : x1;
        y0 = (y0 < 1) ? seg->ROI_left_upper.y : y0;
        y1 = (y0 + y1 > seg->ROI_right_lower.y) ? seg->ROI_right_lower.y-y0 : y1;

        Rect corrArea2(x0, y0, x1, y1);
//qDebug() << "active1";

        Point2d beam_pos_vis(calib->getRectifiedPoint(beam_pos,0));

        ch1_overlay->drawCircle(beam_pos_vis.x,beam_pos_vis.y,seg->last_radius*0.5,lt_ch1);
        ch2_overlay->drawCircle(beam_pos_vis.x,beam_pos_vis.y,seg->last_radius*0.5,lt_ch2);
        ch3_overlay->drawCircle(beam_pos_vis.x,beam_pos_vis.y,seg->last_radius*0.5,lt_ch3);
        ch4_overlay->drawCircle(beam_pos_vis.x,beam_pos_vis.y,seg->last_radius*0.5,lt_ch4);

        log_coords_x.push_back(beam_pos_vis.x);
        log_coords_y.push_back(beam_pos_vis.y);
        log_radius.push_back(seg->last_radius);

        if (correlation>(seg->thres))
        {
            //qDebug() << "active2";
            //qDebug() << "activate";

            // both beam positions are available

            //Point2d beam_proj_l(beam_pos);
            //beam_proj_l = calib->getRectifiedPoint(beam_proj_l,0);

            //seg right
            //ellipse(frame_vis, Point(x+xfrom,y+yfrom), Size(5,5), 0, 0, 360, Scalar( 255, 0, 0 ), 3, 8, 0);

            Mat cam0pnts(1,1,CV_64FC2, Scalar(beam_pos.x,beam_pos.y) );
            Mat cam1pnts(1,1,CV_64FC2, Scalar(x+xfrom,y+yfrom) );
            Mat res = calib->reconstructPoint3D(cam0pnts,cam1pnts); // 1 0

            // do stereo triangulation!
            // convert from homogenous to Eucledian coordinats
            double height = res.at<Vec4d>(0,0)[2] / res.at<Vec4d>(0,0)[3];


            //ellipse(frame_vis, beam_pos_vis, Size(5,5), 0, 0, 360, Scalar( 0, 255, 0 ), 3, 8, 0);

            // visualize height
            if (height_profile==NULL)
            {
                height_profile = new Overlay(seg->res_x,seg->res_y,height-0.1,height+0.1, 99999);
                if (current_channel==0)
                    switchChannel(0);
            }
            if (current_channel==0)
                overlay->drawCurrentVal(height, current_channel);
       //     if (height>height_profile->getUpperBound())
       //         height_profile->setNewInterval(height_profile->getLowerBound(),height+0.05);

       //     if (height<height_profile->getLowerBound())
       //         height_profile->setNewInterval(height-0.05,height_profile->getUpperBound());

            //qDebug() << "Profile Overlay";
            //qDebug() << height;
            //qDebug() << "stereosegm7";
            height_profile->drawCircle(beam_pos_vis.x,beam_pos_vis.y,seg->last_radius*0.5,height);


            log_real_x.push_back(res.at<Vec4d>(0,0)[0] / res.at<Vec4d>(0,0)[3]);
            log_real_y.push_back(res.at<Vec4d>(0,0)[1] / res.at<Vec4d>(0,0)[3]);

            log_height.push_back(height);
            log_disparity_y.push_back(beam_pos.y - y - yfrom);
        }
        else
        {

            //qDebug() << "stereosegm8";
            log_height.push_back(0);
            log_real_x.push_back(0);
            log_real_y.push_back(0);
            log_disparity_y.push_back(100);
        }
    }
    else
    {
       // qDebug() << "stereosegm9";
        log_synchronized.push_back(0);
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
//qDebug() << "stereosegm10";
    if (height_profile!=NULL)
    {
        float alpha = 0.5;
        float beta = 0.5;
        addWeighted( frame_vis, alpha, overlay->mergeOverlay(frame_vis), beta, 0.0, frame_vis);
        //addWeighted( frame_vis, alpha, ch1_overlay->mergeOverlay(frame_vis), beta, 0.0, frame_vis); //frame_vis
//qDebug() << "stereosegm11";
    }
    //qDebug() << "stereosegm12";
}

void StereoSegmentation::setThreshold(double thres)
{
    seg->setThreshold(thres);
}

void StereoSegmentation::switchChannel(int channel)
{
    //qDebug() << "in_set";
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
        //qDebug() << "set";
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
    this->is_synchronized = is_synchronized;
}
