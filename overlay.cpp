#include "overlay.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <fstream>
#include <iostream>
#include <QMessageBox>
#include <QDebug>
#include "stereocalibration.h"

using namespace cv;
using namespace std;

Overlay::Overlay(int size_x, int size_y, double scale_mn, double scale_mx)
{
    //qDebug() << "overlay mono";
    this->stereo_mode = false;
    init(size_x, size_y, scale_mn, scale_mx);
}

Overlay::Overlay(int size_x, int size_y, double scale_mn, double scale_mx, StereoCalibration * calib)
{
    //qDebug() << "overlay stereo";
    this->stereo_mode = true;
    this->calib = calib;
    init(size_x, size_y, scale_mn, scale_mx);
}

void Overlay::init(int size_x, int size_y, double scale_mn, double scale_mx)
{
    RGBimage = Mat(size_y, size_x, CV_8UC3, Scalar(0, 0, 0));
    color_bar = Mat(size_y, size_x, CV_8UC3, Scalar(0, 0, 0));
    accumulator = Mat(size_y, size_x, CV_16UC1, Scalar(0));
    values = Mat(size_y, size_x, CV_32F, Scalar(0));

    // initialize color map
    float r1[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.00588235294117645f,0.02156862745098032f,0.03725490196078418f,0.05294117647058827f,0.06862745098039214f,0.084313725490196f,0.1000000000000001f,0.115686274509804f,0.1313725490196078f,0.1470588235294117f,0.1627450980392156f,0.1784313725490196f,0.1941176470588235f,0.2098039215686274f,0.2254901960784315f,0.2411764705882353f,0.2568627450980392f,0.2725490196078431f,0.2882352941176469f,0.303921568627451f,0.3196078431372549f,0.3352941176470587f,0.3509803921568628f,0.3666666666666667f,0.3823529411764706f,0.3980392156862744f,0.4137254901960783f,0.4294117647058824f,0.4450980392156862f,0.4607843137254901f,0.4764705882352942f,0.4921568627450981f,0.5078431372549019f,0.5235294117647058f,0.5392156862745097f,0.5549019607843135f,0.5705882352941174f,0.5862745098039217f,0.6019607843137256f,0.6176470588235294f,0.6333333333333333f,0.6490196078431372f,0.664705882352941f,0.6803921568627449f,0.6960784313725492f,0.7117647058823531f,0.7274509803921569f,0.7431372549019608f,0.7588235294117647f,0.7745098039215685f,0.7901960784313724f,0.8058823529411763f,0.8215686274509801f,0.8372549019607844f,0.8529411764705883f,0.8686274509803922f,0.884313725490196f,0.8999999999999999f,0.9156862745098038f,0.9313725490196076f,0.947058823529412f,0.9627450980392158f,0.9784313725490197f,0.9941176470588236f,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0.9862745098039216f,0.9705882352941178f,0.9549019607843139f,0.93921568627451f,0.9235294117647062f,0.9078431372549018f,0.892156862745098f,0.8764705882352941f,0.8607843137254902f,0.8450980392156864f,0.8294117647058825f,0.8137254901960786f,0.7980392156862743f,0.7823529411764705f,0.7666666666666666f,0.7509803921568627f,0.7352941176470589f,0.719607843137255f,0.7039215686274511f,0.6882352941176473f,0.6725490196078434f,0.6568627450980391f,0.6411764705882352f,0.6254901960784314f,0.6098039215686275f,0.5941176470588236f,0.5784313725490198f,0.5627450980392159f,0.5470588235294116f,0.5313725490196077f,0.5156862745098039f,0.5f};
    float g1[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.001960784313725483f,0.01764705882352935f,0.03333333333333333f,0.0490196078431373f,0.06470588235294117f,0.08039215686274503f,0.09607843137254901f,0.111764705882353f,0.1274509803921569f,0.1431372549019607f,0.1588235294117647f,0.1745098039215687f,0.1901960784313725f,0.2058823529411764f,0.2215686274509804f,0.2372549019607844f,0.2529411764705882f,0.2686274509803921f,0.2843137254901961f,0.3f,0.3156862745098039f,0.3313725490196078f,0.3470588235294118f,0.3627450980392157f,0.3784313725490196f,0.3941176470588235f,0.4098039215686274f,0.4254901960784314f,0.4411764705882353f,0.4568627450980391f,0.4725490196078431f,0.4882352941176471f,0.503921568627451f,0.5196078431372548f,0.5352941176470587f,0.5509803921568628f,0.5666666666666667f,0.5823529411764705f,0.5980392156862746f,0.6137254901960785f,0.6294117647058823f,0.6450980392156862f,0.6607843137254901f,0.6764705882352942f,0.692156862745098f,0.7078431372549019f,0.723529411764706f,0.7392156862745098f,0.7549019607843137f,0.7705882352941176f,0.7862745098039214f,0.8019607843137255f,0.8176470588235294f,0.8333333333333333f,0.8490196078431373f,0.8647058823529412f,0.8803921568627451f,0.8960784313725489f,0.9117647058823528f,0.9274509803921569f,0.9431372549019608f,0.9588235294117646f,0.9745098039215687f,0.9901960784313726f,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0.9901960784313726f,0.9745098039215687f,0.9588235294117649f,0.943137254901961f,0.9274509803921571f,0.9117647058823528f,0.8960784313725489f,0.8803921568627451f,0.8647058823529412f,0.8490196078431373f,0.8333333333333335f,0.8176470588235296f,0.8019607843137253f,0.7862745098039214f,0.7705882352941176f,0.7549019607843137f,0.7392156862745098f,0.723529411764706f,0.7078431372549021f,0.6921568627450982f,0.6764705882352944f,0.6607843137254901f,0.6450980392156862f,0.6294117647058823f,0.6137254901960785f,0.5980392156862746f,0.5823529411764707f,0.5666666666666669f,0.5509803921568626f,0.5352941176470587f,0.5196078431372548f,0.503921568627451f,0.4882352941176471f,0.4725490196078432f,0.4568627450980394f,0.4411764705882355f,0.4254901960784316f,0.4098039215686273f,0.3941176470588235f,0.3784313725490196f,0.3627450980392157f,0.3470588235294119f,0.331372549019608f,0.3156862745098041f,0.2999999999999998f,0.284313725490196f,0.2686274509803921f,0.2529411764705882f,0.2372549019607844f,0.2215686274509805f,0.2058823529411766f,0.1901960784313728f,0.1745098039215689f,0.1588235294117646f,0.1431372549019607f,0.1274509803921569f,0.111764705882353f,0.09607843137254912f,0.08039215686274526f,0.06470588235294139f,0.04901960784313708f,0.03333333333333321f,0.01764705882352935f,0.001960784313725483f,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    float b1[] = {0.5f,0.5156862745098039f,0.5313725490196078f,0.5470588235294118f,0.5627450980392157f,0.5784313725490196f,0.5941176470588235f,0.6098039215686275f,0.6254901960784314f,0.6411764705882352f,0.6568627450980392f,0.6725490196078432f,0.6882352941176471f,0.7039215686274509f,0.7196078431372549f,0.7352941176470589f,0.7509803921568627f,0.7666666666666666f,0.7823529411764706f,0.7980392156862746f,0.8137254901960784f,0.8294117647058823f,0.8450980392156863f,0.8607843137254902f,0.8764705882352941f,0.892156862745098f,0.907843137254902f,0.9235294117647059f,0.9392156862745098f,0.9549019607843137f,0.9705882352941176f,0.9862745098039216f,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0.9941176470588236f,0.9784313725490197f,0.9627450980392158f,0.9470588235294117f,0.9313725490196079f,0.915686274509804f,0.8999999999999999f,0.884313725490196f,0.8686274509803922f,0.8529411764705883f,0.8372549019607844f,0.8215686274509804f,0.8058823529411765f,0.7901960784313726f,0.7745098039215685f,0.7588235294117647f,0.7431372549019608f,0.7274509803921569f,0.7117647058823531f,0.696078431372549f,0.6803921568627451f,0.6647058823529413f,0.6490196078431372f,0.6333333333333333f,0.6176470588235294f,0.6019607843137256f,0.5862745098039217f,0.5705882352941176f,0.5549019607843138f,0.5392156862745099f,0.5235294117647058f,0.5078431372549019f,0.4921568627450981f,0.4764705882352942f,0.4607843137254903f,0.4450980392156865f,0.4294117647058826f,0.4137254901960783f,0.3980392156862744f,0.3823529411764706f,0.3666666666666667f,0.3509803921568628f,0.335294117647059f,0.3196078431372551f,0.3039215686274508f,0.2882352941176469f,0.2725490196078431f,0.2568627450980392f,0.2411764705882353f,0.2254901960784315f,0.2098039215686276f,0.1941176470588237f,0.1784313725490199f,0.1627450980392156f,0.1470588235294117f,0.1313725490196078f,0.115686274509804f,0.1000000000000001f,0.08431372549019622f,0.06862745098039236f,0.05294117647058805f,0.03725490196078418f,0.02156862745098032f,0.00588235294117645f,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    r = new vector<float> ( r1, r1 + sizeof(r1) / sizeof(int) );
    g = new vector<float> ( g1, g1 + sizeof(g1) / sizeof(int) );
    b = new vector<float> ( b1, b1 + sizeof(b1) / sizeof(int) );
    //qDebug() << "def";

    // determine interval
    scale_max = scale_mx;
    scale_min = scale_mn;


    drawColorBar();
}

Mat Overlay::getOverlay()
{
    return RGBimage;
}

Mat Overlay::mergeOverlay(Mat frame)
{
    Mat mask;
    Mat res = frame.clone();

    cvtColor(RGBimage,mask,CV_BGR2GRAY);
    threshold(mask, mask, 1, 1, 0);

    RGBimage.copyTo(res,mask);

    mask.release();
    return res;
}

Mat Overlay::getColorBar()
{
    return color_bar;
}

double Overlay::getLowerBound()
{
    return scale_min;
}

double Overlay::getUpperBound()
{
    return scale_max;
}

Mat Overlay::getAccumulator()
{
    return accumulator;
}

void Overlay::setNewInterval(double mn, double mx)
{
    scale_max = mx;
    scale_min = mn;

    RGBimage.setTo(Scalar(0, 0, 0));


    int val0, r2, g2, b2;
    for (int x=0; x<values.rows; x++)
        for (int y=0; y<values.cols; y++)
        {
            if (values.at<float>(x,y)<=0.01)
                    continue;
            val0 = (int) ( (values.at<float>(x,y)-scale_min)/(scale_max-scale_min)*255 );
            r2 = (int) (r->at(val0)*255);
            g2 = (int) (g->at(val0)*255);
            b2 = (int) (b->at(val0)*255);
            RGBimage.at<Vec3b>(x,y)[0] = b2;
            RGBimage.at<Vec3b>(x,y)[1] = g2;
            RGBimage.at<Vec3b>(x,y)[2] = r2;
        }
    drawColorBar();
}

void Overlay::drawCircle(int x, int y, int radius, double val)
{
    // highly efficient overlay computation :)

    //qDebug() << stereo_mode;
    // if stereo is enabled, map back to visual image
    if (stereo_mode)
    {
        //qDebug() << "should never go here";
        //qDebug() << x;
    }
    else
    {
        //qDebug() << "stereo off";
        //qDebug() << x;
    }


    // make sure that the beam is inside the overlay
    if (x-radius<0 || y-radius<0 || x+radius+1>accumulator.cols || y+radius+1>accumulator.rows)
        return;


    Point pt1;
    pt1.x = radius;
    pt1.y = radius;


    // cut segment from accumulator
    Rect segmArea(x-radius, y-radius, 2*radius+1, 2*radius+1);
    Mat segm_acc = accumulator(segmArea);

    // define circle in segment
    Mat tmp = Mat(2*radius+1,2*radius+1, CV_16UC1, Scalar(0));
    circle(tmp, pt1, radius, 1, -1);

    // update accumulator
    Mat segm_acc2;
    segm_acc2 = segm_acc + tmp;

    // cut area from lifetime values
    Mat segm_val = values(segmArea);

    // convert accumulator segment to float
    Mat acc_pre;
    Mat acc_post;
    Mat binary_segm;
    segm_acc.convertTo(acc_pre, CV_32F);
    segm_acc2.convertTo(acc_post, CV_32F);
    tmp.convertTo(binary_segm, CV_32F);

    // update values
    Mat const_one(binary_segm.size(), CV_32F, Scalar::all(1));
    Mat tmp1 = (segm_val.mul(acc_pre) + binary_segm*val ); //binary_segm.mul
            //+ (segm_val.mul(const_one-binary_segm)); //*segm_val


    try
    {
        divide(tmp1, acc_post, tmp1);
    }
    catch(int e)
    {
        //qDebug() << "Overlay Update Failed!";
        return;
    }
    if (val<scale_min)
            return;

    if (val>scale_max)
            return;
    //tmp1 = binary_segm.mul(tmp1) + segm_val.mul(const_one-binary_segm);

    // copy values back to frame and accumulator
    tmp1.copyTo( values( segmArea ) );
    segm_acc2.copyTo( accumulator( segmArea ) );

    //Mat dst;
    //normalize(values, dst, 0, 1, cv::NORM_MINMAX);





    // update color image
    Mat rgb_segm = RGBimage(segmArea);

    int val0, r2, g2, b2;
    for (int x=0; x<2*radius+1; x++)
    {

        for (int y=0; y<2*radius+1; y++)
        {
            if (tmp1.at<float>(x,y)<0.001)
                    continue;
            val0 = (int) ( (tmp1.at<float>(x,y)-scale_min)/(scale_max-scale_min)*255 );
            r2 = (int) (r->at(val0)*255);
            g2 = (int) (g->at(val0)*255);
            b2 = (int) (b->at(val0)*255);
            rgb_segm.at<Vec3b>(x,y)[0] = b2;
            rgb_segm.at<Vec3b>(x,y)[1] = g2;
            rgb_segm.at<Vec3b>(x,y)[2] = r2;
        }

    }


    //imshow("activeDisplay", RGBimage);

    return;
}

void Overlay::drawColorBar()
{
    int y_from = 100;
    int y_to = values.rows-100;

    int x_from = values.cols-80;
    int x_to = values.cols-40;


    color_bar.setTo(Scalar(0, 0, 0));
    int r2,g2,b2,val0;

    for (int x=y_from; x<=y_to; x++)
        for (int y=x_from; y<x_to; y++)
        {
            val0 = 255- (int) ( (float) (x-y_from)/ (float) (y_to-y_from)*255 );
            r2 = (int) (r->at(val0)*255);
            g2 = (int) (g->at(val0)*255);
            b2 = (int) (b->at(val0)*255);
            RGBimage.at<Vec3b>(x,y)[0] = b2;
            RGBimage.at<Vec3b>(x,y)[1] = g2;
            RGBimage.at<Vec3b>(x,y)[2] = r2;
        }

    sprintf(str_mx1,"%2.2f",scale_max);
    putText(RGBimage, str_mx1, Point2f(x_from-20,y_from-25), FONT_HERSHEY_PLAIN, 2,  Scalar(0,0,255,255), 2);

    sprintf(str_mn1,"%2.2f",scale_min);
    putText(RGBimage, str_mn1, Point2f(x_from-20,y_to+50),   FONT_HERSHEY_PLAIN, 2,  Scalar(0,0,255,255), 2);

}

void Overlay::drawCurrentVal(double val, int channel)
{
    int fontFace = FONT_HERSHEY_PLAIN;
    int fontScale = 2;
    int thickness = 3;

    int y_from = values.rows-60;
    int y_to = values.rows-20;

    int x_from = values.cols/2-60;
    int x_to = values.cols/2+60;

    int r0, g0, b0;
    int r1, g1, b1;

    if (val>=scale_min && val<=scale_max)
    {
        int val0 = (int) ( (val-scale_min)/(scale_max-scale_min)*255);

        r0 = (int) (r->at(val0)*255);
        g0 = (int) (g->at(val0)*255);
        b0 = (int) (b->at(val0)*255);
    }
    else
    {
        r0 = 0; g0 = 0; b0 = 0;
    }

    if (r0+g0+b0>383)
    {
        r1 = 0; g1 = 0; b1 = 0;
    }
    else
    {
        r1 = 255; g1 = 255; b1 = 255;
    }


    rectangle(RGBimage, Point(x_from,y_from), Point(x_to, y_to), Scalar(b0,g0,r0), cv::FILLED, 8, 0);
    char str[10];
    sprintf(str,"%2.2f",val);
    putText(RGBimage, str, Point2f(x_from+10,y_to-8), fontFace, fontScale,  Scalar(b1,g1,r1,255),thickness);
    char str1[10];
    if (channel>0)
        sprintf(str1,"CH%i",channel);
    else if (channel==0)
        sprintf(str1,"Profile");

    rectangle(RGBimage, Point(x_to+9,y_from), Point(x_to+150, y_to), Scalar(0,0,0), cv::FILLED, 8, 0);
    putText(RGBimage, str1, Point2f(x_to+10,y_to-8), fontFace, fontScale,  Scalar(255,255,255,255),thickness);
}

double Overlay::getValue(int x, int y)
{
    double val = values.at<float>(y,x);
    return val;
}
