#include "stereocalibration.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include "videoinputstereo.h"
#include <iostream>
#include <QDebug>
#include <opencv2/opencv.hpp>



using namespace cv;
using namespace std;

StereoCalibration::StereoCalibration(string filename)
{
    loadCalibration(filename);
    convertMaps(rmap00, rmap01, map_x0, map_y0, CV_32FC1 , false );
    convertMaps(rmap10, rmap11, map_x1, map_y1, CV_32FC1 , false );
    RX = getRotationAngleBetweenCameras();
}

StereoCalibration::StereoCalibration(VideoInputStereo * stereoInput, Size boardSize, float squareSize)
{
    // Size(9,6), 0.5,
    captureCalibImages(stereoInput, boardSize, squareSize);
}

void StereoCalibration::captureCalibImages(VideoInputStereo * stereoInput, Size boardSize, float squareSize)
{
    Mat im_left;
    Mat im_right;

    //cvNamedWindow("calibration", CV_WINDOW_NORMAL);
    cvNamedWindow("calibration", CV_WINDOW_FULLSCREEN);
    //setWindowProperty("calibration",CV_WND_PROP_FULLSCREEN,CV_WINDOW_FULLSCREEN);

    int image_counter = 0;
    int k=0;
    vector<string> imagelist;
    while( true )
    {

        //im_left  = stereoInput->getNextFrame(0); //0
        //im_right = stereoInput->getNextFrame(1); //1
        stereoInput->getNextFrame(im_left, im_right);

        showStereo(im_left, im_right);
        k=waitKey(50);

        if (k==27)
        {
            if( imagelist.size() == 0 )
            {
                qDebug() << "test";
                destroyWindow("calibration");
                return;
            }
            calibrate(imagelist, boardSize, squareSize, true, true, true); //2.letzte false
            break;
        }

        if (k==32)
        {
            image_counter++;
            //imwrite("calib"+ std::to_string(image_counter) + "l.tif",im_left.clone());
            //imwrite("calib"+ std::to_string(image_counter) + "r.tif",im_right.clone());
            string left = "calib"+ std::to_string(image_counter) + "l.tif";
            const char * left1 = left.c_str();
            cvSaveImage(left1, &(IplImage(im_left)));
            string right = "calib"+ std::to_string(image_counter) + "r.tif";
            const char * right1 = right.c_str();
            cvSaveImage(right1, &(IplImage(im_right)));

            imagelist.push_back(left);
            imagelist.push_back(right);
        }

    }

}

void StereoCalibration::showStereo(Mat l, Mat r)
{
    Mat l1;
    Mat r1;
    resize(l,l1,Size(0,0),0.5,0.5);
    resize(r,r1,Size(0,0),0.5,0.5);
    int res_x = l1.cols;
    int res_y = l1.rows;

    // Create 1280x480 mat for window
    cv::Mat win_mat(Size(2*res_x, res_y), CV_8UC3);

    r1.copyTo(win_mat(Rect(0, 0, res_x, res_y)) );
    l1.copyTo(win_mat(Rect(res_x, 0, res_x, res_y)) );
    string str("Push space to capture and ESC to exit");
    putText(win_mat, str, Point2f(20,100), FONT_HERSHEY_PLAIN, 2,  Scalar(0,0,255,255));

    imshow("calibration", win_mat);
}


void StereoCalibration::loadCalibration(string filename)
{

    FileStorage fs;
    fs.open(filename, FileStorage::READ);
    qDebug() << filename.c_str();
    if( fs.isOpened() )
    {
        // load intrinsic parameters
        fs["M1"] >> cameraMatrix0;
        fs["M2"] >> cameraMatrix1;
        fs["D1"] >> distCoeffs0;
        fs["D2"] >> distCoeffs1;

        // load extrinsic parameters
        fs["R"] >> R;
        fs["T"] >> T;
        fs["R1"] >> R1;
        fs["P1"] >> P1;
        fs["R2"] >> R2;
        fs["P2"] >> P2;
        fs["Q"] >> Q;
        fs["PM1"] >> PM1;
        fs["PM2"] >> PM2;

        fs["rmap00"] >> rmap00;
        fs["rmap01"] >> rmap01;
        fs["rmap10"] >> rmap10;
        fs["rmap11"] >> rmap11;

        fs["valid0"] >> valid0;
        fs["valid1"] >> valid1;

        fs.release();
    }
    else
        cout << "Error: can not load the parameters\n";
}

void StereoCalibration::saveCalibration(string filename)
{
    // save intrinsic parameters
    FileStorage fs(filename, FileStorage::WRITE);
    if( fs.isOpened() )
    {
        fs << "M1" << cameraMatrix0 << "D1" << distCoeffs0 <<
            "M2" << cameraMatrix1 << "D2" << distCoeffs1 <<
            "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q << "PM1" << PM1 << "PM2" << PM2 <<
            "valid0" << valid0 << "valid1" << valid1 << "rmap00" << rmap00 << "rmap10" << rmap10 << "rmap01" << rmap01 << "rmap11" << rmap11;
        fs.release();
    }
    else
        cout << "Error: can not save the intrinsic parameters\n";
}


void StereoCalibration::calibrate(const vector<string> imagelist, Size boardSize, float squareSize, bool displayCorners, bool useCalibrated, bool showRectified)
{
    qDebug() << imagelist.size();
    // even number of images!!
    if( imagelist.size() % 2 != 0 )
    {
        cout << "Error: the image list contains odd (non-even) number of elements\n";
        destroyWindow("calibration");
        return;
    }



    const int maxScale = 2;
    // ARRAY AND VECTOR STORAGE:

    vector<vector<Point2f> > imagePoints[2];
    vector<vector<Point3f> > objectPoints;
    Size imageSize;

    int c11=0;

    int i, j, k, nimages = (int)imagelist.size()/2;

    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    vector<string> goodImageList;

    for( i = j = 0; i < nimages; i++ )
    {
        for( k = 0; k < 2; k++ )
        {
            const string& filename = imagelist[i*2+k];
            Mat img = imread(filename, 0);
            if(! img.data )
                {
                    qDebug() << "File not Found";
                    qDebug() << filename.c_str();
                    //exit(-1);
                }

            if(img.empty())
                break;
            if( imageSize == Size() )
                imageSize = img.size();
            else if( img.size() != imageSize )
            {
                cout << "The size of image " << filename << " differs from the first image size. Skipping the pair\n";
                break;
            }
            bool found = false;
            vector<Point2f>& corners = imagePoints[k][j];
            for( int scale = 1; scale <= maxScale; scale++ )
            {
                Mat timg;
                if( scale == 1 )
                    timg = img;
                else
                    resize(img, timg, Size(), scale, scale);
                found = false;
                if (timg.empty())
                    qDebug() << "empty";


                found = findChessboardCorners(timg, boardSize, corners,
                CV_CALIB_CB_FAST_CHECK | CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

                //found = findCirclesGrid(timg,Size(11,4),corners,CALIB_CB_ASYMMETRIC_GRID);

                if( found )
                {
                    if( scale > 1 )
                    {
                        Mat cornersMat(corners);
                        cornersMat *= 1./scale;
                    }
                    break;
                }
            }
            if( displayCorners )
            {
                cout << filename << endl;
                Mat cimg, cimg1;
                cvtColor(img, cimg, COLOR_GRAY2BGR);
                drawChessboardCorners(cimg, boardSize, corners, found);
                double sf = 640./MAX(img.rows, img.cols);
                resize(cimg, cimg1, Size(), sf, sf);
                imshow("calibration", cimg1);
                c11 = waitKey(500);
                if( c11 == 27 ) //Allow ESC to quit
                {
                    exit(-1);
                }
            }
            else
            {
                putchar('.');
            }
            if( !found )
                break;
            cornerSubPix(img, corners, Size(11,11), Size(-1,-1),
                         TermCriteria(TermCriteria::COUNT+TermCriteria::EPS,
                                      30, 0.01));
        }
        if( k == 2 )
        {
            goodImageList.push_back(imagelist[i*2]);
            goodImageList.push_back(imagelist[i*2+1]);
            j++;
        }
    }
    cout << j << " pairs have been successfully detected.\n";
    nimages = j;
    if( nimages < 2 )
    {
        cout << "Error: too little pairs to run the calibration\n";
        destroyWindow("calibration");
        ready=false;
        return;
    }

    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    objectPoints.resize(nimages);

    for( i = 0; i < nimages; i++ )
    {
        for( j = 0; j < boardSize.height; j++ )
            for( k = 0; k < boardSize.width; k++ )
                objectPoints[i].push_back(Point3f(k*squareSize, j*squareSize, 0));
    }

    cout << "Running stereo calibration ...\n";
    Mat cameraMatrix[2], distCoeffs[2];

    //vector<vector<Point2f>> ip0 = imagePoints[0];
    //vector<vector<Point2f>> ip1 = imagePoints[1];


    cameraMatrix[0] = initCameraMatrix2D(objectPoints,imagePoints[0],imageSize,0);
    cameraMatrix[1] = initCameraMatrix2D(objectPoints,imagePoints[1],imageSize,0);

    double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
                    cameraMatrix[0], distCoeffs[0],
                    cameraMatrix[1], distCoeffs[1],
                    imageSize, R, T, E, F,
                    CALIB_FIX_ASPECT_RATIO +
                    CALIB_ZERO_TANGENT_DIST +
                    CALIB_USE_INTRINSIC_GUESS +
                    CALIB_SAME_FOCAL_LENGTH +
                    CALIB_RATIONAL_MODEL +
                    CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,
                    TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 100, 1e-5) );
    cout << "done with RMS error=" << rms << endl;



// CALIBRATION QUALITY CHECK
// because the output fundamental matrix implicitly
// includes all the output information,
// we can check the quality of calibration using the
// epipolar geometry constraint: m2^t*F*m1=0
    double err = 0;
    int npoints = 0;
    vector<Vec3f> lines[2];
    for( i = 0; i < nimages; i++ )
    {
        int npt = (int)imagePoints[0][i].size();
        Mat imgpt[2];
        for( k = 0; k < 2; k++ )
        {
            imgpt[k] = Mat(imagePoints[k][i]);
            undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
            computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);
        }
        for( j = 0; j < npt; j++ )
        {
            double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
                                imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
                           fabs(imagePoints[1][i][j].x*lines[0][j][0] +
                                imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
            err += errij;
        }
        npoints += npt;
    }
    cout << "average epipolar err = " <<  err/npoints << endl;




    Rect validRoi[2];

    stereoRectify(cameraMatrix[0], distCoeffs[0],
                  cameraMatrix[1], distCoeffs[1],
                  imageSize, R, T, R1, R2, P1, P2, Q,
                  0, 1, imageSize, &validRoi[0], &validRoi[1]); //CALIB_ZERO_DISPARITY

    PM1 = P1.clone();
    PM2 = P2.clone();

    // OpenCV can handle left-right
    // or up-down camera arrangements
    bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

// COMPUTE AND DISPLAY RECTIFICATION
    if( !showRectified )
        return;

    Mat rmap[2][2];
// IF BY CALIBRATED (BOUGUET'S METHOD)
    if( useCalibrated )
    {
        // we already computed everything
    }
// OR ELSE HARTLEY'S METHOD
    else
 // use intrinsic parameters of each camera, but
 // compute the rectification transformation directly
 // from the fundamental matrix
    {
        vector<Point2f> allimgpt[2];
        for( k = 0; k < 2; k++ )
        {
            for( i = 0; i < nimages; i++ )
                std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
        }
        F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);

        stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3); // H1, H2

        R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
        R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
        P1 = cameraMatrix[0];
        P2 = cameraMatrix[1];
    }

    std::cout << endl << "P1" << endl;
    std::cout << P1;
    std::cout << endl << "P2" << endl;
    std::cout << P2 << endl << endl;

    std::cout << endl << "R1" << endl;
    std::cout << R1;
    std::cout << endl << "R2" << endl;
    std::cout << R2;


    //Precompute maps for cv::remap()
    initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

   // initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_32F, map_x0, map_y0);
   // initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_32F, map_x1, map_y1);

    std::cout << endl;
    std::cout << P1;
    std::cout << endl;

    Mat canvas;
    double sf;
    int w, h;
    if( !isVerticalStereo )
    {
        sf = 600./MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width*sf);
        h = cvRound(imageSize.height*sf);
        canvas.create(h, w*2, CV_8UC3);
    }
    else
    {
        sf = 300./MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width*sf);
        h = cvRound(imageSize.height*sf);
        canvas.create(h*2, w, CV_8UC3);
    }

    for( i = 0; i < nimages; i++ )
    {
        for( k = 0; k < 2; k++ )
        {
            Mat img = imread(goodImageList[i*2+k], 0), rimg, cimg;
            remap(img, rimg, rmap[k][0], rmap[k][1], INTER_LINEAR);
            cvtColor(rimg, cimg, COLOR_GRAY2BGR);
            Mat canvasPart = !isVerticalStereo ? canvas(Rect(w*k, 0, w, h)) : canvas(Rect(0, h*k, w, h));
            resize(cimg, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
            if( useCalibrated )
            {
                Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
                          cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));
                rectangle(canvasPart, vroi, Scalar(0,0,255), 3, 8);
            }
        }

        if( !isVerticalStereo )
            for( j = 0; j < canvas.rows; j += 16 )
                line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
        else
            for( j = 0; j < canvas.cols; j += 16 )
                line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
        destroyWindow("calibration");
        imshow("rectified", canvas);
    //    char c = (char)waitKey();
    //    if( c == 27 || c == 'q' || c == 'Q' )
    //        break;
    }

    distCoeffs0 = distCoeffs[0];
    distCoeffs1 = distCoeffs[1];
    cameraMatrix0 = cameraMatrix[0];
    cameraMatrix1 = cameraMatrix[1];

    rmap00 = rmap[0][0];
    rmap01 = rmap[0][1];
    rmap10 = rmap[1][0];
    rmap11 = rmap[1][1];

    imSize = imageSize;

    valid0 = validRoi[0];
    valid1 = validRoi[1];

    convertMaps(rmap00, rmap01, map_x0, map_y0, CV_32FC1 , false );
    convertMaps(rmap10, rmap11, map_x1, map_y1, CV_32FC1 , false );

    cout << "Calibration Finished!";

    cout << std::endl << "H1" << std::endl;
    cout << H1;
    cout << std::endl << "H2" << std::endl;
    cout << H2;

    cout << std::endl << "T" << std::endl;
    cout << T;

    //Mat Rt0;
    //Mat T0 = Mat::zeros(3, 1, CV_64F);
    //cv::hconcat(R1,T0,Rt0);
    //PM1 = cameraMatrix0 *Rt0;

    cout << std::endl << "PM1" << std::endl;
    cout << PM1;

    //Mat Rt1;
    //cv::hconcat(R2,T0,Rt1);
    //PM2 = cameraMatrix1 *Rt1;

    cout << std::endl << "PM2" << std::endl;
    cout << PM2;

    cout << std::endl << "Finished!";
    cout << std::endl;
    ready = true;
}

Mat StereoCalibration::getProjectionMatrix(int ind)
{
    Mat res;
    if (ind==0)
        res = getOptimalNewCameraMatrix(cameraMatrix0, distCoeffs0, imSize, 0, imSize);
    else
        res = getOptimalNewCameraMatrix(cameraMatrix1, distCoeffs1, imSize, 0, imSize);
    return res;
}

bool StereoCalibration::isReady()
{
    return ready;
}

Mat StereoCalibration::getRotationAngleBetweenCameras()
{
    // by openCV convention, the x-axis connects both camera origins
    // We want a coordinate system where x and y are parallel to the ground
    // so we rotate the coordinate system around the y-axis.

    double a = T.at<double>(0); // translation obtained from calibration
    double b = T.at<double>(2);
    double rx = atan(b/a); // rotation angle

    // compute rotation matrix
    Mat rotCoord = (Mat_<double>(4,4) <<
                                        cos(rx),        0,          sin(rx),    0,
                                        0,              1,          0,          0,
                                        -sin(rx),       0,          cos(rx),    0,
                                        0,              0,          0,          1);
    return rotCoord;
}

Mat StereoCalibration::reconstructPoint3D(Mat p_l, Mat p_r)
{
    Mat dest_hom(1, 4, CV_32FC4);
    triangulatePoints(PM1, PM2, p_l, p_r, dest_hom);
    return RX*dest_hom;
}

Mat StereoCalibration::getRectifiedIm(Mat img, int camID)
{
    Mat rimg;
    if (camID==0)
        remap(img, rimg, rmap00, rmap01, INTER_LINEAR);
        //remap(img, rimg, map_x0, map_y0, INTER_LINEAR);
    if (camID==1)
        remap(img, rimg, rmap10, rmap11, INTER_LINEAR);
        //remap(img, rimg, map_x1, map_y1, INTER_LINEAR);

    return rimg;
}

Point2d StereoCalibration::getRectifiedPoint(Point2d p, int camID)
{
    Point2d res;
    if (camID==0)
    {
        if (valid0.contains(p))
        {
            res.x = map_x0.at<float>(p.y,p.x);
            res.y = map_y0.at<float>(p.y,p.x);
        }
        else
        {
            res.x = 0;
            res.y = 0;
            std::cout << endl << "out of range" << endl;
        }
    }
    else
    {
        if (valid1.contains(p))
        {
            res.x = map_x1.at<float>(p.y,p.x);
            res.y = map_y1.at<float>(p.y,p.x);
        }
        else
        {
            res.x = 0;
            res.y = 0;
            std::cout << endl << "out of range" << endl;
        }
    }


    return res;
}

Mat StereoCalibration::projectTo3D(Mat frame_l, Mat frame_r)
{
    Mat rect_l = getRectifiedIm(frame_l,0);
    Mat rect_r = getRectifiedIm(frame_r,1);

    cvtColor(rect_l, rect_l, cv::COLOR_RGB2GRAY);
    cvtColor(rect_r, rect_r, cv::COLOR_RGB2GRAY);

    imshow("rectl", rect_l);
    imshow("rectr", rect_r);

    cv::Mat imgDisparity32F = Mat( rect_l.rows, rect_l.cols, CV_16S  );

    Ptr<StereoBM> sbm = cv::StereoBM::create(4*16,15); // 64,19


    sbm->compute(rect_l, rect_r, imgDisparity32F);

    cv::Mat XYZ(imgDisparity32F.size(),CV_32FC3);
    reprojectImageTo3D(imgDisparity32F, XYZ, Q, false, CV_32F);

    double minVal;
    double maxVal;
    minMaxIdx(imgDisparity32F, &minVal, &maxVal);
    Mat adjMap;
    //convertScaleAbs(XYZ, adjMap, 255 / max);
    // Display it as a CV_8UC1 image
    imgDisparity32F.convertTo( adjMap, CV_8UC1, 255/(maxVal - minVal));

    imshow("3d_im",adjMap);
    cout << adjMap;

    return XYZ;
}

Mat StereoCalibration::projectTo3D_(Mat left, Mat right)
{
    int max_disp = 4*16;//multiple of 16
    int wsize = 15;
    double matching_time;

    Mat rect_l = getRectifiedIm(right,0);
    Mat rect_r = getRectifiedIm(left,1);

    Mat left_for_matcher, right_for_matcher;
    Mat left_disp,right_disp;
    Mat filtered_disp;
    Mat XYZ;
    Mat conf_map = Mat(left.rows,left.cols,CV_8U);
    bool no_downscale = 0;
    Ptr< StereoMatcher > wls_filter;

    string filter = "wls_conf";
    string algo = "bm";

    if(filter=="wls_conf") // filtering with confidence (significantly better quality than wls_no_conf)
    {

        left_for_matcher  = left.clone();
        right_for_matcher = right.clone();


        if(algo=="bm")
        {
            // ! [matching]
            Ptr<StereoBM> left_matcher = StereoBM::create(max_disp,wsize);
            //wls_filter = cv::ximgproc::createDisparityWLSFilter(left_matcher);
            //Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);

            cvtColor(left_for_matcher,  left_for_matcher,  COLOR_BGR2GRAY);
            cvtColor(right_for_matcher, right_for_matcher, COLOR_BGR2GRAY);

            matching_time = (double)getTickCount();
            left_matcher-> compute(left_for_matcher, right_for_matcher,left_disp);
            //right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);
            matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();
            // ! [matching]
        }
        else if(algo=="sgbm")
        {
            Ptr<StereoSGBM> left_matcher  = StereoSGBM::create(0,max_disp,wsize);
            left_matcher->setP1(24*wsize*wsize);
            left_matcher->setP2(96*wsize*wsize);
            left_matcher->setPreFilterCap(63);
            //left_matcher->setMode(StereoSGBM::MODE_SGBM_3WAY);
            //wls_filter = createDisparityWLSFilter(left_matcher);
            //Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);

            matching_time = (double)getTickCount();
            left_matcher-> compute(left_for_matcher, right_for_matcher,left_disp);
            //right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);
            matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();
        }
        else
        {
            cout<<"Unsupported algorithm";
            //exit(-1);
        }

        // ! [filtering]
        //wls_filter->setLambda(lambda);
        //wls_filter->setSigmaColor(sigma);
        //filtering_time = (double)getTickCount();
        //wls_filter->filter(left_disp,left,filtered_disp,right_disp);
        //filtering_time = ((double)getTickCount() - filtering_time)/getTickFrequency();
        // ! [filtering]
        //conf_map = wls_filter->getConfidenceMap();

        // Get the ROI that was used in the last filter call:
        //ROI = wls_filter->getROI();
        if(!no_downscale)
        {
            // upscale raw disparity and ROI back for a proper comparison:
            resize(left_disp,left_disp,Size(),2.0,2.0);
            left_disp = left_disp*2.0;
            //ROI = Rect(ROI.x*2,ROI.y*2,ROI.width*2,ROI.height*2);
        }
    }
    else if(filter=="wls_no_conf")
    {
        /* There is no convenience function for the case of filtering with no confidence, so we
        will need to set the ROI and matcher parameters manually */

        left_for_matcher  = left.clone();
        right_for_matcher = right.clone();

        if(algo=="bm")
        {
            Ptr<StereoBM> matcher  = StereoBM::create(max_disp,wsize);
            matcher->setTextureThreshold(0);
            matcher->setUniquenessRatio(0);
            cvtColor(left_for_matcher,  left_for_matcher, COLOR_BGR2GRAY);
            cvtColor(right_for_matcher, right_for_matcher, COLOR_BGR2GRAY);
            //ROI = computeROI(left_for_matcher.size(),matcher);
            //wls_filter = createDisparityWLSFilterGeneric(false);
            //wls_filter->setDepthDiscontinuityRadius((int)ceil(0.33*wsize));

            matching_time = (double)getTickCount();
            matcher->compute(left_for_matcher,right_for_matcher,left_disp);
            matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();
        }
        else if(algo=="sgbm")
        {
            Ptr<StereoSGBM> matcher  = StereoSGBM::create(0,max_disp,wsize);
            matcher->setUniquenessRatio(0);
            matcher->setDisp12MaxDiff(1000000);
            matcher->setSpeckleWindowSize(0);
            matcher->setP1(24*wsize*wsize);
            matcher->setP2(96*wsize*wsize);
            matcher->setMode(StereoSGBM::MODE_SGBM);
            //ROI = computeROI(left_for_matcher.size(),matcher);
            //wls_filter = createDisparityWLSFilterGeneric(false);
            //wls_filter->setDepthDiscontinuityRadius((int)ceil(0.5*wsize));

            matching_time = (double)getTickCount();
            matcher->compute(left_for_matcher,right_for_matcher,left_disp);
            matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();
        }
        else
        {
            cout<<"Unsupported algorithm";
            //exit(-1);
        }
        reprojectImageTo3D(left_disp, XYZ, Q, false, CV_32F);
        //wls_filter->setLambda(lambda);
        //wls_filter->setSigmaColor(sigma);
        //filtering_time = (double)getTickCount();
        //wls_filter->filter(left_disp,left,filtered_disp,Mat(),ROI);
        //filtering_time = ((double)getTickCount() - filtering_time)/getTickFrequency();
    }

    return XYZ;
}


