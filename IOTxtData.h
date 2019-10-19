#ifndef IOTXTDATA_H
#define IOTXTDATA_H

#include <iostream>
#include <fstream>
#include <stdint.h>
#include <QDebug>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "segmentation.h"
#include "stereosegmentation.h"

using namespace std;

class IOTxtData
{
public:
    //IOTxtData();
    static std::vector<double> getLifetimeData(string file, int channel);
    static void writeMatData(string filename, Segmentation * seg);
    static void writeTxtFile(string filename, Segmentation * seg);
    static void writeTxtFile(string filename, StereoSegmentation * seg);
    static void writeJpgFile_stereo(string filename, StereoSegmentation * seg, int channel );
    static void writeJpgFile_mono(string filename, Segmentation * seg, int channel );
    static void writeLogFile(string filename, vector<double> log);
    static void writeSegmentationLog(string filename, vector<double> time, vector<double> log_pulse_thres, vector<double> log_pulse_min, vector<double> log_pulse_max, vector<double> log_pulse_cur);
    static void writeStereoLog(string filename, vector<double> disparity_y, vector<int> is_sync);
    static void writeOneLineTxt(ofstream &outputFile, Segmentation *seg);
};

#endif // IOTxtData_H
