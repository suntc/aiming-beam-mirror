#include "IOTxtData.h"
#include <sstream>
#include <fstream>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include "segmentation.h"
#include "stereosegmentation.h"
#include <QFile>
#include <QDebug>

using namespace std;

// Read Lifetime data from TXT file
vector<double> IOTxtData::getLifetimeData(string data_file, int channel)
{
    vector<double> v;

    string word;
    word.clear();

    ifstream file;
    file.open(data_file);
    int row = 1;

    while ( file >> word )
    {
        if (row==(1+channel))
        {
            if (word=="1.#INF" || word=="-1.#IND")
            {
                v.push_back( 0.0 );
                cout << word << endl;
            }
            else
            {
                v.push_back( atof(word.c_str()) );
            }
        }

        row++;
        word.clear();
        if (row==12)
        {
            row = 1;
        }
    }
    file.close();

    return v;
}

// Writes MAT file for MATLAB
void IOTxtData::writeMatData(string filename, Segmentation * seg)
{
    cv::Mat ch1 = seg->ch1_overlay->values;
    filename.insert (filename.size()-4, "1");
    std::fstream outputFile;

    outputFile.open( filename, std::ios::out ) ;

    for(int i=0; i<ch1.rows; i++)
    {
        for(int j=0; j<ch1.cols; j++)
        {
            outputFile << ch1.at<float>(i,j) << ", ";
        }
        outputFile << endl;

    }
    outputFile.close( );

    ch1 = seg->ch2_overlay->values;
    filename[filename.size()-5] = '2';
    outputFile.open( filename, std::ios::out ) ;

    for(int i=0; i<ch1.rows; i++)
    {
        for(int j=0; j<ch1.cols; j++)
        {
            outputFile << ch1.at<float>(i,j) << ", ";
        }
        outputFile << endl;

    }
    outputFile.close( );

    ch1 = seg->ch3_overlay->values;
    filename[filename.size()-5] = '3';
    outputFile.open( filename, std::ios::out ) ;

    for(int i=0; i<ch1.rows; i++)
    {
        for(int j=0; j<ch1.cols; j++)
        {
            outputFile << ch1.at<float>(i,j) << ", ";
        }
        outputFile << endl;

    }
    outputFile.close( );

    ch1 = seg->ch4_overlay->values;
    filename[filename.size()-5] = '4';
    outputFile.open( filename, std::ios::out ) ;

    for(int i=0; i<ch1.rows; i++)
    {
        for(int j=0; j<ch1.cols; j++)
        {
            outputFile << ch1.at<float>(i,j) << ", ";
        }
        outputFile << endl;

    }
    outputFile.close( );
}

// Writes TXT file for single camera segmentation
void IOTxtData::writeTxtFile(string filename, Segmentation * seg)
{
    ofstream outputFile;
    outputFile.open (filename);
    vector<double> frame_noms = seg->log_frame_no;

    vector<double>::iterator it_frame_noms = frame_noms.begin();
    vector<double>::iterator it_timer = seg->log_timer.begin();
    vector<int>::iterator it_pos_x = seg->log_coords_x.begin();
    vector<int>::iterator it_pos_y = seg->log_coords_y.begin();
    vector<int>::iterator it_rad = seg->log_radius.begin();

    vector<double>::iterator it_ch1 = seg->log_lt_ch1.begin();
    vector<double>::iterator it_ch2 = seg->log_lt_ch2.begin();
    vector<double>::iterator it_ch3 = seg->log_lt_ch3.begin();
    vector<double>::iterator it_ch4 = seg->log_lt_ch4.begin();

    while( it_frame_noms != frame_noms.end() )
    {
        outputFile <<std::to_string((int) *it_frame_noms) << "\t" << *it_ch1 << "\t" << *it_ch2 << "\t" << *it_ch3 << "\t" << *it_ch4 << "\t" << *it_pos_x << "\t" << *it_pos_y << "\t" << *it_rad << "\t" << *it_timer << "\n";
        it_frame_noms++; it_pos_x++; it_pos_y++; it_rad++;
        it_ch1++; it_ch2++; it_ch3++; it_ch4++; it_timer++;
    }
    outputFile.close();

    frame_noms.clear();
    return;
}


void IOTxtData::writeLogFile(string filename, vector<double> log)
{
    ofstream outputFile;
    outputFile.open (filename);
    vector<double>::iterator it = log.begin();
    while( it != log.end() )
    {
        outputFile << *it << " ";
        it++;
    }

    outputFile.close();
    return;
}

void IOTxtData::writeSegmentationLog(string filename, vector<double> time, vector<double> log_pulse_thres, vector<double> log_pulse_min, vector<double> log_pulse_max, vector<double> log_pulse_cur)
{
    // from string to qstring
    QString fileqs = QString::fromStdString(filename);

    QFile file(fileqs);
    if(file.open(QFile::WriteOnly | QFile::Truncate))
    {
        QTextStream output(&file);

        // header
        output << "Time\tThreshold\tMin\tMax\tValue\n";

        // write values
        for(std::vector<int>::size_type i = 0; i != time.size(); i++) {
            output << time[i] << "\t" << log_pulse_thres[i] << "\t" << log_pulse_min[i] << "\t" << log_pulse_max[i] << "\t" << log_pulse_cur[i] << "\n";
        }
    }

    file.close();
}

void IOTxtData::writeStereoLog(string filename, vector<double> disparity_y, vector<int> is_sync)
{
    // from string to qstring
    QString fileqs = QString::fromStdString(filename);

    QFile file(fileqs);
    if(file.open(QFile::WriteOnly | QFile::Truncate))
    {
        QTextStream output(&file);

        // header
        output << "Disparity_y\tSync\n";

        // write values
        for(std::vector<int>::size_type i = 0; i != disparity_y.size(); i++) {
            output << disparity_y[i] << "\t" << is_sync[i]<< "\n";
        }
    }

    file.close();
}



// Writes TXT file for stereo camera segmentation
void IOTxtData::writeTxtFile(string filename, StereoSegmentation * seg)
{
    ofstream outputFile;
    outputFile.open (filename);

    vector<double> frame_noms = seg->seg->log_frame_no;

    vector<double>::iterator it_frame_noms = frame_noms.begin();
    vector<int>::iterator it_pos_x = seg->log_coords_x.begin();
    vector<int>::iterator it_pos_y = seg->log_coords_y.begin();
    vector<int>::iterator it_rad = seg->log_radius.begin();
    vector<double>::iterator it_timer = seg->seg->log_timer.begin();

    vector<double>::iterator it_ch1 = seg->seg->log_lt_ch1.begin();
    vector<double>::iterator it_ch2 = seg->seg->log_lt_ch2.begin();
    vector<double>::iterator it_ch3 = seg->seg->log_lt_ch3.begin();
    vector<double>::iterator it_ch4 = seg->seg->log_lt_ch4.begin();

    vector<double>::iterator it_real_x = seg->log_real_x.begin();
    vector<double>::iterator it_real_y = seg->log_real_y.begin();
    vector<double>::iterator it_height = seg->log_height.begin();

    int x,y;
    double ch1, ch2, ch3, ch4, height;

    while( it_frame_noms != frame_noms.end() )
    {
        x = *it_pos_x;
        y = *it_pos_y;



        if (x>0 && y>0 && x<seg->ch1_overlay->values.rows && y<seg->ch1_overlay->values.cols)
        {
            ch1 = seg->ch1_overlay->getValue(x,y);
            ch2 = seg->ch2_overlay->getValue(x,y);
            ch3 = seg->ch3_overlay->getValue(x,y);
            ch4 = seg->ch4_overlay->getValue(x,y);
            height = seg->height_profile->getValue(x,y); // *it_height; //
        }
        else
        {
            ch1 = *it_ch1;
            ch2 = *it_ch2;
            ch3 = *it_ch3;
            ch4 = *it_ch4;
            height = *it_height;
        }

        outputFile << std::to_string((int) *it_frame_noms) << "\t" << ch1 << "\t" << ch2 << "\t" << ch3 << "\t" << ch4 << "\t" << x << "\t" << y << "\t" << *it_rad << "\t" << *it_timer << "\t" << *it_real_x << "\t" << *it_real_y << "\t" << height << "\n";

        it_frame_noms++; it_pos_x++; it_pos_y++; it_rad++; it_real_x++; it_real_y++; it_height++;
        it_ch1++; it_ch2++; it_ch3++; it_ch4++; it_timer++;
    }
    outputFile.close();
    return;
}

// Writes JPG file
void IOTxtData::writeJpgFile_mono(string filename, Segmentation * seg, int channel )
{
    if (!seg->ch1_overlay)
        return;

    cv::Mat c1;

    if(channel==0)  // raw image, no overlay
        c1 = seg->firstFrame;
    if(channel==1)
        addWeighted( seg->firstFrame, 0.5, seg->ch1_overlay->mergeOverlay(seg->firstFrame), 0.5, 0.0, c1);
    if(channel==2)
        addWeighted( seg->firstFrame, 0.5, seg->ch2_overlay->mergeOverlay(seg->firstFrame), 0.5, 0.0, c1);
    if(channel==3)
        addWeighted( seg->firstFrame, 0.5, seg->ch3_overlay->mergeOverlay(seg->firstFrame), 0.5, 0.0, c1);
    if(channel==4)
        addWeighted( seg->firstFrame, 0.5, seg->ch4_overlay->mergeOverlay(seg->firstFrame), 0.5, 0.0, c1);

    const char * filename1 = filename.c_str();
    cvSaveImage(filename1, &(IplImage(c1)));

    c1.release();
}

void IOTxtData::writeJpgFile_stereo(string filename, StereoSegmentation * seg, int channel )
{
    if(!seg->height_profile)
        return;

    cv::Mat c1;

    if(channel==0)  // raw image, no overlay
        c1 = seg->firstFrame;
    if(channel==1)
        addWeighted( seg->firstFrame, 0.5, seg->ch1_overlay->mergeOverlay(seg->firstFrame), 0.5, 0.0, c1);
    if(channel==2)
        addWeighted( seg->firstFrame, 0.5, seg->ch2_overlay->mergeOverlay(seg->firstFrame), 0.5, 0.0, c1);
    if(channel==3)
        addWeighted( seg->firstFrame, 0.5, seg->ch3_overlay->mergeOverlay(seg->firstFrame), 0.5, 0.0, c1);
    if(channel==4)
        addWeighted( seg->firstFrame, 0.5, seg->ch4_overlay->mergeOverlay(seg->firstFrame), 0.5, 0.0, c1);
    if(channel==5)  // profile
        addWeighted( seg->firstFrame, 0.5, seg->height_profile->mergeOverlay(seg->firstFrame), 0.5, 0.0, c1);

    const char * filename1 = filename.c_str();
    cvSaveImage(filename1, &(IplImage(c1)));

    c1.release();
}
