#include "iopath.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <windows.h>

using namespace std;

std::string IOPath::datapath = "data";
std::string IOPath::counter = "counter.txt";

IOPath::IOPath()
{

}

string IOPath::getAppDir()
{
    TCHAR szFileName[MAX_PATH];
    HINSTANCE hInstance = GetModuleHandle(NULL);
    GetModuleFileName(hInstance, szFileName, MAX_PATH);
    wstring ws(&szFileName[0]); //convert to wstring
    string fullpath(ws.begin(), ws.end()); //and convert to string.

    string directory;
    const size_t last_slash_idx = fullpath.rfind('\\');
    if (std::string::npos != last_slash_idx)
    {
        directory = fullpath.substr(0, last_slash_idx+1);
    }

    return directory;
}

string IOPath::getDataOutputFilename(string infix, string suffix)
{
    string path = getAppDir();
    path.append(datapath);
    path.append("\\");
    path.append(infix);
    path.append(".");
    path.append(suffix);
    return path;
}

string IOPath::getCurrentCounter()
{
    string cntfile = getAppDir();
    cntfile.append(counter);
    ifstream infile(cntfile); //cntfile
    string sLine;
    if (infile.good())
        getline(infile, sLine);
    else
    {
        ofstream newFile(cntfile);
        newFile << "1";
        newFile.close();
        return 0;
    }

    infile.close();
    int i_dec = std::stoi(sLine);

    ofstream newFile(cntfile);
    newFile << std::to_string(i_dec+1);
    newFile.close();

    return sLine;
}
