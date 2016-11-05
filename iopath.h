#ifndef IOPATH_H
#define IOPATH_H
#include <iostream>


class IOPath
{
public:
    IOPath();
    static std::string getAppDir();
    static std::string getDataOutputFilename(std::string infix, std::string suffix, std::string category);
    static std::string getandincreaseCurrentCounter();
    static std::string getCurrentCounter();
private:
    static std::string datapath;
    static std::string videopath;
    static std::string txtpath;
    static std::string figurepath;
    static std::string logpath;
    static std::string counter;

};

#endif // IOPATH_H
