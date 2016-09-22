#ifndef IOPATH_H
#define IOPATH_H
#include <iostream>


class IOPath
{
public:
    IOPath();
    static std::string getAppDir();
    static std::string getDataOutputFilename(std::string infix, std::string suffix);
    static std::string getCurrentCounter();
private:
    static std::string datapath;
    static std::string counter;

};

#endif // IOPATH_H
