#ifndef TCP_IP_H
#define TCP_IP_H
#include <winsock2.h>
#include <cstdlib>
#include <string>
#include <stdio.h>
#include <boost/thread/thread.hpp>
#include <boost/date_time.hpp>

#define DEFAULT_BUFLEN 4096

using namespace std;

class TCP_IP
{
private:
    const char *ip, *port;
    bool isConnected;
    vector<double> tempdata; // temporary vector to hold the data
    char recvbuf[4096];
    SOCKET socketfd;

public:

    TCP_IP(const char *ip, const char *port);
    void write(const char *sendbuf);
    void checkConnection();
    void read(string *output, int *len, vector<double> *data);
    bool disconnect();
    bool isOpen();
    void set_transaction(bool assert);
    bool get_transaction();
    SOCKET get_socket();
    bool inTransaction;





};

#endif // TCP_IP_H
