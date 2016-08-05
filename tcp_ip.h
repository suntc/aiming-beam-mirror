#ifndef TCP_IP_H
#define TCP_IP_H
#include <winsock2.h>
#include <cstdlib>
#include <string>

#define DEFAULT_BUFLEN 512

using namespace std;

class TCP_IP
{
private:
    const char *ip, *port;
    bool isConnected;

    SOCKET socketfd;

public:

    TCP_IP(const char *ip, const char *port);
    void write(const char *sendbuf);
    void checkConnection();
    void read(string *output);
    bool disconnect();
    bool isOpen();
    void set_transaction(bool assert);
    bool get_transaction();
    SOCKET get_socket();
    bool inTransaction;





};

#endif // TCP_IP_H
