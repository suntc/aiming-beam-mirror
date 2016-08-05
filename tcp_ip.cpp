/*
This class is based on Dimitris work for the original
version of the aiming beam software.
*/

#include "tcp_ip.h"
#include <ws2tcpip.h>
#include <stdio.h>
#include <QDebug>
#include <cstdlib>
#include <boost/thread/thread.hpp>
#include <boost/date_time.hpp>

using namespace std;

TCP_IP::TCP_IP(const char *ip, const char *port)
{
    // Connecting to the server
    WSADATA wsaData;

    // initialize variable to hold the socketID
    socketfd = INVALID_SOCKET;
    isConnected = false;
    inTransaction = false;
    int status = -1;
    int WSStatus;

    // The struct that getaddrinfo() fills up with data.
    struct addrinfo host_info;

    // Pointer to the linked list of host_info's.
    struct addrinfo *host_info_list;

    // Initialize Winsock
    WSStatus = WSAStartup(MAKEWORD(2,2), &wsaData);
    if (WSStatus != 0)
    {
        qDebug() << "WSAStartup failed!";
    }
    else
    {

        // The MAN page of getaddrinfo() states "All  the other fields in the structure pointed
        // to by hints must contain either 0 or a null pointer, as appropriate." When a struct
        // is created in c++, it will be given a block of memory. This memory is not nessesary
        // empty. Therefor we use the memset function to make sure all fields are NULL.
        ZeroMemory(&host_info, sizeof(host_info));
        host_info.ai_family = AF_UNSPEC;     // IP version not specified. Can be both.
        host_info.ai_socktype = SOCK_STREAM; // Use SOCK_STREAM for TCP or SOCK_DGRAM for UDP.
        host_info.ai_protocol = IPPROTO_TCP;

        // Now fill up the linked list of host_info structs with server's address information.
        WSStatus = getaddrinfo(ip, port, &host_info, &host_info_list);

        if (WSStatus != 0)
        {
            qDebug() << "Error getting address info!";
        }
        else
        {
            do
            {
                socketfd = socket(host_info_list->ai_family, host_info_list->ai_socktype,host_info_list->ai_protocol);
                if (socketfd == INVALID_SOCKET)
                {
                    qDebug() << "Socket error!";
                    WSACleanup();
                }
                else
                {
                    // connect to server
                    status = connect(socketfd, host_info_list->ai_addr, host_info_list->ai_addrlen);
                    if (status == SOCKET_ERROR)
                    {
                        qDebug() << "Connection Error!";
                        closesocket(socketfd);
                        socketfd = INVALID_SOCKET;

                        // wait 100ms and reconnect
                        Sleep(100);
                    }

                    //qDebug() << status;
                }

            } while (status != 0);
        }

        // free address information struct from getaddinfo
        freeaddrinfo(host_info_list);

        if (socketfd == INVALID_SOCKET) // not connected
        {
            qDebug() << "Unable to connect to server!";
            WSACleanup();
            isConnected = false;
        }
        else
        {
            isConnected = true;
        }
    }

}

/*
 * Write to host
 */
void TCP_IP::write(const char *sendbuf)
{
    int status;

    // length of the command
    int len = strlen(sendbuf);

    // send data to host
    status = send(socketfd, sendbuf, len, 0);

    // check if data was sent. may want to do some cleanup here, or send some feedback
    if (status == SOCKET_ERROR)
    {
        isConnected = false;
    }
    else
    {
        // unknown. keep it connected for now
        isConnected = true;
    }
}

/*
 * Read from host
 */
void TCP_IP::read(string *output)
{
    int status;

    // buffer to hold command sent by master
    char recvbuf[512];

    // init variable for recvbuf length
    int len = 512;

    // receive from host
    status = recv(socketfd, recvbuf, len, 0);

    // clipping buffer, removes weird characters
    if (status > 0 && status < DEFAULT_BUFLEN-1)
    {
        recvbuf[status] = '\0';
    }

    // copy received cmd so that it is available outside the thread
    //strcpy(output, recvbuf);

    //qDebug() << recvbuf;
    *output = recvbuf;
}

void TCP_IP::set_transaction (bool assert)
{
    inTransaction = assert;
}

bool TCP_IP::get_transaction()
{
    return inTransaction;
}


/*
 * Checks socket status
 */
void TCP_IP::checkConnection()
{

    int status;

    // something unexpected arrived.
    // check socket status, to verify connection
    char error_code;
    int error_code_size = sizeof(error_code);
    status = getsockopt(socketfd, SOL_SOCKET, SO_KEEPALIVE, &error_code, &error_code_size);

    // disconnected
    if (error_code == SOCKET_ERROR)
    {
        isConnected = false;
    }
    else
    {
        // unknown. keep it connected for now
        isConnected = true;
    }
}

/*
 * Disconnects the connection and cleans up afterwards
 */
bool TCP_IP::disconnect()
{
    int status;

    // shutdown the connection
    status = shutdown(socketfd, SD_SEND);
    if (status == SOCKET_ERROR)
    {
        qDebug() << WSAGetLastError();
        closesocket(socketfd);
        WSACleanup();
        isConnected = true;
        return false;
    }

    // cleanup
    closesocket(socketfd);
    WSACleanup();
    isConnected = false;

    qDebug() << "Disconnected";
    return true;

}

/*
 * Checks if the connection is opened by probing socketfd
 */
bool TCP_IP::isOpen()
{
    return isConnected;
}

/*
 * Getter for socketfd
 */
SOCKET TCP_IP::get_socket()
{
    return socketfd;
}

