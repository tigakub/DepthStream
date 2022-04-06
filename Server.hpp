//
//  Server.hpp
//  SocketServer
//
//  Created by Edward Janne on 4/1/22.
//

#ifndef Server_hpp
#define Server_hpp

#include <stdio.h>

#include <thread>
#include <mutex>
#include <atomic>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h> // For sockaddr_in
#include <sys/types.h>
#include <ifaddrs.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <unistd.h> // for read()
#include <sys/poll.h> // for pollfd()

#include <iostream>
#include <vector>
#include <deque>
#include <cstring>

#include "Semaphore.hpp"
#include "Exception.hpp"

using namespace std;

#include "Connection.hpp"

class Server {

    public:
        static Server shared;

    protected:
        Server(string iInterface, BfrFunctor &iBfrFunctor)
        : interface(iInterface), listenSock(0), shutdown(false), acceptThread(nullptr), bfrFunctor(iBfrFunctor) { }

        virtual ~Server() {
            stop();
            while(connections.size()) {
                Connection *aConnection = connections.back();
                connections.pop_back();
                delete aConnection;
            }
        }
    
    public:
        const string &ip() const {
            return ipString;
        }

        void start() {
            if(listenSock) return;

            listenSock = socket(AF_INET, SOCK_STREAM, 0);

            // Retrieve file descriptor status flags
            int sockFlags = fcntl(listenSock, F_GETFL);
            // Set non-blocking flag
            sockFlags |= O_NONBLOCK;
            // Store file descriptor status flags
            fcntl(listenSock, F_SETFL, sockFlags);
            
            sockaddr_in sa; socklen_t saLen = sizeof(sa);
            sa.sin_family = AF_INET;
            sa.sin_addr.s_addr = INADDR_ANY;
            sa.sin_port = htons(3060);
            
            int err;
            if((err = ::bind(listenSock, (struct sockaddr *) &sa, saLen)) < 0) {
                throw Exception(__FILE__, __LINE__, err, "Unable to bind server socket");
            }
            
            struct ifreq ifr;
            ifr.ifr_addr.sa_family = AF_INET;
            strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ-1);
            ioctl(listenSock, SIOCGIFADDR, &ifr);
            ipString = string(inet_ntoa(((struct sockaddr_in *) &ifr.ifr_addr)->sin_addr));

            /*
            char ipStrAddr[INET_ADDRSTRLEN+1];
            struct ifaddrs *ipAddrs;
            getifaddrs(&ipAddrs);
            if(ipAddrs != nullptr) {
                for(struct ifaddrs *next = ipAddrs; next != nullptr; next = next->ifa_next) {
                    inet_ntop(AF_INET, next->ifa_addr, ipStrAddr, INET_ADDRSTRLEN);
                    cout << next->ifa_name << " " << ipStrAddr << endl;
                }
                freeifaddrs(ipAddrs);
            }
            */

            if((err = listen(listenSock, 10)) < 0) {
                throw Exception(__FILE__, __LINE__, err, "Failed to place socket into listening mode");
            }

            acceptThread = new thread(Server::acceptThreadProc, this);
        }

        void stop() {
            if(listenSock) {
                close(listenSock);
                listenSock = 0;
            }
            if(acceptThread) {
                acceptThread->join();
                delete acceptThread;
                acceptThread = nullptr;
            }
        }

        void acceptLoop() {
            sockaddr_in peersa;
            auto peersaLen = sizeof(peersa);
            int newSock;
            while(!shutdown) {
                newSock = accept(listenSock, (struct sockaddr *) &peersa, (socklen_t *) &peersaLen);
                if(newSock < 0) {
                    if(errno == EWOULDBLOCK) {
                        sleep(1);
                    } else {
                        shutdown = true;
                    }
                } else if(newSock > 0) {
                    Connection *newConnection = new Connection(bfrFunctor, this);
                    newConnection->accept(newSock, peersa);
                    connections.push_back(newConnection);
                    cout << "  New incoming connection" << endl;
                }
            }
        }

        void sockDidShutdown(Connection *iCnx) {
            cout << "  Connection shutdown" << endl;
            vector<Connection *>::iterator i;
            for(i = connections.begin(); i != connections.end(); i++) {
                if(*i == iCnx) {
                    connections.erase(i);
                    return;
                }
            }
        }

    protected:
        string interface;
        int listenSock;
        string ipString;
        atomic_bool shutdown;
        thread *acceptThread;
        vector<Connection *> connections;
        BfrFunctor &bfrFunctor;

        static void acceptThreadProc(Server *iSelf) {
            iSelf->acceptLoop();
        }
};

#endif /* Server_hpp */
