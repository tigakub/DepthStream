#ifndef __NETWORKING_HPP__
#define __HETWORKING_HPP__

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

#include "Exception.hpp"

using namespace std;

class Server;

class Connection {
    public:
        typedef struct Hdr {
            int payloadSize;

            void hostToNet() {
                payloadSize = htonl(payloadSize);
            }

            void netToHost() {
                payloadSize = ntohl(payloadSize);
            }
        } Hdr;

    class Bfr {
        friend class Connection;

        public:
            Bfr(int iCapacity = 1024000)
            : payload(new char[iCapacity]), capacity(iCapacity) { }

            virtual ~Bfr() {
                if(payload) delete [] payload;
            }

            char *getPayload() { return payload; }

            void growIfNeeded(int iCapacity) {
                if(iCapacity > capacity) {
                    delete [] payload;
                    payload = new char[iCapacity];
                    capacity = iCapacity;
                }
            }

            int getCapacity() { return capacity; }

            Hdr &getHeader() { return header; }

        protected:
            Hdr header;
            char *payload;
            int capacity;
        };

    protected:
        typedef enum {
            IDLE = 0,
            HEADER = 1,
            PAYLOAD = 2
        } Mode;

        class Ndx {
            public:
                Ndx()
                : payload(nullptr), index(0), payloadSize(0) { }

                void set(char *iBuffer, int iDataSize) {
                    payload = iBuffer;
                    index = 0;
                    payloadSize = iDataSize;
                }

                int remaining() {
                    return payloadSize - index;
                }

                char *next() {
                    return payload + index;
                }

                void advance(int iCount) {
                    index += iCount;
                    if(index > payloadSize) {
                        index = payloadSize;
                    }
                }

                char *payload;
                int index;
                int payloadSize;
        };

    public:
        Connection(Server *iServer = nullptr)
        : server(iServer), 
          sock(0), sockPipe(0), peersa(), 
          recvThread(nullptr),
          sendThread(nullptr),
          performShutdown(false),
          sendLock(), sendQueue(),
          returnLock(), returnQueue(),
          recvLock(), recvQueue(),
          poolLock(), poolQueue(),
          sendIndex(), recvIndex() {
        }

        virtual ~Connection() {
            stop();
            sendLock.lock();
            while(sendQueue.size()) {
                Bfr *aBuf = sendQueue.front();
                sendQueue.pop_front();
                delete aBuf;
            }
            sendLock.unlock();
            returnLock.lock();
            while(returnQueue.size()) {
                Bfr *aBuf = returnQueue.front();
                returnQueue.pop_front();
                delete aBuf;
            }
            returnLock.unlock();
            recvLock.lock();
            while(recvQueue.size()) {
                Bfr *aBuf = recvQueue.front();
                recvQueue.pop_front();
                delete aBuf;
            }
            recvLock.unlock();
            poolLock.lock();
            while(poolQueue.size()) {
                Bfr *aBuf = poolQueue.front();
                poolQueue.pop_front();
                delete aBuf;
            }
            poolLock.unlock();
        }

        void accept(int iSocket, struct sockaddr_in iPeersa) {
            // Retrieve file descriptor status flags
            int sockFlags = fcntl(sock, F_GETFL);
            // Set non-blocking flag
            sockFlags |= O_NONBLOCK;
            // Store file descriptor status flags
            fcntl(sock, F_SETFL, sockFlags);

            peersa = iPeersa;

            recvThread = new thread(Connection::recvThreadProc, this);
            sendThread = new thread(Connection::sendThreadProc, this);
        }

        void connect(const string &iHostAddress, int iPort) {
            if(sock) return;
            sock = socket(AF_INET, SOCK_STREAM, 0);
            if(sock <= 0) {
                int retValue = sock;
                sock = 0;
                throw Exception(__FILE__, __LINE__, retValue, "Failed to create socket");
            }
            struct hostent *host;
            host = gethostbyname(iHostAddress.c_str());
            if(!host) {
                sock = 0;
                throw Exception(__FILE__, __LINE__, 0, "Invalid host address");
            }
            bzero((char *) &peersa, sizeof(peersa));
            peersa.sin_family = AF_INET;
            bcopy((char *) host->h_addr,
                (char *) &peersa.sin_addr.s_addr,
                host->h_length);
            peersa.sin_port = htons(iPort);

            int err = ::connect(sock, (const sockaddr *) &peersa, sizeof(peersa));
            if(err < 0) {
                sock = 0;
                throw Exception(__FILE__, __LINE__, err, "Failed to connect to peer");
            }

            // Retrieve file descriptor status flags
            int sockFlags = fcntl(sock, F_GETFL);
            // Set non-blocking flag
            sockFlags |= O_NONBLOCK;
            // Store file descriptor status flags
            fcntl(sock, F_SETFL, sockFlags);

            recvThread = new thread(Connection::recvThreadProc, this);
            sendThread = new thread(Connection::sendThreadProc, this);
        }

        void recvLoop() {
            fd_set sockSet;
            struct timeval timeout;
            int byteCount = 0;
            Bfr *currentBuf;
            Mode mode = IDLE;
            Hdr swappedHeader;
            sockPipe |= 1;
            while(!performShutdown) {
                FD_ZERO(&sockSet);
                FD_SET(sock, &sockSet);
                timeout.tv_sec = 1;
                timeout.tv_usec = 0;
                int n = select(sock+1, &sockSet, NULL, NULL, &timeout);
                if(n == -1) performShutdown = true;
                if(n > 0) {
                    if(!currentBuf) {
                        poolLock.lock();
                        if(poolQueue.size()) {
                            currentBuf = poolQueue.front();
                            poolQueue.pop_front();
                        } else {
                            currentBuf = new Bfr();
                        }
                        poolLock.unlock();
                        recvIndex.set((char *) &swappedHeader, sizeof(swappedHeader));
                        mode = HEADER;
                    }
                    if(currentBuf) {
                        int rcvdBytes = read(sock, recvIndex.next(), recvIndex.remaining());
                        if(rcvdBytes) {
                            recvIndex.advance(rcvdBytes);
                            if(!recvIndex.remaining()) {
                                switch(mode) {
                                    case HEADER:
                                        swappedHeader.netToHost();
                                        currentBuf->header = swappedHeader;
                                        recvIndex.set(currentBuf->getPayload(), currentBuf->header.payloadSize);
                                        currentBuf->growIfNeeded(currentBuf->header.payloadSize);
                                        mode = PAYLOAD;
                                        break;
                                    case PAYLOAD:
                                        recvLock.lock();
                                        recvQueue.push_back(currentBuf);
                                        recvLock.unlock();
                                        currentBuf = nullptr;
                                        mode = IDLE;
                                        break;
                                }
                            }
                        } else {
                            performShutdown = true;
                        }
                    }
                }
            }
            shutdown(sock, SHUT_RD);
            char *tmpBuf[256];
            while(read(sock, tmpBuf, 256) > 0);
            sockPipe &= ~1;
            if(!sockPipe) {
                shutdownNotification();
            }
        }

        void submitSendBuffer(Bfr *iBuf) {
            sendLock.lock();
            sendQueue.push_back(iBuf);
            sendLock.unlock();
        }

        Bfr *recoverSendBuffer() {
            Bfr *theBuf = nullptr;
            returnLock.lock();
            if(returnQueue.size()) {
                theBuf = returnQueue.back();
                returnQueue.pop_back();
            }
            returnLock.unlock();
            return theBuf;
        }

        void sendLoop() {
            Bfr *currentBuf = nullptr;
            Mode mode = IDLE;
            Hdr swappedHeader;
            sockPipe |= 2;
            while(!performShutdown) {
                if(!currentBuf) {
                    sendLock.lock();
                    if(sendQueue.size()) {
                        currentBuf = sendQueue.front();
                        sendQueue.pop_front();
                    }
                    sendLock.unlock();
                    if(currentBuf) {
                        swappedHeader = currentBuf->header;
                        swappedHeader.hostToNet();
                        sendIndex.set((char *) &swappedHeader, sizeof(swappedHeader));
                        mode = HEADER;
                    }
                }
                if(currentBuf) {
                    int bytesToSend = sendIndex.remaining();
                    if(bytesToSend > 1024) bytesToSend = 1024;
                    int sentBytes = send(sock, sendIndex.next(), bytesToSend, 0);
                    if(sentBytes > 0) {
                        sendIndex.advance(sentBytes);
                        if(!sendIndex.remaining()) {
                            switch(mode) {
                                case HEADER:
                                    sendIndex.set(currentBuf->getPayload(), currentBuf->header.payloadSize);
                                    mode = PAYLOAD;
                                    break;
                                case PAYLOAD:
                                    returnLock.lock();
                                    returnQueue.push_back(currentBuf);
                                    returnLock.unlock();
                                    currentBuf = nullptr;
                                    mode = IDLE;
                                    break;
                            }
                        }
                    } else if(sentBytes < 0) {
                        // Error, what to do?
                    } else {
                        // cout << "No bytes sent" << endl;
                    }
                }
            }
            shutdown(sock, SHUT_WR);
            sockPipe &= ~2;
            if(!sockPipe) {
                shutdownNotification();
            }
        }

        void stop() {
            performShutdown = true;
            if(recvThread) {
                recvThread->join();
                delete recvThread;
                recvThread = nullptr;
            }
            if(sendThread) {
                sendThread->join();
                delete sendThread;
                sendThread = nullptr;
            }
            if(sock) close(sock);
            sock = 0;
        }

    protected:
        static void recvThreadProc(Connection *iCnx) {
            iCnx->recvLoop();
        }

        static void sendThreadProc(Connection *iCnx) {
            iCnx->sendLoop();
        }

        void shutdownNotification();

    protected:
        Server *server;
        int sock;
        atomic_int sockPipe;
        struct sockaddr_in peersa;
        thread *recvThread, *sendThread;
        atomic_bool performShutdown;
        mutex sendLock;
        deque<Bfr *> sendQueue;
        mutex returnLock;
        deque<Bfr *> returnQueue;
        mutex recvLock;
        deque<Bfr *> recvQueue;
        mutex poolLock;
        deque<Bfr *> poolQueue;
        Ndx sendIndex, recvIndex;
};

class Server {

    public:
        static Server shared;

    protected:
        Server()
        : listenSock(0), shutdown(false), acceptThread(nullptr) { }

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
            if((err = bind(listenSock, (struct sockaddr *) &sa, sizeof(sa))) < 0) {
                throw Exception(__FILE__, __LINE__, err, "Unable to bind server socket");
            }
            
            struct ifreq ifr;
            ifr.ifr_addr.sa_family = AF_INET;
            strncpy(ifr.ifr_name, "wlan0", IFNAMSIZ-1);
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
                } else {
                    Connection *newConnection = new Connection(this);
                    newConnection->accept(newSock, peersa);
                    connections.push_back(newConnection);
                    cout << "  New incoming connection" << endl;
                }
            }
        }

        void sockDidShutdown(Connection *iCnx) {
            vector<Connection *>::iterator i;
            for(i = connections.begin(); i != connections.end(); i++) {
                if(*i == iCnx) {
                    connections.erase(i);
                    delete iCnx;
                    return;
                }
            }
        }

    protected:
        int listenSock;
        string ipString;
        atomic_bool shutdown;
        thread *acceptThread;
        vector<Connection *> connections;

        static void acceptThreadProc(Server *iSelf) {
            iSelf->acceptLoop();
        }
};

inline void Connection::shutdownNotification() {
    if(server) server->sockDidShutdown(this);
}

#endif
