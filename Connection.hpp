//
//  Connection.hpp
//  SocketServer
//
//  Created by Edward Janne on 4/1/22.
//

#ifndef Connection_hpp
#define Connection_hpp

#include <stdio.h>
#include <sys/types.h>
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
#include "Buffer.hpp"

using namespace std;

namespace ds {
    class Server;

    class Connection {
        protected:
            typedef enum {
                HEADER = 1,
                PAYLOAD = 2
            } Mode;

            class Ndx {
                public:
                    Ndx()
                    : payload(nullptr), index(0), payloadSize(0) { }

                    void set(char *iBuffer, uint32_t iDataSize) {
                        payload = iBuffer;
                        index = 0;
                        payloadSize = iDataSize;
                    }

                    uint32_t remaining() {
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
                    uint32_t index;
                    uint32_t payloadSize;
            };

        public:
            Connection(BufferFunctor &iFunctor, Server *iServer = nullptr)
            : server(iServer),
            sock(0), sockPipe(0), peersa(),
            recvThread(nullptr),
            sendThread(nullptr),
            performShutdown(false),
            sendLock(), sendQueue(),
            returnLock(), returnQueue(),
            recvLock(), recvQueue(),
            poolLock(), poolQueue(),
            sendIndex(), recvIndex(),
            sequenceCount(0),
            bfrHandler(*this, iFunctor) {
            }

            virtual ~Connection() {
                stop();
                sendLock.lock();
                while(sendQueue.size()) {
                    Buffer *aBuf = sendQueue.front();
                    sendQueue.pop_front();
                    delete aBuf;
                }
                sendLock.unlock();
                returnLock.lock();
                while(returnQueue.size()) {
                    Buffer *aBuf = returnQueue.front();
                    returnQueue.pop_front();
                    delete aBuf;
                }
                returnLock.unlock();
                recvLock.lock();
                while(recvQueue.size()) {
                    Buffer *aBuf = recvQueue.front();
                    recvQueue.pop_front();
                    delete aBuf;
                }
                recvLock.unlock();
                poolLock.lock();
                while(poolQueue.size()) {
                    Buffer *aBuf = poolQueue.front();
                    poolQueue.pop_front();
                    delete aBuf;
                }
                poolLock.unlock();
            }

            void accept(int iSocket, struct sockaddr_in iPeersa) {
                sock = iSocket;
                
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
            
            Buffer *popRecvBuffer() {
                Buffer *aBuf = nullptr;
                recvLock.lock();
                if(recvQueue.size()) {
                    aBuf = recvQueue.front();
                    recvQueue.pop_front();
                }
                recvLock.unlock();
                return aBuf;
            }
            
            void returnRecvBuffer(Buffer *iBuf) {
                poolLock.lock();
                poolQueue.push_back(iBuf);
                poolLock.unlock();
            }

            void recvLoop() {
                fd_set sockSet;
                struct timeval timeout;
                Buffer *currentBuf = nullptr;
                Mode mode = HEADER;
                Header swappedHeader;
                sockPipe |= 1;
                bfrHandler.start();
                while(!performShutdown) {
                    if(!currentBuf) {
                        poolLock.lock();
                        if(poolQueue.size()) {
                            currentBuf = poolQueue.front();
                            poolQueue.pop_front();
                        } else {
                            currentBuf = new Buffer(new Header());
                        }
                        poolLock.unlock();
                        recvIndex.set((char *) swappedHeader.startPtr(), swappedHeader.size());
                        mode = HEADER;
                    }
                    if(currentBuf) {
                        FD_ZERO(&sockSet);
                        FD_SET(sock, &sockSet);
                        timeout.tv_sec = 1;
                        timeout.tv_usec = 0;
                        int n = select(sock+1, &sockSet, NULL, NULL, &timeout);
                        if(n < 0) performShutdown = true;
                        if(n > 0) {
                            ssize_t rcvdBytes = read(sock, recvIndex.next(), recvIndex.remaining());
                            if(rcvdBytes > 0) {
                                recvIndex.advance(uint32_t(rcvdBytes));
                                if(!recvIndex.remaining()) {
                                    switch(mode) {
                                        case HEADER:
                                            swappedHeader.netToHost();
                                            currentBuf->getHeader() = swappedHeader;
                                            recvIndex.set(currentBuf->getPayload(), currentBuf->getHeader().getPayloadSize());
                                            currentBuf->growIfNeeded(currentBuf->getHeader().getPayloadSize());
                                            mode = PAYLOAD;
                                            break;
                                        case PAYLOAD:
                                            recvLock.lock();
                                            recvQueue.push_back(currentBuf);
                                            recvLock.unlock();
                                            bfrHandler.signal();
                                            currentBuf = nullptr;
                                            mode = HEADER;
                                            break;
                                    }
                                }
                            } else {
                                performShutdown = true;
                            }
                        }
                    }
                }
                bfrHandler.stop();
                shutdown(sock, SHUT_RD);
                char *tmpBuf[256];
                while(read(sock, tmpBuf, 256) > 0);
                sockPipe &= ~1;
                if(!sockPipe) {
                    shutdownNotification();
                }
            }

            void submitSendBuffer(Buffer *iBuf) {
                sendLock.lock();
                sendQueue.push_back(iBuf);
                sendLock.unlock();
            }

            Buffer *recoverSendBuffer() {
                Buffer *theBuf = nullptr;
                returnLock.lock();
                if(returnQueue.size()) {
                    theBuf = returnQueue.back();
                    returnQueue.pop_back();
                }
                returnLock.unlock();
                return theBuf;
            }

            void sendLoop() {
                fd_set sockSet;
                struct timeval timeout;
                Buffer *currentBuf = nullptr;
                Mode mode = HEADER;
                Header swappedHeader;
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
                            /*
                            swappedHeader = currentBuf->getHeader();
                            swappedHeader.hostToNet();
                            sendIndex.set((char *) &swappedHeader, swappedHeader.size());
                            */
                            Header &header = currentBuf->getHeader();
                            header.signature = 0x626c616d;
                            header.sequenceCount = sequenceCount;
                            if(header.payloadSize == 0) {
                                cout << "Zero payload message ignored" << endl;
                                returnLock.lock();
                                returnQueue.push_back(currentBuf);
                                returnLock.unlock();
                                currentBuf = nullptr;
                            } else {
                                /*
                                uint8_t *hdrPtr = ((uint8_t*) header.startPtr());
                                cout << "Actual header bytes:              ";
                                for(int i = 0; i < sizeof(Header); i++) {
                                    cout << hex << int(hdrPtr[i]) << " ";
                                }
                                cout << endl << endl;
                                */
                                header.hostToNet();
                                sendIndex.set((char *) header.startPtr(), header.size());
                                mode = HEADER;
                            }
                        }
                    }
                    if(currentBuf) {
                        uint32_t bytesToSend = sendIndex.remaining();
                        if(bytesToSend > 1024) bytesToSend = 1024;
                        /*
                        FD_ZERO(&sockSet);
                        FD_SET(sock, &sockSet);
                        timeout.tv_sec = 1;
                        timeout.tv_usec = 0;
                        int n = select(sock+1, &sockSet, NULL, NULL, &timeout);
                        if(n < 0) performShutdown = true;
                        */
                        if(mode == HEADER) {
                            uint8_t *hdrPtr = ((uint8_t*) currentBuf->getHeader().startPtr());
                            cout << "Bytes sent: ";
                            for(int i = 0; i < sizeof(Header); i++) {
                                cout << hex << int(hdrPtr[i]) << " ";
                            }
                            cout << endl;
                        }
                        ssize_t sentBytes = send(sock, sendIndex.next(), bytesToSend, 0);
                        if(sentBytes > 0) {
                            sendIndex.advance(uint32_t(sentBytes));
                            if(!sendIndex.remaining()) {
                                switch(mode) {
                                    case HEADER: {
                                        Header &header = currentBuf->getHeader();
                                        header.netToHost();
                                        sendIndex.set(currentBuf->getPayload(), currentBuf->getHeader().getPayloadSize());
                                        mode = PAYLOAD;
                                        }
                                        break;
                                    case PAYLOAD:
                                        returnLock.lock();
                                        returnQueue.push_back(currentBuf);
                                        returnLock.unlock();
                                        currentBuf = nullptr;
                                        sequenceCount++;
                                        mode = HEADER;
                                        break;
                                }
                            }
                        } else if(sentBytes < 0) {
                            // Error, what to do?
                        } else {
                            cout << "No bytes sent" << endl;
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
            deque<Buffer *> sendQueue;
            mutex returnLock;
            deque<Buffer *> returnQueue;
            mutex recvLock;
            deque<Buffer *> recvQueue;
            mutex poolLock;
            deque<Buffer *> poolQueue;
            Ndx sendIndex, recvIndex;
            uint32_t sequenceCount;
            BufferHandler bfrHandler;
    };

} // ds namespace

#endif /* Connection_hpp */
