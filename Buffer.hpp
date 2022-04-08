#ifndef __BUFFER_HPP__
#define __BUFFER_HPP__

#include <stdio.h>
#include <sys/types.h>
#include <thread>
#include <mutex>
#include <atomic>
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

#include "Semaphore.hpp"
#include "Exception.hpp"

namespace ds {
    class Server;
    class Connection;

    class Header {
        public:
            Header()
            : payloadSize(0) { }
            virtual ~Header() { }
            
            virtual void hostToNet() {
                payloadSize = htonl(payloadSize);
                *((uint32_t *) &qx) = htonl(*((uint32_t *) &qx));
                *((uint32_t *) &qy) = htonl(*((uint32_t *) &qy));
                *((uint32_t *) &qz) = htonl(*((uint32_t *) &qz));
                *((uint32_t *) &qw) = htonl(*((uint32_t *) &qw));
            }

            virtual void netToHost() {
                payloadSize = ntohl(payloadSize);
                *((uint32_t *) &qx) = ntohl(*((uint32_t *) &qx));
                *((uint32_t *) &qy) = ntohl(*((uint32_t *) &qy));
                *((uint32_t *) &qz) = ntohl(*((uint32_t *) &qz));
                *((uint32_t *) &qw) = ntohl(*((uint32_t *) &qw));
            }
            
            virtual uint32_t size() { return sizeof(Header); }
            
            void setPayloadSize(uint32_t iSize) { payloadSize = iSize; }
            uint32_t getPayloadSize() { return payloadSize; }

            void setRotation(float iqx, float iqy, float iqz, float iqw) {
                qx = iqx; qy = iqy; qz = iqz; qw = iqw;
            }
            void getRotation(float &oqx, float &oqy, float &oqz, float &oqw) {
                oqx = qz; oqy = qy; oqz = qz; oqw = qw;
            }
            
        protected:
            uint32_t payloadSize;
            float qx, qy, qz, qw;
    };

    class Buffer {
        friend class Connection;

        public:
            Buffer(Header *iHeader, uint32_t iCapacity = 1024000)
            : header(iHeader), payload(new char[iCapacity]), capacity(iCapacity) { }

            virtual ~Buffer() {
                if(header) delete header;
                if(payload) delete [] payload;
            }

            char *getPayload() { return payload; }

            void growIfNeeded(uint32_t iCapacity) {
                if(iCapacity > capacity) {
                    delete [] payload;
                    payload = new char[iCapacity];
                    capacity = iCapacity;
                }
            }

            uint32_t getCapacity() { return capacity; }

            Header &getHeader() { return *header; }

        protected:
            Header *header;
            char *payload;
            uint32_t capacity;
        };

    class BufferFunctor {
        public:
            virtual void operator()(Connection &, Buffer &) = 0;
    };

    class BufferHandler {
        public:
            BufferHandler(Connection &iCnx, BufferFunctor &iFunctor)
            : cnx(iCnx), handlerThread(nullptr), functor(iFunctor), alive(false), sem() { }
            
            virtual ~BufferHandler() {
                stop();
            }
            
            void start() {
                if(handlerThread) return;
                handlerThread = new thread(BufferHandler::handlerProc, this);
            }
            
            void stop() {
                alive = false;
                if(handlerThread) {
                    sem.signal();
                    handlerThread->join();
                    delete handlerThread;
                    handlerThread = nullptr;
                }
            }
            
            void handlerLoop();
            
            void signal() {
                sem.signal();
            }
            
        protected:
            static void handlerProc(BufferHandler *iSelf);

            Connection &cnx;
            thread *handlerThread;
            BufferFunctor &functor;
            atomic_bool alive;
            Semaphore sem;
    };
}

#endif
