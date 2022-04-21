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

    #pragma pack(push, 4)
    class Header {
        public:
            Header()
            : signature(0x626c616d), sequenceCount(0), payloadSize(0) { }
            virtual ~Header() { }
            
            virtual void hostToNet() {
                signature = htonl(signature);
                sequenceCount = htonl(sequenceCount);
                payloadSize = htonl(payloadSize);
                *((uint32_t *) &qi) = htonl(*((uint32_t *) &qi));
                *((uint32_t *) &qj) = htonl(*((uint32_t *) &qj));
                *((uint32_t *) &qk) = htonl(*((uint32_t *) &qk));
                *((uint32_t *) &qr) = htonl(*((uint32_t *) &qr));
            }

            virtual void netToHost() {
                signature = ntohl(signature);
                sequenceCount = ntohl(sequenceCount);
                payloadSize = ntohl(payloadSize);
                *((uint32_t *) &qi) = ntohl(*((uint32_t *) &qi));
                *((uint32_t *) &qj) = ntohl(*((uint32_t *) &qj));
                *((uint32_t *) &qk) = ntohl(*((uint32_t *) &qk));
                *((uint32_t *) &qr) = ntohl(*((uint32_t *) &qr));
            }
            
            virtual void *startPtr() { return (void *) &signature; }
            virtual uint32_t size() { return sizeof(Header); }
            
            void setPayloadSize(uint32_t iSize) { payloadSize = iSize; }
            uint32_t getPayloadSize() { return payloadSize; }

            void setRotation(float iqi, float iqj, float iqk, float iqr) {
                qi = iqi; qj = iqj; qk = iqk; qr = iqr;
            }
            void getRotation(float &oqi, float &oqj, float &oqk, float &oqr) {
                oqi = qi; oqj = qj; oqk = qk; oqr = qr;
            }

            uint32_t signature;
            uint32_t sequenceCount;
            uint32_t payloadSize;
            float qi, qj, qk, qr;
    };
    #pragma pack(pop)

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
