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

    typedef struct Header {
        uint32_t payloadSize;

        void hostToNet() {
            payloadSize = htonl(payloadSize);
        }

        void netToHost() {
            payloadSize = ntohl(payloadSize);
        }
    } Header;

    class Buffer {
        friend class Connection;

        public:
            Buffer(uint32_t iCapacity = 1024000)
            : payload(new char[iCapacity]), capacity(iCapacity) { }

            virtual ~Buffer() {
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

            Header &getHeader() { return header; }

        protected:
            Header header;
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
