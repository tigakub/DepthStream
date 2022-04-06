#include "Buffer.hpp"
#include "Connection.hpp"

namespace ds {
    void BufferHandler::handlerProc(BufferHandler *iSelf) {
        iSelf->handlerLoop();
    }

    void BufferHandler::handlerLoop() {
        alive = true;
        Buffer *aBuf;
        while(alive) {
            sem.wait();
            if(!alive) return;
            aBuf = cnx.popRecvBuffer();
            if(aBuf) {
                functor(cnx, *aBuf);
                cnx.returnRecvBuffer(aBuf);
            }
        }
    }
}